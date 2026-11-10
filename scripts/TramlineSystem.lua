--[[
    TramlineSystem.lua
    Mission-level glue: owns the store + overlay, hooks save/load, registers
    console commands and key bindings, and injects the TramlineDriver
    specialization into every drivable vehicle type.
]]

TramlineSystem = {}
TramlineSystem.modName = g_currentModName
TramlineSystem.modDir  = g_currentModDirectory

local TramlineSystem_mt = Class(TramlineSystem)

function TramlineSystem.new()
    local self = setmetatable({}, TramlineSystem_mt)
    self.store = TramlineStore.new()
    self.overlay = TramlineOverlay.new(self.store)
    self.menu = TramlineMenu.new(self)
    self.actionEventIds = {}
    return self
end

-------------------------------------------------------------------------------
-- Helpers
-------------------------------------------------------------------------------

function TramlineSystem:getPlayerVehicle()
    local v = g_currentMission.controlledVehicle
    if v == nil and g_localPlayer ~= nil and g_localPlayer.getCurrentVehicle ~= nil then
        v = g_localPlayer:getCurrentVehicle()
    end
    if v == false then v = nil end
    return v
end

function TramlineSystem:getPlayerWorldPos()
    local v = self:getPlayerVehicle()
    local node
    if v ~= nil then
        node = v.rootNode
    elseif g_localPlayer ~= nil then
        node = g_localPlayer.rootNode
    end
    if node == nil then return nil end
    local x, _, z = getWorldTranslation(node)
    return x, z
end

function TramlineSystem:getFieldIdAt(x, z)
    if not TramlineFieldUtil.isFieldGround(x, z) then return nil end
    return TramlineFieldUtil.getFieldIdAt(x, z)
end

function TramlineSystem:notify(text)
    if g_currentMission ~= nil and g_currentMission.hud ~= nil then
        g_currentMission:showBlinkingWarning(text, 2500)
    end
    TramlineLog.info("[TramlinePilot] %s", text)
end

-- a field's settings in one line, for the log
function TramlineSystem.describeSet(set)
    if set == nil then return "none" end
    local dir = math.deg(math.atan2(set.dx or 0, -(set.dz or 1))) % 360
    return string.format("spacing %g m, direction %.0f deg, %d headland pass(es) done %s, corners %s, turns %s, turn radius %g m, wheel track %.2f m, AutoDrive refill %s",
        set.spacing or 0, dir, set.headlands or 0, tostring(set.headlandOrder or "first"), tostring(set.corners or "round"),
        (set.turnStyle == 2) and "loops" or "along the headland", set.turnRadius or 0, set.trackWidth or 2,
        set.refillAD and "on" or "off")
end

-- the same, longer on screen and also in the game's notification list
function TramlineSystem:notifyImportant(text)
    if g_currentMission ~= nil and g_currentMission.hud ~= nil then
        g_currentMission:showBlinkingWarning(text, 6000)
        if g_currentMission.addIngameNotification ~= nil and FSBaseMission ~= nil then
            local kind = FSBaseMission.INGAME_NOTIFICATION_INFO or FSBaseMission.INGAME_NOTIFICATION_OK
            if kind ~= nil then pcall(g_currentMission.addIngameNotification, g_currentMission, kind, "Tramline Pilot: " .. text) end
        end
    end
    TramlineLog.info("[TramlinePilot] %s", text)
end

-------------------------------------------------------------------------------
-- Editing commands (used by both console and keybinds)
-------------------------------------------------------------------------------

function TramlineSystem:cmdSetA()
    local x, z = self:getPlayerWorldPos()
    if x == nil then return "no position" end
    self.overlay.pendingA = { x, z }
    self.overlay.pendingB = nil
    self:notify(string.format("A point set at %.1f, %.1f", x, z))
    return "ok"
end

function TramlineSystem:cmdSetB()
    local x, z = self:getPlayerWorldPos()
    if x == nil then return "no position" end
    if self.overlay.pendingA == nil then
        self:notify("Set A point first")
        return "no A point"
    end
    self.overlay.pendingB = { x, z }
    self:notify(string.format("B point set at %.1f, %.1f", x, z))
    return "ok"
end

function TramlineSystem:cmdWidth(arg)
    local w = tonumber(arg)
    if w == nil or w <= 0 then return "usage: tlWidth <metres>" end
    self.pendingSpacing = w
    self:notify(string.format("Tramline spacing: %.1f m", w))
    return "ok"
end

function TramlineSystem:cmdHeadland(arg)
    local n = tonumber(arg)
    if n == nil or n < 0 then return "usage: tlHeadland <passes>" end
    self.pendingHeadlands = math.floor(n)
    self:notify(string.format("Headland passes: %d", self.pendingHeadlands))
    return "ok"
end

function TramlineSystem:cmdGenerate()
    local x, z = self:getPlayerWorldPos()
    if x == nil then return "no position" end
    local fieldId = self:getFieldIdAt(x, z)
    if fieldId == nil then
        self:notify("Stand on a field to generate tramlines")
        return "not on a field"
    end

    local a = self.overlay.pendingA
    local b = self.overlay.pendingB
    local set = self.store:getOrCreateSet(fieldId)
    set.refX, set.refZ = x, z

    if a ~= nil then
        set.ax, set.az = a[1], a[2]
        if b ~= nil then
            set.dx, set.dz = b[1] - a[1], b[2] - a[2]
        else
            -- no B: use the vehicle's current heading
            local v = self:getPlayerVehicle()
            if v ~= nil then
                local dx, _, dz = localDirectionToWorld(v.rootNode, 0, 0, 1)
                set.dx, set.dz = dx, dz
            end
        end
    end
    if self.pendingSpacing   ~= nil then set.spacing   = self.pendingSpacing end
    if self.pendingHeadlands ~= nil then set.headlands = self.pendingHeadlands end

    local geo = self.store:regenerate(fieldId)
    self.overlay.pendingA, self.overlay.pendingB = nil, nil
    self:notify(string.format("Field %d: %d tramlines @ %.1f m", fieldId, #geo.lines, set.spacing))
    return "ok"
end

function TramlineSystem:cmdMenu()
    self.menu:open()
    return "ok"
end

function TramlineSystem:cmdScan()
    local x, z = self:getPlayerWorldPos()
    if x == nil then return "no position" end
    local info = FieldScanner.scan(x, z)
    if info == nil then return "not on a field" end
    return string.format("field %d: %.2f ha, %.0f x %.0f m, longest edge dir (%.2f, %.2f)",
        info.fieldId, info.areaHa, info.edgeLength, info.edgeWidth, info.edgeDx, info.edgeDz)
end

-- tlPaint [tracks|full] [wheel track m]  — create the stored tramlines of this
-- field in the crop (a width given is kept for the field)
function TramlineSystem:cmdPaint(arg, widthArg)
    local x, z = self:getPlayerWorldPos()
    if x == nil then return "no position" end
    local fieldId = self:getFieldIdAt(x, z)
    if fieldId == nil then return "not on a field" end
    local geo = self.store:getGeometry(fieldId)
    if geo == nil then return "no tramlines stored for this field" end
    if widthArg == nil and tonumber(arg) ~= nil then arg, widthArg = nil, arg end     -- tlPaint 2.25
    local mode = (arg == "full") and TramlinePainter.PAINT_FULL or TramlinePainter.PAINT_TRACKS
    local set = self.store:getSet(fieldId)
    local given = tonumber(widthArg)
    local w = TramlinePainter.clampTrack(given or (set ~= nil and set.trackWidth or nil))
    if given ~= nil and set ~= nil then
        set.trackWidth = w
        self.store.isDirty = true
    end
    local n = TramlinePainter.paint(geo, mode, w)
    local what = (mode == TramlinePainter.PAINT_FULL)
        and string.format("%.2f m wide lanes", TramlinePainter.laneWidth(w))
        or string.format("wheel tracks %.2f m apart", w)
    self:notify(string.format("Field %d: tramlines created in the crop, %s (%d pieces)", fieldId, what, n))
    return "ok"
end

function TramlineSystem:cmdClearAll()
    self.store:removeAll()
    self:notify("Cleared tramlines for all fields")
    return "ok"
end

function TramlineSystem:cmdClear()
    local x, z = self:getPlayerWorldPos()
    if x == nil then return "no position" end
    local fieldId = self:getFieldIdAt(x, z)
    if fieldId == nil then return "not on a field" end
    self.store:removeSet(fieldId)
    self:notify(string.format("Cleared tramlines for field %d", fieldId))
    return "ok"
end

function TramlineSystem:cmdOverlay()
    self:setShowMode(self.overlay.enabled and 1 or (self.store.lastShowMode or 2))
    self:notify("Tramline overlay " .. (self.overlay.enabled and "on" or "off"))
    return "ok"
end

-- 'Show lines': 1 off, 2 this field, 3 all fields. Only the player switches
-- the lines on or off (menu row, Alt+4); saved with the savegame.
function TramlineSystem:setShowMode(mode)
    mode = math.max(1, math.min(3, math.floor(tonumber(mode) or 2)))
    if mode ~= 1 then self.store.lastShowMode = mode end
    self.store.showMode = mode
    self.overlay.enabled = mode ~= 1
    self.store.isDirty = true
end

-------------------------------------------------------------------------------
-- Where and how the worker would start on a field, for a vehicle at (vx,vz)
-- facing (hx,hz). Shared by Alt+T and the start marker.
--   { kind = "resume", route, at, job, pct }  carry on with the unfinished job
--                                        from here (it stopped here, or the
--                                        bit passed over is tiny)
--   { kind = "choice", route, job, pct, x, z, dirX, dirZ,   unfinished job,
--     back, backInfo | backWhy, at }     but not here: 'back' = the way back
--                                        to where it stopped (or why there is
--                                        none), 'at' = carrying on from here
--   { kind = "new", route, at, job, joined }  a new job; joined = it starts on
--                                        a main tramline, not a headland one
--   { kind = "away", x, z, dirX, dirZ }  nothing within reach: where to drive to
-- at = TramlineRoute.resumeIndex result (where on the route it sets off).
-- forceNew: leave an unfinished job aside; preview: don't build routes that
-- are only needed to actually start; vehicle: for its refill gate.
-------------------------------------------------------------------------------

TramlineSystem.SKIP_OK = 20     -- m of spraying Alt+T may pass over without asking

function TramlineSystem:planStart(fieldId, vx, vz, hx, hz, forceNew, preview, vehicle)
    local geo = self.store:getGeometry(fieldId)
    if geo == nil or geo.lines == nil or #geo.lines == 0 then return nil end
    local set = self.store:getSet(fieldId) or {}
    local order = set.headlandOrder or "first"
    local corners = set.corners == "reverse" and (vehicle == nil or TramlineDriver.canReverse(vehicle))
    -- how far behind the vehicle the boom is (how far it backs into a corner)
    local boomBack = nil
    if corners and vehicle ~= nil then
        local ok, mk = pcall(TramlineDriver.findMarkerOffsets, vehicle)
        if ok and mk ~= nil and mk.back ~= nil then boomBack = math.floor(math.max(0, -mk.back) * 10 + 0.5) / 10 end
    end

    -- 1. the unfinished job
    local job = self.store:getJob(fieldId)
    if job ~= nil and not forceNew then
        local route = self.store:getJobRoute(fieldId)
        if route ~= nil then
            local done, total = TramlineRoute.progress(route, job.ix)
            local pct = total > 0 and math.floor(100 * done / total + 0.5) or 0
            local at = TramlineRoute.resumeIndex(route, job.ix, vx, vz, hx, hz, nil, job.x, job.z)
            if at ~= nil and (at.skipped or 0) <= TramlineSystem.SKIP_OK then
                return { kind = "resume", route = route, at = at, job = job, pct = pct }
            end
            -- not where it stopped: the way back there
            local back, info = self:backPlan(fieldId, geo, route, job, vx, vz, hx, hz, preview, vehicle, set)
            return { kind = "choice", route = route, at = at, job = job, pct = pct, unfinished = true,
                     back = back, backInfo = back ~= nil and info or nil, backWhy = back == nil and info or nil,
                     x = job.x or job.sx, z = job.z or job.sz, dirX = job.dirX or job.hx, dirZ = job.dirZ or job.hz }
        end
    end

    -- 2. a new job from a headland tramline (the usual way)
    local sp = TramlineRoute.startPoint(geo, vx, vz, hx, hz)
    if sp ~= nil and sp.ready then
        local at = { ix = 1, skipped = 0, x = sp.x, z = sp.z, dirX = sp.dirX, dirZ = sp.dirZ }
        local plan = { kind = "new", at = at, job = { sx = vx, sz = vz, hx = hx, hz = hz, order = order, corners = corners, boomBack = boomBack } }
        if not preview then
            plan.route = TramlineRoute.tagIndices(TramlineRoute.buildJob(geo, vx, vz, hx, hz,
                { headlandOrder = order, corners = corners, boomBack = boomBack }))
            if plan.route == nil then plan = nil end
        end
        if plan ~= nil then return plan end
    end

    -- 3. a new job from the main tramline or turn the vehicle is on
    local j = TramlineRoute.findJoin(geo, order, vx, vz, hx, hz, corners, boomBack)
    if j ~= nil then
        local c = j.cand
        return { kind = "new", route = c.route, at = j.at, joined = true,
                 job = { sx = c.sx, sz = c.sz, hx = c.hx, hz = c.hz, order = c.order, corners = c.corners, boomBack = c.boomBack } }
    end

    if sp ~= nil then return { kind = "away", x = sp.x, z = sp.z, dirX = sp.dirX, dirZ = sp.dirZ } end
    return nil
end

-- The course (the worker's whole route) for the start and finish flags:
-- where it starts, where it finishes and what is left of it.
local function distText(m)
    if m < 950 then return string.format("%d m", math.floor(m / 10 + 0.5) * 10) end
    return string.format("%.1f km", m / 1000)
end

-- job: the job under way (progress from job.ix and where it is, job.x/z),
-- nil for a job not started yet
function TramlineSystem.makeCourse(route, startIx, showStart, job)
    if route == nil or #route < 2 then return nil end
    startIx = math.max(1, math.min(startIx or 1, #route))
    local s, f = route[startIx], route[#route]
    local course = { sx = s[1], sz = s[2], fx = f[1], fz = f[2], showStart = showStart }
    if job == nil then
        course.finishSub = distText(TramlineRoute.lengthFrom(route, startIx)) .. " course"
        return course
    end
    -- what is left: the rest of the piece it is on, then the pieces after it
    local ix = math.max(1, math.min(job.ix or startIx, #route - 1))
    local a, b = route[ix], route[ix + 1]
    local dx, dz = b[1] - a[1], b[2] - a[2]
    local seg = math.sqrt(dx * dx + dz * dz)
    local along = 0
    if seg > 1e-6 and job.x ~= nil and job.z ~= nil then
        along = math.max(0, math.min(seg, ((job.x - a[1]) * dx + (job.z - a[2]) * dz) / seg))
    end
    local left = TramlineRoute.lengthFrom(route, ix + 1) + seg - along
    local done, total = TramlineRoute.progress(route, ix)
    local pct = total > 0 and math.floor(100 * done / total + 0.5) or 0
    course.finishSub = string.format("%d%% done  ·  %s to go", pct, distText(left))
    return course
end

-- the course of a plan from planStart (preview: the start marker shows the
-- start of a new job, so only its finish gets a flag)
function TramlineSystem:courseFor(fieldId, plan, vx, vz, hx, hz)
    if plan.kind == "new" then
        local route = plan.route or self:previewRoute(fieldId, plan.job, vx, vz, hx, hz)
        local at = plan.at and plan.at.ix or 1
        return TramlineSystem.makeCourse(route, at, false, nil)
    elseif plan.kind == "resume" or plan.kind == "choice" then
        local job = plan.job or {}
        return TramlineSystem.makeCourse(plan.route, job.startIx, true, job)
    end
    return nil
end

-- the route a new job would take from here, for its finish flag: kept while
-- the vehicle stays put, rebuilt at most about once a second while it moves
function TramlineSystem:previewRoute(fieldId, job, vx, vz, hx, hz)
    local geo = self.store:getGeometry(fieldId)
    if geo == nil or job == nil then return nil end
    local c = self.previewRouteCache
    local same = c ~= nil and c.geo == geo and c.order == job.order and c.corners == job.corners and c.boomBack == job.boomBack
    if same and (vx - c.x) ^ 2 + (vz - c.z) ^ 2 < 9 and hx * c.hx + hz * c.hz > 0.9 then return c.route end
    local now = g_time or 0
    if same and now - c.time < 900 then return c.route end
    local route = TramlineRoute.buildJob(geo, vx, vz, hx, hz,
        { headlandOrder = job.order, corners = job.corners, boomBack = job.boomBack })
    self.previewRouteCache = { geo = geo, order = job.order, corners = job.corners, boomBack = job.boomBack,
                               x = vx, z = vz, hx = hx, hz = hz, time = now, route = route }
    return route
end

-- The way back to where the job stopped (TramlineRoute.backToJob); for the
-- marker it is kept while the vehicle stands still.
function TramlineSystem:backPlan(fieldId, geo, route, job, vx, vz, hx, hz, preview, vehicle, set)
    local key = string.format("%d/%d/%.1f/%.1f/%.0f/%.0f/%.0f/%.0f", fieldId, job.ix or 0, job.x or 0, job.z or 0,
        vx, vz, (hx or 0) * 20, (hz or 0) * 20)
    local c = self.backCache
    if preview and c ~= nil and c.key == key and c.route == route then return c.back, c.info end
    local opts = {}
    if vehicle ~= nil and set ~= nil and set.refillAD then
        local d = TramlineAD.destinations(vehicle)
        if d ~= nil then opts.gate = { x = d.gx, z = d.gz } end
    end
    local back, info = TramlineRoute.backToJob(geo, route, job.ix or 1, job.x, job.z, vx, vz, hx, hz, opts)
    self.backCache = { key = key, route = route, back = back, info = info }
    return back, info
end

-- tlRefill: send the running worker off to refill now (to try out the
-- AutoDrive refill trip without waiting for the tank to run dry)
function TramlineSystem:cmdRefill()
    local v = self:getWorkerVehicle()
    if v == nil then return "the tramline worker is not running" end
    local spec = v.spec_tramlineDriver
    if spec.refill ~= nil then return "already on the way to refill" end
    local ok, why = TramlineDriver.startRefill(v)
    if not ok then
        self:notify("Refill: " .. tostring(why))
        return "refill: " .. tostring(why)
    end
    return "ok"
end

-- tlNewJob: forget the unfinished job on this field (the next start begins anew)
function TramlineSystem:cmdNewJob()
    local x, z = self:getPlayerWorldPos()
    if x == nil then return "no position" end
    local fieldId = TramlineFieldUtil.getFieldIdAt(x, z)
    if fieldId == nil or self.store:getJob(fieldId) == nil then return "no unfinished job here" end
    local w = self:getWorkerVehicle()
    if w ~= nil and w.spec_tramlineDriver.fieldId == fieldId then return "stop the worker first (Alt+Q)" end
    self.store:clearJob(fieldId)
    self:notify(string.format("Field %d: unfinished tramline job forgotten - the next start begins anew", fieldId))
    return "ok"
end

-- the vehicle the tramline worker is running in, or nil
function TramlineSystem:getWorkerVehicle()
    local v = self.activeVehicle
    if v ~= nil and v.spec_tramlineDriver ~= nil and v.spec_tramlineDriver.active then return v end
    return nil
end

-- start/stop the worker in the vehicle we sit in; on foot: stop a running one
function TramlineSystem:cmdToggleDriver()
    local v = self:getPlayerVehicle()
    if v == nil or v.tramlineDriverToggle == nil then
        if self:getWorkerVehicle() ~= nil then return self:cmdStopDriver() end
        self:notify("Get into a vehicle first")
        return "no vehicle"
    end
    v:tramlineDriverToggle()
    return "ok"
end

-- stop the tramline worker, wherever it is (never starts one)
function TramlineSystem:cmdStopDriver()
    local v = self:getWorkerVehicle()
    if v == nil then
        local pv = self:getPlayerVehicle()
        if pv ~= nil and pv.spec_tramlineDriver ~= nil and pv.spec_tramlineDriver.active then v = pv end
    end
    if v == nil then
        self:notify("The tramline worker is not running")
        return "not running"
    end
    v:tramlineDriverStop()
    return "stopped"
end

-- start the tramline worker in the vehicle we sit in (never stops one)
function TramlineSystem:cmdStartDriver()
    local v = self:getPlayerVehicle()
    if v == nil or v.tramlineDriverStart == nil then
        self:notify("Get into a vehicle first")
        return "no vehicle"
    end
    if v.spec_tramlineDriver.active then return "already running" end
    v:tramlineDriverStart()
    return "ok"
end

-------------------------------------------------------------------------------
-- Key bindings
-------------------------------------------------------------------------------

-- Called every time the game rebuilds its input events (on foot and in a
-- vehicle), via the FSBaseMission.registerActionEvents hook below. Events
-- registered only once in loadMap are thrown away on the first context change,
-- which is why a plain loadMap registration never fires in a vehicle.
TramlineSystem.ACTIONS = {
    { "TRAMLINEPILOT_SET_A",         "cmdSetA" },
    { "TRAMLINEPILOT_SET_B",         "cmdSetB" },
    { "TRAMLINEPILOT_MENU",          "cmdMenu" },
    { "TRAMLINEPILOT_OVERLAY",       "cmdOverlay" },
    { "TRAMLINEPILOT_TOGGLE_DRIVER", "cmdToggleDriver" },
    { "TRAMLINEPILOT_STOP_DRIVER",   "cmdStopDriver" },
}

-- One key press can reach us twice in the same frame (mission-level and
-- vehicle-level registration): run each action once per 0.3 s.
function TramlineSystem:runAction(method)
    local now = g_time or (os.clock() * 1000)
    self.lastActionAt = self.lastActionAt or {}
    local last = self.lastActionAt[method]
    if last ~= nil and now - last >= 0 and now - last < 300 then return end
    self.lastActionAt[method] = now
    if self[method] ~= nil then TramlineLog.call("the key for " .. method, self[method], self) end
end

function TramlineSystem:registerActionEvents()
    self.actionEventIds = {}
    for _, a in ipairs(TramlineSystem.ACTIONS) do
        local actionName, method = a[1], a[2]
        local action = InputAction[actionName]
        if action == nil then
            TramlineLog.warning("[TramlinePilot] InputAction.%s missing", actionName)
        else
            local ok, id = g_inputBinding:registerActionEvent(action, self, function() self:runAction(method) end, false, true, false, true)
            if ok and id ~= nil then
                g_inputBinding:setActionEventTextPriority(id, GS_PRIO_LOW)
                g_inputBinding:setActionEventTextVisibility(id, true)
                table.insert(self.actionEventIds, id)
            else
                TramlineLog.warning("[TramlinePilot] could not register %s", actionName)
            end
        end
    end
end

function TramlineSystem:removeActionEvents()
    for _, id in ipairs(self.actionEventIds or {}) do
        g_inputBinding:removeActionEvent(id)
    end
    self.actionEventIds = {}
end

FSBaseMission.registerActionEvents = Utils.appendedFunction(FSBaseMission.registerActionEvents, function(mission)
    if g_tramlineSystem ~= nil then g_tramlineSystem:registerActionEvents() end
end)
FSBaseMission.removeActionEvents = Utils.appendedFunction(FSBaseMission.removeActionEvents, function(mission)
    if g_tramlineSystem ~= nil then g_tramlineSystem:removeActionEvents() end
end)

-------------------------------------------------------------------------------
-- Mod event listener plumbing
-------------------------------------------------------------------------------

function TramlineSystem:loadMap()
    TramlineLog.start(TramlineSystem.modName)
    self.store:load()
    -- lines shown or not, as the player left them
    self.overlay.enabled = (self.store.showMode or 2) ~= 1

    -- one-page settings dialog
    pcall(function() g_gui:loadProfiles(TramlineSystem.modDir .. "gui/guiProfiles.xml") end)
    self.dialog = TramlineDialog.new(self)
    local ok, res = pcall(function()
        return g_gui:loadGui(TramlineSystem.modDir .. "gui/TramlineDialog.xml", "TramlineDialog", self.dialog)
    end)
    self.dialogLoaded = ok and res ~= nil
    if not self.dialogLoaded then
        TramlineLog.warning("[TramlinePilot] settings dialog failed to load (%s) - using step-by-step menu", tostring(res))
    end
    TramlineLog.info("[TramlinePilot] loaded — Right Ctrl+M on a field opens the setup menu, or 'tlMenu' in the console")

    addConsoleCommand("tlMenu",     "Tramline Pilot: open setup menu",        "cmdMenu",         self)
    addConsoleCommand("tlScan",     "Tramline Pilot: scan field under player", "cmdScan",        self)
    addConsoleCommand("tlSetA",     "Tramline Pilot: set A point",            "cmdSetA",         self)
    addConsoleCommand("tlSetB",     "Tramline Pilot: set B point",            "cmdSetB",         self)
    addConsoleCommand("tlWidth",    "Tramline Pilot: spacing in metres",      "cmdWidth",        self)
    addConsoleCommand("tlHeadland", "Tramline Pilot: headland passes",        "cmdHeadland",     self)
    addConsoleCommand("tlGenerate", "Tramline Pilot: generate for field",     "cmdGenerate",     self)
    addConsoleCommand("tlPaint",    "Tramline Pilot: create tramlines in the crop [tracks|full] [wheel track m]", "cmdPaint",     self)
    addConsoleCommand("tlClear",    "Tramline Pilot: clear field",            "cmdClear",        self)
    addConsoleCommand("tlClearAll", "Tramline Pilot: clear ALL fields",       "cmdClearAll",     self)
    addConsoleCommand("tlOverlay",  "Tramline Pilot: toggle overlay",         "cmdOverlay",      self)
    addConsoleCommand("tlStart",    "Tramline Pilot: start AI worker",        "cmdStartDriver",  self)
    addConsoleCommand("tlStop",     "Tramline Pilot: stop AI worker",         "cmdStopDriver",   self)
    addConsoleCommand("tlNewJob",   "Tramline Pilot: forget the unfinished job on this field", "cmdNewJob", self)
    addConsoleCommand("tlRefill",   "Tramline Pilot: send the worker to refill with AutoDrive now", "cmdRefill", self)
    addConsoleCommand("tlRoute",    "Tramline Pilot: log the worker route from here", "cmdRoute", self)
    addConsoleCommand("tlDebug",    "Tramline Pilot: toggle driver debug log",  "cmdDebug", self)
    addConsoleCommand("tlPF",       "Tramline Pilot: Precision Farming tramlines [on|fields|off|dump]", "cmdPF", self)
    addConsoleCommand("tlWheelings", "Tramline Pilot: list the saved wheelings of this field", "cmdWheelings", self)
    addConsoleCommand("tlSave",     "Tramline Pilot: save this field's wheelings [name]", "cmdSaveWheeling", self)
    addConsoleCommand("tlLoad",     "Tramline Pilot: load saved wheelings for this field <number|name>", "cmdLoadWheeling", self)
    addConsoleCommand("tlReport",   "Tramline Pilot: write a snapshot for a bug report to TramlinePilot_log.txt", "cmdReport", self)

    -- apply the saved PF switch once everything (incl. PF) has loaded
    self.pendingPFApply = true
end

-- tlReport: a snapshot for a bug report, into the mod's own log
function TramlineSystem:cmdReport()
    local lines = {}
    local function add(f, ...) lines[#lines + 1] = string.format(f, ...) end
    add("===== tlReport =====")
    add("Tramline Pilot %s; %s", tostring(TramlineLog.modVersion(TramlineSystem.modName) or "?"), TramlineLog.mods())
    local x, z = self:getPlayerWorldPos()
    local fieldId = x ~= nil and self:getFieldIdAt(x, z) or nil
    add("player at %s, field %s", x ~= nil and string.format("%.0f %.0f", x, z) or "?", tostring(fieldId))
    if fieldId ~= nil then
        local set = self.store:getSet(fieldId)
        add("field %d settings: %s", fieldId, TramlineSystem.describeSet(set))
        local geo = self.store:getGeometry(fieldId)
        if geo ~= nil then
            add("layout: %d tramlines, %d headland pass(es), %d curves; notes: %s", #(geo.lines or {}), geo.headlands or 0,
                #(geo.curves or {}), #(geo.warnings or {}) > 0 and table.concat(geo.warnings, "; ") or "none")
        end
        local job = self.store:getJob(fieldId)
        if job ~= nil then
            add("job: waypoint %s of %s, started at %s, headland %s", tostring(job.ix), tostring(job.n), tostring(job.startIx), tostring(job.order))
        end
    end
    local v = self:getPlayerVehicle()
    if v ~= nil and v.spec_tramlineDriver ~= nil then
        add("vehicle: %s", TramlineDriver.describeVehicle(v))
        add("worker: %s", TramlineDriver.describeState(v))
    end
    add("work speed %s km/h, turn speed %s km/h, lines %s, PF tramlines %s", tostring(self.store.workSpeed),
        tostring(self.store.turnSpeed), tostring(self.store.showMode), tostring(self.store.pfMode))
    add("====================")
    for _, l in ipairs(lines) do TramlineLog.write("INFO", l) end
    if TramlineLog.path == nil then return "the log file could not be written" end
    local where = "Documents/My Games/FarmingSimulator2025/modSettings/" .. TramlineLibrary.MOD_FOLDER .. "/" .. TramlineLog.FILE
    self:notify("Report added to " .. where)
    return "ok"
end

-- tlRoute: log the route the worker would drive from the current vehicle
function TramlineSystem:cmdRoute()
    local v = self:getPlayerVehicle()
    if v == nil then return "get into a vehicle" end
    local x, _, z = getWorldTranslation(v.rootNode)
    local fieldId = TramlineFieldUtil.getFieldIdAt(x, z)
    if fieldId == nil or self.store:getSet(fieldId) == nil then return "no tramlines here" end
    local geo = self.store:getGeometry(fieldId)
    local set = self.store:getSet(fieldId)
    local hx, _, hz = localDirectionToWorld(v.rootNode, 0, 0, 1)
    local route, why = TramlineRoute.buildJob(geo, x, z, hx, hz, { headlandOrder = set.headlandOrder or "first" })
    if route == nil then return "no route: " .. tostring(why) end
    TramlineLog.info("[TramlinePilot][route] field %d: %d lines in %d block(s), %d headland pass(es), radius %.1f m, %d loop / %d headland turns",
        fieldId, #geo.lines, #geo.blocks, geo.headlands or 0, geo.radius or 0, geo.loopCount or 0, geo.ringTurnCount or 0)
    for _, w in ipairs(geo.warnings or {}) do TramlineLog.info("[TramlinePilot][route] warning: %s", w) end
    local lastKind, from = nil, 1
    for i = 1, #route + 1 do
        local w = route[i]
        local kind = w and string.format("%s%s", w.kind or "?", w.work and " (spray)" or "") or nil
        if kind ~= lastKind then
            if lastKind ~= nil then
                local a, b = route[from], route[i - 1]
                TramlineLog.info("[TramlinePilot][route] %4d-%4d %-20s %8.1f %8.1f -> %8.1f %8.1f", from, i - 1, lastKind, a[1], a[2], b[1], b[2])
            end
            lastKind, from = kind, i
        end
    end
    return string.format("%d waypoints logged", #route)
end

-- the field the player (or their vehicle) is on, or nil
function TramlineSystem:playerFieldId()
    local x, z = self:getPlayerWorldPos()
    if x == nil then return nil end
    return TramlineFieldUtil.getFieldIdAt(x, z)
end

-- tlWheelings: the saved wheelings of this field
function TramlineSystem:cmdWheelings()
    local fieldId = self:playerFieldId()
    local list = TramlineLibrary.list(fieldId)
    if #list == 0 then
        return string.format("no saved wheelings for field %s (folder %s)", tostring(fieldId), tostring(TramlineLibrary.folder()))
    end
    local lines = {}
    for i, w in ipairs(list) do lines[#lines + 1] = string.format("%d: %s", i, TramlineLibrary.describe(w)) end
    return table.concat(lines, "\n")
end

-- tlSave [name]: save this field's tramline settings as a wheeling
function TramlineSystem:cmdSaveWheeling(name)
    local fieldId = self:playerFieldId()
    local set = fieldId and self.store:getSet(fieldId)
    if set == nil then return "no tramlines on this field" end
    if name ~= nil and name == "" then name = nil end
    local saved, path = TramlineLibrary.save(set, name)
    if saved == nil then return "could not save: " .. tostring(path) end
    self:notify("Wheeling saved: " .. saved)
    return "saved to " .. tostring(path)
end

-- tlLoad <number|name>: lay this field's tramlines from a saved wheeling
function TramlineSystem:cmdLoadWheeling(arg)
    local fieldId = self:playerFieldId()
    if fieldId == nil then return "stand on a field" end
    local list = TramlineLibrary.list(fieldId)
    local w = list[tonumber(arg or "") or 0]
    if w == nil and arg ~= nil then
        for _, c in ipairs(list) do if string.lower(c.name) == string.lower(arg) then w = c end end
    end
    if w == nil then return "usage: tlLoad <number|name> (see tlWheelings)" end
    local set = TramlineLibrary.apply(w, TramlineStore.newSet(fieldId))
    set.fieldId = fieldId
    self.store:putSet(fieldId, set)
    local geo = self.store:regenerate(fieldId)
    self:notify(string.format("Field %d: wheeling '%s' loaded (%d tramlines)", fieldId, w.name, geo and #geo.lines or 0))
    return "ok"
end

function TramlineSystem:cmdDebug()
    self.debug = not self.debug
    return "driver debug " .. (self.debug and "on" or "off")
end

function TramlineSystem:cmdPF(arg)
    arg = string.lower(arg or "")
    local modes = { on = 1, fields = 2, off = 3 }
    if modes[arg] ~= nil then
        if not TramlinePFCompat.isPresent() then return "Precision Farming not found" end
        TramlinePFCompat.setMode(modes[arg])
        self.store.pfMode = TramlinePFCompat.mode
        self.store.isDirty = true
        self:notify("Precision Farming tramlines: " .. TramlinePFCompat.MODES[TramlinePFCompat.mode])
        return "PF tramlines: " .. TramlinePFCompat.MODES[TramlinePFCompat.mode]
    elseif arg == "dump" then
        return TramlinePFCompat.dump()
    end
    return "usage: tlPF on|fields|off|dump   (now: " .. TramlinePFCompat.MODES[TramlinePFCompat.mode or 1] .. ")"
end

function TramlineSystem:deleteMap()
    self:removeActionEvents()
    TramlineDraw.delete()
    TramlineLog.finish()
end

-- script errors in the mod's own update and drawing go to its log once
-- (counted after that), not a line in log.txt every frame
function TramlineSystem:update(dt)
    TramlineLog.guard("the update", TramlineSystem.updateInner, self, dt)
end

function TramlineSystem:draw()
    TramlineLog.guard("the drawing", TramlineSystem.drawInner, self)
end

function TramlineSystem:updateInner(dt)
    -- PF registers its globals during mod loading; apply our switch on the
    -- first frame so we're sure everything exists.
    if self.pendingPFApply then
        self.pendingPFApply = nil
        TramlinePFCompat.setMode(self.store.pfMode or 1)
        TramlineLog.info("[TramlinePilot] works with: %s", TramlineLog.mods())
    end

    -- refill trips: when AutoDrive is done, the worker takes over again
    if self.refillVehicles ~= nil then
        for v, _ in pairs(self.refillVehicles) do
            local spec = v.spec_tramlineDriver
            local r = spec ~= nil and spec.refill or nil
            if v.isDeleted or r == nil then
                self.refillVehicles[v] = nil
            else
                if r.stage == "waiting" and TramlineAD.isDriving(v) then
                    -- AutoDrive was started again: wait for it to finish
                    r.stage, r.idle = "ad", 0
                end
                if r.stage == "ad" then
                    -- in case AutoDrive's stop event did not reach us
                    if TramlineAD.isDriving(v) then
                        r.idle = 0
                    else
                        r.idle = (r.idle or 0) + dt
                        if r.idle > 2000 then r.stage = "back" end
                    end
                end
                if r.stage == "back" then
                    self.refillVehicles[v] = nil
                    TramlineDriver.returnFromRefill(v)
                end
            end
        end
    end

    -- Start marker: every 0.3 s, where the worker would set off from if
    -- Alt+T were pressed now (vehicle on a field with tramlines, worker not
    -- running) - or, with an unfinished job, where it last sprayed and the
    -- way back there (every second).
    self.previewTimer = (self.previewTimer or 0) + dt
    if self.previewTimer > (self.previewPeriod or 300) then
        self.previewTimer = 0
        self.previewPeriod = 300
        self.previewStart = nil
        self.course = nil
        -- the course of the job being worked: start and finish flags
        local fid = self.activeFieldId
        if fid ~= nil and self.overlay.enabled then
            local job = self.store:getJob(fid)
            local route = job ~= nil and self.store:getJobRoute(fid) or nil
            if route ~= nil then self.course = TramlineSystem.makeCourse(route, job.startIx, true, job) end
        end
        local v = self:getPlayerVehicle()
        if self.overlay.enabled and v ~= nil and v.spec_tramlineDriver ~= nil and not v.spec_tramlineDriver.active
            and self:getWorkerVehicle() == nil then
            local node = TramlineDriver.getDirectionNode(v)
            local x, _, z = getWorldTranslation(node)
            local fieldId = TramlineFieldUtil.getFieldIdAt(x, z)
            if fieldId ~= nil and self.store:getSet(fieldId) ~= nil then
                local hx, _, hz = localDirectionToWorld(node, 0, 0, 1)
                local ok, plan = pcall(self.planStart, self, fieldId, x, z, hx, hz, false, true, v)
                if ok and plan ~= nil then
                    local p = plan.at or plan
                    if plan.kind == "choice" then
                        p = plan
                        self.previewPeriod = 1000
                    end
                    self.previewStart = { x = p.x, z = p.z, nx = p.x + (p.dirX or 0), nz = p.z + (p.dirZ or 0),
                                          ready = plan.kind ~= "away", kind = plan.kind, pct = plan.pct,
                                          joined = plan.joined, unfinished = plan.unfinished }
                    if plan.kind == "choice" then
                        local ps = self.previewStart
                        local okW, w = pcall(TramlineDriver.toolWords, v)
                        ps.spreader = okW and w ~= nil and w.noun == "spreader"
                        ps.back, ps.backN = plan.back, plan.backInfo and plan.backInfo.n or 0
                        ps.backLen = plan.backInfo and plan.backInfo.total or nil
                        ps.backWhy = plan.backWhy
                        ps.skipped = plan.at and plan.at.skipped or nil
                    end
                    if plan.kind == "away" then self.previewStart.fromX, self.previewStart.fromZ = x, z end
                    local okC, course = pcall(self.courseFor, self, fieldId, plan, x, z, hx, hz)
                    if okC then
                        self.course = course
                    else
                        TramlineLog.warning("[TramlinePilot] course flags: %s", tostring(course))
                    end
                elseif not ok then
                    TramlineLog.warning("[TramlinePilot] start marker: %s", tostring(plan))
                end
            end
        end
    end
end

function TramlineSystem:drawInner()
    self.overlay:draw()
end

-------------------------------------------------------------------------------
-- Hook savegame write
-------------------------------------------------------------------------------

local function onSaveSavegame(mission)
    if g_tramlineSystem ~= nil then
        g_tramlineSystem.store:save()
    end
end
FSBaseMission.saveSavegame = Utils.appendedFunction(FSBaseMission.saveSavegame, onSaveSavegame)

-------------------------------------------------------------------------------
-- Inject TramlineDriver into drivable vehicle types
-------------------------------------------------------------------------------

local function injectDriverSpec(typeManager)
    if typeManager.typeName ~= "vehicle" then return end
    local specName = TramlineSystem.modName .. ".tramlineDriver"
    for typeName, typeEntry in pairs(typeManager:getTypes()) do
        if SpecializationUtil.hasSpecialization(Drivable, typeEntry.specializations)
            and SpecializationUtil.hasSpecialization(AIVehicle, typeEntry.specializations) then
            typeManager:addSpecialization(typeName, specName)
        end
    end
end
TypeManager.finalizeTypes = Utils.prependedFunction(TypeManager.finalizeTypes, injectDriverSpec)

-------------------------------------------------------------------------------
-- Boot
-------------------------------------------------------------------------------

g_tramlineSystem = TramlineSystem.new()
addModEventListener(g_tramlineSystem)
