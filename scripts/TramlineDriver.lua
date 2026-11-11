--[[
    TramlineDriver.lua
    Vehicle specialization: the tramline worker. Drives the job route of the
    field the vehicle stands on (TramlineRoute.buildJob: headland passes,
    tramlines, turns along the headland tramline) and sprays only on the
    working stretches. Every metre it drives is on the tramline layout.

    Implements are handled like Courseplay / the Giants AI field worker, one
    step per frame so every implement sees the events in order:
      1. updateAIFieldWorkerImplementData, onAIFieldWorkerStart
      2. onAIFieldWorkerPrepareForWork (unfolds)
      3. wait until everything is unfolded
      4. at each working stretch: aiImplementStartLine + AI_START_LINE, crawl
         until the implements are lowered (getCanAIFieldWorkerContinueWork),
         then check they are really switched on (switch on if not)
      5. at the end of each stretch: aiImplementEndLine + AI_END_LINE
      6. on stop: onAIFieldWorkerEnd
    Crab steering vehicles are switched to their AI steering mode.
    Jobs survive stopping and save/load: Alt+T anywhere on what is left of
    the job carries on from there (TramlineSystem:planStart).
]]

TramlineDriver = {}

TramlineDriver.WORK_SPEED      = 12     -- km/h on tramlines
TramlineDriver.TURN_SPEED      = 7      -- km/h in turns and tight curves
TramlineDriver.DEFAULT_RADIUS  = 7      -- m, if the vehicle doesn't tell us
TramlineDriver.OFF_TRACK_LIMIT = 15     -- m from the route -> stop
TramlineDriver.UNFOLD_TIMEOUT  = 30000  -- ms
TramlineDriver.LOWER_TIMEOUT   = 10000  -- ms crawling while the implements lower
TramlineDriver.SWITCH_LEAD     = 2.5    -- m: start the line this far before the boom reaches it
TramlineDriver.REVERSE_SPEED   = 4      -- km/h backing into a headland corner

-- (TramlineGeom is looked up when used: this file may be loaded before it)

function TramlineDriver.prerequisitesPresent(specializations)
    return SpecializationUtil.hasSpecialization(Drivable, specializations)
end

function TramlineDriver.registerFunctions(vehicleType)
    SpecializationUtil.registerFunction(vehicleType, "tramlineDriverToggle",  TramlineDriver.tramlineDriverToggle)
    SpecializationUtil.registerFunction(vehicleType, "tramlineDriverStart",   TramlineDriver.tramlineDriverStart)
    SpecializationUtil.registerFunction(vehicleType, "tramlineDriverStop",    TramlineDriver.tramlineDriverStop)
    SpecializationUtil.registerFunction(vehicleType, "tramlineDriverSetWorking", TramlineDriver.tramlineDriverSetWorking)
    SpecializationUtil.registerFunction(vehicleType, "tramlineDriverOutOfSpray", TramlineDriver.tramlineDriverOutOfSpray)
end

function TramlineDriver.registerOverwrittenFunctions(vehicleType)
    SpecializationUtil.registerOverwrittenFunction(vehicleType, "getIsAIActive", TramlineDriver.getIsAIActive)
    SpecializationUtil.registerOverwrittenFunction(vehicleType, "getIsVehicleControlledByPlayer", TramlineDriver.getIsVehicleControlledByPlayer)
    -- the game's own "stop helper" (and implements that run empty, the motor
    -- running out of fuel, ...) stop the AI through this: stop our worker too
    SpecializationUtil.registerOverwrittenFunction(vehicleType, "stopCurrentAIJob", TramlineDriver.stopCurrentAIJob)
end

function TramlineDriver:stopCurrentAIJob(superFunc, message, ...)
    local spec = self.spec_tramlineDriver
    if spec ~= nil and spec.active then
        -- the sprayer reports it is empty: off to refill (if set up)
        local outOfFill = false
        if message ~= nil and message.isa ~= nil and AIMessageErrorOutOfFill ~= nil then
            local ok, isIt = pcall(message.isa, message, AIMessageErrorOutOfFill)
            outOfFill = ok and isIt == true
        end
        if outOfFill then
            if spec.refill == nil then self:tramlineDriverOutOfSpray() end
            return
        end
        local reason = nil
        if message ~= nil and message.getMessage ~= nil then
            local ok, text = pcall(message.getMessage, message)
            if ok and type(text) == "string" and text ~= "" then reason = text end
        end
        TramlineLog.info("[TramlinePilot][drv] stop requested by the game%s", reason and (": " .. reason) or "")
        self:tramlineDriverStop(false, reason)
        return
    end
    return superFunc(self, message, ...)
end

function TramlineDriver:getIsAIActive(superFunc)
    local spec = self.spec_tramlineDriver
    if spec ~= nil and spec.active then return true end
    return superFunc(self)
end

function TramlineDriver:getIsVehicleControlledByPlayer(superFunc)
    local spec = self.spec_tramlineDriver
    if spec ~= nil and spec.active then return false end
    return superFunc(self)
end

function TramlineDriver.registerEventListeners(vehicleType)
    SpecializationUtil.registerEventListener(vehicleType, "onLoad",   TramlineDriver)
    SpecializationUtil.registerEventListener(vehicleType, "onUpdate", TramlineDriver)
    SpecializationUtil.registerEventListener(vehicleType, "onDelete", TramlineDriver)
    SpecializationUtil.registerEventListener(vehicleType, "onRegisterActionEvents", TramlineDriver)
    -- AutoDrive's stop event, on vehicles that have AutoDrive (refill trips)
    if vehicleType.eventListeners == nil or vehicleType.eventListeners.onStopAutoDrive ~= nil then
        SpecializationUtil.registerEventListener(vehicleType, "onStopAutoDrive", TramlineDriver)
    end
end

-- AutoDrive has stopped: if it was our refill trip, take over again (the
-- system's update does it on the next frame, AutoDrive is still busy now)
function TramlineDriver:onStopAutoDrive(isPassingToCP, isStartingAIVE)
    local spec = self.spec_tramlineDriver
    local r = spec ~= nil and spec.refill or nil
    if r ~= nil and r.stage == "ad" then
        r.stage = "back"
        TramlineLog.info("[TramlinePilot][drv] AutoDrive has stopped - taking over again")
    end
end

-- Vehicle-context key bindings. The mission-level registration in
-- TramlineSystem covers on-foot; this covers sitting in a vehicle.
-- While our worker drives, the game counts the vehicle as AI driven and
-- passes isActiveForInputIgnoreSelection = false; like Courseplay we ask
-- getIsActiveForInput(true, true) (= also while AI driven), so the stop and
-- start/stop keys keep working.
TramlineDriver.WHILE_DRIVING = { TRAMLINEPILOT_STOP_DRIVER = true, TRAMLINEPILOT_TOGGLE_DRIVER = true, TRAMLINEPILOT_OVERLAY = true }

function TramlineDriver:onRegisterActionEvents(isActiveForInput, isActiveForInputIgnoreSelection)
    if not self.isClient then return end
    local spec = self.spec_tramlineDriver
    self:clearActionEventsTable(spec.actionEvents)
    local inputActive = isActiveForInputIgnoreSelection
    if not inputActive and self.getIsActiveForInput ~= nil then
        local ok, a = pcall(self.getIsActiveForInput, self, true, true)
        inputActive = ok and a == true
    end
    if not inputActive then return end

    for _, a in ipairs(TramlineSystem.ACTIONS) do
        local action = InputAction[a[1]]
        if action ~= nil and (not spec.active or TramlineDriver.WHILE_DRIVING[a[1]]) then
            local _, id = self:addActionEvent(spec.actionEvents, action, self, TramlineDriver.onActionEvent, false, true, false, true, a[2])
            if id ~= nil then
                local stopKey = spec.active and (a[1] == "TRAMLINEPILOT_STOP_DRIVER" or a[1] == "TRAMLINEPILOT_TOGGLE_DRIVER")
                g_inputBinding:setActionEventTextPriority(id, stopKey and GS_PRIO_HIGH or GS_PRIO_LOW)
                g_inputBinding:setActionEventTextVisibility(id, a[1] ~= "TRAMLINEPILOT_STOP_DRIVER" or spec.active)
                if a[1] == "TRAMLINEPILOT_TOGGLE_DRIVER" and g_inputBinding.setActionEventText ~= nil then
                    g_inputBinding:setActionEventText(id, spec.active and "Tramlines: stop AI worker" or "Tramlines: start AI worker")
                end
            end
        end
    end
end

-- rebuild the key list (start/stop changes which keys we offer)
local function refreshInput(vehicle)
    if vehicle.requestActionEventUpdate ~= nil then pcall(vehicle.requestActionEventUpdate, vehicle) end
end

function TramlineDriver.onActionEvent(self, actionName, inputValue, callbackState)
    local method = callbackState
    if g_tramlineSystem ~= nil and method ~= nil and g_tramlineSystem[method] ~= nil then
        g_tramlineSystem:runAction(method)
    end
end

function TramlineDriver:onLoad(savegame)
    self.spec_tramlineDriver = {
        active = false,
        route = nil,
        isWorking = false,
        actionEvents = {},
    }
end

function TramlineDriver:onDelete()
    if self.spec_tramlineDriver ~= nil and self.spec_tramlineDriver.active then
        self:tramlineDriverStop()
    end
end

-------------------------------------------------------------------------------
-- Vehicle helpers (after Courseplay's AIUtil / Markers)
-------------------------------------------------------------------------------

local function try(obj, fname, ...)
    local f = obj ~= nil and obj[fname] or nil
    if f == nil then return false end
    local ok, a, b, c = pcall(f, obj, ...)
    if not ok then
        TramlineLog.warning("[TramlinePilot][drv] %s failed: %s", fname, tostring(a))
        return false
    end
    return true, a, b, c
end

local function childVehicles(vehicle)
    local list = {}
    if vehicle.getChildVehicles ~= nil then
        for _, v in pairs(vehicle:getChildVehicles()) do list[#list + 1] = v end
    end
    if #list == 0 then list[1] = vehicle end
    return list
end

function TramlineDriver.getDirectionNode(vehicle)
    if vehicle.getAIDirectionNode ~= nil then
        local n = vehicle:getAIDirectionNode()
        if n ~= nil then return n end
    end
    return vehicle.rootNode
end

function TramlineDriver.getSteeringNode(vehicle)
    if vehicle.getAISteeringNode ~= nil then
        local n = vehicle:getAISteeringNode()
        if n ~= nil then return n end
    end
    return TramlineDriver.getDirectionNode(vehicle)
end

function TramlineDriver.getTurningRadius(vehicle)
    local r = vehicle.maxTurningRadius or TramlineDriver.DEFAULT_RADIUS
    if vehicle.getAIMinTurningRadius ~= nil then
        local ok, minR = pcall(vehicle.getAIMinTurningRadius, vehicle)
        if ok and minR ~= nil then r = math.max(r, minR) end
    end
    for _, impl in ipairs(childVehicles(vehicle)) do
        if impl ~= vehicle and impl.getAIMinTurningRadius ~= nil then
            local ok, minR = pcall(impl.getAIMinTurningRadius, impl)
            if ok and minR ~= nil then r = math.max(r, minR) end
        end
    end
    return math.max(3, math.min(r, 20))
end

-- Wheel track of one vehicle: centre to centre of the left and right wheels
-- of its widest axle (wheels grouped by how far forward they sit). nil if it
-- has no axle with a wheel each side, like a mounted sprayer or spreader.
local function wheelTrack(obj, groundOnly)
    local spec = obj.spec_wheels
    if spec == nil or spec.wheels == nil then return nil end
    local root = obj.components ~= nil and obj.components[1] ~= nil and obj.components[1].node or obj.rootNode
    if root == nil then return nil end
    local axles = {}
    for _, wheel in pairs(spec.wheels) do
        local ph = wheel.physics
        if ph ~= nil and wheel.node ~= nil and ph.positionX ~= nil
                and not (groundOnly and ph.hasGroundContact == false) then
            local x, _, z = localToLocal(wheel.node, root, ph.positionX, ph.positionY or 0, ph.positionZ or 0)
            if x ~= nil and z ~= nil then
                local axle = nil
                for _, a in ipairs(axles) do
                    if math.abs(a.z - z) < 0.5 then axle = a; break end
                end
                if axle == nil then
                    axles[#axles + 1] = { z = z, minX = x, maxX = x }
                else
                    axle.minX, axle.maxX = math.min(axle.minX, x), math.max(axle.maxX, x)
                end
            end
        end
    end
    local best = nil
    for _, a in ipairs(axles) do
        if a.minX < -0.3 and a.maxX > 0.3 and (best == nil or a.maxX - a.minX > best) then
            best = a.maxX - a.minX
        end
    end
    return best
end

-- The wheel track the tramlines should have for this machine: the sprayer's
-- or spreader's own wheels when it runs on wheels (trailed or self-propelled),
-- else the vehicle carrying it. Metres to the nearest 5 cm; nil if unknown.
function TramlineDriver.getTrackWidth(vehicle)
    local w = nil
    for _, obj in ipairs(TramlineDriver.getAIImplements(vehicle)) do
        local ok, t = pcall(wheelTrack, obj, obj ~= vehicle)
        if ok and t ~= nil then w = math.max(w or 0, t) end
    end
    if w == nil then
        local ok, t = pcall(wheelTrack, vehicle, false)
        if ok then w = t end
    end
    if w == nil or w < 0.8 or w > 6 then return nil end
    return math.floor(w * 20 + 0.5) / 20
end

-- Where the work area is relative to the direction node, along the vehicle
-- axis: 'front' = the AI left/right markers (where work begins), 'back' = the
-- AI back marker (where work ends). Both are z offsets (forward positive).
function TramlineDriver.findMarkerOffsets(vehicle)
    local dirNode = TramlineDriver.getDirectionNode(vehicle)
    local front, back = nil, nil
    for _, obj in ipairs(childVehicles(vehicle)) do
        if obj.getAIMarkers ~= nil then
            local ok, l, r, b = pcall(obj.getAIMarkers, obj)
            if ok and l ~= nil and r ~= nil then
                local _, _, zl = localToLocal(l, dirNode, 0, 0, 0)
                local _, _, zr = localToLocal(r, dirNode, 0, 0, 0)
                local zf = (zl + zr) * 0.5
                local zb = zf
                if b ~= nil then _, _, zb = localToLocal(b, dirNode, 0, 0, 0) end
                front = front and math.max(front, zf) or zf
                back  = back  and math.min(back, zb)  or zb
            end
        end
    end
    return { front = front or 0, back = back or 0 }
end

-- What the worker is working with, for the messages: a spreader (fertiliser
-- spinner, pneumatic, lime or muck spreader - FS25 runs them on the same
-- sprayer machinery) or a sprayer. An empty tank has no fill type, so the
-- last one seen while working counts too.
local SPREAD_FILLS = { FERTILIZER = true, LIME = true, MANURE = true }
function TramlineDriver.toolWords(vehicle)
    local spec = vehicle.spec_tramlineDriver
    local spreader = false
    for _, obj in ipairs(TramlineDriver.getAIImplements(vehicle)) do
        if string.find(string.lower(tostring(obj.typeName or "")), "spread", 1, true) ~= nil then spreader = true end
    end
    if not spreader then
        local _, _, ft = TramlineDriver.sprayFill(vehicle)
        ft = ft or (spec ~= nil and spec.sprayFillType or nil)
        if ft ~= nil and g_fillTypeManager ~= nil and g_fillTypeManager.getFillTypeNameByIndex ~= nil then
            local ok, name = pcall(g_fillTypeManager.getFillTypeNameByIndex, g_fillTypeManager, ft)
            if ok and name ~= nil and SPREAD_FILLS[string.upper(tostring(name))] then spreader = true end
        end
    end
    if spreader then return { noun = "spreader", Noun = "Spreader", ing = "spreading" } end
    return { noun = "sprayer", Noun = "Sprayer", ing = "spraying" }
end

-- the implements the Giants AI works with (fallback: anything AI capable)
function TramlineDriver.getAIImplements(vehicle)
    local list = {}
    if vehicle.getAttachedAIImplements ~= nil then
        local ok, impls = pcall(vehicle.getAttachedAIImplements, vehicle)
        if ok and impls ~= nil then
            for _, impl in pairs(impls) do
                if impl.object ~= nil then list[#list + 1] = impl.object end
            end
        end
    end
    if #list == 0 then
        for _, obj in ipairs(childVehicles(vehicle)) do
            if obj.aiImplementStartLine ~= nil then list[#list + 1] = obj end
        end
    end
    return list
end

-- Can the vehicle back into a corner on its own? Not with a trailed (towed,
-- wheeled) implement: that would need steering the trailer backwards.
function TramlineDriver.canReverse(vehicle)
    for _, obj in ipairs(TramlineDriver.getAIImplements(vehicle)) do
        if obj ~= vehicle then
            local w = obj.spec_wheels
            if w ~= nil and w.wheels ~= nil and #w.wheels > 0 then return false end
        end
    end
    return true
end

-- the Giants AI event pair: root vehicle event + implement event
function TramlineDriver.raiseAIEvent(vehicle, evt, evtImplement)
    if vehicle.raiseAIEvent ~= nil then
        local ok, err = pcall(vehicle.raiseAIEvent, vehicle, evt, evtImplement)
        if ok then
            TramlineLog.info("[TramlinePilot][drv] AI event %s / %s", evt, evtImplement)
            return true
        end
        TramlineLog.warning("[TramlinePilot][drv] raiseAIEvent %s failed: %s", evt, tostring(err))
    end
    -- no AI event support: raise the plain events
    pcall(SpecializationUtil.raiseEvent, vehicle, evt)
    for _, obj in ipairs(TramlineDriver.getAIImplements(vehicle)) do
        if obj ~= vehicle then pcall(SpecializationUtil.raiseEvent, obj, evtImplement) end
    end
    return false
end

local function isFoldable(obj)
    local fs = obj.spec_foldable
    return fs ~= nil and fs.foldingParts ~= nil and #fs.foldingParts > 0 and obj.getIsUnfolded ~= nil
end

function TramlineDriver.allUnfolded(vehicle)
    for _, obj in ipairs(childVehicles(vehicle)) do
        if isFoldable(obj) then
            local ok, un = pcall(obj.getIsUnfolded, obj)
            if ok and not un then return false end
        end
    end
    return true
end

-- unfold whatever the AI events did not (turnOnFoldDirection leads to work)
function TramlineDriver.forceUnfold(vehicle)
    for _, obj in ipairs(childVehicles(vehicle)) do
        if isFoldable(obj) then
            local ok, un = pcall(obj.getIsUnfolded, obj)
            if ok and not un and obj.setFoldState ~= nil then
                local fs = obj.spec_foldable
                local dir = fs.turnOnFoldDirection or -1
                if fs.foldMoveDirection ~= dir then
                    pcall(obj.setFoldState, obj, dir, false)
                    TramlineLog.info("[TramlinePilot][drv] unfolding %s", tostring(obj.getName and obj:getName() or "?"))
                end
            end
        end
    end
end

-- make sure the working implements are switched on; returns how many were not
function TramlineDriver.ensureTurnedOn(vehicle)
    local forced = 0
    for _, obj in ipairs(TramlineDriver.getAIImplements(vehicle)) do
        if obj.getIsTurnedOn ~= nil and obj.setIsTurnedOn ~= nil then
            local ok, on = pcall(obj.getIsTurnedOn, obj)
            if ok and not on then
                local can = true
                if obj.getCanBeTurnedOn ~= nil then
                    local ok2, c = pcall(obj.getCanBeTurnedOn, obj)
                    can = ok2 and c
                end
                if can then
                    pcall(obj.setIsTurnedOn, obj, true)
                    forced = forced + 1
                end
            end
        end
    end
    return forced
end

-- one log line per sprayer: is it on, lowered, how full (diagnosis)
function TramlineDriver.logImplementState(vehicle, why)
    for _, obj in ipairs(TramlineDriver.getAIImplements(vehicle)) do
        local name = obj.getName ~= nil and obj:getName() or "?"
        local on, low, fill, cap = "-", "-", "-", "-"
        if obj.getIsTurnedOn ~= nil then local ok, v = pcall(obj.getIsTurnedOn, obj); if ok then on = tostring(v) end end
        if obj.getIsLowered ~= nil then local ok, v = pcall(obj.getIsLowered, obj, true); if ok then low = tostring(v) end end
        if obj.spec_sprayer ~= nil and obj.getSprayerFillUnitIndex ~= nil then
            local ok, idx = pcall(obj.getSprayerFillUnitIndex, obj)
            if ok and idx ~= nil then
                fill = string.format("%.0f", obj:getFillUnitFillLevel(idx) or 0)
                cap = string.format("%.0f", obj:getFillUnitCapacity(idx) or 0)
            end
        end
        TramlineLog.info("[TramlinePilot][drv] %s: %s turnedOn=%s lowered=%s fill=%s/%s", why, name, on, low, fill, cap)
    end
end

local function setCrabSteering(vehicle, toAI)
    local cs = vehicle.spec_crabSteering
    local spec = vehicle.spec_tramlineDriver
    if cs == nil or vehicle.setCrabSteering == nil then return end
    if toAI then
        local ai = cs.aiSteeringModeIndex or cs.aiSteeringMode
        if ai ~= nil and cs.state ~= ai then
            spec.crabBefore = cs.state
            pcall(vehicle.setCrabSteering, vehicle, ai)
            TramlineLog.info("[TramlinePilot][drv] crab steering: AI mode %s", tostring(ai))
        end
    elseif spec.crabBefore ~= nil then
        pcall(vehicle.setCrabSteering, vehicle, spec.crabBefore)
        spec.crabBefore = nil
    end
end

-- total heading change along the route over the next 'dist' metres
local function curvatureAhead(route, ix, dist)
    local total, acc, prevH = 0, 0, nil
    for i = math.max(2, ix), #route do
        local a, b = route[i - 1], route[i]
        local dx, dz = b[1] - a[1], b[2] - a[2]
        local l = math.sqrt(dx * dx + dz * dz)
        if l > 0.05 then
            local h = TramlineGeom.atan2(dz, dx)
            if prevH ~= nil then total = total + math.abs(TramlineGeom.wrapPi(h - prevH)) end
            prevH = h
            acc = acc + l
            if acc > dist then break end
        end
    end
    return total
end

-------------------------------------------------------------------------------
-- Start / stop
-------------------------------------------------------------------------------

function TramlineDriver:tramlineDriverToggle()
    if self.spec_tramlineDriver.active then
        self:tramlineDriverStop()
    else
        self:tramlineDriverStart()
    end
end

function TramlineDriver:tramlineDriverStart()
    local spec = self.spec_tramlineDriver
    local sys = g_tramlineSystem
    if TramlineAD.isDriving(self) then
        sys:notify("AutoDrive is driving this vehicle - stop AutoDrive first")
        return
    end
    local x, _, z = getWorldTranslation(self.rootNode)
    local fieldId = TramlineFieldUtil.getFieldIdAt(x, z)
    if fieldId == nil then
        sys:notify("Vehicle is not on a field")
        return
    end
    local geo = sys.store:getGeometry(fieldId)
    if geo == nil or geo.lines == nil or #geo.lines == 0 then
        sys:notify(string.format("No tramlines stored for field %d - open the menu with Right Ctrl+M", fieldId))
        return
    end
    -- a refill trip that AutoDrive did not finish: we take over from here
    if spec.refill ~= nil then
        spec.refill = nil
        sys.gateLines = nil; sys.gateInfo = nil
        if sys.refillVehicles ~= nil then sys.refillVehicles[self] = nil end
    end

    local dirNode = TramlineDriver.getDirectionNode(self)
    local vx, _, vz = getWorldTranslation(dirNode)
    local hx, _, hz = localDirectionToWorld(dirNode, 0, 0, 1)

    -- carry on with the unfinished job where it stopped (or ask, if the
    -- vehicle is somewhere else), or a new job from here
    local set = sys.store:getSet(fieldId) or {}
    sys.cornersVehicle = TramlineDriver.canReverse(self)
    if set.corners == "reverse" and not sys.cornersVehicle then
        sys:notify("Tramline worker: a trailed " .. TramlineDriver.toolWords(self).noun .. " cannot back into the corners - it takes the corners round")
    end
    local plan = sys:planStart(fieldId, vx, vz, hx, hz, false, false, self)
    if plan ~= nil and plan.kind == "choice" then
        TramlineDriver.askWhereToStart(self, fieldId, plan, vx, vz, hx, hz)
        return
    end
    TramlineDriver.startPlan(self, fieldId, plan)
end

-- The unfinished job stopped somewhere else: drive back there (on the
-- tramlines), carry on from here, or start a new job from here?
function TramlineDriver.askWhereToStart(self, fieldId, plan, vx, vz, hx, hz)
    local sys = g_tramlineSystem
    local options, actions = {}, {}
    if plan.back ~= nil then
        options[#options + 1] = string.format("Drive back to where it stopped (%.0f m)", plan.backInfo.total)
        actions[#actions + 1] = "back"
    end
    if plan.at ~= nil then
        options[#options + 1] = string.format("Carry on from here (skips %.0f m)", plan.at.skipped or 0)
        actions[#actions + 1] = "here"
    end
    local newPlan = sys:planStart(fieldId, vx, vz, hx, hz, true, false, self)
    if newPlan ~= nil and newPlan.kind == "new" then
        options[#options + 1] = "Start a new job from here"
        actions[#actions + 1] = "new"
    end
    if #options == 0 then
        sys:notify("Tramline worker: " .. tostring(plan.backWhy or "drive onto a tramline"))
        return
    end
    local function choose(_, index)
        local a = actions[index or 0]
        if a == nil or self.spec_tramlineDriver.active then return end
        if a == "back" then
            TramlineDriver.startPlan(self, fieldId, { kind = "back", route = plan.back, job = plan.job,
                                                      backInfo = plan.backInfo, pct = plan.pct })
        elseif a == "here" then
            TramlineDriver.startPlan(self, fieldId, { kind = "resume", route = plan.route, at = plan.at,
                                                      job = plan.job, pct = plan.pct })
        else
            TramlineDriver.startPlan(self, fieldId, newPlan)
        end
    end
    local w = TramlineDriver.toolWords(self)
    local text = plan.back ~= nil
        and ("The " .. w.noun .. " is not where the job stopped. It can drive back there on the tramlines and carry on " .. w.ing .. " from that spot.")
        or ("The " .. w.noun .. " is not where the job stopped, and " .. tostring(plan.backWhy or "there is no way back from here") .. ".")
    local title = string.format("Field %d: unfinished job (%d%% done)", fieldId, plan.pct or 0)
    if not TramlineMenu.showOptions(title, text, options, choose, nil) then
        choose(nil, 1)       -- no dialog: the first choice (back, if possible)
    end
end

-- set off with a plan from planStart (or { kind = "back" }: the way back to
-- where the unfinished job stopped)
function TramlineDriver.startPlan(self, fieldId, plan)
    local sys = g_tramlineSystem
    local geo = sys.store:getGeometry(fieldId)
    local set = sys.store:getSet(fieldId) or {}
    if plan ~= nil and plan.wrongWay then
        local way = plan.way == "cw" and "clockwise" or "anticlockwise"
        sys:notify(string.format("Tramline worker: this field goes %s round the headland - turn round to start", way))
        return
    end
    if geo == nil or plan == nil or plan.kind == "away" or plan.kind == "choice" then
        sys:notify("Tramline worker: drive onto the headland tramline (or a tramline) to start")
        return
    end
    TramlineLog.info("[TramlinePilot][drv] %s on field %d: %s", tostring(plan.kind), fieldId, TramlineDriver.describeVehicle(self))
    TramlineLog.info("[TramlinePilot][drv] field %d settings: %s", fieldId, TramlineSystem.describeSet(set))
    local route, job, startIx = plan.route, plan.job, 1
    if plan.kind == "back" then
        -- the job keeps where it stopped until the vehicle is back there
        sys:notify(string.format("Tramline worker: driving back to where it stopped (%.0f m, %d%% done)",
            plan.backInfo and plan.backInfo.total or 0, plan.pct or 0))
    else
        local at = plan.at
        startIx = math.max(1, at.ix or 1)
        if plan.kind == "new" then
            job.sig = TramlineRoute.layoutSignature(geo)
            job.n = #route
            job.route, job.geo = route, geo
            job.startIx = startIx               -- where the course starts (start flag)
            sys.store:setJob(fieldId, job)      -- replaces an unfinished one
        end
        job.ix = startIx
        job.x, job.z, job.dirX, job.dirZ = at.x, at.z, at.dirX, at.dirZ
        local done, total = TramlineRoute.progress(route, startIx)
        local pct = total > 0 and math.floor(100 * done / total + 0.5) or 0
        local big = (at.skipped or 0) > TramlineSystem.SKIP_OK
        if plan.kind == "resume" then
            sys:notify(string.format("Tramline worker: carrying on (%d%% done)%s", pct,
                big and string.format(" - skips %.0f m of %s", at.skipped, TramlineDriver.toolWords(self).ing) or ""))
        elseif plan.joined then
            sys:notify(string.format("Tramline worker: starting from this tramline%s%s",
                big and string.format(" - skips the %.0f m of tramline before it", at.skipped) or "",
                job.order == "off" and (geo.headlands or 0) > 0 and " (headland not included)" or ""))
        end
    end

    -- refill with AutoDrive: check it is set up now, not when the tank is empty
    if set.refillAD then
        local d, why = TramlineAD.destinations(self)
        local gate = nil
        if d ~= nil then gate, why = TramlineRoute.gate(geo, d.gx, d.gz) end
        if gate == nil then
            sys:notify("Refill with AutoDrive: " .. tostring(why))
            sys.gateLines = nil; sys.gateInfo = nil
        else
            sys.gateLines = TramlineRoute.gateLines(gate)
            sys.gateInfo = { x = gate.x, z = gate.z, name = d.gateName }
            TramlineLog.info("[TramlinePilot][drv] refill: gate '%s' (%.0f m from the headland tramline), refill point '%s'",
                d.gateName, gate.dist, d.refillName)
        end
    end

    TramlineDriver.beginDriving(self, fieldId, geo, route, startIx, job)
    if plan.kind == "new" and not plan.joined then
        sys:notify(string.format("Tramline worker started: %d waypoints (Alt+Q stops)", #route))
    end
end

-- set off on a route (job route, or the way back from a refill)
function TramlineDriver.beginDriving(self, fieldId, geo, route, startIx, job)
    local spec = self.spec_tramlineDriver
    local sys = g_tramlineSystem
    local radius = geo.radius or TramlineDriver.getTurningRadius(self)
    spec.route = route
    spec.fieldId = fieldId
    spec.geo = geo
    spec.job = job
    spec.refill = nil
    spec.ppc = TramlineRoute.PPC.new(route, radius, startIx)
    spec.markers = TramlineDriver.findMarkerOffsets(self)
    spec.isWorking = false
    spec.state = "start"
    spec.timer = 0
    spec.lowering = false
    spec.checkTimer = nil
    spec.stuckTimer = 0
    spec.active = true
    sys.activeRoute = route
    sys.activeRouteIx = startIx
    sys.activeFieldId = fieldId
    TramlineLog.info("[TramlinePilot][drv] field %d: %d waypoints from %d, layout radius %.1f m, vehicle radius %.1f m, boom markers front %.1f back %.1f",
        fieldId, #route, startIx, radius, TramlineDriver.getTurningRadius(self), spec.markers.front, spec.markers.back)

    TramlineDriver.clearBrakeToStop(self)
    if self.setCruiseControlState ~= nil then
        self:setCruiseControlState(Drivable.CRUISECONTROL_STATE_OFF, true)
    end
    if self.startMotor ~= nil and self.getIsMotorStarted ~= nil and not self:getIsMotorStarted() then
        self:startMotor()
    end
    setCrabSteering(self, true)
    sys.activeVehicle = self
    refreshInput(self)
    self:raiseActive()
end

-- finished: the job is done (nothing to carry on); reason: shown to the player
function TramlineDriver:tramlineDriverStop(finished, reason)
    local spec = self.spec_tramlineDriver
    if not spec.active then return end
    self:tramlineDriverSetWorking(false)
    spec.active = false
    spec.refill = nil
    g_tramlineSystem.gateLines = nil; g_tramlineSystem.gateInfo = nil
    -- the job stays stored (with how far it got) until it is finished
    if finished then
        g_tramlineSystem.store:clearJob(spec.fieldId)
    elseif spec.job ~= nil then
        TramlineLog.info("[TramlinePilot][drv] stopped at waypoint %d of %d", spec.job.ix or 0, spec.job.n or 0)
    end
    spec.job = nil
    if spec.state ~= "start" then
        TramlineDriver.raiseAIEvent(self, "onAIFieldWorkerEnd", "onAIImplementEnd")
    end
    if self.rootVehicle ~= nil and self.rootVehicle.actionController ~= nil then
        pcall(self.rootVehicle.actionController.resetCurrentState, self.rootVehicle.actionController)
    end
    setCrabSteering(self, false)
    g_tramlineSystem.activeRoute = nil
    g_tramlineSystem.activeFieldId = nil
    if g_tramlineSystem.activeVehicle == self then g_tramlineSystem.activeVehicle = nil end
    refreshInput(self)
    TramlineDriver.brakeAfterStop(self)
    if finished then
        g_tramlineSystem:notify("Tramline worker finished the field")
    elseif reason ~= nil then
        g_tramlineSystem:notify("Tramline worker stopped: " .. reason .. " - Alt+T here carries on")
    else
        g_tramlineSystem:notify("Tramline worker stopped - Alt+T here carries on")
    end
end

-------------------------------------------------------------------------------
-- Refilling with AutoDrive
--   out of spray -> stop, boom up and folded ("prefold") -> the shortest way
--   on the tramlines to the field gate ("exit") -> at the gate ("fold") ->
--   AutoDrive: refill and back ("ad") -> AutoDrive stopped ("back") -> back
--   on the tramlines to where the spray ran out, and on with the job.
--   AutoDrive stopped somewhere it cannot come back from: "waiting"
-------------------------------------------------------------------------------

-- spray left: level, capacity, fill type (sprayer tanks and their extra tanks)
function TramlineDriver.sprayFill(vehicle)
    local level, cap, ft = 0, 0, nil
    for _, obj in ipairs(TramlineDriver.getAIImplements(vehicle)) do
        local ss = obj.spec_sprayer
        if ss ~= nil and obj.getSprayerFillUnitIndex ~= nil then
            local ok, idx = pcall(obj.getSprayerFillUnitIndex, obj)
            if ok and idx ~= nil then
                level = level + (obj:getFillUnitFillLevel(idx) or 0)
                cap = cap + (obj:getFillUnitCapacity(idx) or 0)
                if obj.getFillUnitFillType ~= nil then
                    local f = obj:getFillUnitFillType(idx)
                    if f ~= nil and (FillType == nil or f ~= FillType.UNKNOWN) then ft = f end
                end
                -- extra tanks feeding this sprayer (front tanks)
                local srcs = ft ~= nil and ss.fillTypeSources ~= nil and ss.fillTypeSources[ft] or nil
                for _, src in ipairs(srcs or {}) do
                    if src.vehicle ~= nil and src.vehicle ~= obj and src.vehicle.getFillUnitFillLevel ~= nil then
                        level = level + (src.vehicle:getFillUnitFillLevel(src.fillUnitIndex) or 0)
                        cap = cap + (src.vehicle:getFillUnitCapacity(src.fillUnitIndex) or 0)
                    end
                end
            end
        end
    end
    return level, cap, ft
end

-- the sprayer is empty: refill with AutoDrive if set up, else stop
function TramlineDriver:tramlineDriverOutOfSpray()
    local spec = self.spec_tramlineDriver
    if not spec.active or spec.refill ~= nil then return end
    local set = g_tramlineSystem.store:getSet(spec.fieldId) or {}
    if not set.refillAD then
        self:tramlineDriverStop(false, "the " .. TramlineDriver.toolWords(self).noun .. " is empty")
        return
    end
    local ok, why = TramlineDriver.startRefill(self)
    if not ok then
        self:tramlineDriverStop(false, "the " .. TramlineDriver.toolWords(self).noun .. " is empty (refill: " .. tostring(why) .. ")")
    end
end

-- Out of spray: stop, boom up and fold, then the shortest way on the
-- tramlines to the field gate (AutoDrive takes over there). The job waits
-- where the spray ran out.
function TramlineDriver.startRefill(self)
    local spec = self.spec_tramlineDriver
    if spec.job == nil or spec.geo == nil or spec.ppc == nil then return false, "no job running" end
    local d, why = TramlineAD.destinations(self)
    if d == nil then return false, why end
    local gate, why2 = TramlineRoute.gate(spec.geo, d.gx, d.gz)
    if gate == nil then return false, why2 end
    local steerNode = TramlineDriver.getSteeringNode(self)
    local vx, _, vz = getWorldTranslation(steerNode)
    local dirNode = TramlineDriver.getDirectionNode(self)
    local hx, _, hz = localDirectionToWorld(dirNode, 0, 0, 1)
    local exit, dist, how = TramlineRoute.toGate(spec.geo, spec.route, spec.ppc.relevantIx, vx, vz, hx, hz, gate)
    if exit == nil then
        local why3
        exit, why3 = TramlineRoute.exitRoute(spec.geo, spec.route, spec.ppc.relevantIx, vx, vz, gate)
        if exit == nil then return false, why3 end
        how = "old way"
    end
    -- metres to go, from each waypoint
    local cum, total = {}, 0
    cum[1] = 0
    for i = 2, #exit do
        total = total + MathUtil.vector2Length(exit[i][1] - exit[i - 1][1], exit[i][2] - exit[i - 1][2])
        cum[i] = total
    end
    local job = spec.job
    self:tramlineDriverSetWorking(false)
    spec.refill = { stage = "prefold", timer = 0, gate = gate, dest = d, fieldId = spec.fieldId, cum = cum, total = total }
    spec.job = nil                       -- the job waits where the spray ran out
    spec.route = exit
    spec.ppc = TramlineRoute.PPC.new(exit, spec.geo.radius or 7, 1)
    g_tramlineSystem.activeRoute = exit
    g_tramlineSystem.activeRouteIx = 1
    g_tramlineSystem.gateLines = TramlineRoute.gateLines(gate)
    g_tramlineSystem.gateInfo = { x = gate.x, z = gate.z, name = d.gateName }
    TramlineLog.info("[TramlinePilot][drv] out of spray at job waypoint %d - folding, then %.0f m (%d waypoints, %s) to the gate '%s'",
        job.ix or 0, total, #exit, tostring(how), d.gateName)
    g_tramlineSystem:notifyImportant(string.format("%s empty: %s%.0f m on the tramlines to '%s' - AutoDrive refills it at '%s'",
        TramlineDriver.toolWords(self).Noun, TramlineDriver.allFolded(self) and "" or "folding up, then ", total, d.gateName, d.refillName))
    return true
end

local function isFoldableObj(obj)
    local fs = obj.spec_foldable
    return fs ~= nil and fs.foldingParts ~= nil and #fs.foldingParts > 0
end

-- fold everything away for the road
-- is this implement folded away (fold animation at the end away from work)?
local function objFolded(obj)
    local fs = obj.spec_foldable
    local t = fs.foldAnimTime
    if t == nil and obj.getFoldAnimTime ~= nil then
        local ok, v = pcall(obj.getFoldAnimTime, obj)
        if ok then t = v end
    end
    if t == nil then return true end
    local dir = fs.turnOnFoldDirection or -1
    if dir < 0 then return t >= 0.99 end
    return t <= 0.01
end

-- fold everything away for the road
function TramlineDriver.forceFold(vehicle)
    for _, obj in ipairs(childVehicles(vehicle)) do
        if isFoldableObj(obj) and obj.setFoldState ~= nil and not objFolded(obj) then
            local fs = obj.spec_foldable
            local dir = -(fs.turnOnFoldDirection or -1)
            local allowed = true
            if obj.getIsFoldAllowed ~= nil then
                local ok, a = pcall(obj.getIsFoldAllowed, obj, dir, false)
                allowed = (not ok) or a ~= false
            end
            if allowed then
                pcall(obj.setFoldState, obj, dir, false)
                TramlineLog.info("[TramlinePilot][drv] folding %s", tostring(obj.getName and obj:getName() or "?"))
            end
        end
    end
end

function TramlineDriver.allFolded(vehicle)
    for _, obj in ipairs(childVehicles(vehicle)) do
        if isFoldableObj(obj) and not objFolded(obj) then return false end
    end
    return true
end

-- our worker steps aside for AutoDrive (the job and the refill trip stay)
function TramlineDriver.release(self)
    local spec = self.spec_tramlineDriver
    spec.active = false
    spec.isWorking = false
    setCrabSteering(self, false)
    local sys = g_tramlineSystem
    sys.activeRoute = nil
    sys.activeFieldId = nil
    if sys.activeVehicle == self then sys.activeVehicle = nil end
    if self.rootVehicle ~= nil and self.rootVehicle.actionController ~= nil then
        pcall(self.rootVehicle.actionController.resetCurrentState, self.rootVehicle.actionController)
    end
    refreshInput(self)
end

-- folded at the gate: over to AutoDrive
function TramlineDriver.handToAD(self)
    local spec = self.spec_tramlineDriver
    local r = spec.refill
    TramlineDriver.release(self)
    local ok, why = TramlineAD.startRefillTrip(self, spec.sprayFillType)
    if not ok then
        spec.refill = nil
        g_tramlineSystem.gateLines = nil; g_tramlineSystem.gateInfo = nil
        g_tramlineSystem:notify("Tramline worker: AutoDrive could not take over (" .. tostring(why) .. ") - Alt+T here carries on after refilling")
        return
    end
    r.stage = "ad"
    r.idle = 0
    local sys = g_tramlineSystem
    sys.refillVehicles = sys.refillVehicles or {}
    sys.refillVehicles[self] = true
    local w = TramlineDriver.toolWords(self)
    sys:notifyImportant(string.format("AutoDrive is taking the %s to '%s' - it carries on %s afterwards", w.noun, r.dest.refillName, w.ing))
end

-- AutoDrive is done: back along the tramlines to where the spray ran out,
-- from wherever AutoDrive stopped. If that is not possible (not refilled,
-- or stopped off the field) the trip waits: when AutoDrive is started again
-- we take over after it, or Alt+T on the field drives back from there.
function TramlineDriver.returnFromRefill(self)
    local spec = self.spec_tramlineDriver
    local r = spec.refill
    local sys = g_tramlineSystem
    if r == nil then return end
    local function wait(text)
        r.stage = "waiting"
        sys.refillVehicles = sys.refillVehicles or {}
        sys.refillVehicles[self] = true
        sys:notify(text)
    end
    local level, cap = TramlineDriver.sprayFill(self)
    if cap > 0 and level < cap * 0.1 then
        wait("Tramline worker: the " .. TramlineDriver.toolWords(self).noun .. " was not refilled - start AutoDrive again, or refill it and press Alt+T on the field")
        return
    end
    local geo = sys.store:getGeometry(r.fieldId)
    local job = sys.store:getJob(r.fieldId)
    local jobRoute = job ~= nil and sys.store:getJobRoute(r.fieldId) or nil
    if geo == nil or jobRoute == nil then
        spec.refill = nil
        sys.gateLines = nil; sys.gateInfo = nil
        sys:notify("Tramline worker: the job on this field is gone - Alt+T starts a new one")
        return
    end
    local dirNode = TramlineDriver.getDirectionNode(self)
    local vx, _, vz = getWorldTranslation(dirNode)
    local hx, _, hz = localDirectionToWorld(dirNode, 0, 0, 1)
    local route, info = TramlineRoute.backToJob(geo, jobRoute, job.ix or 1, job.x, job.z, vx, vz, hx, hz,
        { gate = r.gate and { x = r.gate.x, z = r.gate.z } or nil })
    if route == nil then
        TramlineLog.info("[TramlinePilot][drv] no way back from where AutoDrive stopped: %s", tostring(info))
        wait("Tramline worker: AutoDrive stopped away from the field gate - drive onto the field and press Alt+T, it drives back to where it ran out")
        return
    end
    spec.refill = nil
    sys.gateLines = nil; sys.gateInfo = nil
    TramlineLog.info("[TramlinePilot][drv] refilled (%.0f/%.0f) - %d waypoints (%.0f m, %s) back to job waypoint %d",
        level, cap, info.n, info.total, info.how, job.ix or 0)
    TramlineDriver.beginDriving(self, r.fieldId, geo, route, 1, job)
    sys:notifyImportant(string.format("Refilled - driving back to where it ran out (%.0f m)", info.total))
end

-------------------------------------------------------------------------------
-- Implement control
-------------------------------------------------------------------------------

function TramlineDriver:tramlineDriverSetWorking(working)
    local spec = self.spec_tramlineDriver
    if spec.isWorking == working then return end
    spec.isWorking = working
    for _, obj in ipairs(TramlineDriver.getAIImplements(self)) do
        try(obj, working and "aiImplementStartLine" or "aiImplementEndLine")
    end
    if self.raiseStateChange ~= nil and VehicleStateChange ~= nil then
        local st = working and VehicleStateChange.AI_START_LINE or VehicleStateChange.AI_END_LINE
        if st ~= nil then pcall(self.raiseStateChange, self, st) end
    end
    if working then
        spec.lowering = true          -- crawl until the implements are down
        spec.lowerTimer = 0
        spec.checkTimer = 3000        -- then check they really are switched on
    else
        spec.lowering = false
        spec.checkTimer = nil
    end
    TramlineLog.info("[TramlinePilot][drv] start line %s", working and "ON" or "OFF")
end

-------------------------------------------------------------------------------
-- Handing the vehicle back standing still
--   The game's AI driving (AIVehicleUtil.driveToPoint) switches cruise
--   control on at the speed it is given, and it stays on when the worker
--   stops: the vehicle would carry on at the player's cruise control speed.
--   So: cruise control off, then brake every frame until it stands (like
--   Courseplay at the end of a job, after Drivable:brakeToStop).
-------------------------------------------------------------------------------

TramlineDriver.BRAKE_TIMEOUT = 8000      -- ms; gives up braking after this

function TramlineDriver.brakeAfterStop(self)
    local spec = self.spec_tramlineDriver
    if self.setCruiseControlState ~= nil and Drivable ~= nil and Drivable.CRUISECONTROL_STATE_OFF ~= nil then
        pcall(self.setCruiseControlState, self, Drivable.CRUISECONTROL_STATE_OFF, true)
    end
    if self.isServer and WheelsUtil ~= nil and WheelsUtil.updateWheelsPhysics ~= nil then
        pcall(WheelsUtil.updateWheelsPhysics, self, 0, 0, 0, true, true)
    end
    spec.brakeToStop = true
    spec.brakeTimer = 0
    TramlineDriver.updateBrakeToStop(self, 0)
end

function TramlineDriver.clearBrakeToStop(self)
    local spec = self.spec_tramlineDriver
    if not spec.brakeToStop then return end
    spec.brakeToStop = false
    local d = self.spec_drivable
    if d ~= nil and d.lastInputValues ~= nil then
        d.lastInputValues.targetSpeed = nil
        d.lastInputValues.targetDirection = nil
    end
end

function TramlineDriver.updateBrakeToStop(self, dt)
    local spec = self.spec_tramlineDriver
    local d = self.spec_drivable
    spec.brakeTimer = (spec.brakeTimer or 0) + (dt or 0)
    local speed = math.abs(self.getLastSpeed ~= nil and self:getLastSpeed() or 0)
    if d == nil or d.lastInputValues == nil or speed < 1 or spec.brakeTimer > TramlineDriver.BRAKE_TIMEOUT then
        TramlineDriver.clearBrakeToStop(self)
        return
    end
    d.lastInputValues.targetSpeed = 0.51
    d.lastInputValues.targetDirection = 1
    if self.raiseActive ~= nil then self:raiseActive() end
end

-------------------------------------------------------------------------------
-- Driving loop
-------------------------------------------------------------------------------

-- Every drivable vehicle has this spec: idle ones return at once. A script
-- error while working is written to the mod's log with what the worker was
-- doing, and the worker stops and brakes instead of driving on unsteered.
function TramlineDriver:onUpdate(dt, isActiveForInput, isActiveForInputIgnoreSelection, isSelected)
    local spec = self.spec_tramlineDriver
    if spec == nil or (not spec.active and not spec.brakeToStop) then return end
    if not TramlineLog.guard("the worker", TramlineDriver.updateDriving, self, dt) then
        TramlineDriver.failSafe(self)
    end
end

function TramlineDriver.failSafe(self)
    local spec = self.spec_tramlineDriver
    if spec.failing then return end
    spec.failing = true
    TramlineLog.write("ERROR", "  worker: " .. TramlineDriver.describeState(self))
    TramlineLog.write("ERROR", "  machine: " .. TramlineDriver.describeVehicle(self))
    if spec.active then
        local ok = pcall(self.tramlineDriverStop, self, false, "a script error - please send TramlinePilot_log.txt")
        if not ok then
            spec.active = false
            pcall(TramlineDriver.brakeAfterStop, self)
        end
    else
        -- the error was in the braking: let go, but no cruise control
        spec.brakeToStop = false
        if self.setCruiseControlState ~= nil and Drivable ~= nil then
            pcall(self.setCruiseControlState, self, Drivable.CRUISECONTROL_STATE_OFF, true)
        end
    end
    spec.failing = false
end

-- what the worker is doing, for the log
function TramlineDriver.describeState(self)
    local spec = self.spec_tramlineDriver
    local ppc = spec.ppc
    local parts = { string.format("field %s, state %s", tostring(spec.fieldId), tostring(spec.state)) }
    if ppc ~= nil and spec.route ~= nil then
        parts[#parts + 1] = string.format("waypoint %s of %d%s", tostring(ppc.currentIx), #spec.route, ppc.reversing and " reversing" or "")
    end
    if spec.refill ~= nil then parts[#parts + 1] = "refill " .. tostring(spec.refill.stage) end
    local ok, v = pcall(function() return self:getLastSpeed() end)
    if ok and v ~= nil then parts[#parts + 1] = string.format("%.1f km/h", v) end
    parts[#parts + 1] = spec.isWorking and "spraying" or "not spraying"
    if not spec.active then parts[#parts + 1] = "stopped" end
    return table.concat(parts, ", ")
end

-- the machine, for the log: vehicle, the implements it works with, the tank
function TramlineDriver.describeVehicle(self)
    local function name(o)
        local ok, n = pcall(function() return o:getName() end)
        return string.format("%s (%s)", ok and tostring(n) or "?", tostring(o.typeName or "?"))
    end
    local parts = { name(self) }
    local ok, impls = pcall(TramlineDriver.getAIImplements, self)
    if ok and impls ~= nil then
        local list = {}
        for _, o in ipairs(impls) do if o ~= self then list[#list + 1] = name(o) end end
        if #list > 0 then parts[#parts + 1] = "with " .. table.concat(list, ", ") end
    end
    local okF, level, cap, ft = pcall(TramlineDriver.sprayFill, self)
    if okF and cap ~= nil and cap > 0 then
        local ftName = ft
        if ft ~= nil and g_fillTypeManager ~= nil and g_fillTypeManager.getFillTypeNameByIndex ~= nil then
            local okN, n = pcall(g_fillTypeManager.getFillTypeNameByIndex, g_fillTypeManager, ft)
            if okN and n ~= nil then ftName = n end
        end
        parts[#parts + 1] = string.format("tank %.0f/%.0f %s", level or 0, cap, tostring(ftName or "empty"))
    end
    local okT, w = pcall(TramlineDriver.getTrackWidth, self)
    if okT and w ~= nil then parts[#parts + 1] = string.format("wheel track %.2f m", w) end
    local okR, r = pcall(TramlineDriver.getTurningRadius, self)
    if okR and r ~= nil then parts[#parts + 1] = string.format("turning radius %.1f m", r) end
    return table.concat(parts, "; ")
end

function TramlineDriver:updateDriving(dt)
    local spec = self.spec_tramlineDriver
    if spec.brakeToStop and not spec.active then TramlineDriver.updateBrakeToStop(self, dt) end
    if not spec.active then return end
    if not self.isServer then return end
    self:raiseActive()

    local route, ppc = spec.route, spec.ppc
    if ppc == nil or route == nil then return end
    spec.timer = spec.timer + dt

    -- preparation, one step per frame, standing still
    if spec.state ~= "drive" then
        AIVehicleUtil.driveToPoint(self, dt, 0, false, true, 0, 1, 0)
        if spec.state == "start" then
            try(self, "updateAIFieldWorkerImplementData")
            TramlineDriver.raiseAIEvent(self, "onAIFieldWorkerStart", "onAIImplementStart")
            spec.state = "prepare"
        elseif spec.state == "prepare" then
            TramlineDriver.raiseAIEvent(self, "onAIFieldWorkerPrepareForWork", "onAIImplementPrepareForWork")
            spec.state = "unfold"
            spec.timer = 0
            TramlineDriver.logImplementState(self, "start")
        elseif spec.state == "unfold" then
            local done = TramlineDriver.allUnfolded(self)
            if not done and spec.timer > 4000 and not spec.forcedUnfold then
                spec.forcedUnfold = true
                TramlineDriver.forceUnfold(self)
            end
            if (done and spec.timer > 500) or spec.timer > TramlineDriver.UNFOLD_TIMEOUT then
                TramlineLog.info("[TramlinePilot][drv] unfolded (%s) after %.1f s", done and "ok" or "timeout", spec.timer / 1000)
                spec.state = "drive"
                spec.forcedUnfold = nil
            end
        end
        return
    end

    -- out of spray: standing, boom up and folded, then off to the gate
    if spec.refill ~= nil and spec.refill.stage == "prefold" then
        local r = spec.refill
        AIVehicleUtil.driveToPoint(self, dt, 0, false, true, 0, 1, 0)
        r.timer = r.timer + dt
        if r.foldAt == nil or r.timer - r.foldAt > 2000 then
            r.foldAt = r.timer
            TramlineDriver.forceFold(self)
        end
        local folded = TramlineDriver.allFolded(self)
        if (r.timer > 1500 and folded) or r.timer > 25000 then
            TramlineLog.info("[TramlinePilot][drv] folded (%s) after %.1f s - %.0f m to the gate", folded and "ok" or "timeout", r.timer / 1000, r.total or 0)
            r.stage, r.timer, r.toldAt = "exit", 0, 0
        end
        return
    end
    if spec.refill ~= nil and spec.refill.stage == "exit" and spec.refill.cum ~= nil then
        local r = spec.refill
        r.timer = (r.timer or 0) + dt
        if r.timer - (r.toldAt or 0) > 30000 then
            r.toldAt = r.timer
            local left = (r.total or 0) - (r.cum[ppc.relevantIx] or 0)
            g_tramlineSystem:notify(string.format("Tramline worker: %.0f m to '%s' for the refill", math.max(0, left), r.dest.gateName))
        end
    end

    -- at the field gate: standing, folding, then over to AutoDrive
    if spec.refill ~= nil and spec.refill.stage == "fold" then
        AIVehicleUtil.driveToPoint(self, dt, 0, false, true, 0, 1, 0)
        spec.refill.timer = spec.refill.timer + dt
        if (spec.refill.timer > 1500 and TramlineDriver.allFolded(self)) or spec.refill.timer > 25000 then
            TramlineDriver.handToAD(self)
        end
        return
    end

    local dirNode = TramlineDriver.getDirectionNode(self)
    local steerNode = TramlineDriver.getSteeringNode(self)
    local sx, _, sz = getWorldTranslation(steerNode)
    local dx, _, dz = getWorldTranslation(dirNode)
    local hx, _, hz = localDirectionToWorld(dirNode, 0, 0, 1)

    ppc:update(sx, sz)
    g_tramlineSystem.activeRouteIx = ppc.relevantIx
    if (ppc.reversing == true) ~= (spec.wasReversing == true) then
        spec.wasReversing = ppc.reversing == true
        TramlineLog.info("[TramlinePilot][drv] %s at waypoint %d", spec.wasReversing and "backing into a headland corner" or "driving on", ppc.relevantIx)
    end

    -- how far the job has got (kept in the store, saved with the savegame)
    local job = spec.job
    if job ~= nil and not ppc.finished then
        local ix = ppc.relevantIx
        local a, b = route[ix], route[ix + 1]
        -- only on the job's own waypoints (not on the way back from a refill)
        if a ~= nil and b ~= nil and a.jix ~= nil then
            job.ix = a.jix
            local ddx, ddz, l = TramlineGeom.normalize(b[1] - a[1], b[2] - a[2])
            if l > 0.05 then job.dirX, job.dirZ = ddx, ddz end
            job.x, job.z = ppc.projX or a[1], ppc.projZ or a[2]
        end
    end

    if ppc.finished then
        if spec.refill ~= nil and spec.refill.stage == "exit" then
            -- at the gate: boom up, fold, then AutoDrive
            spec.refill.stage = "fold"
            spec.refill.timer = 0
            self:tramlineDriverSetWorking(false)
            TramlineDriver.raiseAIEvent(self, "onAIFieldWorkerEnd", "onAIImplementEnd")
            TramlineDriver.forceFold(self)
            AIVehicleUtil.driveToPoint(self, dt, 0, false, true, 0, 1, 0)
        else
            self:tramlineDriverStop(true)
        end
        return
    end

    -- spray left? (every half second while working on the job)
    if spec.refill == nil and spec.job ~= nil then
        spec.fillTimer = (spec.fillTimer or 0) + dt
        if spec.fillTimer > 500 then
            spec.fillTimer = 0
            local level, cap, ft = TramlineDriver.sprayFill(self)
            if ft ~= nil then spec.sprayFillType = ft end
            if cap > 0 and level <= math.max(2, cap * 0.002) then
                TramlineLog.info("[TramlinePilot][drv] %s empty (%.1f of %.0f l)", TramlineDriver.toolWords(self).noun, level, cap)
                self:tramlineDriverOutOfSpray()
                return
            end
        end
    end
    if not ppc.initializing and ppc.offTrack > TramlineDriver.OFF_TRACK_LIMIT then
        TramlineLog.info("[TramlinePilot][drv] off the route by %.1f m at waypoint %d", ppc.offTrack, ppc.currentIx)
        self:tramlineDriverStop(false, "it left the tramlines")
        return
    end

    -- boom: on from just before the front of the work area reaches a
    -- working stretch until the back of it has left the stretch (only on the
    -- stretch driven in the current direction; never while reversing)
    local rev = ppc.reversing == true
    local m = spec.markers or { front = 0, back = 0 }
    local pf = m.front + TramlineDriver.SWITCH_LEAD
    local lo, hi = ppc.sectionStart, ppc.sectionEnd
    local work = not rev and (TramlineRoute.workAt(route, ppc.currentIx, dx + hx * pf, dz + hz * pf, lo, hi)
        or TramlineRoute.workAt(route, ppc.currentIx, dx + hx * m.back, dz + hz * m.back, lo, hi))
    self:tramlineDriverSetWorking(work)

    local workSpeed = g_tramlineSystem.store.workSpeed or TramlineDriver.WORK_SPEED
    local turnSpeed = g_tramlineSystem.store.turnSpeed or TramlineDriver.TURN_SPEED
    local wp = route[ppc.currentIx] or route[#route]
    local maxSpeed = workSpeed
    if wp.turn or curvatureAhead(route, ppc.currentIx, 12) > math.rad(25) then
        maxSpeed = math.min(maxSpeed, turnSpeed)
    end
    if rev then maxSpeed = math.min(maxSpeed, TramlineDriver.REVERSE_SPEED) end
    -- slow down to stop where the direction changes, and at the end of the
    -- route (the end of the course, or the field gate for a refill) - that
    -- one measured along the route: a headland round ends where it began
    local toStop = nil
    if ppc.sectionEnd ~= nil and ppc.sectionEnd < #route then
        toStop = ppc.toSectionEnd
    elseif route[ppc.currentIx] ~= nil then
        local p = route[ppc.currentIx]
        toStop = math.sqrt((p[1] - sx) ^ 2 + (p[2] - sz) ^ 2) + TramlineRoute.lengthFrom(route, ppc.currentIx)
    end
    if toStop ~= nil and toStop < 10 then
        maxSpeed = math.min(maxSpeed, math.max(2, 1 + toStop * 0.8))
    end

    -- after a line start: crawl until the implements are down, then check them
    if spec.lowering then
        spec.lowerTimer = spec.lowerTimer + dt
        local ready = true
        if self.getCanAIFieldWorkerContinueWork ~= nil then
            local ok, can = pcall(self.getCanAIFieldWorkerContinueWork, self)
            ready = (not ok) or can
        end
        if ready or spec.lowerTimer > TramlineDriver.LOWER_TIMEOUT then
            spec.lowering = false
            if not ready then TramlineLog.info("[TramlinePilot][drv] implements not ready after %.0f s - driving on", spec.lowerTimer / 1000) end
        else
            maxSpeed = math.min(maxSpeed, 2)
        end
    end
    if spec.checkTimer ~= nil then
        spec.checkTimer = spec.checkTimer - dt
        if spec.checkTimer <= 0 then
            spec.checkTimer = nil
            local forced = TramlineDriver.ensureTurnedOn(self)
            TramlineDriver.logImplementState(self, forced > 0 and "switched on" or "working")
        end
    end

    -- blocked?
    if math.abs(self:getLastSpeed()) < 0.5 and not spec.lowering then
        spec.stuckTimer = spec.stuckTimer + dt
        if spec.stuckTimer > 30000 then
            self:tramlineDriverStop(false, "blocked")
            return
        end
    else
        spec.stuckTimer = 0
    end

    local gy = getTerrainHeightAtWorldPos(g_currentMission.terrainRootNode, ppc.goalX, 0, ppc.goalZ)
    local lx, _, lz = worldToLocal(steerNode, ppc.goalX, gy, ppc.goalZ)
    -- in reverse the goal point is behind: same steering circle, driven backwards
    AIVehicleUtil.driveToPoint(self, dt, 1, true, not rev, lx, lz, maxSpeed)

    if g_tramlineSystem.debug then
        spec.debugTimer = (spec.debugTimer or 0) + dt
        if spec.debugTimer > 1000 then
            spec.debugTimer = 0
            TramlineLog.info("[TramlinePilot][drv] wp %d/%d %s cte %.2f off %.2f la %.1f goal %.1f/%.1f work=%s v=%.1f max %.0f",
                ppc.currentIx, #route, tostring(wp.kind), ppc.cte, ppc.offTrack, ppc.lookahead, lx, lz,
                tostring(spec.isWorking), self:getLastSpeed(), maxSpeed)
        end
    end
end
