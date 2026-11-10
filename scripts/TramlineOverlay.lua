--[[
    TramlineOverlay.lua
    The tramlines in the field, drawn with TramlineDraw: smooth lines that
    follow the ground (thinner and fainter with distance), the worker's way
    ahead, the course's start and finish flags, and markers with label tags
    (carry on, last sprayed, the refill gate, A/B points). Nothing touches
    the ground; Alt+4 / the menu's 'Show lines' switch it.
]]

TramlineOverlay = {}
local TramlineOverlay_mt = Class(TramlineOverlay)

TramlineOverlay.DRAW_RADIUS = 650       -- metres from the camera

-- colours
TramlineOverlay.COLOR_LINE     = { 0.97, 0.95, 0.86 }   -- main tramlines
TramlineOverlay.COLOR_HEADLAND = { 1.00, 0.70, 0.22 }   -- headland tramline
TramlineOverlay.COLOR_CURVE    = { 1.00, 0.84, 0.56 }   -- turns, crossings, corner tramlines
TramlineOverlay.COLOR_WORK     = { 0.47, 0.88, 0.32 }   -- worker's way ahead: spraying
TramlineOverlay.COLOR_ROUTE    = { 0.55, 0.80, 1.00 }   -- worker's way ahead / ways back: not spraying
TramlineOverlay.COLOR_AB       = { 0.42, 0.66, 1.00 }   -- A / B points
TramlineOverlay.COLOR_START    = { 0.58, 0.84, 0.20 }   -- start / carry on
TramlineOverlay.COLOR_LAST     = { 1.00, 0.45, 0.32 }   -- where the unfinished job last sprayed
TramlineOverlay.COLOR_AWAY     = { 1.00, 0.70, 0.22 }   -- drive to here
TramlineOverlay.COLOR_FINISH   = { 0.97, 0.97, 0.95 }   -- finish of the course

-- widths in pixels at 40 m
TramlineOverlay.W_LINE, TramlineOverlay.W_HEADLAND, TramlineOverlay.W_CURVE = 2.6, 3.0, 2.2
TramlineOverlay.W_ROUTE = 3.8
TramlineOverlay.DASH = { 4.5, 3.0 }
TramlineOverlay.AHEAD = 300             -- m of the worker's way ahead shown

function TramlineOverlay.new(store)
    local self = setmetatable({}, TramlineOverlay_mt)
    self.store = store
    self.enabled = true
    self.pendingA = nil     -- {x,z} while user is setting up a field
    self.pendingB = nil
    return self
end

function TramlineOverlay:delete()
    TramlineDraw.delete()
end

-- the layout of one field
local function drawLayout(geo, cam)
    local D = TramlineDraw
    local O = TramlineOverlay
    local paths = D.layoutPaths(geo)
    local r2 = O.DRAW_RADIUS * O.DRAW_RADIUS
    for _, l in ipairs(geo.lines or {}) do
        if (l[1] - cam[1]) ^ 2 + (l[2] - cam[3]) ^ 2 < r2 or (l[3] - cam[1]) ^ 2 + (l[4] - cam[3]) ^ 2 < r2 then
            D.groundSegment(l[1], l[2], l[3], l[4], O.COLOR_LINE, O.W_LINE, 0.92, nil, cam)
        end
    end
    for _, pl in ipairs(paths.curves) do D.groundPath(pl, false, O.COLOR_CURVE, O.W_CURVE, 0.88, nil, cam, O.DRAW_RADIUS) end
    for _, pl in ipairs(paths.rings) do D.groundPath(pl, true, O.COLOR_HEADLAND, O.W_HEADLAND, 0.95, nil, cam, O.DRAW_RADIUS) end
end

-- the worker's way ahead: green where it sprays, blue where it does not,
-- chevrons just in front of it
local function drawWayAhead(route, ix, cam)
    local D = TramlineDraw
    local O = TramlineOverlay
    ix = math.max(1, math.min(ix or 1, #route - 1))
    local left = O.AHEAD
    for i = ix + 1, #route do
        local a, b = route[i - 1], route[i]
        local l = MathUtil.vector2Length(b[1] - a[1], b[2] - a[2])
        D.groundSegment(a[1], a[2], b[1], b[2], b.work and O.COLOR_WORK or O.COLOR_ROUTE, O.W_ROUTE, 0.95, nil, cam)
        left = left - l
        if left <= 0 then break end
    end
end

function TramlineOverlay:draw()
    if not self.enabled then return end
    if g_currentMission == nil or g_currentMission.terrainRootNode == nil then return end
    local sys = g_tramlineSystem
    local D = TramlineDraw
    local O = TramlineOverlay

    local cx, cy, cz = getWorldTranslation(getCamera())
    local cam = { cx, cy, cz }
    local r2 = O.DRAW_RADIUS * O.DRAW_RADIUS
    D.beginFrame()
    local callouts = {}
    local function callout(c) if c ~= nil then callouts[#callouts + 1] = c end end

    -- which fields to draw: mode 2 = only the field the player / active
    -- worker is on, 3 = all fields
    local mode = math.max(2, self.store.showMode or 2)
    local onlyField = nil
    if mode == 2 then
        onlyField = sys ~= nil and sys.activeFieldId or nil
        if onlyField == nil and sys ~= nil then
            local px, pz = sys:getPlayerWorldPos()
            if px ~= nil then onlyField = TramlineFieldUtil.getFieldIdAt(px, pz) end
        end
    end
    if mode == 3 or onlyField ~= nil then
        for fieldId, _ in pairs(self.store.sets) do
            local geo = (onlyField == nil or fieldId == onlyField) and self.store:getGeometry(fieldId) or nil
            if geo ~= nil and geo.bounds ~= nil then
                local b = geo.bounds
                local mx, mz = (b[1] + b[3]) * 0.5, (b[2] + b[4]) * 0.5
                if MathUtil.vector2LengthSq(mx - cx, mz - cz) < r2 * 4 then drawLayout(geo, cam) end
            end
        end
    end

    -- the worker's way ahead
    local route = sys ~= nil and sys.activeRoute or nil
    if route ~= nil and #route > 1 then drawWayAhead(route, sys.activeRouteIx, cam) end

    -- the way between the headland tramline and the field gate (refills)
    if sys ~= nil and sys.gateLines ~= nil then
        for _, pl in ipairs(sys.gateLines) do D.groundPath(pl, false, O.COLOR_ROUTE, O.W_HEADLAND, 0.9, O.DASH, cam) end
        local g = sys.gateInfo
        if g ~= nil then callout(D.callout(g.x, g.z, 2.2, g.name or "Field gate", "Refill gate - AutoDrive takes over here", O.COLOR_ROUTE, cam)) end
    end

    -- where Alt+T would set off from / where the unfinished job stopped
    local ps = sys ~= nil and sys.previewStart or nil
    if ps ~= nil then
        local accent = O.COLOR_START
        local title, sub
        local icon = nil        -- a new job: this is the start of the course (start flag)
        local pct = string.format("%d%% done", ps.pct or 0)
        if ps.kind == "choice" then
            accent = O.COLOR_LAST
            title = ps.spreader and "Last spread here" or "Last sprayed here"
            if ps.back ~= nil then
                sub = string.format("%s  ·  Alt+T drives back (%.0f m)", pct, ps.backLen or 0)
            else
                sub = pct .. "  ·  drive back here to carry on"
            end
        elseif ps.kind == "resume" then
            title, sub = "Carry on here", pct .. "  ·  Alt+T"
        elseif ps.unfinished then
            accent = O.COLOR_LAST
            title, sub = "Stopped here", "Drive back here to carry on"
        elseif ps.ready then
            title = ps.joined and "Start from this tramline" or "Start here"
            sub = "Alt+T starts the tramline worker"
            icon = "start"
        else
            accent = O.COLOR_AWAY
            title, sub = "Headland tramline", "Drive onto it here to start"
        end
        -- direction it sets off in
        local dx, dz = (ps.nx or ps.x) - ps.x, (ps.nz or ps.z) - ps.z
        if MathUtil.vector2Length(dx, dz) > 0.1 then
            D.chevrons(ps.x, ps.z, dx, dz, 3, 4.5, accent, cam)
        end
        -- too far away: the way from the vehicle to the headland tramline
        if ps.fromX ~= nil then
            D.groundSegment(ps.fromX, ps.fromZ, ps.x, ps.z, O.COLOR_AWAY, O.W_HEADLAND, 0.9, O.DASH, cam)
        end
        -- the way back to where it stopped
        if ps.back ~= nil then
            local b = ps.back
            local pts = {}
            for i = 1, math.min(ps.backN or #b, #b) do pts[i] = b[i] end
            D.groundPath(pts, false, O.COLOR_ROUTE, O.W_ROUTE, 0.9, O.DASH, cam, O.DRAW_RADIUS)
        end
        callout(D.callout(ps.x, ps.z, 2.2, title, sub, accent, cam, icon))
    end

    -- the course: green flag where it starts, chequered flag where it finishes
    local course = sys ~= nil and sys.course or nil
    if course ~= nil then
        if course.showStart then
            callout(D.callout(course.sx, course.sz, 2.2, "Start", nil, O.COLOR_START, cam, "start"))
        end
        callout(D.callout(course.fx, course.fz, 2.2, "Finish", course.finishSub, O.COLOR_FINISH, cam, "finish", "chequer"))
    end

    -- A / B points while setting a direction
    if self.pendingA ~= nil and self.pendingB ~= nil then
        D.groundSegment(self.pendingA[1], self.pendingA[2], self.pendingB[1], self.pendingB[2], O.COLOR_AB, O.W_HEADLAND, 0.9, O.DASH, cam)
    end
    if self.pendingA ~= nil then
        callout(D.callout(self.pendingA[1], self.pendingA[2], 1.6, "Point A", self.pendingB == nil and "Alt+2 sets point B" or nil, O.COLOR_AB, cam))
    end
    if self.pendingB ~= nil then
        callout(D.callout(self.pendingB[1], self.pendingB[2], 1.6, "Point B", "Right Ctrl+M: direction A-B", O.COLOR_AB, cam))
    end

    D.endFrame()
    -- labels on top of the lines, kept clear of each other, nearest in front
    D.layoutCallouts(callouts)
    for _, c in ipairs(callouts) do D.drawCallout(c) end
end
