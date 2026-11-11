--[[
    TramlineDialog.lua
    One-page settings dialog (gui/TramlineDialog.xml), built on the same
    FS25 profiles as Precision Farming's tramline dialog. Opened with
    Right Ctrl+M while on a field.
]]

TramlineDialog = {}
local TramlineDialog_mt = Class(TramlineDialog, DialogElement)

TramlineDialog.DIRECTIONS = { "Along longest edge", "Vehicle heading", "A - B points", "Angle", "As stored" }
TramlineDialog.TURNS      = { "Along headland tramline", "Loops where they fit" }
TramlineDialog.CORNERS    = { "Round", "Back into corners" }
-- headland direction: as the vehicle faces (turns both ways), or one way
-- round with every turn out of a tramline that way (the field in two halves)
TramlineDialog.HEADLAND_DIRS     = { "As the vehicle faces", "Clockwise", "Anticlockwise" }
TramlineDialog.HEADLAND_DIR_KEYS = { "either", "cw", "acw" }
TramlineDialog.ORDERS     = { "First", "Last", "Skip" }
TramlineDialog.ORDER_KEYS = { "first", "last", "off" }
TramlineDialog.RADIUS_MIN, TramlineDialog.RADIUS_MAX = 4, 15            -- m
TramlineDialog.PAINTS     = { "No", "Wheel tracks", "Full lane" }
TramlineDialog.ONOFF      = { "Off", "On" }
TramlineDialog.SHOW       = { "Off", "This field", "All fields" }
TramlineDialog.WORK_SPEED_MIN, TramlineDialog.WORK_SPEED_MAX = 4, 25   -- km/h
TramlineDialog.TURN_SPEED_MIN, TramlineDialog.TURN_SPEED_MAX = 3, 15

function TramlineDialog.new(target, customMt)
    local self = DialogElement.new(target, customMt or TramlineDialog_mt)
    self.system = target
    return self
end

function TramlineDialog:onCreate()
end

-- values -----------------------------------------------------------------

local function buildAngles()
    local t = {}
    for a = 0, 355, 5 do table.insert(t, string.format("%d°", a)) end
    return t
end

-- wheel track widths, 5 cm steps; the one measured on the vehicle we sit in is marked
local function trackCount()
    return math.floor((TramlinePainter.TRACK_MAX - TramlinePainter.TRACK_MIN) * 20 + 0.5) + 1
end
local function trackOf(i)
    return TramlinePainter.clampTrack(TramlinePainter.TRACK_MIN + (i - 1) * 0.05)
end
local function trackIndex(w)
    local i = math.floor((TramlinePainter.clampTrack(w) - TramlinePainter.TRACK_MIN) * 20 + 0.5) + 1
    return math.max(1, math.min(trackCount(), i))
end
TramlineDialog.trackOf, TramlineDialog.trackIndex = trackOf, trackIndex
local function buildTracks(measured)
    local t = {}
    for i = 1, trackCount() do
        local w = trackOf(i)
        t[i] = string.format("%.2f m", w)
        if measured ~= nil and math.abs(w - measured) < 0.001 then t[i] = t[i] .. " (your vehicle)" end
    end
    return t
end

local function buildSpacings()
    local t = {}
    for w = 6, 60, 1 do table.insert(t, string.format("%d m", w)) end
    return t
end

local function buildHeadlands()
    return { "0", "1", "2", "3", "4" }
end

local function buildSpeeds(lo, hi)
    local t = {}
    for v = lo, hi do table.insert(t, string.format("%d km/h", v)) end
    return t
end

local function buildRadii()
    local t = {}
    for r = TramlineDialog.RADIUS_MIN, TramlineDialog.RADIUS_MAX do table.insert(t, string.format("%d m", r)) end
    return t
end

-- angle in degrees (0 = north, clockwise) of a world direction (FS: +x east, -z north)
local function angleOf(dx, dz)
    local a = math.deg(TramlineGeom.atan2(dx, -dz))
    return (a + 360) % 360
end

-- current direction's angle for the state (what the angle row should show)
function TramlineDialog:currentAngle()
    local st = self.state
    local dir = st.direction or 1
    if dir == 4 then return st.angle or 0 end
    local tmp = {}
    self.system.menu:applyDirection(tmp, st, true)
    return angleOf(tmp.dx or 0, tmp.dz or -1)
end

-- open ------------------------------------------------------------------

-- st: the menu state (field info, set, flags) prepared by TramlineMenu
function TramlineDialog:setup(st)
    self.state = st
end

function TramlineDialog:onOpen()
    TramlineDialog:superClass().onOpen(self)
    local st = self.state
    local sys = self.system

    self.fieldText:setText(string.format("Field %d  -  %.1f ha  (approx. %.0f x %.0f m)",
        st.info.fieldId, st.info.areaHa, st.info.edgeLength, st.info.edgeWidth))

    self.angleOption:setTexts(buildAngles())
    self.spacingOption:setTexts(buildSpacings())
    self.headlandOption:setTexts(buildHeadlands())
    self.turnOption:setTexts(TramlineDialog.TURNS)
    self.radiusOption:setTexts(buildRadii())
    self.orderOption:setTexts(TramlineDialog.ORDERS)
    self.cornerOption:setTexts(TramlineDialog.CORNERS)
    self.headlandDirOption:setTexts(TramlineDialog.HEADLAND_DIRS)
    self.refillOption:setTexts(TramlineDialog.ONOFF)
    self.refillOption:setDisabled(TramlineAD.get() == nil)

    self.workSpeedOption:setTexts(buildSpeeds(TramlineDialog.WORK_SPEED_MIN, TramlineDialog.WORK_SPEED_MAX))
    self.workSpeedOption:setState(math.max(1, math.min(TramlineDialog.WORK_SPEED_MAX - TramlineDialog.WORK_SPEED_MIN + 1,
        math.floor((sys.store.workSpeed or 12) + 0.5) - TramlineDialog.WORK_SPEED_MIN + 1)))
    self.turnSpeedOption:setTexts(buildSpeeds(TramlineDialog.TURN_SPEED_MIN, TramlineDialog.TURN_SPEED_MAX))
    self.turnSpeedOption:setState(math.max(1, math.min(TramlineDialog.TURN_SPEED_MAX - TramlineDialog.TURN_SPEED_MIN + 1,
        math.floor((sys.store.turnSpeed or 7) + 0.5) - TramlineDialog.TURN_SPEED_MIN + 1)))

    self.paintOption:setTexts(TramlineDialog.PAINTS)
    self.paintOption:setState(st.paintMode or 1)
    self.trackOption:setTexts(buildTracks(st.measuredTrack))

    self.linesOption:setTexts(TramlineDialog.SHOW)
    self.linesOption:setState(sys.overlay.enabled and math.max(2, sys.store.showMode or 2) or 1)

    self.pfOption:setTexts(TramlinePFCompat.MODES)
    self.pfOption:setState(TramlinePFCompat.mode or 1)
    self.pfOption:setDisabled(not TramlinePFCompat.isPresent())

    self:syncRows()
end

-- show the state's set (after opening, or after loading a saved wheeling)
function TramlineDialog:syncRows()
    local st = self.state
    local set = st.set
    local dirs = {}
    for i = 1, 4 do dirs[i] = TramlineDialog.DIRECTIONS[i] end
    if st.storedDir ~= nil then dirs[5] = TramlineDialog.DIRECTIONS[5] end
    if (st.direction or 1) > #dirs then st.direction = 1 end
    self.directionOption:setTexts(dirs)
    self.directionOption:setState(st.direction or 1)
    self:syncAngleRow()
    self.spacingOption:setState(math.max(1, math.min(55, math.floor((set.spacing or 24) + 0.5) - 5)))
    self.headlandOption:setState(math.max(1, math.min(5, (set.headlands or 1) + 1)))
    self.turnOption:setState(set.turnStyle or 1)
    self.radiusOption:setState(math.max(1, math.min(TramlineDialog.RADIUS_MAX - TramlineDialog.RADIUS_MIN + 1,
        math.floor((set.turnRadius or 7) + 0.5) - TramlineDialog.RADIUS_MIN + 1)))
    local oi = 1
    for i, k in ipairs(TramlineDialog.ORDER_KEYS) do if k == set.headlandOrder then oi = i end end
    self.orderOption:setState(oi)
    self.cornerOption:setState(set.corners == "reverse" and 2 or 1)
    self.cornerOption:setDisabled((set.headlands or 0) < 1)
    local di = 1
    for i, k in ipairs(TramlineDialog.HEADLAND_DIR_KEYS) do if k == set.headlandDir then di = i end end
    self.headlandDirOption:setState(di)
    self.headlandDirOption:setDisabled((set.headlands or 0) < 1)
    self.refillOption:setState(set.refillAD and 2 or 1)
    self.trackOption:setState(trackIndex(set.trackWidth))
    self:updatePreview()
end

function TramlineDialog:onClose()
    self.previewGeo = nil
    TramlineDialog:superClass().onClose(self)
end

-- Regenerate the tramline geometry for the current settings (preview only;
-- nothing is stored until Generate).
function TramlineDialog:updatePreview()
    local st, sys = self.state, self.system
    local info = st.info
    local tmp = {}
    for k, v in pairs(st.set) do tmp[k] = v end
    self.system.menu:applyDirection(tmp, st, true)
    tmp.preview = true
    local ok, geo = pcall(TramlineGenerator.generate, tmp, info)
    if ok then
        self.previewGeo = geo
    else
        self.previewGeo = nil
        TramlineLog.warning("[TramlinePilot] preview failed: %s", tostring(geo))
    end
    local n = (self.previewGeo and #self.previewGeo.lines) or 0
    self.previewText:setText(string.format("%d tramlines @ %d m, %d headland pass(es), turn radius %d m, direction %d°%s",
        n, tmp.spacing or 0, tmp.headlands or 0, tmp.turnRadius or 0, math.floor(self:currentAngle() + 0.5),
        st.hadExisting and "\nGenerate overwrites the stored set" or ""))
end

-- Horizontal spans { z, {x1, x2, x3, x4, ...} } filling the polygons 'loops'
-- (even-odd), 'rows' rows across z0..z1 (for the preview map's fills)
local function fillSpans(loops, z0, z1, rows)
    local out = {}
    local dz = (z1 - z0) / rows
    for k = 0, rows - 1 do
        local z = z0 + (k + 0.5) * dz
        local xs = {}
        for _, lp in ipairs(loops) do
            local n = #lp
            for i = 1, n do
                local a, b = lp[i], lp[(i % n) + 1]
                if (a[2] <= z and b[2] > z) or (b[2] <= z and a[2] > z) then
                    xs[#xs + 1] = a[1] + (z - a[2]) / (b[2] - a[2]) * (b[1] - a[1])
                end
            end
        end
        table.sort(xs)
        out[#out + 1] = { z = z, xs = xs }
    end
    return out, dz
end

TramlineDialog.MAP = {
    band   = { 0.50, 0.55, 0.30 },      -- headland band
    inner  = { 0.33, 0.45, 0.21 },      -- main crop area
    edge   = { 0.90, 0.86, 0.68 },      -- field edge
    line   = { 0.97, 0.95, 0.86 },      -- tramlines
    ring   = { 1.00, 0.70, 0.22 },      -- headland tramline
    arrow  = { 1.00, 1.00, 1.00 },
    car    = { 0.36, 0.72, 1.00 },      -- the vehicle
}

-- 2D preview inside the right-hand box: a little map of the field - the
-- headland band a shade lighter than the main area, the tramlines, the
-- headland tramline, the direction, the vehicle, north and a scale
function TramlineDialog:draw(...)
    TramlineDialog:superClass().draw(self, ...)
    local box = self.previewBox
    local st = self.state
    if box == nil or st == nil or st.info == nil or box.absPosition == nil then return end
    local info = st.info
    local b = info.bounds
    local bw, bh = b[3] - b[1], b[4] - b[2]
    if bw < 1 or bh < 1 then return end
    local D, M = TramlineDraw, TramlineDialog.MAP

    -- box in screen units; keep the field's aspect (screen x is squashed by the aspect ratio)
    local px, py = box.absPosition[1], box.absPosition[2]
    local sw, sh = box.absSize[1], box.absSize[2]
    local pad = 0.018
    px, py, sw, sh = px + pad, py + pad + 0.045, sw - 2 * pad, sh - 2 * pad - 0.055   -- room for the caption
    local aspect = g_screenAspectRatio or 1.777
    local scale = math.min(sw * aspect / bw, sh / bh)      -- screen-y units per metre
    local fw, fh = bw * scale / aspect, bh * scale
    local ox = px + (sw - fw) * 0.5
    local oy = py + (sh - fh) * 0.5
    local function toScreen(x, z)
        -- north (-z) up
        return ox + (x - b[1]) * scale / aspect, oy + (b[4] - z) * scale
    end

    local geo = self.previewGeo
    local poly = info.polygon or info.boundary or {}

    -- fills (worked out again only when the preview changes)
    local fc = self.fillCache
    if fc == nil or fc.geo ~= geo or fc.info ~= info then
        local rows = 160
        local outer, dz = fillSpans({ poly }, b[2], b[4], rows)
        local inner = nil
        local H = geo and geo.headlands or 0
        if geo ~= nil and H > 0 and geo.rings ~= nil and geo.rings[H] ~= nil then
            inner = fillSpans(geo.rings[H], b[2], b[4], rows)
        end
        fc = { geo = geo, info = info, outer = outer, inner = inner, dz = dz }
        self.fillCache = fc
    end
    local rowH = fc.dz * scale + 0.0004
    local function fill(spans, c, a)
        for _, row in ipairs(spans) do
            local xs = row.xs
            for i = 1, #xs - 1, 2 do
                local x1, y1 = toScreen(xs[i], row.z + fc.dz * 0.5)
                local x2 = toScreen(xs[i + 1], row.z)
                if x2 > x1 then drawFilledRect(x1, y1, x2 - x1, rowH, c[1], c[2], c[3], a) end
            end
        end
    end
    fill(fc.outer, fc.inner ~= nil and M.band or M.inner, 0.95)
    if fc.inner ~= nil then fill(fc.inner, M.inner, 0.97) end

    -- field edge
    local n = #poly
    if n > 1 then
        local stepI = math.max(1, math.floor(n / 300))
        local i = 1
        while i <= n do
            local j = i + stepI
            local a, c = poly[i], poly[(j > n) and 1 or j]
            local x1, y1 = toScreen(a[1], a[2]); local x2, y2 = toScreen(c[1], c[2])
            D.line(x1, y1, x2, y2, 1.6, M.edge[1], M.edge[2], M.edge[3], 0.9)
            i = j
        end
    end

    if geo ~= nil then
        -- tramlines and the headland tramline, with a soft dark outline
        for _, l in ipairs(geo.lines) do
            local x1, y1 = toScreen(l[1], l[2]); local x2, y2 = toScreen(l[3], l[4])
            D.outlined(x1, y1, x2, y2, 1.5, M.line, 0.95)
        end
        for _, loops in pairs(geo.rings or {}) do
            for _, lp in ipairs(loops) do
                local m = #lp
                local stepI = math.max(1, math.floor(m / 240))
                local i = 1
                while i <= m do
                    local j = i + stepI
                    local a, c = lp[i], lp[(j > m) and 1 or j]
                    local x1, y1 = toScreen(a[1], a[2]); local x2, y2 = toScreen(c[1], c[2])
                    D.outlined(x1, y1, x2, y2, 1.9, M.ring, 0.95)
                    i = j
                end
            end
        end

        -- direction arrow through the field centre
        local tmp = {}
        self.system.menu:applyDirection(tmp, st, true)
        local dx, dz = tmp.dx or 0, tmp.dz or -1
        local l = MathUtil.vector2Length(dx, dz); if l > 0 then dx, dz = dx / l, dz / l end
        local len = math.min(bw, bh) * 0.22
        local cx, cz = info.centerX, info.centerZ
        local x1, y1 = toScreen(cx - dx * len, cz - dz * len)
        local x2, y2 = toScreen(cx + dx * len, cz + dz * len)
        D.outlined(x1, y1, x2, y2, 3.2, M.arrow, 1)
        local hx, hz = cx + dx * len, cz + dz * len
        local nx, nz = -dz, dx
        local a1x, a1y = toScreen(hx - dx * len * 0.3 + nx * len * 0.18, hz - dz * len * 0.3 + nz * len * 0.18)
        local a2x, a2y = toScreen(hx - dx * len * 0.3 - nx * len * 0.18, hz - dz * len * 0.3 - nz * len * 0.18)
        D.outlined(a1x, a1y, x2, y2, 3.2, M.arrow, 1)
        D.outlined(a2x, a2y, x2, y2, 3.2, M.arrow, 1)
    end

    -- the vehicle
    local vx, vz = st.startX, st.startZ
    if vx ~= nil then
        local sx, sy = toScreen(vx, vz)
        D.dot(sx, sy, 15, 0, 0, 0, 0.5)
        D.dot(sx, sy, 11, M.car[1], M.car[2], M.car[3], 1)
        D.ring(sx, sy, 24, M.car[1], M.car[2], M.car[3], 0.8)
    end

    -- north and a scale bar
    local ts = getCorrectTextSize(0.013)
    local nX, nY = px + sw - D.pxX(18), py + sh - D.pxY(40)
    D.outlined(nX, nY, nX, nY + D.pxY(22), 2.2, M.arrow, 0.9)
    D.outlined(nX - D.pxX(6), nY + D.pxY(14), nX, nY + D.pxY(22), 2.2, M.arrow, 0.9)
    D.outlined(nX + D.pxX(6), nY + D.pxY(14), nX, nY + D.pxY(22), 2.2, M.arrow, 0.9)
    setTextAlignment(RenderText.ALIGN_CENTER)
    setTextBold(true)
    setTextColor(1, 1, 1, 0.9)
    renderText(nX, nY + D.pxY(26), ts, "N")
    local metres = 10
    for _, m in ipairs({ 10, 20, 50, 100, 200, 500, 1000 }) do
        if m * scale / aspect <= sw * 0.28 then metres = m end
    end
    local sbX, sbY = px + D.pxX(6), py + D.pxY(8)
    local sbW = metres * scale / aspect
    D.outlined(sbX, sbY, sbX + sbW, sbY, 2, M.arrow, 0.9)
    D.outlined(sbX, sbY - D.pxY(4), sbX, sbY + D.pxY(4), 2, M.arrow, 0.9)
    D.outlined(sbX + sbW, sbY - D.pxY(4), sbX + sbW, sbY + D.pxY(4), 2, M.arrow, 0.9)
    setTextBold(false)
    renderText(sbX + sbW * 0.5, sbY + D.pxY(7), ts, string.format("%d m", metres))
    setTextAlignment(RenderText.ALIGN_LEFT)
    setTextColor(1, 1, 1, 1)
end

-- row callbacks (nothing applied until Generate, except the live toggles)

function TramlineDialog:syncAngleRow()
    local a = self:currentAngle()
    self.syncingAngle = true
    self.angleOption:setState(math.floor((a % 360) / 5 + 0.5) % 72 + 1)
    self.syncingAngle = false
end

function TramlineDialog:onClickDirection(state)
    self.state.direction = state
    self:syncAngleRow()
    self:updatePreview()
end

-- turning the angle row switches to 'Angle' direction, like PF's work direction
function TramlineDialog:onClickAngle(state)
    if self.syncingAngle then return end
    self.state.angle = (state - 1) * 5
    if self.state.direction ~= 4 then
        self.state.direction = 4
        self.directionOption:setState(4)
    end
    self:updatePreview()
end
function TramlineDialog:onClickSpacing(state)   self.state.set.spacing = state + 5; self:updatePreview() end
function TramlineDialog:onClickHeadland(state)
    self.state.set.headlands = state - 1
    self.cornerOption:setDisabled(state - 1 < 1)
    self.headlandDirOption:setDisabled(state - 1 < 1)
    self:updatePreview()
end
function TramlineDialog:onClickCorners(state)  self.state.set.corners = (state == 2) and "reverse" or "round" end
function TramlineDialog:onClickHeadlandDir(state)
    self.state.set.headlandDir = TramlineDialog.HEADLAND_DIR_KEYS[state] or "either"
    self:updatePreview()
end
function TramlineDialog:onClickWorkSpeed(state)
    self.system.store.workSpeed = state + TramlineDialog.WORK_SPEED_MIN - 1
    self.system.store.isDirty = true
end
function TramlineDialog:onClickTurnSpeed(state)
    self.system.store.turnSpeed = state + TramlineDialog.TURN_SPEED_MIN - 1
    self.system.store.isDirty = true
end
function TramlineDialog:onClickTurn(state)     self.state.set.turnStyle = state; self:updatePreview() end
function TramlineDialog:onClickRadius(state)
    self.state.set.turnRadius = state + TramlineDialog.RADIUS_MIN - 1
    self:updatePreview()
end
function TramlineDialog:onClickOrder(state)
    self.state.set.headlandOrder = TramlineDialog.ORDER_KEYS[state] or "first"
end
function TramlineDialog:onClickPaint(state)     self.state.paintMode = state end
function TramlineDialog:onClickTrack(state)     self.state.set.trackWidth = trackOf(state) end
function TramlineDialog:onClickRefill(state)    self.state.set.refillAD = (state == 2) end
function TramlineDialog:onClickLines(state)
    self.system:setShowMode(state)
end
function TramlineDialog:onClickPF(state)
    TramlinePFCompat.setMode(state)
    self.system.store.pfMode = TramlinePFCompat.mode
    self.system.store.pfAsked = true
    self.system.store.isDirty = true
end

-- buttons ----------------------------------------------------------------

function TramlineDialog:onClickBack()
    self:close()
end

-- the saved wheelings of this field: pick one to load its settings
function TramlineDialog:onClickLibrary()
    local st = self.state
    local list = TramlineLibrary.list(st.info.fieldId)
    if #list == 0 then
        self.system:notify(string.format("No saved wheelings for field %d yet - they are saved when you press Generate", st.info.fieldId))
        return
    end
    local names = {}
    for i, w in ipairs(list) do names[i] = TramlineLibrary.describe(w) end
    -- the list replaces this dialog for a moment; it comes back afterwards
    self:close()
    TramlineMenu.showOptions(string.format("Saved wheelings - field %d", st.info.fieldId),
        "Load the settings of a saved wheeling (Generate then lays exactly those tramlines again):", names,
        function(_, index)
            local w = list[index or 0]
            if w ~= nil then
                TramlineLibrary.apply(w, st.set)
                st.set.fieldId = st.info.fieldId
                st.storedDir = TramlineMenu.storedDirOf(st.set)
                st.direction = TramlineMenu.DIR_STORED
                st.loadedName = w.name
                self.system:notify("Loaded wheeling '" .. w.name .. "' - press Generate to use it")
            end
            self:setup(st)
            g_gui:showDialog("TramlineDialog")
        end, nil)
end

function TramlineDialog:onClickClear()
    local st = self.state
    self.system.store:removeSet(st.info.fieldId)
    self.system:notify(string.format("Cleared tramlines for field %d", st.info.fieldId))
    self:close()
end

function TramlineDialog:onClickOk()
    local st = self.state
    self:close()
    self.system.menu:applyAndGenerate(st)
end
