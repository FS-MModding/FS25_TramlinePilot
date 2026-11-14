--[[
    TramlineMenu.lua
    The user-facing setup flow. Opened with Right Ctrl+M (or 'tlMenu' in console)
    while standing on a field:

      1. field is scanned automatically (ID, area, outline, longest edge)
      2. direction  : longest edge (auto) / vehicle heading / A-B points / typed angle
      3. spacing    : metres between tramlines (sprayer width)
      4. headland   : number of headland passes
      5. tramlines are generated, overlay switched on, summary shown

    Built entirely from base-game dialogs (OptionDialog / TextInputDialog /
    InfoDialog) so there is no custom GUI XML to maintain. A single-screen
    panel can replace this later without touching the generator.
]]

TramlineMenu = {}
local TramlineMenu_mt = Class(TramlineMenu)

TramlineMenu.DIR_AUTO    = 1
TramlineMenu.DIR_VEHICLE = 2
TramlineMenu.DIR_AB      = 3
TramlineMenu.DIR_ANGLE   = 4
TramlineMenu.DIR_STORED  = 5     -- as stored (the field's set, or a loaded wheeling)

-- the parts of a set that fix where the tramlines lie
TramlineMenu.DIR_KEYS = { "ax", "az", "dx", "dz", "anchorToEdge", "offset" }
function TramlineMenu.storedDirOf(set)
    local d = {}
    for _, k in ipairs(TramlineMenu.DIR_KEYS) do d[k] = set[k] end
    return d
end

function TramlineMenu.new(system)
    local self = setmetatable({}, TramlineMenu_mt)
    self.system = system
    self.lastSpacing = 24
    self.lastHeadlands = 1
    self.state = nil
    return self
end

-------------------------------------------------------------------------------
-- Dialog compatibility shims
-- FS25 moved dialogs to static <Dialog>.show(...) functions; FS22 used
-- g_gui:show<Dialog>Dialog({...}). Try the new one first, fall back.
-------------------------------------------------------------------------------

-- returns false if no option dialog could be shown
local function showOptions(title, text, options, callback, target)
    if OptionDialog ~= nil and OptionDialog.show ~= nil then
        -- FS25: OptionDialog.show(callback, title, text, options) — callback(item), no target slot
        OptionDialog.show(function(item) callback(target, item) end, title, text, options)
    elseif g_gui ~= nil and g_gui.showOptionDialog ~= nil then
        g_gui:showOptionDialog({ title = title, text = text, options = options, callback = callback, target = target })
    else
        TramlineLog.error("[TramlinePilot] no option dialog available")
        return false
    end
    return true
end
TramlineMenu.showOptions = showOptions

local function showTextInput(prompt, default, maxChars, callback, target)
    if TextInputDialog ~= nil and TextInputDialog.show ~= nil then
        -- FS25: TextInputDialog.show(callback, target, defaultText, dialogPrompt, imePrompt, maxCharacters, confirmText)
        TextInputDialog.show(callback, target, tostring(default), prompt, prompt, maxChars, g_i18n:getText("button_ok"))
    elseif g_gui.showTextInputDialog ~= nil then
        g_gui:showTextInputDialog({ defaultText = tostring(default), dialogPrompt = prompt, maxCharacters = maxChars, callback = callback, target = target })
    else
        TramlineLog.error("[TramlinePilot] no text input dialog available")
    end
end

local function showInfo(title, text)
    if InfoDialog ~= nil and InfoDialog.show ~= nil then
        -- FS25: InfoDialog.show(text, callback, target, dialogType) — no title slot
        local dialogType = (DialogElement ~= nil and DialogElement.TYPE_INFO) or nil
        InfoDialog.show(title .. "\n\n" .. text, nil, nil, dialogType)
    elseif g_gui.showInfoDialog ~= nil then
        g_gui:showInfoDialog({ title = title, text = text })
    else
        TramlineLog.info("[TramlinePilot] %s: %s", title, text)
    end
end

-------------------------------------------------------------------------------
-- Entry
-------------------------------------------------------------------------------

function TramlineMenu:open()
    local sys = self.system
    local x, z = sys:getPlayerWorldPos()
    if x == nil then return end

    local info = FieldScanner.scan(x, z)
    if info == nil then
        sys:notify("Stand on a field to set up tramlines")
        return
    end

    -- work on a copy; it is stored only when the tramlines are generated
    local existing = sys.store:getSet(info.fieldId)
    local set = TramlineStore.newSet(info.fieldId)
    if existing ~= nil then
        for k, v in pairs(existing) do set[k] = v end
    end
    self.state = {
        info = info,
        set = set,
        startX = x, startZ = z,
        hadExisting = existing ~= nil,
    }
    -- a point surely on the field, to find it again after loading
    set.refX, set.refZ = x, z
    -- the wheel track of the machine we sit in: shown in the menu's list,
    -- and what a new field starts with
    local v = sys:getPlayerVehicle()
    if v ~= nil and TramlineDriver ~= nil and TramlineDriver.getTrackWidth ~= nil then
        local ok, w = pcall(TramlineDriver.getTrackWidth, v)
        if ok and w ~= nil then
            self.state.measuredTrack = TramlinePainter.clampTrack(w)
        elseif not ok then
            TramlineLog.warning("[TramlinePilot] could not measure the wheel track: %s", tostring(w))
        end
    end
    if existing == nil then
        -- new field: turn radius from the vehicle we sit in (a bit roomier
        -- than its tightest turn), at least 6 m
        if v ~= nil and v.spec_tramlineDriver ~= nil then
            set.turnRadius = math.max(6, math.ceil(TramlineDriver.getTurningRadius(v) + 1))
        end
        set.trackWidth = self.state.measuredTrack or set.trackWidth
    end
    set.trackWidth = TramlinePainter.clampTrack(set.trackWidth)
    set.turnRadius = set.turnRadius or 7
    set.turnStyle = set.turnStyle or 1
    set.headlandOrder = set.headlandOrder or "first"
    sys.store.fieldInfo = sys.store.fieldInfo or {}
    sys.store.fieldInfo[info.fieldId] = info

    if existing ~= nil then
        self.lastSpacing = existing.spacing or self.lastSpacing
        self.lastHeadlands = existing.headlands or self.lastHeadlands
    end

    -- an existing field keeps where its tramlines lie unless changed
    if existing ~= nil then self.state.storedDir = TramlineMenu.storedDirOf(existing) end

    -- One-page dialog if it loaded, else the step-by-step wizard below
    if sys.dialogLoaded and sys.dialog ~= nil then
        self.state.direction = existing ~= nil and TramlineMenu.DIR_STORED or TramlineMenu.DIR_AUTO
        self.state.paintMode = 1
        sys.dialog:setup(self.state)
        g_gui:showDialog("TramlineDialog")
        return
    end

    -- First time only, if Precision Farming is present: ask whether PF should
    -- stop laying its own tramlines. Changeable later with 'tlPF on|off'.
    if not sys.store.pfAsked and (rawget(_G, "g_precisionFarming") ~= nil or rawget(_G, "ExtendedSowingMachine") ~= nil) then
        sys.store.pfAsked = true
        sys.store.isDirty = true
        showOptions("Precision Farming detected",
            "PF creates its own tramlines when seeding. Disable them so only this mod's tramlines are used?\n(Change later with 'tlPF on' / 'tlPF off' in the console.)",
            { "Yes – disable PF tramlines", "No – keep both" }, self.onPFChosen, self)
        return
    end

    self:askDirection()
end

function TramlineMenu:onPFChosen(index)
    if index == 1 then
        TramlinePFCompat.setEnabled(true)
        self.system.store.isDirty = true
    end
    if self.state ~= nil then self:askDirection() end
end

-------------------------------------------------------------------------------
-- Step 2: direction
-------------------------------------------------------------------------------

function TramlineMenu:askDirection()
    local info = self.state.info
    local header = string.format("%s  –  %.1f ha  (approx. %.0f × %.0f m)",
        TramlineDialog.fieldName(info), info.areaHa, info.edgeLength, info.edgeWidth)

    local options = {
        "Along longest edge (auto)",
        "Vehicle heading",
        "A – B points",
        "Enter angle in degrees",
    }
    if self.state.hadExisting then
        table.insert(options, "Keep current direction")
    end

    showOptions("Tramline direction", header, options, self.onDirectionChosen, self)
end

function TramlineMenu:onDirectionChosen(index)
    if index == nil or index == 0 then self.state = nil; return end
    local st = self.state
    local set, info, sys = st.set, st.info, self.system

    set.anchorToEdge = false
    if index == TramlineMenu.DIR_AUTO then
        set.ax, set.az = info.centerX, info.centerZ
        set.dx, set.dz = info.edgeDx, info.edgeDz
        set.anchorToEdge = true
    elseif index == TramlineMenu.DIR_VEHICLE then
        local v = sys:getPlayerVehicle()
        set.ax, set.az = st.startX, st.startZ
        if v ~= nil then
            local dx, _, dz = localDirectionToWorld(v.rootNode, 0, 0, 1)
            set.dx, set.dz = dx, dz
        else
            set.dx, set.dz = info.edgeDx, info.edgeDz
        end
    elseif index == TramlineMenu.DIR_AB then
        local a, b = sys.overlay.pendingA, sys.overlay.pendingB
        if a == nil or b == nil then
            sys:notify("Set A (Alt+1) and B (Alt+2) first, then reopen the menu")
            self.state = nil
            return
        end
        set.ax, set.az = a[1], a[2]
        set.dx, set.dz = b[1] - a[1], b[2] - a[2]
    elseif index == TramlineMenu.DIR_ANGLE then
        showTextInput("Direction angle (0–359°, 0 = north, 90 = east)", 0, 4, self.onAngleEntered, self)
        return
    else
        -- keep current: leave ax/az/dx/dz as stored
    end

    self:askSpacing()
end

function TramlineMenu:onAngleEntered(text, confirmed)
    if confirmed == false then self.state = nil; return end
    local deg = tonumber(text)
    if deg == nil then
        self.system:notify("Angle must be a number")
        self.state = nil
        return
    end
    local a = math.rad(deg)
    local set, info = self.state.set, self.state.info
    set.ax, set.az = info.centerX, info.centerZ
    -- FS world: +X east, -Z north. 0° = north, clockwise.
    set.dx, set.dz = math.sin(a), -math.cos(a)
    set.anchorToEdge = true
    self:askSpacing()
end

-------------------------------------------------------------------------------
-- Step 3: spacing
-------------------------------------------------------------------------------

function TramlineMenu:askSpacing()
    showTextInput("Tramline spacing in metres (working width)", self.lastSpacing, 5, self.onSpacingEntered, self)
end

function TramlineMenu:onSpacingEntered(text, confirmed)
    if confirmed == false then self.state = nil; return end
    local w = tonumber(text)
    if w == nil or w < 1 or w > 100 then
        self.system:notify("Spacing must be between 1 and 100 m")
        self.state = nil
        return
    end
    self.lastSpacing = w
    self.state.set.spacing = w
    self:askHeadland()
end

-------------------------------------------------------------------------------
-- Step 4: headland
-------------------------------------------------------------------------------

function TramlineMenu:askHeadland()
    showTextInput("Headland passes (0 = none)", self.lastHeadlands, 2, self.onHeadlandEntered, self)
end

function TramlineMenu:onHeadlandEntered(text, confirmed)
    if confirmed == false then self.state = nil; return end
    local n = tonumber(text)
    if n == nil or n < 0 or n > 6 then
        self.system:notify("Headland passes must be 0–6")
        self.state = nil
        return
    end
    self.lastHeadlands = math.floor(n)
    self.state.set.headlands = self.lastHeadlands
    self:askPaint()
end

-------------------------------------------------------------------------------
-- Step 5: create the tramlines in the crop now?
-------------------------------------------------------------------------------

function TramlineMenu:askPaint()
    local w = TramlinePainter.clampTrack(self.state.set.trackWidth)
    local options = {
        "No – overlay only",
        string.format("Wheel tracks (2 narrow tracks, %.2f m apart)", w),
        string.format("Full lane (%.2f m wide)", TramlinePainter.laneWidth(w)),
    }
    showOptions("Create tramlines in the crop now?",
        "Cultivates narrow strips along each tramline so the crop is removed there straight away. Headland tramline is included.",
        options, self.onPaintChosen, self)
end

function TramlineMenu:onPaintChosen(index)
    if index == nil or index == 0 then self.state = nil; return end
    self.state.paintMode = index   -- matches TramlinePainter.PAINT_* numbering
    self:finish()
end

-------------------------------------------------------------------------------
-- Step 5: generate + summary
-------------------------------------------------------------------------------

function TramlineMenu:finish()
    local st = self.state
    local sys = self.system
    sys.store.fieldInfo = sys.store.fieldInfo or {}
    sys.store.fieldInfo[st.info.fieldId] = st.info
    sys.store:putSet(st.info.fieldId, st.set)
    local geo = sys.store:regenerate(st.info.fieldId)

    sys.overlay.pendingA, sys.overlay.pendingB = nil, nil

    local totalLen = 0
    for _, l in ipairs(geo.lines) do
        totalLen = totalLen + MathUtil.vector2Length(l[3] - l[1], l[4] - l[2])
    end

    local paintLine = "Crop: untouched (lines shown only)"
    local mode = st.paintMode or TramlinePainter.PAINT_NONE
    if mode ~= TramlinePainter.PAINT_NONE then
        local w = TramlinePainter.clampTrack(st.set.trackWidth)
        TramlinePainter.paint(geo, mode, w)
        if mode == TramlinePainter.PAINT_FULL then
            paintLine = string.format("Crop: tramlines created, %.2f m wide lanes", TramlinePainter.laneWidth(w))
        else
            paintLine = string.format("Crop: tramlines created, wheel tracks %.2f m apart", w)
        end
    end

    TramlineLog.info("[TramlinePilot] generated field %d: %s; %s", st.info.fieldId, TramlineSystem.describeSet(st.set), paintLine)

    -- keep the wheelings for later crops / other savegames
    local saveLine = ""
    local okSave, wname = pcall(TramlineLibrary.save, st.set)
    if okSave and wname ~= nil then
        saveLine = string.format("\nSaved as wheeling '%s'", wname)
    elseif not okSave then
        TramlineLog.warning("[TramlinePilot] could not save the wheeling: %s", tostring(wname))
    end

    local turns = string.format("Turns: %d along the headland tramline, %d loops (radius %.0f m)",
        geo.ringTurnCount or 0, geo.loopCount or 0, geo.radius or 0)
    if st.set.corners == "reverse" and (geo.cornerCount or 0) > 0 then
        turns = turns .. string.format("\nHeadland corners: backs into %d corner(s)", geo.cornerCount)
    end
    local warn = ""
    for _, w in ipairs(geo.warnings or {}) do warn = warn .. "\nNote: " .. w end

    local text = string.format(
        "%s\nArea: %.1f ha\nTramlines: %d @ %.1f m\nHeadland passes: %d\nTotal tramline length: %.0f m\n%s\n%s%s%s\n\nPark on the headland tramline (orange) and press Alt+T to start the worker.",
        TramlineDialog.fieldName(st.info), st.info.areaHa, #geo.lines, st.set.spacing, st.set.headlands, totalLen, turns, paintLine, saveLine, warn)

    showInfo("Tramlines set", text)
    self.state = nil
end


-------------------------------------------------------------------------------
-- Called by TramlineDialog's Generate button with the edited state
-------------------------------------------------------------------------------

-- Apply the chosen direction (st.direction / st.angle) to 'set'. Returns
-- false if A-B points are needed but missing (quiet = no notification).
function TramlineMenu:applyDirection(set, st, quiet)
    local info, sys = st.info, self.system
    local dir = st.direction or 1
    set.anchorToEdge = false
    if dir == TramlineMenu.DIR_AUTO then
        set.ax, set.az = info.centerX, info.centerZ
        set.dx, set.dz = info.edgeDx, info.edgeDz
        set.anchorToEdge = true
    elseif dir == TramlineMenu.DIR_VEHICLE then
        local v = sys:getPlayerVehicle()
        set.ax, set.az = st.startX, st.startZ
        if v ~= nil then
            local dx, _, dz = localDirectionToWorld(v.rootNode, 0, 0, 1)
            set.dx, set.dz = dx, dz
        else
            set.dx, set.dz = info.edgeDx, info.edgeDz
        end
    elseif dir == TramlineMenu.DIR_AB then
        local a, b = sys.overlay.pendingA, sys.overlay.pendingB
        if a == nil or b == nil then
            if not quiet then sys:notify("Set A (Alt+1) and B (Alt+2) first, then reopen the menu") end
            -- preview: fall back to auto so something is shown
            set.ax, set.az = info.centerX, info.centerZ
            set.dx, set.dz = info.edgeDx, info.edgeDz
            set.anchorToEdge = true
            return false
        end
        set.ax, set.az = a[1], a[2]
        set.dx, set.dz = b[1] - a[1], b[2] - a[2]
    elseif dir == TramlineMenu.DIR_ANGLE then
        local a = math.rad(st.angle or 0)
        set.ax, set.az = info.centerX, info.centerZ
        set.dx, set.dz = math.sin(a), -math.cos(a)
        set.anchorToEdge = true
    elseif dir == TramlineMenu.DIR_STORED then
        local d = st.storedDir
        if d ~= nil and d.dx ~= nil then
            for _, k in ipairs(TramlineMenu.DIR_KEYS) do set[k] = d[k] end
        else
            set.ax, set.az = info.centerX, info.centerZ
            set.dx, set.dz = info.edgeDx, info.edgeDz
            set.anchorToEdge = true
        end
    end
    return true
end

function TramlineMenu:applyAndGenerate(st)
    self.state = st
    local set = st.set
    if not self:applyDirection(set, st, false) then
        self.state = nil
        return
    end
    self.lastSpacing = set.spacing
    self.lastHeadlands = set.headlands
    self:finish()
end

function TramlineMenu:_oldApplyAndGenerate(st)
    self.state = st
    local set, info, sys = st.set, st.info, self.system
    local dir = st.direction or 1

    set.anchorToEdge = false
    if dir == TramlineMenu.DIR_AUTO then
        set.ax, set.az = info.centerX, info.centerZ
        set.dx, set.dz = info.edgeDx, info.edgeDz
        set.anchorToEdge = true
    elseif dir == TramlineMenu.DIR_VEHICLE then
        local v = sys:getPlayerVehicle()
        set.ax, set.az = st.startX, st.startZ
        if v ~= nil then
            local dx, _, dz = localDirectionToWorld(v.rootNode, 0, 0, 1)
            set.dx, set.dz = dx, dz
        else
            set.dx, set.dz = info.edgeDx, info.edgeDz
        end
    elseif dir == TramlineMenu.DIR_AB then
        local a, b = sys.overlay.pendingA, sys.overlay.pendingB
        if a == nil or b == nil then
            sys:notify("Set A (Alt+1) and B (Alt+2) first, then reopen the menu")
            self.state = nil
            return
        end
        set.ax, set.az = a[1], a[2]
        set.dx, set.dz = b[1] - a[1], b[2] - a[2]
    elseif dir == TramlineMenu.DIR_ANGLE then
        local a = math.rad(st.angle or 0)
        set.ax, set.az = info.centerX, info.centerZ
        set.dx, set.dz = math.sin(a), -math.cos(a)
        set.anchorToEdge = true
    end

    self.lastSpacing = set.spacing
    self.lastHeadlands = set.headlands
    self:finish()
end
