--[[
    TramlinePFCompat.lua
    Switch off Precision Farming's own tramlines.

    PF (FS25_precisionFarming) keeps its classes and g_precisionFarming in its
    own mod environment, reached as FS25_precisionFarming.<name>. Its seeders
    ask PF's tramline map for the tramline width at the seeding position
    (TramlineMap:getTramlineWidthAtWorldPos - set per field in PF's map screen,
    'Tramlines' button); no width = no tramlines. This is the same point the
    [TH] Precision Farming Configurator uses to switch them off.

    Modes (menu row 'Precision Farming tramlines', console tlPF):
      1 On                      PF as normal
      2 Off on tramline fields  no PF tramlines where this mod has a layout
      3 Off everywhere          no PF tramlines at all
    Tramlines PF already made stay in the crop until the next seeding.
]]

TramlinePFCompat = {}

TramlinePFCompat.MODES = { "On", "Off on tramline fields", "Off everywhere" }
TramlinePFCompat.ON, TramlinePFCompat.OFF_FIELDS, TramlinePFCompat.OFF_ALL = 1, 2, 3
TramlinePFCompat.mode = 1
TramlinePFCompat.enabled = false     -- PF tramlines are switched off somewhere
TramlinePFCompat.hooked = false
TramlinePFCompat.blockedCalls = 0

-- PF's mod environment, or nil
function TramlinePFCompat.env()
    local ok, env = pcall(function() return FS25_precisionFarming end)
    if not ok or type(env) ~= "table" then return nil end
    local ok2, tm = pcall(function() return env.TramlineMap end)
    if (not ok2 or tm == nil) and type(rawget(env, "_G")) == "table" then env = rawget(env, "_G") end
    return env
end

function TramlinePFCompat.isPresent()
    local env = TramlinePFCompat.env()
    if env == nil then return false end
    local ok, tm = pcall(function() return env.TramlineMap end)
    return ok and type(tm) == "table"
end

-- should PF lay no tramline at this position?
function TramlinePFCompat.blockedAt(x, z)
    local m = TramlinePFCompat.mode
    if m == TramlinePFCompat.OFF_ALL then return true end
    if m ~= TramlinePFCompat.OFF_FIELDS or type(x) ~= "number" or type(z) ~= "number" then return false end
    if g_tramlineSystem == nil or g_tramlineSystem.store == nil then return false end
    local ok, fieldId = pcall(TramlineFieldUtil.getFieldIdAt, x, z)
    return ok and fieldId ~= nil and g_tramlineSystem.store:getSet(fieldId) ~= nil
end

local function wrap(original)
    return function(self, x, z, ...)
        if TramlinePFCompat.mode ~= TramlinePFCompat.ON then
            -- (x, z), or (x, y, z): then the third number is z
            local third = ...
            local blocked = TramlinePFCompat.blockedAt(x, z)
            if not blocked and type(third) == "number" then blocked = TramlinePFCompat.blockedAt(x, third) end
            if blocked then
                TramlinePFCompat.blockedCalls = TramlinePFCompat.blockedCalls + 1
                return nil
            end
        end
        return original(self, x, z, ...)
    end
end

-- put our check in front of PF's tramline width (once; the mode decides)
function TramlinePFCompat.hook()
    if TramlinePFCompat.hooked then return true end
    local env = TramlinePFCompat.env()
    if env == nil then
        TramlineLog.info("[TramlinePilot][PF] Precision Farming not loaded - nothing to switch off")
        return false
    end
    local ok, TM = pcall(function() return env.TramlineMap end)
    if not ok or type(TM) ~= "table" or type(TM.getTramlineWidthAtWorldPos) ~= "function" then
        TramlineLog.warning("[TramlinePilot][PF] Precision Farming found, but not its tramline width function - PF tramlines cannot be switched off")
        return false
    end
    TM.getTramlineWidthAtWorldPos = wrap(TM.getTramlineWidthAtWorldPos)
    -- the tramline map object may carry its own copy
    local pf = nil
    pcall(function() pf = env.g_precisionFarming end)
    local inst = type(pf) == "table" and pf.tramlineMap or nil
    if type(inst) == "table" and type(rawget(inst, "getTramlineWidthAtWorldPos")) == "function" then
        inst.getTramlineWidthAtWorldPos = wrap(rawget(inst, "getTramlineWidthAtWorldPos"))
    end
    TramlinePFCompat.hooked = true
    TramlineLog.info("[TramlinePilot][PF] Precision Farming tramlines: switch in place (TramlineMap.getTramlineWidthAtWorldPos)")
    return true
end

function TramlinePFCompat.setMode(mode)
    mode = math.max(1, math.min(3, math.floor(tonumber(mode) or 1)))
    TramlinePFCompat.mode = mode
    TramlinePFCompat.enabled = mode ~= TramlinePFCompat.ON
    if mode ~= TramlinePFCompat.ON or TramlinePFCompat.isPresent() then TramlinePFCompat.hook() end
    TramlineLog.info("[TramlinePilot][PF] Precision Farming tramlines: %s", TramlinePFCompat.MODES[mode])
end

-- (old name) true = off everywhere, false = on
function TramlinePFCompat.setEnabled(off)
    TramlinePFCompat.setMode(off and TramlinePFCompat.OFF_ALL or TramlinePFCompat.ON)
end

-- tlPF dump: what PF's tramline parts look like (for the log)
function TramlinePFCompat.dump()
    local env = TramlinePFCompat.env()
    if env == nil then
        TramlineLog.info("[TramlinePilot][PF] FS25_precisionFarming environment not found")
        return "Precision Farming not found"
    end
    local n = 0
    local function list(label, t)
        if type(t) ~= "table" then
            TramlineLog.info("[TramlinePilot][PF] %s: %s", label, type(t))
            return
        end
        local seen, cur = {}, t
        while type(cur) == "table" and not seen[cur] do
            seen[cur] = true
            for k, v in pairs(cur) do
                if type(k) == "string" and (type(v) == "function" or string.find(string.lower(k), "tramline")) then
                    TramlineLog.info("[TramlinePilot][PF] %s.%s (%s)", label, k, type(v))
                    n = n + 1
                end
            end
            local mt = getmetatable(cur)
            cur = mt ~= nil and mt.__index or nil
        end
    end
    local ok, TM = pcall(function() return env.TramlineMap end)
    list("TramlineMap", ok and TM or nil)
    local pf = nil
    pcall(function() pf = env.g_precisionFarming end)
    if type(pf) == "table" then list("g_precisionFarming.tramlineMap", pf.tramlineMap) end
    TramlineLog.info("[TramlinePilot][PF] mode %s, hooked %s, tramlines refused %d times",
        TramlinePFCompat.MODES[TramlinePFCompat.mode], tostring(TramlinePFCompat.hooked), TramlinePFCompat.blockedCalls)
    return string.format("%d entries logged to log.txt", n)
end
