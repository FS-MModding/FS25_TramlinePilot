--[[
    TramlineLog.lua
    Tramline Pilot's own log, for bug reports. Everything the mod writes to
    the game's log.txt goes here too, plus what each job was done with and
    any script error together with what the worker was doing, in one small
    text file next to the saved wheelings:
      Documents/My Games/FarmingSimulator2025/modSettings/FS25_TramlinePilot/TramlinePilot_log.txt
    A new file each game session; the one before is kept as
    TramlinePilot_log_previous.txt. Windows user names are left out of any
    path written. 'tlReport' in the console adds a snapshot of the field,
    the job and the vehicle.

    The game lets mods open files for writing only ("w": no appending, no
    reading), so the session's lines are kept here and the whole file is
    written again when something new came in: straight away for warnings
    and errors, otherwise at most every 2 seconds, and at the end.
]]

TramlineLog = {}

TramlineLog.FILE      = "TramlinePilot_log.txt"
TramlineLog.PREVIOUS  = "TramlinePilot_log_previous.txt"
TramlineLog.MAX_LINES = 5000          -- per session; more is not written
TramlineLog.FLUSH_MS  = 2000          -- new INFO lines reach the file within this
TramlineLog.lines     = 0
TramlineLog.buffer    = {}
TramlineLog.dirty     = false
TramlineLog.flushIn   = 0
TramlineLog.errorsSeen = {}           -- message -> times seen (each one logged once in full)

local function folder()
    if getUserProfileAppPath == nil then return nil end
    local sub = (TramlineLibrary ~= nil and TramlineLibrary.MOD_FOLDER) or "FS25_TramlinePilot"
    return getUserProfileAppPath() .. "modSettings/" .. sub .. "/"
end

-- no Windows user name in what is written: C:/Users/<name>/... -> C:/Users/<user>/...
function TramlineLog.scrub(s)
    s = tostring(s)
    s = s:gsub("([A-Za-z]:[/\\][Uu][Ss][Ee][Rr][Ss][/\\])[^/\\]+", "%1<user>")
    return s
end

local function stamp(fmt)
    if getDate ~= nil then
        local ok, d = pcall(getDate, fmt)
        if ok and d ~= nil then return d end
    end
    return ""
end

-- the whole session's lines to the file
function TramlineLog.flush()
    TramlineLog.dirty = false
    local path = TramlineLog.path
    if path == nil or TramlineLog.broken or io == nil or io.open == nil then return end
    local ok = pcall(function()
        local f = io.open(path, "w")
        if f == nil then error("cannot open") end
        f:write(table.concat(TramlineLog.buffer, "\n"), "\n")
        f:close()
    end)
    if not ok then TramlineLog.broken = true end
end

local function append(text, now)
    TramlineLog.buffer[#TramlineLog.buffer + 1] = text
    TramlineLog.dirty = true
    if now then TramlineLog.flush() end
end

-- every frame: new lines reach the file within FLUSH_MS
function TramlineLog.update(dt)
    if not TramlineLog.dirty then return end
    TramlineLog.flushIn = TramlineLog.flushIn - (dt or 0)
    if TramlineLog.flushIn <= 0 then
        TramlineLog.flushIn = TramlineLog.FLUSH_MS
        TramlineLog.flush()
    end
end

-- one line in the mod's file (and nowhere else)
function TramlineLog.write(level, msg)
    if TramlineLog.path == nil then return end
    if TramlineLog.lines >= TramlineLog.MAX_LINES then
        if not TramlineLog.full then
            TramlineLog.full = true
            append(stamp("%H:%M:%S") .. " ----- log full, nothing more this session", true)
        end
        return
    end
    TramlineLog.lines = TramlineLog.lines + 1
    local text = TramlineLog.scrub(msg):gsub("\n", "\n" .. string.rep(" ", 15))    -- message lines kept together
    append(string.format("%s %-5s %s", stamp("%H:%M:%S"), level, text), level ~= "INFO")
end

local function fmt(f, ...)
    if select("#", ...) == 0 then return tostring(f) end
    local ok, s = pcall(string.format, f, ...)
    if ok then return s end
    return tostring(f)
end

-- the mod's messages: to the game's log.txt as before, and to the mod's file
function TramlineLog.info(f, ...)
    local s = fmt(f, ...)
    Logging.info("%s", s)
    TramlineLog.write("INFO", s)
end
function TramlineLog.warning(f, ...)
    local s = fmt(f, ...)
    Logging.warning("%s", s)
    TramlineLog.write("WARN", s)
end
function TramlineLog.error(f, ...)
    local s = fmt(f, ...)
    Logging.error("%s", s)
    TramlineLog.write("ERROR", s)
end

local function modVersion(name)
    if g_modManager == nil or g_modManager.getModByName == nil or name == nil then return nil end
    local ok, mod = pcall(g_modManager.getModByName, g_modManager, name)
    if ok and mod ~= nil then return mod.version end
    return nil
end
TramlineLog.modVersion = modVersion

-- new file for this session (the last one kept), with what it runs on
function TramlineLog.start(modName)
    if TramlineLog.path ~= nil and TramlineLog.dirty then TramlineLog.flush() end   -- a session not ended
    local dir = folder()
    if dir == nil or io == nil or io.open == nil then return end
    if createFolder ~= nil then pcall(createFolder, getUserProfileAppPath() .. "modSettings/") end
    if createFolder ~= nil then pcall(createFolder, dir) end
    local path = dir .. TramlineLog.FILE
    -- the last session's file becomes the previous one (the game's own copy:
    -- mods can't read files back)
    if copyFile ~= nil then pcall(copyFile, path, dir .. TramlineLog.PREVIOUS, true) end
    TramlineLog.path = path
    TramlineLog.broken = false
    TramlineLog.buffer = {}
    TramlineLog.lines = 0
    TramlineLog.full = false
    TramlineLog.errorsSeen = {}
    TramlineLog.modName = modName
    local mi = g_currentMission ~= nil and g_currentMission.missionInfo or nil
    append(string.format("Tramline Pilot %s - log started %s", tostring(modVersion(modName) or "?"), stamp("%Y-%m-%d %H:%M:%S")))
    append(string.format("Game %s, language %s, map '%s' (%s)", tostring(g_gameVersionDisplay or g_gameVersion or "?"),
        tostring(g_languageShort or "?"), tostring(mi and mi.mapTitle or "?"), tostring(mi and mi.mapId or "?")))
    append("Send this file with a bug report: it holds Tramline Pilot's messages only, without your Windows user name.")
    append(string.rep("-", 70), true)
    if TramlineLog.broken then TramlineLog.path = nil end
end

-- the other mods it works with (after they have loaded)
function TramlineLog.mods()
    local parts = {}
    local ad = TramlineAD ~= nil and TramlineAD.get ~= nil and TramlineAD.get() or nil
    parts[#parts + 1] = "AutoDrive " .. (ad ~= nil and tostring(modVersion("FS25_AutoDrive") or "yes") or "no")
    -- other mods' globals live in their own environments: ask the game which are loaded
    local cp = g_modIsLoaded ~= nil and g_modIsLoaded["FS25_Courseplay"]
    parts[#parts + 1] = "Courseplay " .. (cp and tostring(modVersion("FS25_Courseplay") or "yes") or "no")
    local pf = TramlinePFCompat ~= nil and TramlinePFCompat.isPresent ~= nil and TramlinePFCompat.isPresent()
    parts[#parts + 1] = "Precision Farming " .. (pf and "yes" or "no")
    return table.concat(parts, ", ")
end

function TramlineLog.finish()
    if TramlineLog.path ~= nil then
        append(string.format("%s ----- session ended", stamp("%H:%M:%S")), true)
    end
end

local function traceback()
    if debug ~= nil and debug.traceback ~= nil then
        local ok, tb = pcall(debug.traceback, "", 3)
        if ok and tb ~= nil then return tb end
    end
    return ""
end

-- a script error: in full the first time, then counted; also to log.txt
function TramlineLog.scriptError(where, err, context)
    local msg = tostring(err)
    local n = (TramlineLog.errorsSeen[msg] or 0) + 1
    TramlineLog.errorsSeen[msg] = n
    if n == 1 then
        Logging.error("[TramlinePilot] script error in %s: %s", tostring(where), msg)
        TramlineLog.write("ERROR", string.format("script error in %s: %s", tostring(where), msg))
        if context ~= nil and context ~= "" then TramlineLog.write("ERROR", "  while: " .. context) end
        local tb = TramlineLog.lastTrace
        if tb ~= nil and tb ~= "" then
            for line in tostring(tb):gmatch("[^\n]+") do TramlineLog.write("ERROR", "  " .. line) end
        end
    elseif n == 10 or n == 100 or n == 1000 then
        TramlineLog.write("ERROR", string.format("the same error in %s %d times now", tostring(where), n))
    end
end

local function onError(e)
    TramlineLog.lastTrace = traceback()
    return e
end

-- fn(a, b, c, d) catching script errors (for every-frame calls); true if it ran
function TramlineLog.guard(where, fn, a, b, c, d)
    local ok, err = xpcall(function() fn(a, b, c, d) end, onError)
    if not ok then TramlineLog.scriptError(where, err) end
    return ok
end

local unpackArgs = unpack or table.unpack
local function pack(...) return { n = select("#", ...), ... } end

-- run fn(...) catching script errors; returns true, results... or false
function TramlineLog.call(where, fn, ...)
    local args = pack(...)
    local res = pack(xpcall(function() return fn(unpackArgs(args, 1, args.n)) end, onError))
    if not res[1] then
        TramlineLog.scriptError(where, res[2])
        return false
    end
    return unpackArgs(res, 1, res.n)
end
