--[[
    TramlineLibrary.lua
    Saved wheelings: every tramline layout you generate is also written to

      <user folder>/modSettings/FS25_TramlinePilot/<map id>/<name>.xml

    (Documents\My Games\FarmingSimulator2025\modSettings\...), outside the
    savegame, so the same wheelings can be loaded again for the next crop,
    in another savegame on the same map, or after clearing a field.

    A wheeling holds the settings the layout is made from (direction, anchor,
    spacing, headland passes, turns, ...); the field outline comes from the
    map, so loading it gives exactly the same tramlines again.
]]

TramlineLibrary = {}

TramlineLibrary.MOD_FOLDER = "FS25_TramlinePilot"

-- set fields kept in a wheeling, by type
TramlineLibrary.FLOATS  = { "ax", "az", "dx", "dz", "offset", "spacing", "turnRadius", "refX", "refZ", "trackWidth" }
TramlineLibrary.INTS    = { "headlands", "turnStyle" }
TramlineLibrary.STRINGS = { "headlandOrder", "corners" }
TramlineLibrary.BOOLS   = { "anchorToEdge", "refillAD" }

local function safe(s)
    return (tostring(s or ""):gsub("[^%w%.%-_ ]", "_"))
end

-- the map's id (folder name)
function TramlineLibrary.mapId()
    local id = nil
    pcall(function()
        local mi = g_currentMission.missionInfo
        id = mi.mapId or (mi.map ~= nil and mi.map.id) or nil
    end)
    return safe(id or "unknownMap")
end

-- the folder for this map (created when needed), or nil
function TramlineLibrary.folder()
    if getUserProfileAppPath == nil then return nil end
    local base = getUserProfileAppPath() .. "modSettings/"
    local root = base .. TramlineLibrary.MOD_FOLDER .. "/"
    local dir = root .. TramlineLibrary.mapId() .. "/"
    if createFolder ~= nil then
        pcall(createFolder, base)
        pcall(createFolder, root)
        pcall(createFolder, dir)
    end
    return dir
end

-- direction as a compass angle (0 = north, clockwise)
local function angleOf(dx, dz)
    local a = math.deg(TramlineGeom.atan2(dx or 0, -(dz or 1)))
    return math.floor((a + 360) % 360 + 0.5) % 360
end

function TramlineLibrary.defaultName(set)
    local name = string.format("Field %d - %g m - %d deg", set.fieldId or 0, set.spacing or 0, angleOf(set.dx, set.dz))
    if (set.headlands or 1) ~= 1 then
        name = name .. string.format(" - %d headland passes", set.headlands or 0)
    end
    if set.corners == "reverse" then name = name .. " - corners" end
    return name
end

-- Save a set as a wheeling. Returns the name, path or nil, reason
function TramlineLibrary.save(set, name)
    local dir = TramlineLibrary.folder()
    if dir == nil then return nil, "no user folder" end
    name = name or TramlineLibrary.defaultName(set)
    local path = dir .. safe(name) .. ".xml"
    local xml = XMLFile.create("tramlineWheeling", path, "wheeling")
    if xml == nil then return nil, "could not write " .. path end
    xml:setString("wheeling#name", name)
    xml:setString("wheeling#map", TramlineLibrary.mapId())
    xml:setInt("wheeling#fieldId", set.fieldId or 0)
    local date = ""
    if getDate ~= nil then
        local ok, d = pcall(getDate, "%Y-%m-%d %H:%M")
        if ok and d ~= nil then date = d end
    end
    xml:setString("wheeling#saved", date)
    for _, k in ipairs(TramlineLibrary.FLOATS) do
        if set[k] ~= nil then xml:setFloat("wheeling.set#" .. k, set[k]) end
    end
    for _, k in ipairs(TramlineLibrary.INTS) do
        if set[k] ~= nil then xml:setInt("wheeling.set#" .. k, set[k]) end
    end
    for _, k in ipairs(TramlineLibrary.STRINGS) do
        if set[k] ~= nil then xml:setString("wheeling.set#" .. k, tostring(set[k])) end
    end
    for _, k in ipairs(TramlineLibrary.BOOLS) do
        xml:setBool("wheeling.set#" .. k, set[k] == true)
    end
    xml:save()
    xml:delete()
    TramlineLog.info("[TramlinePilot] wheeling saved: '%s' (%s)", name, path)
    return name, path
end

-- read one wheeling file: { name, fieldId, saved, path, values = { set fields } } or nil
function TramlineLibrary.read(path)
    if fileExists ~= nil and not fileExists(path) then return nil end
    local xml = XMLFile.load("tramlineWheeling", path)
    if xml == nil then return nil end
    local w = { path = path, values = {} }
    w.name = xml:getString("wheeling#name", path)
    w.fieldId = xml:getInt("wheeling#fieldId", 0)
    w.saved = xml:getString("wheeling#saved", "")
    local v = w.values
    for _, k in ipairs(TramlineLibrary.FLOATS) do v[k] = xml:getFloat("wheeling.set#" .. k) end
    for _, k in ipairs(TramlineLibrary.INTS) do v[k] = xml:getInt("wheeling.set#" .. k) end
    for _, k in ipairs(TramlineLibrary.STRINGS) do v[k] = xml:getString("wheeling.set#" .. k) end
    for _, k in ipairs(TramlineLibrary.BOOLS) do v[k] = xml:getBool("wheeling.set#" .. k, false) end
    xml:delete()
    if v.spacing == nil or v.dx == nil then return nil end
    return w
end

-- the saved wheelings of this map (of one field if fieldId is given), sorted by name
function TramlineLibrary.list(fieldId)
    local dir = TramlineLibrary.folder()
    local out = {}
    if dir == nil or getFiles == nil then return out end
    local files = {}
    local collector = {}
    function collector:onFile(name, isDirectory)
        if not isDirectory and string.lower(string.sub(name, -4)) == ".xml" then files[#files + 1] = name end
    end
    pcall(getFiles, dir, "onFile", collector)
    for _, f in ipairs(files) do
        local ok, w = pcall(TramlineLibrary.read, dir .. f)
        if ok and w ~= nil and (fieldId == nil or w.fieldId == fieldId) then out[#out + 1] = w end
    end
    table.sort(out, function(a, b) return a.name < b.name end)
    return out
end

-- one line describing a wheeling (for lists)
function TramlineLibrary.describe(w)
    local v = w.values
    local s = string.format("%s  (%g m, %d headland pass%s", w.name, v.spacing or 0, v.headlands or 0,
        (v.headlands or 0) == 1 and "" or "es")
    if w.saved ~= nil and w.saved ~= "" then s = s .. ", " .. w.saved end
    return s .. ")"
end

-- copy a wheeling's settings into a set
function TramlineLibrary.apply(w, set)
    for k, v in pairs(w.values) do set[k] = v end
    return set
end
