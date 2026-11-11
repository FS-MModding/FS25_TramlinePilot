--[[
    TramlineStore.lua
    Holds one tramline set per field, saves/loads them next to the savegame
    (tramlines.xml) and caches the generated geometry.
]]

TramlineStore = {}
local TramlineStore_mt = Class(TramlineStore)

TramlineStore.FILENAME = "tramlines.xml"

function TramlineStore.new()
    local self = setmetatable({}, TramlineStore_mt)
    self.sets = {}          -- [fieldId] = set
    self.geometry = {}      -- [fieldId] = TramlineGenerator.generate(set)
    self.jobs = {}          -- [fieldId] = the worker job in progress (see getJob)
    self.isDirty = false
    return self
end

function TramlineStore:getSet(fieldId)
    return self.sets[fieldId]
end

function TramlineStore.newSet(fieldId)
    return {
        fieldId = fieldId,
        ax = 0, az = 0,
        dx = 0, dz = 1,
        spacing = 24,
        headlands = 1,
        offset = 0,
        turnRadius = 7,
        turnStyle = 1,              -- 1 = along the headland tramline, 2 = loops
        headlandOrder = "first",    -- "first" | "last" | "off"
        refillAD = false,           -- out of spray: refill with AutoDrive
        corners = "round",          -- "round" | "reverse" (back into the headland corners)
        trackWidth = 2.0,           -- m between the wheel centres, for the tracks created in the crop
        headlandDir = "either",     -- "either" (as the vehicle faces) | "cw" | "acw" (one way round, turns all that way)
    }
end

-- store a set (copy of the given values) for its field
function TramlineStore:putSet(fieldId, values)
    local set = self.sets[fieldId] or TramlineStore.newSet(fieldId)
    for k, v in pairs(values) do set[k] = v end
    set.fieldId = fieldId
    self.sets[fieldId] = set
    self.isDirty = true
    return set
end

function TramlineStore:getOrCreateSet(fieldId)
    local set = self.sets[fieldId]
    if set == nil then
        set = {
            fieldId = fieldId,
            ax = 0, az = 0,
            dx = 0, dz = 1,
            spacing = 24,
            headlands = 1,
            offset = 0,
            turnRadius = 7,
            turnStyle = 1,              -- 1 = along the headland tramline, 2 = loops
            headlandOrder = "first",    -- "first" | "last" | "off"
            trackWidth = 2.0,
            headlandDir = "either",
        }
        self.sets[fieldId] = set
    end
    return set
end

-------------------------------------------------------------------------------
-- The worker job in progress on a field: how its route was made (start point,
-- heading, headland order), how far it got (ix = route index; x, z, dirX,
-- dirZ = that spot) and the layout it belongs to (sig). Saved with the
-- savegame, so a job carries on where it stopped even after a reload.
-------------------------------------------------------------------------------

function TramlineStore:getJob(fieldId)
    return self.jobs[fieldId]
end

function TramlineStore:setJob(fieldId, job)
    self.jobs[fieldId] = job
    self.isDirty = true
end

function TramlineStore:clearJob(fieldId)
    if self.jobs[fieldId] ~= nil then
        self.jobs[fieldId] = nil
        self.isDirty = true
    end
end

-- the job's route (rebuilt from how it was made, after a reload); nil and the
-- job is dropped if the field's layout has changed since
function TramlineStore:getJobRoute(fieldId)
    local job = self.jobs[fieldId]
    if job == nil then return nil end
    local geo = self:getGeometry(fieldId)
    if geo == nil then return nil end
    if job.route ~= nil and job.geo == geo then return job.route end
    local ok = job.sig == TramlineRoute.layoutSignature(geo)
    local route = nil
    if ok then
        route = TramlineRoute.tagIndices(TramlineRoute.buildJob(geo, job.sx, job.sz, job.hx, job.hz,
            { headlandOrder = job.order, corners = job.corners == true, boomBack = job.boomBack }))
        ok = route ~= nil and #route == job.n
    end
    if not ok then
        TramlineLog.info("[TramlinePilot] field %d: the layout has changed, the unfinished job is dropped", fieldId)
        self:clearJob(fieldId)
        return nil
    end
    job.route, job.geo = route, geo
    return route
end

function TramlineStore:removeSet(fieldId)
    self.sets[fieldId] = nil
    self.geometry[fieldId] = nil
    self.jobs[fieldId] = nil
    if self.fieldInfo ~= nil then self.fieldInfo[fieldId] = nil end
    self.isDirty = true
end

-- Scan result for a field (transient, not saved). After a savegame load it is
-- rebuilt from the stored reference point (where the menu was opened, always
-- on the field), falling back to the A point.
function TramlineStore:getFieldInfo(fieldId)
    self.fieldInfo = self.fieldInfo or {}
    local info = self.fieldInfo[fieldId]
    if info == nil then
        local set = self.sets[fieldId]
        if set ~= nil and g_currentMission ~= nil and g_currentMission.terrainRootNode ~= nil then
            local tries = {}
            if set.refX ~= nil then tries[#tries + 1] = { set.refX, set.refZ } end
            tries[#tries + 1] = { set.ax, set.az }
            for _, p in ipairs(tries) do
                local ok, res = pcall(FieldScanner.scan, p[1], p[2])
                if ok and res ~= nil and res.fieldId == fieldId then info = res; break end
            end
            self.fieldInfo[fieldId] = info
        end
    end
    return info
end

function TramlineStore:removeAll()
    self.sets, self.geometry, self.fieldInfo, self.jobs = {}, {}, {}, {}
    self.isDirty = true
end

function TramlineStore:regenerate(fieldId)
    local set = self.sets[fieldId]
    if set == nil then return nil end
    local info = self:getFieldInfo(fieldId)
    if info ~= nil then set.scanBounds = info.bounds end
    local geo = TramlineGenerator.generate(set, info)
    self.geometry[fieldId] = geo
    -- an unfinished job only carries on if the tramlines stayed the same
    local job = self.jobs[fieldId]
    if job ~= nil and job.sig ~= TramlineRoute.layoutSignature(geo) then
        TramlineLog.info("[TramlinePilot] field %d: new tramline layout, the unfinished job is dropped", fieldId)
        self.jobs[fieldId] = nil
    end
    self.isDirty = true
    return geo
end

function TramlineStore:getGeometry(fieldId)
    if self.geometry[fieldId] == nil and self.sets[fieldId] ~= nil then
        self:regenerate(fieldId)
    end
    return self.geometry[fieldId]
end

-------------------------------------------------------------------------------
-- Save / load
-------------------------------------------------------------------------------

function TramlineStore:getSavegamePath()
    local mission = g_currentMission
    if mission == nil or mission.missionInfo == nil or mission.missionInfo.savegameDirectory == nil then
        return nil
    end
    return mission.missionInfo.savegameDirectory .. "/" .. TramlineStore.FILENAME
end

function TramlineStore:save()
    local path = self:getSavegamePath()
    if path == nil then return end

    local xmlFile = XMLFile.create("tramlinesXML", path, "tramlines")
    if xmlFile == nil then
        TramlineLog.error("[TramlinePilot] could not create %s", path)
        return
    end

    xmlFile:setInt("tramlines.settings#pfMode", TramlinePFCompat.mode or 1)
    xmlFile:setBool("tramlines.settings#disablePFTramlines", (TramlinePFCompat.mode or 1) == 3)
    xmlFile:setBool("tramlines.settings#pfAsked", self.pfAsked == true)
    xmlFile:setInt("tramlines.settings#showMode", self.showMode or 2)
    xmlFile:setFloat("tramlines.settings#workSpeed", self.workSpeed or 12)
    xmlFile:setFloat("tramlines.settings#turnSpeed", self.turnSpeed or 7)

    local i = 0
    for fieldId, set in pairs(self.sets) do
        local key = string.format("tramlines.set(%d)", i)
        xmlFile:setInt(key .. "#fieldId", fieldId)
        xmlFile:setFloat(key .. "#ax", set.ax)
        xmlFile:setFloat(key .. "#az", set.az)
        xmlFile:setFloat(key .. "#dx", set.dx)
        xmlFile:setFloat(key .. "#dz", set.dz)
        xmlFile:setFloat(key .. "#spacing", set.spacing)
        xmlFile:setInt(key .. "#headlands", set.headlands)
        xmlFile:setFloat(key .. "#offset", set.offset)
        xmlFile:setBool(key .. "#anchorToEdge", set.anchorToEdge == true)
        xmlFile:setFloat(key .. "#turnRadius", set.turnRadius or 7)
        xmlFile:setInt(key .. "#turnStyle", set.turnStyle or 1)
        xmlFile:setString(key .. "#headlandOrder", set.headlandOrder or "first")
        xmlFile:setBool(key .. "#refillAD", set.refillAD == true)
        xmlFile:setString(key .. "#corners", set.corners or "round")
        xmlFile:setFloat(key .. "#trackWidth", set.trackWidth or 2.0)
        xmlFile:setString(key .. "#headlandDir", set.headlandDir or "either")
        if set.refX ~= nil then
            xmlFile:setFloat(key .. "#refX", set.refX)
            xmlFile:setFloat(key .. "#refZ", set.refZ)
        end
        local job = self.jobs[fieldId]
        if job ~= nil and job.n ~= nil then
            local jk = key .. ".job"
            xmlFile:setString(jk .. "#sig", job.sig or "")
            xmlFile:setString(jk .. "#order", job.order or "first")
            xmlFile:setFloat(jk .. "#sx", job.sx)
            xmlFile:setFloat(jk .. "#sz", job.sz)
            xmlFile:setFloat(jk .. "#hx", job.hx)
            xmlFile:setFloat(jk .. "#hz", job.hz)
            xmlFile:setInt(jk .. "#n", job.n)
            xmlFile:setInt(jk .. "#ix", job.ix or 1)
            if job.startIx ~= nil then xmlFile:setInt(jk .. "#startIx", job.startIx) end
            xmlFile:setFloat(jk .. "#x", job.x or job.sx)
            xmlFile:setFloat(jk .. "#z", job.z or job.sz)
            xmlFile:setFloat(jk .. "#dirX", job.dirX or job.hx)
            xmlFile:setFloat(jk .. "#dirZ", job.dirZ or job.hz)
            xmlFile:setBool(jk .. "#corners", job.corners == true)
            if job.boomBack ~= nil then xmlFile:setFloat(jk .. "#boomBack", job.boomBack) end
        end
        i = i + 1
    end

    xmlFile:save()
    xmlFile:delete()
    self.isDirty = false
    TramlineLog.info("[TramlinePilot] saved %d tramline set(s)", i)
end

function TramlineStore:load()
    local path = self:getSavegamePath()
    if path == nil or not fileExists(path) then return end

    local xmlFile = XMLFile.load("tramlinesXML", path)
    if xmlFile == nil then return end

    self.sets = {}
    self.geometry = {}
    self.jobs = {}

    self.disablePFTramlines = xmlFile:getBool("tramlines.settings#disablePFTramlines", false)
    self.pfMode = xmlFile:getInt("tramlines.settings#pfMode", self.disablePFTramlines and 3 or 1)
    self.pfAsked = xmlFile:getBool("tramlines.settings#pfAsked", false)
    self.showMode = xmlFile:getInt("tramlines.settings#showMode", 2)
    self.workSpeed = xmlFile:getFloat("tramlines.settings#workSpeed", 12)
    self.turnSpeed = xmlFile:getFloat("tramlines.settings#turnSpeed", 7)

    xmlFile:iterate("tramlines.set", function(_, key)
        local fieldId = xmlFile:getInt(key .. "#fieldId")
        if fieldId ~= nil then
            self.sets[fieldId] = {
                fieldId   = fieldId,
                ax        = xmlFile:getFloat(key .. "#ax", 0),
                az        = xmlFile:getFloat(key .. "#az", 0),
                dx        = xmlFile:getFloat(key .. "#dx", 0),
                dz        = xmlFile:getFloat(key .. "#dz", 1),
                spacing   = xmlFile:getFloat(key .. "#spacing", 24),
                headlands = xmlFile:getInt(key .. "#headlands", 1),
                offset    = xmlFile:getFloat(key .. "#offset", 0),
                anchorToEdge = xmlFile:getBool(key .. "#anchorToEdge", false),
                turnRadius = xmlFile:getFloat(key .. "#turnRadius", 7),
                turnStyle = xmlFile:getInt(key .. "#turnStyle", 1),
                headlandOrder = xmlFile:getString(key .. "#headlandOrder", "first"),
                refillAD = xmlFile:getBool(key .. "#refillAD", false),
                corners = xmlFile:getString(key .. "#corners", "round"),
                trackWidth = xmlFile:getFloat(key .. "#trackWidth", 2.0),
                headlandDir = xmlFile:getString(key .. "#headlandDir", "either"),
                refX = xmlFile:getFloat(key .. "#refX"),
                refZ = xmlFile:getFloat(key .. "#refZ"),
            }
            local jk = key .. ".job"
            local n = xmlFile:getInt(jk .. "#n")
            if n ~= nil then
                self.jobs[fieldId] = {
                    sig   = xmlFile:getString(jk .. "#sig", ""),
                    order = xmlFile:getString(jk .. "#order", "first"),
                    sx = xmlFile:getFloat(jk .. "#sx", 0), sz = xmlFile:getFloat(jk .. "#sz", 0),
                    hx = xmlFile:getFloat(jk .. "#hx", 1), hz = xmlFile:getFloat(jk .. "#hz", 0),
                    n = n, ix = xmlFile:getInt(jk .. "#ix", 1), startIx = xmlFile:getInt(jk .. "#startIx"),
                    x = xmlFile:getFloat(jk .. "#x"), z = xmlFile:getFloat(jk .. "#z"),
                    dirX = xmlFile:getFloat(jk .. "#dirX"), dirZ = xmlFile:getFloat(jk .. "#dirZ"),
                    corners = xmlFile:getBool(jk .. "#corners", false),
                    boomBack = xmlFile:getFloat(jk .. "#boomBack"),
                }
            end
        end
    end)
    xmlFile:delete()

    local n = 0
    for _ in pairs(self.sets) do n = n + 1 end
    TramlineLog.info("[TramlinePilot] loaded %d tramline set(s)", n)
end
