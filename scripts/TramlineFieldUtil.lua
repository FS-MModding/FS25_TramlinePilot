--[[
    TramlineFieldUtil.lua
    FS25 field lookups.
      - field id for a position comes from the farmland (one field per farmland)
      - "is this field ground" is answered by the GROUND_TYPE density map
        (value > 0 means worked field ground), which stops exactly at the
        field edge instead of the farmland edge.
]]

TramlineFieldUtil = {}

local groundModifier, groundFilter

local function initGround()
    if groundModifier ~= nil then return true end
    if g_currentMission == nil or g_currentMission.fieldGroundSystem == nil then return false end
    local ok, mapId, first, num = pcall(function()
        return g_currentMission.fieldGroundSystem:getDensityMapData(FieldDensityMap.GROUND_TYPE)
    end)
    if not ok or mapId == nil then return false end
    groundModifier = DensityMapModifier.new(mapId, first, num, g_currentMission.terrainRootNode)
    groundFilter = DensityMapFilter.new(groundModifier)
    groundFilter:setValueCompareParams(DensityValueCompareType.GREATER, 0)
    return true
end

-- density map id of the ground type map (for its size), or nil
function TramlineFieldUtil.getGroundMapId()
    if g_currentMission == nil or g_currentMission.fieldGroundSystem == nil then return nil end
    local ok, mapId = pcall(function()
        return g_currentMission.fieldGroundSystem:getDensityMapData(FieldDensityMap.GROUND_TYPE)
    end)
    if ok then return mapId end
    return nil
end

-- Field object (or nil) for a world position, via its farmland.
function TramlineFieldUtil.getFieldAt(x, z)
    local farmland = g_farmlandManager:getFarmlandAtWorldPosition(x, z)
    if farmland ~= nil and farmland.getField ~= nil then
        return farmland:getField()
    end
    return nil
end

-- Fields joined into another field's layout: [field id] = id of the field
-- the layout belongs to (shared with TramlineStore.joins, saved with it).
TramlineFieldUtil.joins = TramlineFieldUtil.joins or {}

-- Field id (or nil) of the farmland at a world position, as the map has it.
function TramlineFieldUtil.getRawFieldIdAt(x, z)
    local field = TramlineFieldUtil.getFieldAt(x, z)
    if field == nil then return nil end
    if field.getId ~= nil then return field:getId() end
    return field.fieldId
end

-- Field id (or nil) for a world position. A field joined into another
-- field's layout (fields joined by ploughing between them) counts as that one
-- while its land is owned (once sold it is a field of its own again).
function TramlineFieldUtil.getFieldIdAt(x, z)
    local id = TramlineFieldUtil.getRawFieldIdAt(x, z)
    if id == nil then return nil end
    local main = TramlineFieldUtil.joins[id]
    if main ~= nil and TramlineFieldUtil.getOwnerAt(x, z) ~= nil then return main end
    return id
end

-- Farm that owns the farmland at a world position (nil: nobody, or unknown).
function TramlineFieldUtil.getOwnerAt(x, z)
    local ok, owner = pcall(function()
        local farmland = g_farmlandManager:getFarmlandAtWorldPosition(x, z)
        if farmland == nil then return nil end
        return g_farmlandManager:getFarmlandOwner(farmland.id)
    end)
    if not ok or owner == nil or owner <= 0 then return nil end
    return owner
end

-- True if the ground at x,z is actual field ground (not grass, track or trees).
function TramlineFieldUtil.isFieldGround(x, z)
    if initGround() then
        local s = 1.0
        groundModifier:setParallelogramWorldCoords(x - s * 0.5, z - s * 0.5, s, 0, 0, s, DensityCoordType.POINT_VECTOR_VECTOR)
        local _, area = groundModifier:executeGet(groundFilter)
        return (area or 0) > 0
    end
    -- fallback
    local y = getTerrainHeightAtWorldPos(g_currentMission.terrainRootNode, x, 1, z)
    local isOnField = FSDensityMapUtil.getFieldDataAtWorldPosition(x, y, z)
    return isOnField == true
end

-- True if x,z is field ground belonging to fieldId.
function TramlineFieldUtil.isOnField(fieldId, x, z)
    if not TramlineFieldUtil.isFieldGround(x, z) then return false end
    return TramlineFieldUtil.getFieldIdAt(x, z) == fieldId
end

-- Exact single-pixel test (what Courseplay's edge tracer uses), so the edge
-- found is not widened by the 1 m sample square of isFieldGround.
function TramlineFieldUtil.isFieldPoint(x, z)
    local ok, isOnField = pcall(function()
        local y = getTerrainHeightAtWorldPos(g_currentMission.terrainRootNode, x, 1, z)
        return FSDensityMapUtil.getFieldDataAtWorldPosition(x, y, z)
    end)
    if ok and isOnField ~= nil then return isOnField == true end
    return TramlineFieldUtil.isFieldGround(x, z)
end

function TramlineFieldUtil.isOnFieldPoint(fieldId, x, z)
    if not TramlineFieldUtil.isFieldPoint(x, z) then return false end
    return TramlineFieldUtil.getFieldIdAt(x, z) == fieldId
end

-- True if a hedge, wall, fence, tree or building stands at x,z: a 2 m box from
-- 0.2 m above the ground (collision flags as Courseplay's obstacle checks).
-- Vehicles and the terrain don't count. If it can't be checked: true, so a
-- strip is never crossed blind.
TramlineFieldUtil.obstacleHits = 0
function TramlineFieldUtil.hasObstacleAt(x, z)
    if overlapBox == nil or CollisionFlag == nil or g_currentMission == nil then return true end
    local ok = pcall(function()
        local y = getTerrainHeightAtWorldPos(g_currentMission.terrainRootNode, x, 0, z)
        local mask = CollisionFlag.DEFAULT + CollisionFlag.TREE + CollisionFlag.BUILDING + CollisionFlag.STATIC_OBJECT
        TramlineFieldUtil.obstacleHits = 0
        overlapBox(x, y + 1.2, z, 0, 0, 0, 1.0, 1.0, 1.0, "onObstacleOverlap", TramlineFieldUtil, mask, true, true, true, true)
    end)
    if not ok then return true end
    return TramlineFieldUtil.obstacleHits > 0
end

function TramlineFieldUtil:onObstacleOverlap(transformId)
    if transformId == nil or transformId == 0 then return end
    if getHasClassId ~= nil and ClassIds ~= nil and ClassIds.TERRAIN_TRANSFORM_GROUP ~= nil
        and getHasClassId(transformId, ClassIds.TERRAIN_TRANSFORM_GROUP) then
        return
    end
    local obj = g_currentMission ~= nil and g_currentMission.nodeToObject ~= nil and g_currentMission.nodeToObject[transformId] or nil
    if obj ~= nil and obj.getRootVehicle ~= nil then return end     -- a vehicle
    TramlineFieldUtil.obstacleHits = TramlineFieldUtil.obstacleHits + 1
end

-- The map's own outline of the field (as drawn by the map maker), or nil.
-- Returns a list of {x, z}.
function TramlineFieldUtil.getFieldPolygon(field)
    if field == nil or field.getDensityMapPolygon == nil then return nil end
    local ok, flat = pcall(function() return field:getDensityMapPolygon():getVerticesList() end)
    if not ok or type(flat) ~= "table" or #flat < 6 then return nil end
    local poly = {}
    for i = 1, #flat - 1, 2 do
        poly[#poly + 1] = { flat[i], flat[i + 1] }
    end
    return poly
end

-- number of ground map pixels in the parallelogram (sx,sz)->(wx,wz)->(hx,hz), or nil
function TramlineFieldUtil.countPixels(sx, sz, wx, wz, hx, hz)
    if not initGround() then return nil end
    local ok, _, _, total = pcall(function()
        groundModifier:setParallelogramWorldCoords(sx, sz, wx, wz, hx, hz, DensityCoordType.POINT_POINT_POINT)
        return groundModifier:executeGet()
    end)
    if ok and type(total) == "number" then return total end
    return nil
end

-------------------------------------------------------------------------------
-- Ground writing (used by the painter)
-------------------------------------------------------------------------------

-- Remove any crop and set the ground to cultivated inside the parallelogram
-- (sx,sz)->(wx,wz)->(hx,hz). Returns true if at least one layer was written.
--
-- The crops share one density map: each cell holds a crop type and a growth
-- state, and a write through a crop's plane makes the cell that crop. So each
-- crop is cleared only where that crop grows (filter: its state > 0), the
-- way the game's own cultivating does it. Writing 0 through every crop's
-- plane unfiltered left every cleared cell as the LAST crop in the list
-- (onion on a multifruit map: issue #1).
function TramlineFieldUtil.cultivateArea(sx, sz, wx, wz, hx, hz)
    local wrote = false

    -- 1. clear each crop where it grows (modifiers and filters created once, reused)
    if TramlineFieldUtil.fruitModifiers == nil then
        TramlineFieldUtil.fruitModifiers = {}
        local fruitTypes = g_fruitTypeManager ~= nil and g_fruitTypeManager:getFruitTypes() or {}
        local seenPlane = {}
        for _, ft in pairs(fruitTypes) do
            if ft.terrainDataPlaneId ~= nil and ft.startStateChannel ~= nil and ft.numStateChannels ~= nil then
                local key = tostring(ft.terrainDataPlaneId) .. ":" .. ft.startStateChannel .. ":" .. ft.numStateChannels
                if not seenPlane[key] then
                    seenPlane[key] = true
                    local ok, m, f = pcall(function()
                        local mod = DensityMapModifier.new(ft.terrainDataPlaneId, ft.startStateChannel, ft.numStateChannels, g_currentMission.terrainRootNode)
                        local fil = DensityMapFilter.new(mod)
                        fil:setValueCompareParams(DensityValueCompareType.GREATER, 0)
                        return mod, fil
                    end)
                    if ok and m ~= nil and f ~= nil then
                        table.insert(TramlineFieldUtil.fruitModifiers, { m = m, f = f, name = ft.name })
                    end
                end
            end
        end
    end
    for _, fm in ipairs(TramlineFieldUtil.fruitModifiers) do
        local ok, err = pcall(function()
            fm.m:setParallelogramWorldCoords(sx, sz, wx, wz, hx, hz, DensityCoordType.POINT_POINT_POINT)
            fm.m:executeSet(0, fm.f)
        end)
        if ok then wrote = true elseif not fm.warned then
            fm.warned = true
            TramlineLog.warning("[TramlinePilot] fruit clear failed (%s): %s", tostring(fm.name), tostring(err))
        end
    end

    -- 2. ground type -> cultivated (only where it is already field ground)
    if initGround() then
        local ok, err = pcall(function()
            local value = g_currentMission.fieldGroundSystem:getFieldGroundValue(FieldGroundType.CULTIVATED)
            groundModifier:setParallelogramWorldCoords(sx, sz, wx, wz, hx, hz, DensityCoordType.POINT_POINT_POINT)
            groundModifier:executeSet(value, groundFilter)
        end)
        if ok then wrote = true else TramlineLog.warning("[TramlinePilot] ground type set failed: %s", tostring(err)) end
    end

    return wrote
end
