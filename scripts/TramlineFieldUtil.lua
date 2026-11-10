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

-- Field id (or nil) for a world position.
function TramlineFieldUtil.getFieldIdAt(x, z)
    local field = TramlineFieldUtil.getFieldAt(x, z)
    if field == nil then return nil end
    if field.getId ~= nil then return field:getId() end
    return field.fieldId
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
function TramlineFieldUtil.cultivateArea(sx, sz, wx, wz, hx, hz)
    local wrote = false

    -- 1. clear every fruit type's growth state (modifiers created once, reused)
    if TramlineFieldUtil.fruitModifiers == nil then
        TramlineFieldUtil.fruitModifiers = {}
        local fruitTypes = g_fruitTypeManager ~= nil and g_fruitTypeManager:getFruitTypes() or {}
        local seenPlane = {}
        for _, ft in pairs(fruitTypes) do
            if ft.terrainDataPlaneId ~= nil and ft.startStateChannel ~= nil and ft.numStateChannels ~= nil then
                local key = tostring(ft.terrainDataPlaneId) .. ":" .. ft.startStateChannel .. ":" .. ft.numStateChannels
                if not seenPlane[key] then
                    seenPlane[key] = true
                    local ok, m = pcall(DensityMapModifier.new, ft.terrainDataPlaneId, ft.startStateChannel, ft.numStateChannels, g_currentMission.terrainRootNode)
                    if ok and m ~= nil then
                        table.insert(TramlineFieldUtil.fruitModifiers, { m = m, name = ft.name })
                    end
                end
            end
        end
    end
    for _, fm in ipairs(TramlineFieldUtil.fruitModifiers) do
        local ok, err = pcall(function()
            fm.m:setParallelogramWorldCoords(sx, sz, wx, wz, hx, hz, DensityCoordType.POINT_POINT_POINT)
            fm.m:executeSet(0)
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
