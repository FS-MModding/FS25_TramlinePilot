--[[
    FieldScanner.lua
    Finds the field under a position and returns an accurate outline.

    Sources, best first:
      1. "map"    - the map's own field polygon (field:getDensityMapPolygon()),
                    used only if it agrees with the ground (sampled along every edge)
      2. "traced" - our own edge tracer: walks round the field edge with a
                    rotating probe (the idea Courseplay's field scanner uses),
                    adaptive step 0.5-4 m, edge located to < 1 cm per probe;
                    traced again across narrow non-field strips (an old hedge
                    line, a path) that the map's outline has in the field and
                    nothing stands in
      3. "grid"   - flood fill on a 2 m grid + contour (old method, last resort)

    scan(x, z) returns nil if not on a field, otherwise:
    {
        fieldId, source,
        polygon   = { {x,z}, ... }  CCW (inside on the left of each edge)
        areaHa, centerX, centerZ, bounds = {x0,z0,x1,z1}, refX, refZ,
        edgeDx, edgeDz             unit direction of the longest straight edge
        edgeLength, edgeWidth      extent along / across that direction (m)
    }
]]

FieldScanner = {}

FieldScanner.TRACE_LMAX     = 4.0     -- probe length on straight edges (m)
FieldScanner.TRACE_LMIN     = 0.5     -- probe length in corners (m)
FieldScanner.TRACE_TURN_MAX = math.rad(20)
FieldScanner.VALIDATE_MIN   = 0.8     -- share of map-polygon samples that must agree
FieldScanner.GRID_STEP      = 2
FieldScanner.GRID_MAX_CELLS = 300000
-- narrow strips of non-field ground across a field (where a hedge was taken
-- out, a path): crossed when the map's own outline has them in the field and
-- nothing stands in them
FieldScanner.GAP_HALF       = 4.0     -- strips up to about 8 m wide
FieldScanner.GAP_MIN_HITS   = 3       -- field ground seen across a strip this often
FieldScanner.GAP_CELL       = 2.0     -- obstacle checks cached per cell (m)

local G = TramlineGeom
local atan2 = G.atan2
local sqrt, abs, cos, sin, floor = math.sqrt, math.abs, math.cos, math.sin, math.floor

-- Injectable for testing; in game these use TramlineFieldUtil
FieldScanner.api = {
    fieldIdAt = function(x, z)
        if not TramlineFieldUtil.isFieldPoint(x, z) then return nil end
        return TramlineFieldUtil.getFieldIdAt(x, z)
    end,
    onField = function(fieldId, x, z) return TramlineFieldUtil.isOnFieldPoint(fieldId, x, z) end,
    mapPolygon = function(x, z) return TramlineFieldUtil.getFieldPolygon(TramlineFieldUtil.getFieldAt(x, z)) end,
    -- a hedge, wall, fence, tree or building standing at x,z?
    obstacleAt = function(x, z) return TramlineFieldUtil.hasObstacleAt(x, z) end,
}

-------------------------------------------------------------------------------

function FieldScanner.scan(x, z)
    local api = FieldScanner.api
    local fieldId = api.fieldIdAt(x, z)
    if fieldId == nil then return nil end
    local onField = function(px, pz) return api.onField(fieldId, px, pz) end

    local poly, source = nil, nil
    local mapPoly = nil     -- the map's own outline: says what is one field even where its edges are off

    -- 1. map polygon
    local ok, mp = pcall(api.mapPolygon, x, z)
    if ok and mp ~= nil and #mp >= 3 then
        mp = G.makeCCW(G.dedupe(mp, 0.05, true))
        if G.pointInPolygon(mp, x, z) then
            mapPoly = mp
            local ratio = FieldScanner.validatePolygon(mp, onField)
            TramlineLog.info("[TramlinePilot] field %d: map outline %d points, %.0f%% agrees with the ground", fieldId, #mp, ratio * 100)
            if ratio >= FieldScanner.VALIDATE_MIN then
                poly, source = mp, "map"
            end
        end
    end

    -- 2. edge tracer
    if poly == nil then
        local ok2, tp = pcall(FieldScanner.traceEdge, x, z, onField)
        if ok2 and tp ~= nil then
            poly, source = tp, "traced"
        elseif not ok2 then
            TramlineLog.warning("[TramlinePilot] edge tracer failed: %s", tostring(tp))
        end
        if poly ~= nil and mapPoly ~= nil then
            local okB, bp = pcall(FieldScanner.acrossStrips, x, z, poly, mapPoly, onField, fieldId)
            if okB and bp ~= nil then
                poly = bp
            elseif not okB then
                TramlineLog.warning("[TramlinePilot] field %d: looking across narrow strips failed: %s", fieldId, tostring(bp))
            end
        end
    end

    -- 3. grid
    if poly == nil then
        poly = FieldScanner.gridPolygon(x, z, onField)
        source = "grid"
    end
    if poly == nil or #poly < 3 then return nil end

    local info = FieldScanner.describe(poly)
    info.fieldId, info.source, info.refX, info.refZ = fieldId, source, x, z
    TramlineLog.info("[TramlinePilot] field %d scanned (%s outline, %d points): %.2f ha, longest edge %.0f m",
        fieldId, source, #poly, info.areaHa, info.edgeLength)
    return info
end

-- area, centre, bounds and longest edge of a CCW polygon
function FieldScanner.describe(poly)
    local info = { polygon = poly }
    info.boundary = poly
    info.areaHa = G.signedArea(poly) / 10000
    local cx, cz = G.centroid(poly)
    if not G.pointInPolygon(poly, cx, cz) then
        -- concave field: use the middle of the widest horizontal chord instead
        local x0, z0, x1, z1 = G.bounds(poly)
        local zm = (z0 + z1) * 0.5
        local iv = G.clipLine(poly, x0 - 1, zm, 1, 0)
        local best, bl = nil, -1
        for _, v in ipairs(iv) do if v[2] - v[1] > bl then best, bl = v, v[2] - v[1] end end
        if best ~= nil then cx, cz = x0 - 1 + (best[1] + best[2]) * 0.5, zm end
    end
    info.centerX, info.centerZ = cx, cz
    local x0, z0, x1, z1 = G.bounds(poly)
    info.bounds = { x0, z0, x1, z1 }

    -- longest straight edge (on a simplified outline so traced edges count as one)
    local simple = G.simplify(poly, 1.5, true)
    local n = #simple
    local bestL, bdx, bdz = -1, 1, 0
    for i = 1, n do
        local a, b = simple[i], simple[(i % n) + 1]
        local dx, dz, l = G.normalize(b[1] - a[1], b[2] - a[2])
        if l > bestL then bestL, bdx, bdz = l, dx, dz end
    end
    info.edgeDx, info.edgeDz = bdx, bdz
    local mnA, mxA, mnC, mxC = math.huge, -math.huge, math.huge, -math.huge
    for _, p in ipairs(poly) do
        local a = p[1] * bdx + p[2] * bdz
        local c = -p[1] * bdz + p[2] * bdx
        if a < mnA then mnA = a end
        if a > mxA then mxA = a end
        if c < mnC then mnC = c end
        if c > mxC then mxC = c end
    end
    info.edgeLength, info.edgeWidth = mxA - mnA, mxC - mnC
    return info
end

-- share of samples along the polygon edges where "just inside" is field and
-- "just outside" is not
function FieldScanner.validatePolygon(poly, onField)
    local n = #poly
    local good, total = 0, 0
    for i = 1, n do
        local a, b = poly[i], poly[(i % n) + 1]
        local dx, dz, l = G.normalize(b[1] - a[1], b[2] - a[2])
        if l > 0.5 then
            local nx, nz = -dz, dx                     -- inward (CCW)
            local k = math.max(1, floor(l / 6))
            for j = 1, k do
                local t = (j - 0.5) / k * l
                local px, pz = a[1] + dx * t, a[2] + dz * t
                total = total + 1
                if onField(px + nx * 0.75, pz + nz * 0.75) and not onField(px - nx * 1.25, pz - nz * 1.25) then
                    good = good + 1
                end
            end
        end
    end
    if total == 0 then return 0 end
    return good / total
end

-------------------------------------------------------------------------------
-- Narrow strips across a field. Where a hedge between two old fields was taken
-- out (or a path crosses the field) the ground there is still not field, so
-- the traced outline goes round the strip (or stops at it) and the headland
-- splits. If the map's own outline has the strip in the field and nothing
-- stands in it, the field is traced again with the strip counted as field.
-------------------------------------------------------------------------------

-- How often field ground of this field shows up just across a narrow gap,
-- looking out from the traced outline (inside the map's outline).
function FieldScanner.countNarrowGaps(poly, onField, inMap)
    local n = #poly
    local hits = 0
    local far = 2 * FieldScanner.GAP_HALF
    for i = 1, n do
        local a, b = poly[i], poly[(i % n) + 1]
        local dx, dz, l = G.normalize(b[1] - a[1], b[2] - a[2])
        if l > 0.5 then
            local ox, oz = dz, -dx                     -- outward (CCW: field on the left)
            local k = math.max(1, floor(l / 8))
            for j = 1, k do
                local t = (j - 0.5) / k * l
                local px, pz = a[1] + dx * t, a[2] + dz * t
                if not onField(px + ox, pz + oz) then
                    for _, d in ipairs({ 3, 5, 7, far }) do
                        local qx, qz = px + ox * d, pz + oz * d
                        if onField(qx, qz) then
                            if inMap(qx, qz) then hits = hits + 1 end
                            break
                        end
                    end
                end
            end
        end
    end
    return hits
end

-- onField that also counts a point as field when it lies in a narrow strip:
-- inside the map's outline, field ground on both sides within GAP_HALF, and
-- nothing standing there. stats counts the strip spots crossed and blocked.
function FieldScanner.bridgedOnField(onField, inMap, obstacleAt)
    local r = FieldScanner.GAP_HALF
    local d = r * 0.7071
    local axes = { { r, 0 }, { 0, r }, { d, d }, { d, -d } }
    local decided, blockedCell = {}, {}
    local stats = { bridged = 0, blocked = 0 }
    local cell, ocell = 0.5, FieldScanner.GAP_CELL
    local function key(px, pz, c) return floor(px / c) * 1000003 + floor(pz / c) end
    local fn = function(px, pz)
        if onField(px, pz) then return true end
        local k = key(px, pz, cell)
        local v = decided[k]
        if v == nil then
            v = false
            if inMap(px, pz) then
                for _, a in ipairs(axes) do
                    if onField(px + a[1], pz + a[2]) and onField(px - a[1], pz - a[2]) then
                        local ok2 = key(px, pz, ocell)
                        local blocked = blockedCell[ok2]
                        if blocked == nil then
                            blocked = obstacleAt(px, pz) == true
                            blockedCell[ok2] = blocked
                        end
                        if blocked then stats.blocked = stats.blocked + 1 else stats.bridged = stats.bridged + 1 end
                        v = not blocked
                        break
                    end
                end
            end
            decided[k] = v
        end
        return v
    end
    return fn, stats
end

-- The traced outline again, across narrow strips with nothing in them; nil
-- when there are none (or when hedges, walls or trees stand in them).
function FieldScanner.acrossStrips(x, z, poly, mapPoly, onField, fieldId)
    local inMap = function(px, pz) return G.pointInPolygon(mapPoly, px, pz) end
    local gaps = FieldScanner.countNarrowGaps(poly, onField, inMap)
    if gaps < FieldScanner.GAP_MIN_HITS then return nil end
    local api = FieldScanner.api
    local bridged, stats = FieldScanner.bridgedOnField(onField, inMap,
        function(px, pz) return api.obstacleAt ~= nil and api.obstacleAt(px, pz) end)
    local bp = FieldScanner.traceEdge(x, z, bridged)
    local a0, am = G.signedArea(poly), abs(G.signedArea(mapPoly))
    local a1 = (bp ~= nil) and G.signedArea(bp) or 0
    -- something stands in the strips (a hedge, wall or trees): keep going round them
    if bp == nil or stats.bridged == 0 or stats.blocked > stats.bridged * 0.25 then
        TramlineLog.info("[TramlinePilot] field %d: narrow strip(s) across the field kept (%d spots with a hedge, wall or trees, %d free)",
            fieldId or -1, stats.blocked, stats.bridged)
        return nil
    end
    if a1 < a0 * 0.98 or a1 > am * 1.15 + 2000 then
        TramlineLog.info("[TramlinePilot] field %d: outline across narrow strips not used (%.2f ha, %.2f ha before, map %.2f ha)",
            fieldId or -1, a1 / 10000, a0 / 10000, am / 10000)
        return nil
    end
    TramlineLog.info("[TramlinePilot] field %d: crossing narrow strip(s) of non-field ground with nothing in them (%.2f ha -> %.2f ha, %d point(s))",
        fieldId or -1, a0 / 10000, a1 / 10000, #bp)
    return bp
end

-------------------------------------------------------------------------------
-- Edge tracer. Keeps the field on the LEFT, so the result is CCW.
-------------------------------------------------------------------------------

function FieldScanner.traceEdge(x, z, onField)
    for attempt = 0, 5 do
        local a0 = math.rad(23 + 61 * attempt)
        local poly = FieldScanner.traceFrom(x, z, a0, onField)
        if poly ~= nil then return poly end
    end
    TramlineLog.warning("[TramlinePilot] could not trace the field edge")
    return nil
end

function FieldScanner.traceFrom(x, z, a0, onField)
    -- 1. walk outwards until we leave the field, then pin the edge down
    local dx, dz = cos(a0), sin(a0)
    local s, lastOn = 0, 0
    while s < 4000 and onField(x + dx * s, z + dz * s) do lastOn = s; s = s + 1 end
    if s >= 4000 then return nil end
    local lo, hi = lastOn, s
    for _ = 1, 16 do
        local mid = (lo + hi) * 0.5
        if onField(x + dx * mid, z + dz * mid) then lo = mid else hi = mid end
    end
    local sx, sz = x + dx * lo, z + dz * lo
    -- along the edge with the field on the left: outward (dx,dz) rotated right
    local h = atan2(dx, -dz)

    local Lmax, Lmin = FieldScanner.TRACE_LMAX, FieldScanner.TRACE_LMIN
    local L = Lmax
    local px, pz = sx, sz
    local pts = { { sx, sz } }
    local travelled, totalTurn = 0, 0
    local stepRad = math.rad(8)
    local closed = false
    for _ = 1, 40000 do
        local function on(phi) return onField(px + L * cos(phi), pz + L * sin(phi)) end
        -- bracket the on/off transition by rotating from the current heading
        local phiOff, phiOn
        if on(h) then
            local phi = h
            for _ = 1, 46 do
                phi = phi - stepRad
                if not on(phi) then phiOff, phiOn = phi, phi + stepRad; break end
            end
        else
            local phi = h
            for _ = 1, 46 do
                phi = phi + stepRad
                if on(phi) then phiOff, phiOn = phi - stepRad, phi; break end
            end
        end
        if phiOff == nil then return nil end       -- isolated pixel or lost
        for _ = 1, 12 do
            local mid = (phiOff + phiOn) * 0.5
            if on(mid) then phiOn = mid else phiOff = mid end
        end
        local phi = phiOn
        local turn = G.wrapPi(phi - h)
        if abs(turn) > FieldScanner.TRACE_TURN_MAX and L > Lmin + 1e-6 then
            L = math.max(Lmin, L * 0.5)            -- corner: look closer
        else
            px, pz = px + L * cos(phi), pz + L * sin(phi)
            travelled = travelled + L
            totalTurn = totalTurn + turn
            h = phi
            pts[#pts + 1] = { px, pz }
            if abs(turn) < math.rad(4) then L = math.min(Lmax, L * 1.5) end
            -- back at the start after going round?
            if travelled > 6 * Lmax and (px - sx) ^ 2 + (pz - sz) ^ 2 < (Lmax * 1.05) ^ 2 and abs(totalTurn) > math.pi then
                closed = true
                break
            end
            if abs(totalTurn) > 5 * math.pi then return nil end   -- lost
        end
    end
    if not closed or #pts < 8 then return nil end
    local poly = G.dedupe(pts, 0.05, true)
    -- a CCW trace of the outer edge is positive; negative means we went round an island
    if G.signedArea(poly) <= 0 then return nil end
    if not G.pointInPolygon(poly, x, z) then return nil end
    -- straighten the pixel staircase on diagonal edges, then put back the
    -- corners the probe cut
    poly = G.simplify(poly, 0.45, true)
    poly = G.sharpenCorners(poly, 6, 5, 4)
    return poly
end

-------------------------------------------------------------------------------
-- Grid fallback: flood fill + Moore contour of the field cells
-------------------------------------------------------------------------------

function FieldScanner.gridPolygon(x, z, onField)
    local step = FieldScanner.GRID_STEP
    local ox, oz = floor(x / step) * step, floor(z / step) * step
    local function key(ix, iz) return ix * 100003 + iz end
    local cells, queue, head = {}, { { 0, 0 } }, 1
    cells[key(0, 0)] = true
    local count = 0
    local minIz, minIzIx = 0, 0
    local dirs = { { 1, 0 }, { -1, 0 }, { 0, 1 }, { 0, -1 } }
    while head <= #queue and count < FieldScanner.GRID_MAX_CELLS do
        local c = queue[head]; head = head + 1
        count = count + 1
        if c[2] < minIz or (c[2] == minIz and c[1] < minIzIx) then minIz, minIzIx = c[2], c[1] end
        for _, d in ipairs(dirs) do
            local nx, nz = c[1] + d[1], c[2] + d[2]
            local k = key(nx, nz)
            if cells[k] == nil then
                if onField(ox + nx * step, oz + nz * step) then
                    cells[k] = true
                    queue[#queue + 1] = { nx, nz }
                else
                    cells[k] = false
                end
            end
        end
    end
    local function inR(ix, iz) return cells[key(ix, iz)] == true end
    local M = { { -1, 0 }, { -1, -1 }, { 0, -1 }, { 1, -1 }, { 1, 0 }, { 1, 1 }, { 0, 1 }, { -1, 1 } }
    local function idxOf(ddx, ddz)
        for i, m in ipairs(M) do if m[1] == ddx and m[2] == ddz then return i end end
        return 1
    end
    local sx, sz = minIzIx, minIz
    local cx, cz, bx, bz = sx, sz, sx - 1, sz
    local bx0, bz0 = bx, bz
    local contour = {}
    for _ = 1, count * 4 + 100 do
        contour[#contour + 1] = { ox + cx * step, oz + cz * step }
        local startIdx = idxOf(bx - cx, bz - cz)
        local found, qx, qz = false, bx, bz
        for i = 1, 8 do
            local mi = ((startIdx - 1 + i) % 8) + 1
            local nx, nz = cx + M[mi][1], cz + M[mi][2]
            if inR(nx, nz) then bx, bz, cx, cz, found = qx, qz, nx, nz, true; break end
            qx, qz = nx, nz
        end
        if not found then break end
        if cx == sx and cz == sz and bx == bx0 and bz == bz0 then break end
    end
    if #contour < 4 then return nil end
    local poly = G.makeCCW(G.simplify(G.dedupe(contour, 0.05, true), 1.2, true))
    return G.sharpenCorners(poly, 8, 6, 5)
end
