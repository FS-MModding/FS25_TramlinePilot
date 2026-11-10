--[[
    TramlineGeom.lua
    Pure 2D geometry used by the tramline layout and the worker route.
    No game API in here, so it can be tested outside the game.

    Conventions
      * points are arrays { x, z } (extra fields allowed)
      * the x/z plane is treated like a normal maths X/Y plane:
        heading 0 = +x, angles increase towards +z ("left" = counter-clockwise)
      * polygons are closed implicitly (last point connects to the first)
      * a polygon is "CCW" when its signed area is positive; its inside is then
        on the LEFT of every edge, so the inward normal of a direction (dx, dz)
        is (-dz, dx)

    Written for Lua 5.1 (Farming Simulator): no math.atan(y, x), no //, no goto.
]]

TramlineGeom = {}
local G = TramlineGeom

local sqrt, abs, min, max, floor, ceil = math.sqrt, math.abs, math.min, math.max, math.floor, math.ceil
local sin, cos, acos, pi = math.sin, math.cos, math.acos, math.pi
local atan2 = math.atan2 or function(y, x) return math.atan(y, x) end
G.atan2 = atan2

local TWO_PI = 2 * pi

function G.mod2pi(a)
    a = a % TWO_PI
    if a < 0 then a = a + TWO_PI end
    return a
end

-- wrap to (-pi, pi]
function G.wrapPi(a)
    a = (a + pi) % TWO_PI
    if a <= 0 then a = a + TWO_PI end
    return a - pi
end

function G.len(x, z) return sqrt(x * x + z * z) end

function G.dist(a, b) return sqrt((a[1] - b[1]) ^ 2 + (a[2] - b[2]) ^ 2) end

function G.normalize(x, z)
    local l = sqrt(x * x + z * z)
    if l < 1e-12 then return 0, 0, 0 end
    return x / l, z / l, l
end

-------------------------------------------------------------------------------
-- Polygons
-------------------------------------------------------------------------------

function G.signedArea(poly)
    local n, a = #poly, 0
    for i = 1, n do
        local p, q = poly[i], poly[(i % n) + 1]
        a = a + p[1] * q[2] - q[1] * p[2]
    end
    return a * 0.5
end

function G.reverse(pts)
    local out = {}
    for i = #pts, 1, -1 do out[#out + 1] = pts[i] end
    return out
end

function G.makeCCW(poly)
    if G.signedArea(poly) < 0 then return G.reverse(poly) end
    return poly
end

function G.centroid(poly)
    local n, a, cx, cz = #poly, 0, 0, 0
    for i = 1, n do
        local p, q = poly[i], poly[(i % n) + 1]
        local c = p[1] * q[2] - q[1] * p[2]
        a = a + c
        cx = cx + (p[1] + q[1]) * c
        cz = cz + (p[2] + q[2]) * c
    end
    if abs(a) < 1e-9 then return poly[1][1], poly[1][2] end
    return cx / (3 * a), cz / (3 * a)
end

function G.bounds(pts)
    local x0, z0, x1, z1 = math.huge, math.huge, -math.huge, -math.huge
    for _, p in ipairs(pts) do
        if p[1] < x0 then x0 = p[1] end
        if p[1] > x1 then x1 = p[1] end
        if p[2] < z0 then z0 = p[2] end
        if p[2] > z1 then z1 = p[2] end
    end
    return x0, z0, x1, z1
end

-- even-odd point in polygon
function G.pointInPolygon(poly, x, z)
    local inside = false
    local n = #poly
    local j = n
    for i = 1, n do
        local xi, zi = poly[i][1], poly[i][2]
        local xj, zj = poly[j][1], poly[j][2]
        if ((zi > z) ~= (zj > z)) and (x < (xj - xi) * (z - zi) / (zj - zi) + xi) then
            inside = not inside
        end
        j = i
    end
    return inside
end

-- distance from point to segment, and the parameter t (0..1) of the foot point
function G.distPointSeg(px, pz, ax, az, bx, bz)
    local dx, dz = bx - ax, bz - az
    local l2 = dx * dx + dz * dz
    local t = 0
    if l2 > 1e-12 then
        t = ((px - ax) * dx + (pz - az) * dz) / l2
        if t < 0 then t = 0 elseif t > 1 then t = 1 end
    end
    local fx, fz = ax + dx * t, az + dz * t
    return sqrt((px - fx) ^ 2 + (pz - fz) ^ 2), t
end

function G.distToPolyline(pts, closed, x, z)
    local n = #pts
    local best = math.huge
    local last = closed and n or n - 1
    for i = 1, last do
        local a, b = pts[i], pts[(i % n) + 1]
        local d = G.distPointSeg(x, z, a[1], a[2], b[1], b[2])
        if d < best then best = d end
    end
    return best
end

-------------------------------------------------------------------------------
-- Edge index: buckets polygon edges into a grid so "distance to the nearest
-- edge" and "anything closer than r?" are fast on fields with many vertices.
-------------------------------------------------------------------------------

function G.buildEdgeIndex(poly, cell)
    local idx = { poly = poly, cell = cell, buckets = {} }
    local n = #poly
    local function key(ix, iz) return ix * 1000003 + iz end
    idx.key = key
    for i = 1, n do
        local a, b = poly[i], poly[(i % n) + 1]
        local x0, x1 = min(a[1], b[1]), max(a[1], b[1])
        local z0, z1 = min(a[2], b[2]), max(a[2], b[2])
        for ix = floor(x0 / cell), floor(x1 / cell) do
            for iz = floor(z0 / cell), floor(z1 / cell) do
                local k = key(ix, iz)
                local bkt = idx.buckets[k]
                if bkt == nil then bkt = {}; idx.buckets[k] = bkt end
                bkt[#bkt + 1] = i
            end
        end
    end
    return idx
end

-- smallest distance from (x,z) to any edge, searching only within 'radius'
-- (returns math.huge if nothing is that close)
function G.nearestEdgeDist(idx, x, z, radius)
    local cell, poly, n = idx.cell, idx.poly, #idx.poly
    local r = ceil(radius / cell)
    local cx, cz = floor(x / cell), floor(z / cell)
    local best = math.huge
    local seen = {}
    for ix = cx - r, cx + r do
        for iz = cz - r, cz + r do
            local bkt = idx.buckets[idx.key(ix, iz)]
            if bkt ~= nil then
                for _, i in ipairs(bkt) do
                    if not seen[i] then
                        seen[i] = true
                        local a, b = poly[i], poly[(i % n) + 1]
                        local d = G.distPointSeg(x, z, a[1], a[2], b[1], b[2])
                        if d < best then best = d end
                    end
                end
            end
        end
    end
    return best
end

-------------------------------------------------------------------------------
-- Polyline helpers
-------------------------------------------------------------------------------

function G.polyLength(pts, closed)
    local n, l = #pts, 0
    local last = closed and n or n - 1
    for i = 1, last do
        local a, b = pts[i], pts[(i % n) + 1]
        l = l + sqrt((b[1] - a[1]) ^ 2 + (b[2] - a[2]) ^ 2)
    end
    return l
end

-- insert points so no segment is longer than 'step'
function G.densify(pts, step, closed)
    local out = {}
    local n = #pts
    local last = closed and n or n - 1
    for i = 1, last do
        local a, b = pts[i], pts[(i % n) + 1]
        out[#out + 1] = { a[1], a[2] }
        local l = sqrt((b[1] - a[1]) ^ 2 + (b[2] - a[2]) ^ 2)
        local k = floor(l / step)
        if k * step >= l - 1e-9 then k = k - 1 end
        for j = 1, k do
            local t = j * step / l
            out[#out + 1] = { a[1] + (b[1] - a[1]) * t, a[2] + (b[2] - a[2]) * t }
        end
    end
    if not closed then out[#out + 1] = { pts[n][1], pts[n][2] } end
    return out
end

-- remove consecutive points closer than eps
function G.dedupe(pts, eps, closed)
    local out = {}
    for _, p in ipairs(pts) do
        local q = out[#out]
        if q == nil or (p[1] - q[1]) ^ 2 + (p[2] - q[2]) ^ 2 > eps * eps then
            out[#out + 1] = p
        end
    end
    if closed and #out > 2 then
        local a, b = out[1], out[#out]
        if (a[1] - b[1]) ^ 2 + (a[2] - b[2]) ^ 2 <= eps * eps then out[#out] = nil end
    end
    return out
end

-- Douglas-Peucker on an open polyline (iterative)
local function dpOpen(pts, tol)
    local n = #pts
    if n < 3 then return pts end
    local keep = { [1] = true, [n] = true }
    local stack = { { 1, n } }
    while #stack > 0 do
        local seg = table.remove(stack)
        local i1, i2 = seg[1], seg[2]
        local a, b = pts[i1], pts[i2]
        local bestD, bestI = -1, nil
        for i = i1 + 1, i2 - 1 do
            local d = G.distPointSeg(pts[i][1], pts[i][2], a[1], a[2], b[1], b[2])
            if d > bestD then bestD, bestI = d, i end
        end
        if bestI ~= nil and bestD > tol then
            keep[bestI] = true
            stack[#stack + 1] = { i1, bestI }
            stack[#stack + 1] = { bestI, i2 }
        end
    end
    local out = {}
    for i = 1, n do if keep[i] then out[#out + 1] = pts[i] end end
    return out
end

function G.simplify(pts, tol, closed)
    if not closed then return dpOpen(pts, tol) end
    local n = #pts
    if n < 4 then return pts end
    -- split the ring at the point furthest from the first one
    local far, farD = 1, -1
    for i = 2, n do
        local d = (pts[i][1] - pts[1][1]) ^ 2 + (pts[i][2] - pts[1][2]) ^ 2
        if d > farD then far, farD = i, d end
    end
    local a, b = {}, {}
    for i = 1, far do a[#a + 1] = pts[i] end
    for i = far, n do b[#b + 1] = pts[i] end
    b[#b + 1] = pts[1]
    local sa, sb = dpOpen(a, tol), dpOpen(b, tol)
    local out = {}
    for i = 1, #sa do out[#out + 1] = sa[i] end
    for i = 2, #sb - 1 do out[#out + 1] = sb[i] end
    return out
end

-- cumulative arc length table: cum[i] = distance from pts[1] to pts[i];
-- for closed polylines cum[n+1] = total length
function G.cumulative(pts, closed)
    local cum = { 0 }
    local n = #pts
    for i = 2, n do
        cum[i] = cum[i - 1] + sqrt((pts[i][1] - pts[i - 1][1]) ^ 2 + (pts[i][2] - pts[i - 1][2]) ^ 2)
    end
    if closed then
        cum[n + 1] = cum[n] + sqrt((pts[1][1] - pts[n][1]) ^ 2 + (pts[1][2] - pts[n][2]) ^ 2)
    end
    return cum
end

-- Parametrised polyline (open or closed) for walking along it
function G.makePath(pts, closed)
    local path = { pts = pts, closed = closed, cum = G.cumulative(pts, closed) }
    local n = #pts
    path.length = closed and path.cum[n + 1] or path.cum[n]
    return path
end

-- nearest point on the path: returns s (distance along), x, z, distance
function G.project(path, x, z)
    local pts, n = path.pts, #path.pts
    local last = path.closed and n or n - 1
    local bestD, bestS, bx, bz = math.huge, 0, pts[1][1], pts[1][2]
    for i = 1, last do
        local a, b = pts[i], pts[(i % n) + 1]
        local d, t = G.distPointSeg(x, z, a[1], a[2], b[1], b[2])
        if d < bestD then
            local segLen = path.cum[i + 1] - path.cum[i]
            bestD, bestS = d, path.cum[i] + segLen * t
            bx, bz = a[1] + (b[1] - a[1]) * t, a[2] + (b[2] - a[2]) * t
        end
    end
    return bestS, bx, bz, bestD
end

-- point and unit tangent at distance s
function G.pointAt(path, s)
    local pts, n, cum = path.pts, #path.pts, path.cum
    if path.closed then
        s = s % path.length
        if s < 0 then s = s + path.length end
    else
        if s < 0 then s = 0 elseif s > path.length then s = path.length end
    end
    local last = path.closed and n or n - 1
    -- binary search for the segment
    local lo, hi = 1, last
    while lo < hi do
        local mid = floor((lo + hi + 1) / 2)
        if cum[mid] <= s then lo = mid else hi = mid - 1 end
    end
    local i = lo
    local a, b = pts[i], pts[(i % n) + 1]
    local segLen = cum[i + 1] - cum[i]
    local t = segLen > 1e-9 and (s - cum[i]) / segLen or 0
    local tx, tz = G.normalize(b[1] - a[1], b[2] - a[2])
    return a[1] + (b[1] - a[1]) * t, a[2] + (b[2] - a[2]) * t, tx, tz
end

-- Points along the path from s0 to s1. dir = +1 (increasing s) or -1.
-- For closed paths the walk wraps around; 'fullLoop' walks one whole lap.
-- Includes both end points and every vertex in between.
function G.walk(path, s0, s1, dir, fullLoop)
    local out = {}
    local pts, n, cum, L = path.pts, #path.pts, path.cum, path.length
    local x, z = G.pointAt(path, s0)
    out[1] = { x, z }
    local dist
    if path.closed then
        if dir > 0 then dist = (s1 - s0) % L else dist = (s0 - s1) % L end
        if fullLoop and dist < 1e-6 then dist = L end
    else
        dist = abs(s1 - s0)
        dir = (s1 >= s0) and 1 or -1
    end
    -- vertex positions in the walking order
    local verts = {}
    for i = 1, n do
        local d
        if path.closed then
            if dir > 0 then d = (cum[i] - s0) % L else d = (s0 - cum[i]) % L end
        else
            if dir > 0 then d = cum[i] - s0 else d = s0 - cum[i] end
        end
        if d > 1e-6 and d < dist - 1e-6 then verts[#verts + 1] = { d, i } end
    end
    table.sort(verts, function(p, q) return p[1] < q[1] end)
    for _, v in ipairs(verts) do
        local p = pts[v[2]]
        out[#out + 1] = { p[1], p[2] }
    end
    local ex, ez = G.pointAt(path, path.closed and (s0 + dir * dist) or s1)
    out[#out + 1] = { ex, ez }
    return out
end

-- Restore corners that an edge tracer has cut off: wherever two long edges
-- (>= longLen) are separated by a short chain (total < maxChain metres) and
-- meet at a real angle, replace the chain by the intersection of the two
-- long edges (if it lies within maxShift of the chain).
function G.sharpenCorners(poly, longLen, maxChain, maxShift)
    local n = #poly
    if n < 4 then return poly end
    local segLen = {}
    for i = 1, n do segLen[i] = G.dist(poly[i], poly[(i % n) + 1]) end
    local longIdx = {}
    for i = 1, n do if segLen[i] >= longLen then longIdx[#longIdx + 1] = i end end
    if #longIdx < 2 then return poly end
    local replace = {}   -- vertex index -> { x, z } or false (drop)
    for k = 1, #longIdx do
        local e1 = longIdx[k]
        local e2 = longIdx[(k % #longIdx) + 1]
        -- vertices strictly between the end of e1 and the start of e2
        local first = (e1 % n) + 1                  -- end vertex of e1
        local last = e2                             -- start vertex of e2
        local chainLen, j, cnt = 0, first, 0
        while j ~= last and cnt < n do
            chainLen = chainLen + segLen[j]
            j = (j % n) + 1
            cnt = cnt + 1
        end
        if cnt >= 1 and chainLen < maxChain then
            local a1, b1 = poly[e1], poly[(e1 % n) + 1]
            local a2, b2 = poly[e2], poly[(e2 % n) + 1]
            local ux, uz = G.normalize(b1[1] - a1[1], b1[2] - a1[2])
            local vx, vz = G.normalize(b2[1] - a2[1], b2[2] - a2[2])
            -- take the edge lines from a least-squares fit over the straight
            -- run (up to 40 m), leaving out the last 2.5 m before the corner:
            -- the vertex next to a corner is the one the probe displaced
            local function fitDir(fromIdx, step, dx0, dz0)
                local p0 = poly[fromIdx]
                local run = { p0 }
                local acc, idx = 0, fromIdx
                for _ = 1, n do
                    local nxt = ((idx - 1 + step) % n) + 1
                    local q = poly[nxt]
                    local ddx, ddz, l = G.normalize(q[1] - poly[idx][1], q[2] - poly[idx][2])
                    if step < 0 then ddx, ddz = -ddx, -ddz end
                    if l > 1 and (ddx * dx0 + ddz * dz0) < math.cos(math.rad(10)) then break end
                    run[#run + 1] = q
                    acc = acc + l
                    idx = nxt
                    if acc >= 40 then break end
                end
                if #run < 2 then return dx0, dz0, p0 end
                local dense = G.densify(run, 1.0, false)
                local sx, sz, cnt = 0, 0, 0
                local used = {}
                for _, q in ipairs(dense) do
                    if G.dist(q, p0) > 2.5 then used[#used + 1] = q; sx = sx + q[1]; sz = sz + q[2]; cnt = cnt + 1 end
                end
                if cnt < 3 then return dx0, dz0, p0 end
                local mx, mz = sx / cnt, sz / cnt
                local sxx, szz, sxz = 0, 0, 0
                for _, q in ipairs(used) do
                    local ex, ez = q[1] - mx, q[2] - mz
                    sxx, szz, sxz = sxx + ex * ex, szz + ez * ez, sxz + ex * ez
                end
                local ang = 0.5 * atan2(2 * sxz, sxx - szz)
                local fx, fz = cos(ang), sin(ang)
                if fx * dx0 + fz * dz0 < 0 then fx, fz = -fx, -fz end
                return fx, fz, { mx, mz }
            end
            ux, uz, a1 = fitDir((e1 % n) + 1, -1, ux, uz)
            vx, vz, a2 = fitDir(e2, 1, vx, vz)
            local den = ux * vz - uz * vx
            if abs(den) > math.sin(math.rad(20)) then
                local t = ((a2[1] - a1[1]) * vz - (a2[2] - a1[2]) * vx) / den
                local X = { a1[1] + ux * t, a1[2] + uz * t }
                local mx, mz = (poly[first][1] + poly[last][1]) * 0.5, (poly[first][2] + poly[last][2]) * 0.5
                if sqrt((X[1] - mx) ^ 2 + (X[2] - mz) ^ 2) < maxShift then
                    local v = first
                    replace[v] = X
                    v = (v % n) + 1
                    while v ~= last do replace[v] = false; v = (v % n) + 1 end
                    replace[last] = false
                end
            end
        end
    end
    local out = {}
    for i = 1, n do
        local r = replace[i]
        if r == nil then out[#out + 1] = poly[i]
        elseif r ~= false then out[#out + 1] = r end
    end
    return G.dedupe(out, 0.05, true)
end

-------------------------------------------------------------------------------
-- Inward offset of a CCW polygon by distance d.
--
-- 1. raw offset curve: each edge moved inwards by d; convex corners meet at
--    the mitre point, reflex corners are joined by an arc of radius d around
--    the original vertex
-- 2. the raw curve is sampled densely and every sample that is closer than d
--    to the boundary is thrown away (those are the parts that cross over
--    where the field is narrower than 2d, or where short edges get "eaten")
-- 3. the remaining runs are stitched back together at the crossing points
--    into one or more closed loops
-- Returns a list of CCW loops (largest first).
-------------------------------------------------------------------------------

-- sign = +1 inward, -1 outward (see G.offsetOutward)
local function offsetCore(poly, d, step, sign)
    step = step or 0.5
    poly = G.makeCCW(G.dedupe(poly, 0.01, true))
    local n = #poly
    if n < 3 or d <= 0 then return { poly } end

    -- 1. raw curve
    local raw = {}
    for i = 1, n do
        local p0, p1, p2 = poly[((i - 2) % n) + 1], poly[i], poly[(i % n) + 1]
        local ux, uz = G.normalize(p1[1] - p0[1], p1[2] - p0[2])
        local vx, vz = G.normalize(p2[1] - p1[1], p2[2] - p1[2])
        local n1x, n1z = -uz * sign, ux * sign     -- offset normals (left = inward)
        local n2x, n2z = -vz * sign, vx * sign
        local cross = (ux * vz - uz * vx) * sign   -- > 0: the offset edges meet (mitre)
        local dot = ux * vx + uz * vz
        if cross > 1e-9 or (cross >= -1e-9 and dot > 0) then
            local den = 1 + (n1x * n2x + n1z * n2z)
            if den > 0.08 then
                raw[#raw + 1] = { p1[1] + d * (n1x + n2x) / den, p1[2] + d * (n1z + n2z) / den }
            else
                -- extremely sharp spike: bevel, the trimming sorts it out
                raw[#raw + 1] = { p1[1] + d * n1x, p1[2] + d * n1z }
                raw[#raw + 1] = { p1[1] + d * n2x, p1[2] + d * n2z }
            end
        else
            -- the offset edges part: arc of radius d around p1 from n1 to n2
            local a0, a1 = atan2(n1z, n1x), atan2(n2z, n2x)
            local sweep = G.wrapPi(a1 - a0)
            if sign > 0 and sweep > 0 then sweep = sweep - TWO_PI end
            if sign < 0 and sweep < 0 then sweep = sweep + TWO_PI end
            local k = max(1, ceil(abs(sweep) * d / step))
            for j = 0, k do
                local a = a0 + sweep * j / k
                raw[#raw + 1] = { p1[1] + d * cos(a), p1[2] + d * sin(a) }
            end
        end
    end

    -- 2. sample and classify
    local dense = G.densify(raw, step, true)
    local m = #dense
    local idx = G.buildEdgeIndex(poly, max(d, 4))
    local tol = 0.02 + d * 0.002
    local tolFar = 0.05 + d * 0.004
    local valid, nValid = {}, 0
    for i = 1, m do
        local p = dense[i]
        local dd = G.nearestEdgeDist(idx, p[1], p[2], d)
        -- every point of the true offset curve is exactly d from the outline:
        -- closer is where the field is narrower than 2d, further is an offset
        -- line carried on past the end of a short edge (a small hook or notch
        -- in the outline throws a spike like that into the field)
        valid[i] = dd >= d - tol and dd <= d + tolFar
        if valid[i] then nValid = nValid + 1 end
    end
    if nValid == 0 then return {} end
    if nValid == m then
        local loop = G.dedupe(dense, 0.05, true)
        return { G.makeCCW(loop) }
    end

    -- 3. runs of valid samples
    local runs = {}
    local startI = nil
    for i = 1, m do
        if not valid[i] and valid[(i % m) + 1] then startI = (i % m) + 1; break end
    end
    if startI == nil then return {} end
    local i, cnt = startI, 0
    while cnt < m do
        if valid[i] then
            local run = {}
            local first = i
            while valid[i] and cnt < m do
                run[#run + 1] = { dense[i][1], dense[i][2] }
                i = (i % m) + 1; cnt = cnt + 1
            end
            -- segment leaving the run and segment entering it (for stitching)
            local prevI = ((first - 2) % m) + 1
            run.inA, run.inB = dense[prevI], dense[first]
            local lastI = ((i - 2) % m) + 1
            run.outA, run.outB = dense[lastI], dense[i]
            runs[#runs + 1] = run
        else
            i = (i % m) + 1; cnt = cnt + 1
        end
    end

    local function segIntersect(a, b, c, e)
        local rx, rz = b[1] - a[1], b[2] - a[2]
        local sx, sz = e[1] - c[1], e[2] - c[2]
        local den = rx * sz - rz * sx
        if abs(den) < 1e-12 then return nil end
        local t = ((c[1] - a[1]) * sz - (c[2] - a[2]) * sx) / den
        local u = ((c[1] - a[1]) * rz - (c[2] - a[2]) * rx) / den
        if t < -0.5 or t > 1.5 or u < -0.5 or u > 1.5 then return nil end
        return { a[1] + rx * t, a[2] + rz * t }
    end

    -- stitch: the end of a run continues at the start of the run whose
    -- entering segment crosses our leaving segment (nearest one)
    local used = {}
    local loops = {}
    for r = 1, #runs do
        if not used[r] then
            local loop = {}
            local cur = r
            local guard = 0
            while cur ~= nil and not used[cur] and guard <= #runs do
                guard = guard + 1
                used[cur] = true
                local run = runs[cur]
                for _, p in ipairs(run) do loop[#loop + 1] = p end
                local last = run[#run]
                local bestR, bestD = nil, 6 * step + 1
                for q = 1, #runs do
                    if q == r or not used[q] then
                        local f = runs[q][1]
                        local dd = sqrt((f[1] - last[1]) ^ 2 + (f[2] - last[2]) ^ 2)
                        if dd < bestD then bestR, bestD = q, dd end
                    end
                end
                if bestR == nil then
                    -- nothing starts close by: a small hook or notch in the outline
                    -- left a wider gap (e.g. 6 m at d = 18). Carry on with the next
                    -- run along the curve rather than closing the loop with a long
                    -- straight line across the field.
                    local q = (cur % #runs) + 1
                    if q == r or not used[q] then
                        local f = runs[q][1]
                        local dd = sqrt((f[1] - last[1]) ^ 2 + (f[2] - last[2]) ^ 2)
                        if dd <= max(4, 1.5 * d) then bestR = q end
                    end
                end
                if bestR ~= nil then
                    local x = segIntersect(run.outA, run.outB, runs[bestR].inA, runs[bestR].inB)
                    if x ~= nil then loop[#loop + 1] = x end
                end
                if bestR == r then break end
                cur = bestR
            end
            loop = G.dedupe(loop, 0.05, true)
            if #loop >= 3 and G.signedArea(loop) > 0 then
                loops[#loops + 1] = loop
            end
        end
    end
    table.sort(loops, function(a, b) return G.signedArea(a) > G.signedArea(b) end)
    -- drop slivers
    local out = {}
    for _, l in ipairs(loops) do
        if G.signedArea(l) > max(20, d * d * 0.5) then out[#out + 1] = l end
    end
    return out
end

function G.offsetInward(poly, d, step) return offsetCore(poly, d, step, 1) end

-- Outward offset of a CCW polygon by d: convex corners become arcs of radius
-- d, reflex corners mitre; where the polygon has inlets narrower than 2d
-- they close up. Returns a list of CCW loops (largest first).
function G.offsetOutward(poly, d, step) return offsetCore(poly, d, step, -1) end

-- Same shape with every convex corner rounded to at least radius r (the
-- part of the polygon a disc of radius r can reach - "opening"). Never
-- grows the polygon. Returns a list of CCW loops.
function G.roundConvex(poly, r, step)
    local out = {}
    for _, lp in ipairs(offsetCore(poly, r, step, 1)) do
        for _, op in ipairs(offsetCore(G.simplify(lp, 0.02, true), r, step, -1)) do
            out[#out + 1] = op
        end
    end
    table.sort(out, function(a, b) return G.signedArea(a) > G.signedArea(b) end)
    return out
end

-------------------------------------------------------------------------------
-- Round the corners of a closed (or open) polyline with arcs of radius r.
-- Corners turning less than minDeg are left alone; the arc's tangent length
-- is limited to half of each neighbouring segment so arcs never overlap.
-------------------------------------------------------------------------------

function G.fillet(pts, r, closed, minDeg, arcStep)
    minDeg = minDeg or 8
    arcStep = arcStep or 1.0
    local n = #pts
    if n < 3 or r <= 0 then return pts end
    local minA = math.rad(minDeg)
    local out = {}
    local first, last = 1, n
    if not closed then out[1] = { pts[1][1], pts[1][2] }; first, last = 2, n - 1 end
    for i = first, last do
        local p0, p1, p2 = pts[((i - 2) % n) + 1], pts[i], pts[(i % n) + 1]
        local ux, uz, l1 = G.normalize(p1[1] - p0[1], p1[2] - p0[2])
        local vx, vz, l2 = G.normalize(p2[1] - p1[1], p2[2] - p1[2])
        local dot = max(-1, min(1, ux * vx + uz * vz))
        local theta = acos(dot)
        if theta < minA or l1 < 1e-6 or l2 < 1e-6 or theta > math.rad(175) then
            out[#out + 1] = { p1[1], p1[2] }
        else
            local rr = r
            local t = rr * math.tan(theta / 2)
            local tMax = min(l1, l2) * 0.5
            if t > tMax then t = tMax; rr = t / math.tan(theta / 2) end
            local sx, sz = p1[1] - ux * t, p1[2] - uz * t
            local ex, ez = p1[1] + vx * t, p1[2] + vz * t
            local bx, bz = G.normalize(vx - ux, vz - uz)
            local cd = rr / cos(theta / 2)
            local cx, cz = p1[1] + bx * cd, p1[2] + bz * cd
            local a0 = atan2(sz - cz, sx - cx)
            local a1 = atan2(ez - cz, ex - cx)
            local sweep = G.wrapPi(a1 - a0)
            local k = max(2, ceil(rr * abs(sweep) / arcStep))
            for j = 0, k do
                local a = a0 + sweep * j / k
                out[#out + 1] = { cx + rr * cos(a), cz + rr * sin(a) }
            end
        end
    end
    if not closed then out[#out + 1] = { pts[n][1], pts[n][2] } end
    return G.dedupe(out, 0.05, closed)
end

-------------------------------------------------------------------------------
-- Clip the infinite line  o + t * dir  (dir unit) against a polygon.
-- Returns the inside intervals { {t0, t1}, ... } sorted along the line.
-------------------------------------------------------------------------------

function G.clipLine(poly, ox, oz, dx, dz)
    local ts = {}
    local n = #poly
    local function side(p) return dx * (p[2] - oz) - dz * (p[1] - ox) end
    for i = 1, n do
        local a, b = poly[i], poly[(i % n) + 1]
        local sa, sb = side(a), side(b)
        if (sa > 0) ~= (sb > 0) then
            local f = sa / (sa - sb)
            local x, z = a[1] + (b[1] - a[1]) * f, a[2] + (b[2] - a[2]) * f
            ts[#ts + 1] = (x - ox) * dx + (z - oz) * dz
        end
    end
    table.sort(ts)
    local out = {}
    for i = 1, #ts - 1, 2 do
        if ts[i + 1] - ts[i] > 1e-6 then out[#out + 1] = { ts[i], ts[i + 1] } end
    end
    return out
end

-------------------------------------------------------------------------------
-- Dubins paths: shortest forward-only path between two poses with a minimum
-- turning radius. Headings in radians (0 = +x, counter-clockwise positive).
-------------------------------------------------------------------------------

local function dubinsWords(alpha, beta, d)
    local sa, sb, ca, cb = sin(alpha), sin(beta), cos(alpha), cos(beta)
    local cab = cos(alpha - beta)
    local res = {}
    local mod = G.mod2pi
    -- LSL
    do
        local p2 = 2 + d * d - 2 * cab + 2 * d * (sa - sb)
        if p2 >= 0 then
            local tmp = atan2(cb - ca, d + sa - sb)
            res[#res + 1] = { "LSL", mod(-alpha + tmp), sqrt(p2), mod(beta - tmp) }
        end
    end
    -- RSR
    do
        local p2 = 2 + d * d - 2 * cab + 2 * d * (sb - sa)
        if p2 >= 0 then
            local tmp = atan2(ca - cb, d - sa + sb)
            res[#res + 1] = { "RSR", mod(alpha - tmp), sqrt(p2), mod(-beta + tmp) }
        end
    end
    -- LSR
    do
        local p2 = -2 + d * d + 2 * cab + 2 * d * (sa + sb)
        if p2 >= 0 then
            local p = sqrt(p2)
            local tmp = atan2(-ca - cb, d + sa + sb) - atan2(-2, p)
            res[#res + 1] = { "LSR", mod(-alpha + tmp), p, mod(-mod(beta) + tmp) }
        end
    end
    -- RSL
    do
        local p2 = -2 + d * d + 2 * cab - 2 * d * (sa + sb)
        if p2 >= 0 then
            local p = sqrt(p2)
            local tmp = atan2(ca + cb, d - sa - sb) - atan2(2, p)
            res[#res + 1] = { "RSL", mod(alpha - tmp), p, mod(beta - tmp) }
        end
    end
    -- RLR
    do
        local tmp = (6 - d * d + 2 * cab + 2 * d * (sa - sb)) / 8
        if abs(tmp) <= 1 then
            local p = mod(TWO_PI - acos(tmp))
            local t = mod(alpha - atan2(ca - cb, d - sa + sb) + p / 2)
            res[#res + 1] = { "RLR", t, p, mod(alpha - beta - t + p) }
        end
    end
    -- LRL
    do
        local tmp = (6 - d * d + 2 * cab + 2 * d * (-sa + sb)) / 8
        if abs(tmp) <= 1 then
            local p = mod(TWO_PI - acos(tmp))
            local t = mod(-alpha - atan2(ca - cb, d + sa - sb) + p / 2)
            res[#res + 1] = { "LRL", t, p, mod(mod(beta) - alpha - t + p) }
        end
    end
    return res
end

-- advance a pose along one Dubins segment of (normalised) length t
local function dubinsStep(x, z, h, kind, t, r)
    if kind == "L" then
        return x + r * (sin(h + t) - sin(h)), z - r * (cos(h + t) - cos(h)), h + t
    elseif kind == "R" then
        return x - r * (sin(h - t) - sin(h)), z + r * (cos(h - t) - cos(h)), h - t
    else
        return x + t * r * cos(h), z + t * r * sin(h), h
    end
end

-- Returns the shortest path { word, t, p, q, length, r } or nil.
-- 'filter' (optional) gets the sampled points and can reject a candidate
function G.dubins(x0, z0, h0, x1, z1, h1, r, filter, step)
    local dx, dz = x1 - x0, z1 - z0
    local D = sqrt(dx * dx + dz * dz)
    local d = D / r
    local theta = G.mod2pi(atan2(dz, dx))
    local alpha = G.mod2pi(h0 - theta)
    local beta = G.mod2pi(h1 - theta)
    local cands = dubinsWords(alpha, beta, d)
    table.sort(cands, function(a, b) return (a[2] + a[3] + a[4]) < (b[2] + b[3] + b[4]) end)
    for _, c in ipairs(cands) do
        local path = { word = c[1], t = c[2], p = c[3], q = c[4], length = (c[2] + c[3] + c[4]) * r, r = r,
                       x0 = x0, z0 = z0, h0 = h0 }
        if filter == nil or filter(G.dubinsPoints(path, step or 1.0)) then
            return path
        end
    end
    return nil
end

-- sample a Dubins path every 'step' metres (includes start and end)
function G.dubinsPoints(path, step)
    local pts = { { path.x0, path.z0 } }
    local x, z, h = path.x0, path.z0, path.h0
    local r = path.r
    local lens = { path.t, path.p, path.q }
    for k = 1, 3 do
        local kind = string.sub(path.word, k, k)
        local seglen = lens[k] * r
        local nsteps = max(1, ceil(seglen / step))
        local dt = lens[k] / nsteps
        for _ = 1, nsteps do
            x, z, h = dubinsStep(x, z, h, kind, dt, r)
            pts[#pts + 1] = { x, z }
        end
    end
    return G.dedupe(pts, 0.01, false), x, z, h
end

-------------------------------------------------------------------------------
-- Fillet between two straight directions meeting at a point J:
-- coming in along unit u, leaving along unit v, radius r.
-- Returns arc points from the tangent point on the way in to the one on the
-- way out, and the tangent length.
-------------------------------------------------------------------------------

function G.cornerArc(jx, jz, ux, uz, vx, vz, r, arcStep)
    arcStep = arcStep or 0.5
    local dot = max(-1, min(1, ux * vx + uz * vz))
    local theta = acos(dot)
    if theta < 1e-3 then return { { jx, jz } }, 0 end
    local t = r * math.tan(theta / 2)
    local sx, sz = jx - ux * t, jz - uz * t
    local ex, ez = jx + vx * t, jz + vz * t
    local bx, bz = G.normalize(vx - ux, vz - uz)
    local cd = r / cos(theta / 2)
    local cx, cz = jx + bx * cd, jz + bz * cd
    local a0 = atan2(sz - cz, sx - cx)
    local a1 = atan2(ez - cz, ex - cx)
    local sweep = G.wrapPi(a1 - a0)
    local k = max(2, ceil(r * abs(sweep) / arcStep))
    local out = {}
    for j = 0, k do
        local a = a0 + sweep * j / k
        out[#out + 1] = { cx + r * cos(a), cz + r * sin(a) }
    end
    return out, t
end

-------------------------------------------------------------------------------
-- Junction: curve from a straight line onto a closed path (headland ring).
--
-- The line arrives at J = (jx, jz), the path point at distance sJ, travelling
-- along unit (ux, uz). The curve leaves the line 'a' metres before J and
-- joins the path at distance 's', from then on heading along the path in
-- direction 'sense' (+1 = increasing s). Where the path allows it this is an
-- exact fillet of radius r (a circle touching both the line and the path);
-- where the path itself bends too tightly for that (rounded ring corners) it
-- is the shortest arc-straight-arc curve with both arcs turning the same way.
-- The curve never crosses the path.
--
-- Returns { pts = { line ... path }, a, s, kind } or nil.
-------------------------------------------------------------------------------

-- the path's segments touching the distance window [s0, s1] (wraps if closed)
function G.windowSegments(path, s0, s1)
    local pts, n, cum, L = path.pts, #path.pts, path.cum, path.length
    local segs = {}
    local last = path.closed and n or n - 1
    local whole = path.closed and (s1 - s0) >= L
    for i = 1, last do
        local a, b = cum[i], cum[i + 1]
        local hit = whole
        if not hit then
            if path.closed then
                for k = -1, 1 do
                    if b + k * L >= s0 and a + k * L <= s1 then hit = true end
                end
            else
                hit = b >= s0 and a <= s1
            end
        end
        if hit then
            local p, q = pts[i], pts[(i % n) + 1]
            segs[#segs + 1] = { p[1], p[2], q[1], q[2], a, b - a }
        end
    end
    return segs
end

-- nearest point on a segment list: distance, s, x, z, segment
function G.nearestOnSegments(segs, x, z)
    local best, bs, bx, bz, bseg = math.huge, 0, x, z, nil
    for _, sg in ipairs(segs) do
        local d, t = G.distPointSeg(x, z, sg[1], sg[2], sg[3], sg[4])
        if d < best then
            best, bs, bseg = d, sg[5] + sg[6] * t, sg
            bx, bz = sg[1] + (sg[3] - sg[1]) * t, sg[2] + (sg[4] - sg[2]) * t
        end
    end
    return best, bs, bx, bz, bseg
end

-- which side of the path a point is on: +1 left, -1 right (of increasing s), 0 on it
local function sideOf(segs, x, z, eps)
    local d, _, px, pz, sg = G.nearestOnSegments(segs, x, z)
    if sg == nil or d < (eps or 0.02) then return 0 end
    local tx, tz = G.normalize(sg[3] - sg[1], sg[4] - sg[2])
    local c = tx * (z - pz) - tz * (x - px)
    if abs(c) < 1e-9 then
        -- nearest point is a vertex seen square-on: use the offset direction
        c = tx * (z - sg[2]) - tz * (x - sg[1])
    end
    return c > 0 and 1 or -1
end

function G.junction(path, sJ, jx, jz, ux, uz, sense, r, arcStep)
    arcStep = arcStep or 0.5
    local L = path.length
    local _, _, tx, tz = G.pointAt(path, sJ)
    local vx, vz = tx * sense, tz * sense
    local cross = ux * vz - uz * vx
    if abs(cross) < 0.03 then return nil end          -- line runs along the path
    local left = cross > 0
    local nx, nz
    if left then nx, nz = -uz, ux else nx, nz = uz, -ux end
    local W = 8 * r + 30
    local segs = G.windowSegments(path, sJ - W, sJ + W)
    -- side of the path the line comes from
    local approachSide = (tx * (-uz) - tz * (-ux)) > 0 and 1 or -1
    local function aheadOf(s)
        local d = (s - sJ) * sense
        if path.closed then
            d = d % L
            if d > L * 0.5 then d = d - L end
        end
        return d
    end
    local aMax = 6 * r + 10

    -- 1. exact fillet: centre r beside the line, r away from the path
    local function f(a)
        local cx, cz = jx - ux * a + nx * r, jz - uz * a + nz * r
        return (G.nearestOnSegments(segs, cx, cz)) - r, cx, cz
    end
    if f(0) <= 0.02 then
        local lo, hi = 0, nil
        local a = 0.5
        while a <= aMax do
            if f(a) >= 0 then hi = a; break end
            lo = a
            a = a + 0.5
        end
        if hi ~= nil then
            for _ = 1, 14 do
                local mid = (lo + hi) * 0.5
                if f(mid) >= 0 then hi = mid else lo = mid end
            end
            a = hi
            local _, cx, cz = f(a)
            local _, s2, px, pz = G.nearestOnSegments(segs, cx, cz)
            local p1x, p1z = jx - ux * a, jz - uz * a
            local a0 = atan2(p1z - cz, p1x - cx)
            local a1 = atan2(pz - cz, px - cx)
            local sweep = left and G.mod2pi(a1 - a0) or -G.mod2pi(a0 - a1)
            local _, _, t2x, t2z = G.pointAt(path, s2)
            local rx, rz = (px - cx) / r, (pz - cz) / r
            local atx, atz
            if left then atx, atz = -rz, rx else atx, atz = rz, -rx end
            if abs(sweep) < pi * 0.995 and (atx * t2x + atz * t2z) * sense > 0.985 and aheadOf(s2) > -0.3 then
                local k = max(2, ceil(r * abs(sweep) / arcStep))
                local pts = {}
                for i = 0, k do
                    local ang = a0 + sweep * i / k
                    pts[#pts + 1] = { cx + r * cos(ang), cz + r * sin(ang) }
                end
                pts[#pts] = { px, pz }
                return { pts = pts, a = a, s = s2 % L, kind = "fillet" }
            end
        end
    end

    -- 2. arc - straight - arc, searched over where it leaves the line (a)
    --    and where it joins the path (d ahead of J)
    local h0 = atan2(uz, ux)
    local function csc(a, d)
        local sx, sz = jx - ux * a, jz - uz * a
        local c1x, c1z = sx + nx * r, sz + nz * r
        local s2 = sJ + sense * d
        local ex, ez, etx, etz = G.pointAt(path, s2)
        etx, etz = etx * sense, etz * sense
        local n2x, n2z
        if left then n2x, n2z = -etz, etx else n2x, n2z = etz, -etx end
        local c2x, c2z = ex + n2x * r, ez + n2z * r
        local wx, wz = c2x - c1x, c2z - c1z
        local wl = sqrt(wx * wx + wz * wz)
        local h1 = atan2(etz, etx)
        local t, q
        if wl < 1e-6 then
            t = left and G.mod2pi(h1 - h0) or G.mod2pi(h0 - h1)
            q = 0
        else
            local hs = atan2(wz, wx)
            if left then t, q = G.mod2pi(hs - h0), G.mod2pi(h1 - hs)
            else t, q = G.mod2pi(h0 - hs), G.mod2pi(hs - h1) end
        end
        local net = left and G.mod2pi(h1 - h0) or G.mod2pi(h0 - h1)
        if t + q > net + 0.05 or t + q > pi * 1.1 then return nil end   -- would loop
        return { cost = r * (t + q) + wl - a - d, t = t, q = q, wl = wl, sx = sx, sz = sz, s2 = s2, a = a, d = d }
    end
    local function curvePts(c)
        local word = left and "LSL" or "RSR"
        local p = { word = word, t = c.t, p = c.wl / r, q = c.q, r = r, x0 = c.sx, z0 = c.sz, h0 = h0 }
        return (G.dubinsPoints(p, arcStep))
    end
    local function clean(c)
        local pts = curvePts(c)
        for i = 2, #pts - 1 do
            local sd = sideOf(segs, pts[i][1], pts[i][2], 0.05)
            if sd ~= 0 and sd ~= approachSide then return nil end
        end
        return pts
    end
    local cands = {}
    local dMax = 6 * r + 10
    for a = 0, aMax, 1 do
        for d = 0, dMax, 1 do
            local c = csc(a, d)
            if c ~= nil then cands[#cands + 1] = c end
        end
    end
    table.sort(cands, function(p, q)
        if abs(p.cost - q.cost) > 0.01 then return p.cost < q.cost end
        return p.a + p.d < q.a + q.d
    end)
    local best, bestPts = nil, nil
    for i = 1, min(#cands, 40) do
        local pts = clean(cands[i])
        if pts ~= nil then best, bestPts = cands[i], pts; break end
    end
    if best == nil then return nil end
    -- refine around the best grid point
    for da = -1, 1, 0.25 do
        for dd = -1, 1, 0.25 do
            local a, d = best.a + da, best.d + dd
            if a >= 0 and d >= 0 then
                local c = csc(a, d)
                if c ~= nil and c.cost < best.cost - 0.01 then
                    local pts = clean(c)
                    if pts ~= nil then best, bestPts = c, pts end
                end
            end
        end
    end
    local ex, ez = G.pointAt(path, best.s2)
    bestPts[#bestPts] = { ex, ez }
    return { pts = bestPts, a = best.a, s = best.s2 % L, kind = "csc" }
end
