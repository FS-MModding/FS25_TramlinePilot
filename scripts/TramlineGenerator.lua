--[[
    TramlineGenerator.lua
    Builds the complete tramline layout of a field from its outline.

    Everything the worker drives on is part of the layout, so it can all be
    drawn and cut into the ground:
      * headland rings   - constant distance from the field edge
                           ((i - 0.5) * spacing for headland pass i), corners
                           rounded to the turning radius
      * main tramlines   - parallel, 'spacing' apart, ending exactly on the
                           innermost ring
      * connections      - between neighbouring lines at alternating ends:
                           a curve onto the headland ring, along it, and a
                           curve back into the next line; only where those
                           curves would not fit (turn too tight) a loop out in
                           the headland (Dubins path, must stay in the field)
      * junctions        - curves between the ring and the free line ends
                           (where the worker enters and leaves each block)

    set: { fieldId, dx, dz, ax, az, offset, spacing, headlands, turnRadius,
           turnStyle (1 = along the headland tramline, loops only when too
           tight; 2 = loops wherever they fit), anchorToEdge,
           preview (true = rings and lines only, for the menu) }
    info: FieldScanner.scan() result (needs info.polygon)

    Output 'geo':
    {
        polygon, spacing, headlands, radius, dirX, dirZ,
        rings     = { [level] = { loop, ... } }    level 1 = outermost
        lines     = { { x1,z1,x2,z2, k, len, w0, w1, block } }  A end = x1,z1
                    w0/w1 = working stretch as distances from the A end
        blocks    = { { lines = {i,...}, conns = { conn, ... }, ends = {E1, E2} } }
                    conn = { kind = "loop"|"ring", from=i, to=i+1, side="A"|"B",
                             pts (loop) | exitArc, entryArc, ringLoop (ring) }
                    E = { line=i, side="A"|"B", x, z, arcs = { [1]=arc for ring
                          travel +1, [-1]=arc for travel -1 } }  arc runs ring -> line
        bounds, warnings,
        headland = flat segment list of the rings (drawing)
        extras   = flat segment list of loops, curves, junctions (drawing)
        curves   = the same loops / curves / junctions as polylines (painting)
        spokes   = crossings between headland passes (see buildSpokes)
    }
]]

TramlineGenerator = {}

local G = TramlineGeom
local sqrt, abs, min, max, floor, ceil = math.sqrt, math.abs, math.min, math.max, math.floor, math.ceil

TramlineGenerator.MIN_LINE_LEN   = 12    -- shorter pieces are dropped
TramlineGenerator.EDGE_MARGIN    = 1.5   -- loops keep this far from the field edge
TramlineGenerator.MAX_LINES      = 400
TramlineGenerator.SWATH_MIN      = 0.2   -- share of the boom over unsprayed main area to spray
TramlineGenerator.MIN_WORK_LEN   = 5     -- lines spraying less than this are dropped
TramlineGenerator.CORNER_MIN_DEG = 50    -- headland bends sharper than this are corners
TramlineGenerator.CORNER_BOOM    = 1.5   -- m: least the boom is behind the vehicle (how far the corner tramlines reach)
TramlineGenerator.CORNER_RUNOUT  = 5     -- m past the corner curve before backing up

-- The rounded corners of one headland ring (closed loop lp, rounded to R) and,
-- for each, the way of backing into it. s = spacing (the pass is s wide).
--   { sA, sB = curve start / end (along increasing s), P = sharp corner,
--     a, b = directions before / after the curve (increasing s),
--     legA = { A0, P, KA }, legB = { B0, P, KB }  (tramlines into the corner),
--     m = { [sense] = { sStart, sQ, sResume, Q, R0 (= B0 / A0), P, dir, u, emax } } }
-- Travelling +s: round the curve to sQ (just past B0), back along the next
-- side (direction dir) through P until the boom reaches the outer edge of
-- the pass - u metres past P, less the boom's distance behind the vehicle,
-- at most emax (the end of the corner tramline) - then spray out of the
-- corner back to B0 and on (and the other way round for -s).
function TramlineGenerator.findCorners(lp, R, s, poly, edgeIdx)
    local out = {}
    local rp = G.makePath(lp, true)
    local L = rp.length
    local step = 0.5
    local n = floor(L / step)
    if n < 16 then return out end
    local orient = G.signedArea(lp) > 0 and 1 or -1
    local hs = {}
    for k = 0, n - 1 do
        local _, _, tx, tz = G.pointAt(rp, k * step)
        hs[k] = G.atan2(tz, tx)
    end
    -- turning per metre over a 2 m window (the ring is a polyline: its
    -- heading only changes at the vertices)
    local w = 2
    local curv = {}
    for k = 0, n - 1 do curv[k] = G.wrapPi(hs[(k + w) % n] - hs[(k - w) % n]) * orient / (2 * w * step) end
    local thr = 0.5 / R
    local start = nil
    for k = 0, n - 1 do if curv[k] < thr then start = k; break end end
    if start == nil then return out end
    local found = {}
    local k = 0
    while k < n do
        if curv[(start + k) % n] >= thr then
            local j, turn = k, 0
            while j < n and curv[(start + j) % n] >= thr do
                turn = turn + curv[(start + j) % n] * step
                j = j + 1
            end
            if turn >= math.rad(TramlineGenerator.CORNER_MIN_DEG) and turn < math.rad(170) then
                found[#found + 1] = { sA = ((start + k) * step) % L, sB = ((start + j) * step) % L, turn = turn }
            end
            k = j
        else
            k = k + 1
        end
    end
    local function inside(x, z)
        return G.pointInPolygon(poly, x, z) and G.nearestEdgeDist(edgeIdx, x, z, 3) >= 1.0
    end
    local runout = TramlineGenerator.CORNER_RUNOUT
    for ci, c in ipairs(found) do
        local A0x, A0z = G.pointAt(rp, c.sA)
        local B0x, B0z = G.pointAt(rp, c.sB)
        local _, _, ax, az = G.pointAt(rp, c.sA - 1.0)
        local _, _, bx, bz = G.pointAt(rp, c.sB + 1.0)
        local cr = ax * bz - az * bx
        if abs(cr) > 0.2 then
            -- sharp corner P: where the two straight sides meet
            local t1 = ((B0x - A0x) * bz - (B0z - A0z) * bx) / cr
            local Px, Pz = A0x + ax * t1, A0z + az * t1
            local t2 = (B0x - Px) * bx + (B0z - Pz) * bz
            if t1 > -0.5 and t2 > -0.5 and t1 < 3 * R + 5 and t2 < 3 * R + 5 then
                -- how far past P to back up: until the boom reaches the outer
                -- edge of this pass (half the spacing out from the tramline)
                local u = 0.5 * s / abs(cr)
                local function leg(dx, dz)
                    local e = max(0, u - TramlineGenerator.CORNER_BOOM)
                    while e > 0 and not inside(Px + dx * e, Pz + dz * e) do e = e - 0.5 end
                    return { Px + dx * max(e, 0), Pz + dz * max(e, 0) }, max(e, 0)
                end
                c.u = u
                local KB, eB = leg(-bx, -bz)      -- travelling +s: back along side b
                local KA, eA = leg(ax, az)        -- travelling -s: back along side a
                c.P, c.a, c.b = { Px, Pz }, { ax, az }, { bx, bz }
                c.legA = { { A0x, A0z }, { Px, Pz }, KA }
                c.legB = { { B0x, B0z }, { Px, Pz }, KB }
                c.eA, c.eB = eA, eB
                -- room to run out past the curve before backing up
                local nxt = found[(ci % #found) + 1]
                local prv = found[((ci - 2) % #found) + 1]
                local afterB = (#found > 1) and ((nxt.sA - c.sB) % L) or L
                local beforeA = (#found > 1) and ((c.sA - prv.sB) % L) or L
                c.m = {}
                if afterB >= runout + 1 then
                    local sQ = (c.sB + runout) % L
                    local Qx, Qz = G.pointAt(rp, sQ)
                    c.m[1] = { sStart = c.sA, sQ = sQ, sResume = c.sB, Q = { Qx, Qz }, R0 = { B0x, B0z },
                               P = { Px, Pz }, dir = { -bx, -bz }, u = u, emax = eB, back = max(0, t2) }
                end
                if beforeA >= runout + 1 then
                    local sQ = (c.sA - runout) % L
                    local Qx, Qz = G.pointAt(rp, sQ)
                    c.m[-1] = { sStart = c.sB, sQ = sQ, sResume = c.sA, Q = { Qx, Qz }, R0 = { A0x, A0z },
                                P = { Px, Pz }, dir = { ax, az }, u = u, emax = eA, back = max(0, t1) }
                end
                out[#out + 1] = c
            end
        end
    end
    return out
end

-- Working stretches of one line, as distances from its A end:
-- boom on where the cross section of the boom (+-s/2) overlaps the work
-- region by at least SWATH_MIN * s. Coarse sampling + bisection at changes.
function TramlineGenerator.workStretches(workLoops, ox, oz, dx, dz, nx, nz, t0, t1, s)
    local len = t1 - t0
    if workLoops == nil then return { { 0, len } } end
    local need = TramlineGenerator.SWATH_MIN * s
    local function on(t)
        local px, pz = ox + dx * (t0 + t), oz + dz * (t0 + t)
        local tot = 0
        for _, lp in ipairs(workLoops) do
            for _, iv in ipairs(G.clipLine(lp, px, pz, nx, nz)) do
                local a, b = math.max(iv[1], -s * 0.5), math.min(iv[2], s * 0.5)
                if b > a then tot = tot + (b - a) end
            end
        end
        return tot >= need
    end
    local step = 4
    local n = math.max(2, math.ceil(len / step))
    local samples = {}
    for i = 0, n do samples[i] = on(len * i / n) end
    local out = {}
    local curStart = samples[0] and 0 or nil
    for i = 1, n do
        if samples[i] ~= samples[i - 1] then
            -- refine the change between i-1 and i
            local lo, hi = len * (i - 1) / n, len * i / n
            for _ = 1, 6 do
                local mid = (lo + hi) * 0.5
                if on(mid) == samples[i - 1] then lo = mid else hi = mid end
            end
            local tChange = (lo + hi) * 0.5
            if samples[i] then curStart = tChange else
                if curStart ~= nil then out[#out + 1] = { curStart, tChange } end
                curStart = nil
            end
        end
    end
    if curStart ~= nil then out[#out + 1] = { curStart, len } end
    return out
end

-- Connectors between headland passes: one straight spoke square across the
-- headland band, in the middle of the longest straight stretch of the
-- innermost ring (preferably a side the main tramlines run along, where no
-- lines end), with curves onto every ring it crosses.
--   trans[a][b][sense] (a < b): from ring a to ring b, travelling 'sense' on
--   both: { pts, sFrom (ring a), sTo (ring b), n1 = points of the first curve }
--   The way back (b -> a, travelling sense) is trans[a][b][-sense] reversed.
--   draw: polylines to show / paint
function TramlineGenerator.buildSpokes(geo, R, dirX, dirZ)
    local H = geo.headlands
    local paths = {}
    for h = 1, H do
        if geo.rings[h] == nil or #geo.rings[h] ~= 1 then return nil end
        paths[h] = G.makePath(geo.rings[h][1], true)
    end
    local ring = geo.rings[H][1]
    local n = #ring
    if n < 3 then return nil end
    local function segHeading(k)
        local a, b = ring[((k - 1) % n) + 1], ring[(k % n) + 1]
        return G.atan2(b[2] - a[2], b[1] - a[1]), G.dist(a, b)
    end
    local tol = math.rad(3)
    -- start scanning at a bend so no straight stretch is split in two
    local k0 = 1
    for k = 1, n do
        local hp = segHeading(k - 1 + n)
        local hk = segHeading(k)
        if abs(G.wrapPi(hk - hp)) > tol then k0 = k; break end
    end
    local need = 4 * R + 10
    local bestScore, bestS = -1, nil
    local cum = paths[H].cum
    local k = 0
    while k < n do
        local i = ((k0 - 1 + k) % n) + 1
        local h0, len = segHeading(i)
        local cnt = 1
        while k + cnt < n do
            local hj, lj = segHeading(((i - 1 + cnt) % n) + 1)
            if abs(G.wrapPi(hj - h0)) > tol then break end
            len = len + lj
            cnt = cnt + 1
        end
        if len >= need then
            local ux, uz = math.cos(h0), math.sin(h0)
            local par = abs(ux * dirX + uz * dirZ) > math.cos(math.rad(15))
            local score = len * (par and 2 or 1)
            if score > bestScore then bestScore, bestS = score, cum[i] + len * 0.5 end
        end
        k = k + cnt
    end
    if bestS == nil then return nil end

    local px, pz, tx, tz = G.pointAt(paths[H], bestS)
    local Nx, Nz = tz, -tx                      -- outward (ring is CCW)
    local T, Q = {}, {}
    T[H] = 0
    Q[H] = { px, pz, (G.project(paths[H], px, pz)) }
    for h = H - 1, 1, -1 do
        local th = nil
        for _, iv in ipairs(G.clipLine(geo.rings[h][1], px, pz, Nx, Nz)) do
            if iv[1] < 0 and iv[2] > 0 then th = iv[2] end
        end
        if th == nil or th < T[h + 1] + 0.5 * geo.spacing or th > T[h + 1] + 1.6 * geo.spacing then return nil end
        T[h] = th
        local qx, qz = px + Nx * th, pz + Nz * th
        Q[h] = { qx, qz, (G.project(paths[h], qx, qz)) }
    end
    -- curves: jOut[h][sense] spoke arriving at ring h from inside (heading out),
    --         jIn[h][sense]  spoke arriving at ring h from outside (heading in)
    local jOut, jIn = {}, {}
    for h = 1, H do
        jOut[h], jIn[h] = {}, {}
        for _, sense in ipairs({ 1, -1 }) do
            if h < H then jOut[h][sense] = G.junction(paths[h], Q[h][3], Q[h][1], Q[h][2], Nx, Nz, sense, R, 0.5) end
            if h > 1 then jIn[h][sense] = G.junction(paths[h], Q[h][3], Q[h][1], Q[h][2], -Nx, -Nz, sense, R, 0.5) end
        end
    end
    local out = { trans = {}, draw = {}, Q = Q }
    local used = {}
    local function use(j) if j ~= nil and not used[j] then used[j] = true; out.draw[#out.draw + 1] = j.pts end end
    out.draw[#out.draw + 1] = { { Q[1][1], Q[1][2] }, { Q[H][1], Q[H][2] } }
    for a = 1, H - 1 do
        out.trans[a] = {}
        for b = a + 1, H do
            out.trans[a][b] = {}
            for _, sense in ipairs({ 1, -1 }) do
                local jo, ji = jOut[a][-sense], jIn[b][sense]
                local tr = nil
                if jo ~= nil and ji ~= nil and jo.a + ji.a <= (T[a] - T[b]) - 0.1 then
                    local pts = G.reverse(jo.pts)
                    local n1 = #pts
                    for _, p in ipairs(ji.pts) do pts[#pts + 1] = p end
                    tr = { pts = pts, sFrom = jo.s, sTo = ji.s, n1 = n1 }
                    use(jo); use(ji)
                else
                    -- band too narrow for square curves: S-curve between the rings
                    for _, f in ipairs({ 1, 1.5, 2, 3 }) do
                        local s0 = Q[a][3] - sense * f * R
                        local s1 = Q[b][3] + sense * f * R
                        local x0, z0, t0x, t0z = G.pointAt(paths[a], s0)
                        local x1, z1, t1x, t1z = G.pointAt(paths[b], s1)
                        local path = G.dubins(x0, z0, G.atan2(t0z * sense, t0x * sense),
                                              x1, z1, G.atan2(t1z * sense, t1x * sense), R)
                        if path ~= nil and path.t + path.q + (string.sub(path.word, 2, 2) ~= "S" and path.p or 0) < math.pi * 1.2 then
                            local pts = G.dubinsPoints(path, 0.5)
                            tr = { pts = pts, sFrom = s0 % paths[a].length, sTo = s1 % paths[b].length, n1 = math.floor(#pts / 2) }
                            out.draw[#out.draw + 1] = pts
                            break
                        end
                    end
                end
                if tr == nil then return nil end
                out.trans[a][b][sense] = tr
            end
        end
    end
    return out
end

local function segmentsOf(pts, closed, out)
    local n = #pts
    local last = closed and n or n - 1
    for i = 1, last do
        local a, b = pts[i], pts[(i % n) + 1]
        out[#out + 1] = { a[1], a[2], b[1], b[2] }
    end
end

-- all edges of several loops as one polygon edge list (for even-odd clipping)
local function clipMulti(loops, ox, oz, dx, dz)
    local all = {}
    for _, lp in ipairs(loops) do
        local iv = G.clipLine(lp, ox, oz, dx, dz)
        for _, v in ipairs(iv) do all[#all + 1] = v end
    end
    table.sort(all, function(a, b) return a[1] < b[1] end)
    return all
end

local function insideAny(loops, x, z)
    for _, lp in ipairs(loops) do if G.pointInPolygon(lp, x, z) then return true end end
    return false
end

function TramlineGenerator.generate(set, info)
    local geo = { lines = {}, blocks = {}, rings = {}, headland = {}, extras = {}, curves = {}, warnings = {} }
    if info == nil or info.polygon == nil then
        geo.warnings[#geo.warnings + 1] = "no field outline"
        return geo
    end
    local poly = G.makeCCW(info.polygon)
    geo.polygon = poly
    local s = max(set.spacing or 24, 3)
    local H = max(0, floor(set.headlands or 1))
    local R = max(set.turnRadius or 7, 2)
    geo.spacing, geo.headlands, geo.radius = s, H, R

    local dx, dz = G.normalize(set.dx or 1, set.dz or 0)
    if dx == 0 and dz == 0 then dx, dz = 1, 0 end
    local nx, nz = -dz, dx
    geo.dirX, geo.dirZ = dx, dz

    local edgeIdx = G.buildEdgeIndex(poly, 8)

    ---------------------------------------------------------------------------
    -- 1. headland rings
    ---------------------------------------------------------------------------
    for i = 1, H do
        -- (i - 0.5) * s in from the edge; convex corners rounded to the turn
        -- radius (offset in by d + R, back out by R), so the worker can follow
        -- every corner and the ring never comes closer to the edge than d
        local raw = G.offsetInward(poly, (i - 0.5) * s + R, 0.5)
        local loops = {}
        for _, lp in ipairs(raw) do
            for _, op in ipairs(G.offsetOutward(G.simplify(lp, 0.02, true), R, 0.5)) do
                local simple = G.simplify(op, 0.03, true)
                local f = G.fillet(simple, R, true, 8, 1.0)
                if #f >= 3 then loops[#loops + 1] = G.makeCCW(f) end
            end
        end
        table.sort(loops, function(a, b) return G.signedArea(a) > G.signedArea(b) end)
        geo.rings[i] = loops
        for _, lp in ipairs(loops) do segmentsOf(lp, true, geo.headland) end
    end
    local inner = (H > 0) and geo.rings[H] or nil
    if H > 0 and (inner == nil or #inner == 0) then
        geo.warnings[#geo.warnings + 1] = "field too narrow for the headland"
        return geo
    end

    -- region the main lines are clipped to: the innermost ring, or (no
    -- headland) the field shrunk so turn loops fit inside it
    local clipLoops = inner
    if H == 0 then
        clipLoops = G.offsetInward(poly, R + 2, 0.5)
        if #clipLoops == 0 then
            geo.warnings[#geo.warnings + 1] = "field too small"
            return geo
        end
    end
    -- region where the boom is on: inside the inner edge of the headland band
    local workLoops = nil
    if H > 0 then
        workLoops = {}
        for _, lp in ipairs(G.offsetInward(poly, H * s, 0.5)) do
            workLoops[#workLoops + 1] = G.simplify(lp, 0.05, true)
        end
    end

    ---------------------------------------------------------------------------
    -- 2. line positions
    ---------------------------------------------------------------------------
    local cMin, cMax = math.huge, -math.huge
    for _, p in ipairs(poly) do
        local c = p[1] * nx + p[2] * nz
        if c < cMin then cMin = c end
        if c > cMax then cMax = c end
    end
    local cFirst
    if set.anchorToEdge then
        -- the longest field edge running (nearly) parallel to the lines; the
        -- first main tramline sits H*s + s/2 in from it, like a real first pass
        local simple = G.simplify(poly, 1.5, true)
        local n = #simple
        local bestL, bestC, inwardSign = -1, nil, 1
        local ccx = (info.centerX or 0) * nx + (info.centerZ or 0) * nz
        for i = 1, n do
            local a, b = simple[i], simple[(i % n) + 1]
            local ex, ez, l = G.normalize(b[1] - a[1], b[2] - a[2])
            if l > bestL and abs(ex * dx + ez * dz) > math.cos(math.rad(12)) then
                bestL = l
                bestC = ((a[1] + b[1]) * 0.5) * nx + ((a[2] + b[2]) * 0.5) * nz
                inwardSign = (ccx >= bestC) and 1 or -1
            end
        end
        if bestC ~= nil then
            cFirst = bestC + inwardSign * (H * s + s * 0.5)
        else
            cFirst = cMin + H * s + s * 0.5
        end
    else
        cFirst = (set.ax or 0) * nx + (set.az or 0) * nz + (set.offset or 0)
    end
    local kMin = ceil((cMin - cFirst) / s) - 1
    local kMax = floor((cMax - cFirst) / s) + 1

    ---------------------------------------------------------------------------
    -- 3. clip lines; group into blocks (boustrophedon)
    ---------------------------------------------------------------------------
    local perK = {}
    for k = kMin, kMax do
        local c = cFirst + k * s
        local ox, oz = nx * c, nz * c
        local iv = clipMulti(clipLoops, ox, oz, dx, dz)
        local list = {}
        for _, v in ipairs(iv) do
            if v[2] - v[1] >= TramlineGenerator.MIN_LINE_LEN then
                local line = { ox + dx * v[1], oz + dz * v[1], ox + dx * v[2], oz + dz * v[2],
                               k = k, t0 = v[1], t1 = v[2], len = v[2] - v[1], c = c }
                -- boom on wherever at least SWATH_MIN of the boom width is over
                -- the main (not headland) area; lines with no such stretch lie
                -- in the headland band and are dropped
                line.work = TramlineGenerator.workStretches(workLoops, ox, oz, dx, dz, nx, nz, v[1], v[2], s)
                local wl = 0
                for _, w in ipairs(line.work) do wl = wl + (w[2] - w[1]) end
                if wl >= TramlineGenerator.MIN_WORK_LEN then
                    line.w0 = line.work[1][1]
                    line.w1 = line.work[#line.work][2]
                    list[#list + 1] = line
                end
            end
        end
        if #list > 0 then perK[#perK + 1] = { k = k, list = list } end
        if #geo.lines + #list > TramlineGenerator.MAX_LINES then break end
        for _, l in ipairs(list) do geo.lines[#geo.lines + 1] = l end
    end
    if #geo.lines == 0 then
        geo.warnings[#geo.warnings + 1] = "no tramlines fit this field"
        return geo
    end
    for i, l in ipairs(geo.lines) do l.id = i end

    local open = {}     -- open blocks keyed by their last line
    local function overlap(a, b) return min(a.t1, b.t1) - max(a.t0, b.t0) end
    local prevList = nil
    for _, entry in ipairs(perK) do
        local newOpen = {}
        for _, l in ipairs(entry.list) do
            local cont = nil
            if prevList ~= nil and prevList.k == entry.k - 1 then
                -- continue a block if exactly one previous line overlaps us and
                -- we are the only line overlapping it
                local hits = {}
                for _, p in ipairs(prevList.list) do if overlap(l, p) > 0 then hits[#hits + 1] = p end end
                if #hits == 1 then
                    local back = 0
                    for _, q in ipairs(entry.list) do if overlap(q, hits[1]) > 0 then back = back + 1 end end
                    if back == 1 and open[hits[1]] ~= nil then cont = open[hits[1]] end
                end
            end
            if cont == nil then
                cont = { lines = {} }
                geo.blocks[#geo.blocks + 1] = cont
            end
            cont.lines[#cont.lines + 1] = l.id
            l.block = #geo.blocks
            newOpen[l] = cont
        end
        open = newOpen
        prevList = entry
    end
    -- fix block index on lines (blocks may have been created out of order)
    for b, blk in ipairs(geo.blocks) do for _, id in ipairs(blk.lines) do geo.lines[id].block = b end end

    if set.preview then
        -- menu preview: rings and lines are enough (turns take the time)
        local x0, z0, x1, z1 = G.bounds(poly)
        geo.bounds = { x0, z0, x1, z1 }
        return geo
    end

    ---------------------------------------------------------------------------
    -- 4. turns between the lines of a block, curves at the free ends
    ---------------------------------------------------------------------------
    local ringPaths = {}
    if inner ~= nil then
        for i, lp in ipairs(inner) do ringPaths[i] = G.makePath(lp, true) end
    end
    local function nearestRing(x, z)
        local best, bi, bs = math.huge, nil, nil
        for i, rp in ipairs(ringPaths) do
            local sp, _, _, d = G.project(rp, x, z)
            if d < best then best, bi, bs = d, i, sp end
        end
        return bi, bs, best
    end
    local function endPoint(l, side)
        if side == "A" then return l[1], l[2], -dx, -dz end   -- outward direction
        return l[3], l[4], dx, dz
    end
    local function loopOk(pts)
        local m = #pts
        for i = 1, m do
            local p = pts[i]
            if not G.pointInPolygon(poly, p[1], p[2]) then return false end
            if G.nearestEdgeDist(edgeIdx, p[1], p[2], TramlineGenerator.EDGE_MARGIN) < TramlineGenerator.EDGE_MARGIN then return false end
            -- must not cut into the cropped middle (allow the first/last 2 m)
            if inner ~= nil and i > 4 and i < m - 3 and insideAny(inner, p[1], p[2]) then return false end
        end
        return true
    end
    -- curve from a line end out onto the ring, then travelling 'sense' on it
    -- { pts (line -> ring), a = metres of line it cuts off, s = ring position, ring }
    local jcache = {}
    local function toRing(l, side, sense)
        local key = l.id .. side .. sense
        local j = jcache[key]
        if j == nil then
            j = false
            local ex, ez, ux, uz = endPoint(l, side)
            local ri, sJ, d = nearestRing(ex, ez)
            if ri ~= nil and d < 0.5 then
                local res = G.junction(ringPaths[ri], sJ, ex, ez, ux, uz, sense, R, 0.5)
                if res ~= nil and res.a <= l.len * 0.5 then
                    res.ring = ri
                    j = res
                end
            end
            jcache[key] = j
        end
        return j or nil
    end
    local function drawn(pts)
        segmentsOf(pts, false, geo.extras)
        geo.curves[#geo.curves + 1] = pts
    end
    -- the way off each line end onto the ring, per direction of travel on it
    -- (only curves that are part of the layout): l.exits[side][sense]
    local function recordExit(l, side, sense, j)
        l.exits = l.exits or { A = {}, B = {} }
        l.exits[side][sense] = j
    end
    local drawnJ = {}
    local function drawJ(j) if j ~= nil and not drawnJ[j] then drawnJ[j] = true; drawn(j.pts) end end

    -- turn along the headland tramline: curve onto it, along it, curve into the
    -- next line; nil where the two curves would overlap (turn too tight)
    local function ringTurn(la, lb, side)
        if inner == nil then return nil end
        local ax_, az_ = endPoint(la, side)
        local bx_, bz_ = endPoint(lb, side)
        local ri, sa = nearestRing(ax_, az_)
        local rib, sb = nearestRing(bx_, bz_)
        if ri == nil or ri ~= rib then return nil end
        local L = ringPaths[ri].length
        local fwd = (sb - sa) % L
        local sense = (fwd <= L * 0.5) and 1 or -1
        local dAB = (sense > 0) and fwd or (L - fwd)
        local ex = toRing(la, side, sense)        -- line A -> ring
        local en = toRing(lb, side, -sense)       -- reversed: ring -> line B
        if ex == nil or en == nil then return nil end
        local function signedAhead(s0, s1)
            local d = ((s1 - s0) * sense) % L
            if d > L * 0.5 then d = d - L end
            return d
        end
        local dEx = signedAhead(sa, ex.s)
        local dEn = signedAhead(en.s, sb)
        if dEx < -0.3 or dEn < -0.3 or max(dEx, 0) + max(dEn, 0) > dAB + 0.05 then return nil end
        return { kind = "ring", from = la.id, to = lb.id, side = side, sense = sense, ringIndex = ri,
                 exitArc = ex.pts, exitTrim = ex.a, s0 = ex.s,
                 entryArc = G.reverse(en.pts), entryTrim = en.a, s1 = en.s, jEx = ex, jEn = en }
    end
    -- loop out into the headland (or the field margin without headland)
    local function loopTurn(la, lb, side, forced)
        local ax_, az_, ux, uz = endPoint(la, side)
        local bx_, bz_ = endPoint(lb, side)
        local h0, h1 = G.atan2(uz, ux), G.atan2(-uz, -ux)
        local path = G.dubins(ax_, az_, h0, bx_, bz_, h1, R, (not forced) and loopOk or nil, 1.0)
        if path == nil then return nil end
        return { kind = "loop", from = la.id, to = lb.id, side = side, pts = (G.dubinsPoints(path, 0.5)), forced = forced }
    end

    local style = set.turnStyle or 1
    local loopsUsed, ringUsed, forcedUsed = 0, 0, 0
    for _, blk in ipairs(geo.blocks) do
        blk.conns = {}
        local m = #blk.lines
        for j = 1, m - 1 do
            local la, lb = geo.lines[blk.lines[j]], geo.lines[blk.lines[j + 1]]
            local side = (j % 2 == 1) and "B" or "A"
            local conn
            if style == 2 then
                conn = loopTurn(la, lb, side) or ringTurn(la, lb, side)
            else
                conn = ringTurn(la, lb, side) or loopTurn(la, lb, side)
            end
            if conn == nil then
                conn = loopTurn(la, lb, side, true)
                forcedUsed = forcedUsed + 1
            end
            if conn.kind == "loop" then loopsUsed = loopsUsed + 1 else ringUsed = ringUsed + 1 end
            blk.conns[j] = conn
            if conn.pts ~= nil then drawn(conn.pts) end
            if conn.jEx ~= nil then
                drawJ(conn.jEx); drawJ(conn.jEn)
                recordExit(la, side, conn.sense, conn.jEx)
                recordExit(lb, side, -conn.sense, conn.jEn)
            end
        end
        -- free ends: line 1 at side A, line m at the side the serpentine leaves it
        local first, last = geo.lines[blk.lines[1]], geo.lines[blk.lines[m]]
        local lastSide = (m % 2 == 1) and "B" or "A"
        local firstSide = "A"
        blk.ends = {}
        for e, spec in ipairs({ { first, firstSide }, { last, lastSide } }) do
            local l, side = spec[1], spec[2]
            local ex, ez, ux, uz = endPoint(l, side)
            -- entry[sense]: ring (travelling sense) -> line;  exit[sense]: line -> ring, then sense
            local E = { line = l.id, side = side, x = ex, z = ez, ux = ux, uz = uz, entry = {}, exit = {} }
            if inner ~= nil then
                for _, sense in ipairs({ 1, -1 }) do
                    local jn = toRing(l, side, sense)
                    if jn ~= nil then
                        E.exit[sense] = { pts = jn.pts, s = jn.s, trim = jn.a }
                        E.entry[-sense] = { pts = G.reverse(jn.pts), s = jn.s, trim = jn.a }
                        E.ring = jn.ring
                        drawJ(jn)
                        recordExit(l, side, sense, jn)
                    end
                end
            end
            blk.ends[e] = E
        end
    end

    ---------------------------------------------------------------------------
    -- 4b. headland corners (backing into them: tramlines on into the corner)
    ---------------------------------------------------------------------------
    geo.corners = {}
    geo.cornerStyle = set.corners or "round"
    local cornerCount = 0
    for level = 1, H do
        geo.corners[level] = {}
        for idx, lp in ipairs(geo.rings[level] or {}) do
            local list = TramlineGenerator.findCorners(lp, R, s, poly, edgeIdx)
            geo.corners[level][idx] = list
            if geo.cornerStyle == "reverse" then
                for _, c in ipairs(list) do
                    if c.m[1] ~= nil then drawn(c.legB) end
                    if c.m[-1] ~= nil then drawn(c.legA) end
                    cornerCount = cornerCount + 1
                end
            end
        end
    end
    geo.cornerCount = cornerCount

    ---------------------------------------------------------------------------
    -- 5. connectors between headland passes (2+ passes)
    ---------------------------------------------------------------------------
    if H >= 2 then
        geo.spokes = TramlineGenerator.buildSpokes(geo, R, dx, dz)
        if geo.spokes == nil then
            geo.warnings[#geo.warnings + 1] = "no room for a crossing between the headland passes"
        else
            for _, pts in ipairs(geo.spokes.draw) do drawn(pts) end
        end
    end

    local x0, z0, x1, z1 = G.bounds(poly)
    geo.bounds = { x0, z0, x1, z1 }
    if forcedUsed > 0 then
        geo.warnings[#geo.warnings + 1] = string.format("%d turn(s) do not fit the headland - use a smaller turn radius or more headland passes", forcedUsed)
    end
    geo.loopCount, geo.ringTurnCount, geo.forcedCount = loopsUsed, ringUsed, forcedUsed
    TramlineLog.info("[TramlinePilot] field %d: %d tramlines in %d block(s), %d headland ring(s), %d loop turns, %d headland turns",
        set.fieldId or -1, #geo.lines, #geo.blocks, H, loopsUsed, ringUsed)
    return geo
end
