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
           headlandDir ("either" | "cw" | "acw": with cw / acw the worker goes
           round the headland that way, turns onto and off it only that way
           (one curve at each end of a block) and backs into each corner from
           one side; the tramlines are still done up and down),
           strip (m, > 0: no headland tramline, the crop grows to a strip of
           bare soil this wide where the tramlines end; the worker folds and
           turns round on it - see planStripTurn),
           preview (true = rings and lines only, for the menu) }
    info: FieldScanner.scan() result (needs info.polygon)

    Output 'geo':
    {
        polygon, spacing, headlands, radius, dirX, dirZ,
        rings     = { [level] = { loop, ... } }    level 1 = outermost
        lines     = { { x1,z1,x2,z2, k, len, w0, w1, block } }  A end = x1,z1
                    w0/w1 = working stretch as distances from the A end
        blocks    = { { lines = {i,...}, conns = { conn, ... }, ends = {E1, E2} } }
                    conn = { kind = "loop"|"ring"|"strip", from=i, to=i+1, side="A"|"B",
                             pts (loop) | exitArc, entryArc, ringLoop (ring) |
                             turn (strip: planStripTurn for the default vehicle) }
                    E = { line=i, side="A"|"B", x, z, entry = { [sense] = curve ring
                          (travelling sense) -> line }, exit = { [sense] = curve line ->
                          ring, then sense } }; with a headland direction one end
                          has only entry[way] and the other only exit[way]
        bounds, warnings,
        headland = flat segment list of the rings (drawing)
        extras   = flat segment list of loops, curves, junctions (drawing)
        curves   = the same loops / curves / junctions as polylines (painting)
        spokes   = crossings between headland passes (see buildSpokes)
    }
]]

TramlineGenerator = {}

local G = TramlineGeom

-- The way along headland loop lp (as a ring path's sense) that is clockwise
-- or anticlockwise ("cw" / "acw") seen from above, north up. z points south,
-- so a loop with positive signed area in x/z runs clockwise on the map.
function TramlineGenerator.dirSense(lp, dir)
    local cw = G.signedArea(lp) > 0 and 1 or -1
    return (dir == "acw") and -cw or cw
end
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

-------------------------------------------------------------------------------
-- Soil strip at the ends (crops grown to the end of the field: potatoes,
-- onions). No headland tramline: the tramlines run to a strip of bare soil
-- 'W' metres wide wherever they end (along the sides the crop goes right to
-- the edge). At the end of each tramline the worker drives on until the boom
-- has left the crop, folds it, turns round on the strip - forwards, or with
-- a three-point turn where it is tight - and unfolds lined up with the next
-- tramline, the boom just over the edge of the crop.
-------------------------------------------------------------------------------

TramlineGenerator.STRIP_WIDTHS      = { 6, 8, 10, 12, 14, 16, 18, 20, 24, 30 }   -- m (menu)
TramlineGenerator.STRIP_END_ANGLE   = math.rad(12)   -- outline edges this far off the tramlines are ends
TramlineGenerator.STRIP_CROP_MARGIN = 1.75  -- m the wheels' centre line keeps off the crop (half a wheel track and some)
TramlineGenerator.STRIP_EDGE_MARGIN = 1.0   -- m the vehicle's front and back keep from the field edge
TramlineGenerator.STRIP_LEAD        = 4.0   -- m lined up with the next tramline before the crop (the front wheels in its track)
TramlineGenerator.STRIP_BOOM_BACK   = 3.0   -- m the boom is behind the vehicle if the vehicle doesn't say
TramlineGenerator.STRIP_FOLD_EXTRA  = 0.5   -- m on past where the boom leaves the crop before folding
TramlineGenerator.STRIP_STEP        = 0.5   -- m between the points of a turn
TramlineGenerator.STRIP_SHIFT_COST  = 15    -- m a stop to change direction is worth
TramlineGenerator.STRIP_RUN_OUT     = 2.0   -- m the boom goes on past the last of the crop (anywhere across it) before folding, room allowing
TramlineGenerator.STRIP_RUN_OUT_MIN = 0.6   -- m of that kept where a shorter run out lets it turn (or turn without backing up)
TramlineGenerator.STRIP_RUN_IN      = 1.0   -- m before the first of the crop the boom comes on, starting a tramline
TramlineGenerator.STRIP_STOP_TOL    = 0.3   -- m short of a fold stop the vehicle may halt (TramlineRoute.ACT_REACH)
-- the vehicle as the turns see it: from the point it steers by (the rear
-- axle on most machines) to its front axle, front end and back end, and to
-- its rear axle where that is behind the point (a machine that steers from
-- its middle)
TramlineGenerator.STRIP_WHEELBASE   = 4.0
TramlineGenerator.STRIP_FRONT       = 5.5
TramlineGenerator.STRIP_REAR        = 2.0

-- The ends of the field for a soil strip W wide with tramlines running
-- (dx,dz) along the CCW outline poly: which of its edges are ends - judged
-- by the way the outline runs there overall (simplified, so the steps of a
-- traced edge don't count), at least STRIP_END_ANGLE off the tramlines -
-- and an index of them.
function TramlineGenerator.stripContext(poly, dx, dz, W, edgeIdx)
    local n = #poly
    local sinMin = math.sin(TramlineGenerator.STRIP_END_ANGLE)
    local keptAt = {}
    for _, p in ipairs(G.simplify(poly, 2.0, true)) do keptAt[p] = true end
    -- the simplified edge each edge of the outline belongs to: from the last
    -- point kept before it to the next one kept after it
    local firstKept = nil
    for i = 1, n do if keptAt[poly[i]] then firstKept = i; break end end
    local isEnd = {}
    if firstKept == nil then
        for i = 1, n do isEnd[i] = true end
    else
        local i = firstKept
        repeat
            local j = (i % n) + 1
            while not keptAt[poly[j]] do j = (j % n) + 1 end
            local a, b = poly[i], poly[j]
            local ex, ez, l = G.normalize(b[1] - a[1], b[2] - a[2])
            local e = l > 0.01 and abs(ex * dz - ez * dx) >= sinMin
            local k = i
            repeat isEnd[k] = e; k = (k % n) + 1 until k == j
            i = j
        until i == firstKept
    end
    local sp = poly
    -- where the outline bends into the field between two end edges, the
    -- strip goes round the bend (distance to the corner itself: inside the
    -- field there, neither edge is straight across); elsewhere it is just the
    -- band along each end edge
    local roundAt = {}
    for i = 1, n do
        local prev = ((i - 2) % n) + 1
        if isEnd[i] and isEnd[prev] then
            local a, b, c = sp[prev], sp[i], sp[(i % n) + 1]
            roundAt[i] = (b[1] - a[1]) * (c[2] - b[2]) - (b[2] - a[2]) * (c[1] - b[1]) < 0
        end
    end
    return { W = W, sp = sp, isEnd = isEnd, roundAt = roundAt, edgeIdx = edgeIdx, poly = poly,
             endIdx = G.buildEdgeIndex(sp, max(8, W + 2), function(i) return isEnd[i] end) }
end

-- Distance from (x,z) to the ends of the field (only looked for up to about
-- W + 2 m away: math.huge beyond). Inside the strip: < W.
function TramlineGenerator.distEnd(ctx, x, z)
    local idx = ctx.endIdx
    local cell, sp = idx.cell, ctx.sp
    local n = #sp
    local cx, cz = floor(x / cell), floor(z / cell)
    local best = math.huge
    local mark = idx.mark
    local stamp = idx.stamp + 1
    idx.stamp = stamp
    for ix = cx - 1, cx + 1 do
        for iz = cz - 1, cz + 1 do
            local bkt = idx.buckets[idx.key(ix, iz)]
            if bkt ~= nil then
                for _, i in ipairs(bkt) do
                    if mark[i] ~= stamp then
                        mark[i] = stamp
                        local j = (i % n) + 1
                        local a, b = sp[i], sp[j]
                        local ex, ez = b[1] - a[1], b[2] - a[2]
                        local L2 = ex * ex + ez * ez
                        local t = ((x - a[1]) * ex + (z - a[2]) * ez) / L2
                        local d = nil
                        if t >= 0 and t <= 1 then
                            d = abs((x - a[1]) * ez - (z - a[2]) * ex) / sqrt(L2)
                        elseif t < 0 and ctx.roundAt[i] then
                            d = sqrt((x - a[1]) ^ 2 + (z - a[2]) ^ 2)
                        elseif t > 1 and ctx.roundAt[j] then
                            d = sqrt((x - b[1]) ^ 2 + (z - b[2]) ^ 2)
                        end
                        if d ~= nil and d < best then best = d end
                    end
                end
            end
        end
    end
    return best
end

-- The crop part of the line piece ox,oz + t*(dx,dz), t0..t1: in from each
-- end until the strip is behind. Returns a, b or nil (all strip).
function TramlineGenerator.trimToCrop(ctx, ox, oz, dx, dz, t0, t1)
    local W = ctx.W
    local distEnd = TramlineGenerator.distEnd
    local function inStrip(t) return distEnd(ctx, ox + dx * t, oz + dz * t) < W end
    local function walk(tFrom, tTo)
        local dir = (tTo > tFrom) and 1 or -1
        if not inStrip(tFrom) then return tFrom end
        local t = tFrom
        while (tTo - t) * dir > 1e-6 do
            local tn = t + dir * 0.5
            if (tTo - tn) * dir < 0 then tn = tTo end
            if not inStrip(tn) then
                local a, b = t, tn
                for _ = 1, 12 do
                    local m = (a + b) * 0.5
                    if inStrip(m) then a = m else b = m end
                end
                return b
            end
            t = tn
        end
        return nil
    end
    local a = walk(t0, t1)
    if a == nil then return nil end
    local b = walk(t1, a)
    if b == nil or b - a < 1 then return nil end
    return a, b
end

-- Is (x,z) a place for the vehicle's point 'kind' on the strip?
--   "wheels": on the strip, the wheels off the crop and the field edge
--   "body":   the front or back end of the vehicle, inside the field
local function stripFree(ctx, x, z, kind)
    local de = G.nearestEdgeDist(ctx.edgeIdx, x, z, 4)
    if kind == "body" then return de >= TramlineGenerator.STRIP_EDGE_MARGIN end
    if de < TramlineGenerator.STRIP_EDGE_MARGIN + 0.5 then return false end
    return TramlineGenerator.distEnd(ctx, x, z) <= ctx.W - TramlineGenerator.STRIP_CROP_MARGIN
end

-- How far on from (px,pz) - where the crop stops on a tramline, (ux,uz) the
-- way out of it - there is still crop somewhere across the boom (hw either
-- side): where the end of the field isn't square to the tramlines the crop
-- goes on further at one end of the boom. Looked for up to 'cap' m.
function TramlineGenerator.cropBeyond(ctx, px, pz, ux, uz, hw, cap)
    local W, distEnd = ctx.W, TramlineGenerator.distEnd
    local vx, vz = -uz, ux
    local n = max(2, ceil(2 * hw))          -- about every metre across
    local best = 0
    for i = 0, n do
        local a = -hw + 2 * hw * i / n
        local ox, oz = px + vx * a, pz + vz * a
        local function crop(t)
            local x, z = ox + ux * t, oz + uz * t
            return distEnd(ctx, x, z) >= W and G.pointInPolygon(ctx.poly, x, z)
        end
        local t = 0
        if crop(0) then
            while t < cap and crop(t) do t = t + 0.25 end
            if t < cap then
                -- where it stops, between t - 0.25 and t
                local lo, hi = t - 0.25, t
                for _ = 1, 5 do
                    local m = (lo + hi) * 0.5
                    if crop(m) then lo = m else hi = m end
                end
                t = hi
            end
        end
        if t > best then best = t end
    end
    return min(best, cap)
end

local function boomBackOf(opts)
    return max(0, ((opts and opts.foldBack) or (TramlineGenerator.STRIP_BOOM_BACK + TramlineGenerator.STRIP_FOLD_EXTRA))
        - TramlineGenerator.STRIP_FOLD_EXTRA)
end

-- the furthest y from y0 towards y1 (0.25 m at a time) that ok(y) allows all
-- the way; nil if not even y0
local function furthest(y0, y1, ok)
    if not ok(y0) then return nil end
    local y = y0
    while y < y1 - 1e-6 do
        local yn = min(y1, y + 0.25)
        if not ok(yn) then return y end
        y = yn
    end
    return y
end

-- The end of a tramline at the strip: (px,pz) where the crop stops on it,
-- (ux,uz) on out. The worker drives on until the boom (opts.foldBack less
-- STRIP_FOLD_EXTRA behind) is STRIP_RUN_OUT past the last of the crop
-- anywhere across it - as far as there is room for the front of the vehicle
-- in the field - and folds there. Returns { fb = m on to the fold, workEnd
-- = m on the boom stays on, clear = m the boom gets past the last of the
-- crop (< 0: it can't get right out), delta = m the crop goes on past
-- (px,pz) across the boom }, or nil if the boom can't even get off the
-- crop where the tramline runs.
function TramlineGenerator.stripFoldAt(geo, px, pz, ux, uz, opts)
    local ctx = geo.strip
    if ctx == nil then return nil end
    opts = opts or {}
    local FRONT = opts.front or TramlineGenerator.STRIP_FRONT
    local bb = boomBackOf(opts)
    local delta = TramlineGenerator.cropBeyond(ctx, px, pz, ux, uz, (geo.spacing or 24) * 0.5, ctx.W + 2)
    local fbMin = bb + TramlineGenerator.STRIP_FOLD_EXTRA
    local fbWant = max(fbMin, bb + delta + TramlineGenerator.STRIP_RUN_OUT)
    local fb = furthest(fbMin, fbWant, function(y)
        return stripFree(ctx, px + ux * (y + FRONT), pz + uz * (y + FRONT), "body")
    end)
    if fb == nil then return nil end
    -- (the vehicle stops up to STRIP_STOP_TOL short of it)
    return { fb = fb, workEnd = max(0, fb - bb - 0.3), clear = fb - TramlineGenerator.STRIP_STOP_TOL - bb - delta, delta = delta }
end

-- The start of a tramline from the strip: (px,pz) where the crop starts on
-- it, (ux,uz) back out. The worker drives in until the boom is over the edge
-- of the crop, unfolds, backs up out of the tramline to yS out on the strip
-- (at least yMin - the end of the turn - if there is room, and further if
-- the crop starts further out somewhere across the boom) and sprays from
-- there, the boom coming on yW out. A machine that can't back up unfolds
-- where the boom is yW out instead (on the strip, or in the tramline) and
-- sprays on from there. Returns { ub = m in to unfold (< 0: out on the
-- strip), yS (nil: no backing up), yW, delta }.
function TramlineGenerator.stripStartAt(geo, px, pz, ux, uz, opts, yMin)
    local ctx = geo.strip
    opts = opts or {}
    local bb = boomBackOf(opts)
    local REAR = opts.rear or TramlineGenerator.STRIP_REAR
    local RA = opts.rearAxle or 0
    local delta = ctx ~= nil and TramlineGenerator.cropBeyond(ctx, px, pz, ux, uz, (geo.spacing or 24) * 0.5, ctx.W + 2) or 0
    local yW = delta + TramlineGenerator.STRIP_RUN_IN
    if opts.reverse == false then
        return { ub = max(0.5 - TramlineGenerator.STRIP_LEAD, bb - yW), yS = nil, yW = yW, delta = delta }
    end
    local want = max(yMin or 0, yW - bb)
    local yS = nil
    if ctx ~= nil then
        local wheelDist = TramlineGenerator.STRIP_EDGE_MARGIN + 0.5
        yS = furthest(0, want, function(y)
            -- facing into the tramline: its back end out on the strip, its
            -- wheels (on the tramline's own line) off the field edge
            if not stripFree(ctx, px + ux * (y + REAR), pz + uz * (y + REAR), "body") then return false end
            if y > 0 and G.nearestEdgeDist(ctx.edgeIdx, px + ux * y, pz + uz * y, 4) < wheelDist then return false end
            return RA <= 0.3 or G.nearestEdgeDist(ctx.edgeIdx, px + ux * (y + RA), pz + uz * (y + RA), 4) >= wheelDist
        end)
    end
    return { ub = bb, yS = yS, yW = yW, delta = delta }
end

-- A turn on the soil strip from the end of line la (side sA) to the end of
-- line lb (side sB), both where the crop stops. opts:
--   foldBack  the boom's distance behind the vehicle + STRIP_FOLD_EXTRA
--   reverse   the vehicle can back up (three-point turns)
--   R         turn radius;  wheelbase, front, rear, rearAxle: the vehicle (defaults above)
-- Returns { P1, F, B, P2, S, pieces = { { pts, rev }, ... }, len, shifts,
--           fold = stripFoldAt, start = stripStartAt }:
--   P1 -> F straight on out of the crop (the boom on fold.workEnd past P1),
--   fold at F, the pieces (forwards or backwards), B -> P2 straight into the
--   next tramline, on in start.ub to unfold, back out to S and away;
-- or nil if no turn fits.
function TramlineGenerator.planStripTurn(geo, la, sA, lb, sB, opts)
    local ctx = geo.strip
    if ctx == nil then return nil end
    opts = opts or {}
    local R = opts.R or geo.radius or 7
    local W = ctx.W
    local STEP = TramlineGenerator.STRIP_STEP
    local WB = opts.wheelbase or TramlineGenerator.STRIP_WHEELBASE
    local FRONT = opts.front or TramlineGenerator.STRIP_FRONT
    local REAR = opts.rear or TramlineGenerator.STRIP_REAR
    local RA = opts.rearAxle or 0
    local lead = TramlineGenerator.STRIP_LEAD
    local dx, dz = geo.dirX, geo.dirZ
    -- end points (where the crop stops) and outward directions
    local function endOf(l, side)
        if side == "A" then return l[1], l[2], -dx, -dz end
        return l[3], l[4], dx, dz
    end
    local p1x, p1z, ux, uz = endOf(la, sA)
    local p2x, p2z, ux2, uz2 = endOf(lb, sB)
    if ux * ux2 + uz * uz2 < 0.99 then return nil end        -- not the same end of the field
    -- local frame: y outwards (u), x across towards the next tramline (v)
    local rx, rz = p2x - p1x, p2z - p1z
    local y2 = rx * ux + rz * uz
    local vx, vz, sl = G.normalize(rx - ux * y2, rz - uz * y2)
    if sl < 1 then return nil end
    local function W2(x, y) return p1x + vx * x + ux * y, p1z + vz * x + uz * y end
    local yB = y2 + lead
    -- where it folds: on out until the boom has left the crop (room allowing)
    local fold = TramlineGenerator.stripFoldAt(geo, p1x, p1z, ux, uz, opts)
    if fold == nil then return nil end
    local fb = fold.fb

    -- a candidate is a list of moves from (0, fb) facing outwards (phi = 0,
    -- phi grows turning towards the next tramline):
    --   { "S", dir, len }  straight (dir 1 forwards, -1 backwards)
    --   { "A", dir, ang }  arc of radius R, phi growing by ang (forwards: a
    --                      turn towards the next tramline; backwards:
    --                      steering the other way)
    -- Check it: every STEP the wheels (reference point, front axle and a rear
    -- axle behind the point) on the strip, the ends of the vehicle in the
    -- field; on the first and last tramline's own line (lined up, in its
    -- track) anything goes.
    local cosA, sinA = math.cos, math.sin
    -- in a tramline's own track (tol: how far off its middle the wheels may be)
    local function onOwnLine(x, y, tol)
        return (abs(x) <= tol and y <= fb + 0.1) or (abs(x - sl) <= tol and y <= yB + 0.1)
    end
    -- what each quarter metre square around the turn allows, worked out
    -- when first asked: 2 = wheels may go there, 1 = only the body, 0 = nothing
    local memo = {}
    local function free(x, y)
        local ix, iy = floor(x * 4 + 0.5), floor(y * 4 + 0.5)
        local key = ix * 65536 + iy
        local v = memo[key]
        if v == nil then
            local wx, wz = W2(ix * 0.25, iy * 0.25)
            if stripFree(ctx, wx, wz, "wheels") then v = 2
            elseif stripFree(ctx, wx, wz, "body") then v = 1
            else v = 0 end
            memo[key] = v
        end
        return v
    end
    local function poseOk(x, y, phi)
        local sp, cp = sinA(phi), cosA(phi)
        if not onOwnLine(x, y, 0.6) and free(x, y) < 2 then return false end
        local ax, ay = x + sp * WB, y + cp * WB
        if not onOwnLine(ax, ay, 1.0) and free(ax, ay) < 2 then return false end
        if RA > 0.3 then
            local bx, by = x - sp * RA, y - cp * RA
            if not onOwnLine(bx, by, 1.0) and free(bx, by) < 2 then return false end
        end
        if free(x + sp * FRONT, y + cp * FRONT) < 1 then return false end
        return free(x - sp * REAR, y - cp * REAR) >= 1
    end
    -- walk the moves; returns pieces (split where the direction changes) or nil
    local function trace(moves, check, step)
        step = step or STEP
        local x, y, phi = 0, fb, 0
        local pieces, cur = {}, nil
        for mi, mv in ipairs(moves) do
            local kind, dir, amount = mv[1], mv[2], mv[3]
            if amount > 1e-6 then
                local rev = dir < 0
                if cur == nil or cur.rev ~= rev then
                    cur = { pts = { { W2(x, y) } }, rev = rev }
                    pieces[#pieces + 1] = cur
                end
                local len = (kind == "S") and amount or amount * R
                local k = max(1, ceil(len / step))
                local cxc, cyc, side
                if kind == "A" then
                    side = dir    -- forwards: centre on the turning side; backwards: the other
                    cxc, cyc = x + side * R * cosA(phi), y - side * R * sinA(phi)
                end
                for j = 1, k do
                    local f = j / k
                    local nx_, ny_, nphi
                    if kind == "S" then
                        nx_, ny_, nphi = x + dir * amount * f * sinA(phi), y + dir * amount * f * cosA(phi), phi
                    else
                        nphi = phi + amount * f
                        nx_, ny_ = cxc - side * R * cosA(nphi), cyc + side * R * sinA(nphi)
                    end
                    if check and not poseOk(nx_, ny_, nphi) then return nil, mi, amount * f end
                    local wx, wz = W2(nx_, ny_)
                    cur.pts[#cur.pts + 1] = { wx, wz }
                    if j == k then x, y, phi = nx_, ny_, nphi end
                end
            end
        end
        return pieces, x, y, phi
    end

    -- the candidates, cheapest first (angles every 'stepDeg' from 'offDeg')
    local function search(stepDeg, offDeg, starts, a2s, fwdOnly)
    local cands = {}
    local SHIFT = TramlineGenerator.STRIP_SHIFT_COST
    local function addCand(moves)
        local len, shifts, lastDir = 0, 0, 1
        for _, mv in ipairs(moves) do
            if mv[3] > 1e-6 then
                len = len + ((mv[1] == "S") and mv[3] or mv[3] * R) * ((mv[2] < 0) and 1.3 or 1)
                if mv[2] ~= lastDir then shifts = shifts + 1; lastDir = mv[2] end
            end
        end
        if shifts > 0 and (fwdOnly or not opts.reverse) then return end
        cands[#cands + 1] = { moves = moves, cost = len + shifts * SHIFT, shifts = shifts }
    end
    local d2r = math.pi / 180
    -- back up first (down the tramline, as far as the crop edge or a little
    -- into the track) for more room, or not
    -- Two shapes, each: back up (or not), forward arc al towards the next
    -- tramline, backward arc be (the back swinging out), forward arc into the
    -- next tramline, and one straight to make it end on it:
    --   1. the straight after the backward arc (out with a three-point turn,
    --      across, round into it; without be: an ordinary U-turn)
    --   2. the straight after the first arc (out round, along the strip, then
    --      back the rear round and forwards into it); with a2: round towards
    --      the crop a2 first, then back round (coming at it from this side)
    local maxStraight = 3 * W + sl
    for _, yr in ipairs(starts) do
        local back = fb - yr
        -- the angles on the grid, and the way straight at the next tramline
        -- (along a strip that runs off at an angle)
        local alphas = {}
        for a = offDeg, 177, stepDeg do alphas[#alphas + 1] = a end
        local aim = math.deg(math.atan2(sl, yB - yr))
        if aim > 1 and aim < 177 then
            for _, k in ipairs({ -3, -1.5, 0, 1.5, 3 }) do alphas[#alphas + 1] = aim + k end
        end
        for _, a in ipairs(alphas) do
            for b = 0, 177 - a, stepDeg do
                local al, be = a * d2r, b * d2r
                local p2 = al + be
                if p2 > 2 * d2r and p2 < 178 * d2r then
                    local ca, sa, c2, s2 = cosA(al), sinA(al), cosA(p2), sinA(p2)
                    -- 1.
                    local x, y = R - R * ca, yr + R * sa                 -- after the forward arc
                    x, y = x - R * ca + R * c2, y + R * sa - R * s2       -- after the backward arc
                    local bb = (x + R * c2 + R - sl) / s2                -- straight: backwards if > 0
                    if abs(bb) <= maxStraight then
                        local y6 = y - bb * c2 - R * s2
                        local c = y6 - yB
                        if c >= -1e-6 then
                            local moves = { { "S", -1, back }, { "A", 1, al }, { "A", -1, be } }
                            if bb >= 0 then moves[#moves + 1] = { "S", -1, bb } else moves[#moves + 1] = { "S", 1, -bb } end
                            moves[#moves + 1] = { "A", 1, math.pi - p2 }
                            moves[#moves + 1] = { "S", 1, max(0, c) }
                            addCand(moves)
                        end
                    end
                    -- 2.
                    if b > 0 and sa > 0.05 then
                        for _, a2d in ipairs(a2s) do
                            local a2 = a2d * d2r
                            local pa, pb = al + a2, p2 + a2                      -- heading before / after backing round
                            if pb < 177 * d2r then
                                local cpa, spa, cpb, spb = cosA(pa), sinA(pa), cosA(pb), sinA(pb)
                                local f = (sl - 2 * R + 2 * R * cpa - 2 * R * cpb) / sa   -- straight: forwards if > 0
                                if abs(f) <= maxStraight then
                                    local y5 = yr + f * ca + 2 * R * spa - 2 * R * spb
                                    local c = y5 - yB
                                    if c >= -1e-6 then
                                        local moves = { { "S", -1, back }, { "A", 1, al } }
                                        if f >= 0 then moves[#moves + 1] = { "S", 1, f } else moves[#moves + 1] = { "S", -1, -f } end
                                        moves[#moves + 1] = { "A", 1, a2 }
                                        moves[#moves + 1] = { "A", -1, be }
                                        moves[#moves + 1] = { "A", 1, math.pi - pb }
                                        moves[#moves + 1] = { "S", 1, max(0, c) }
                                        addCand(moves)
                                    end
                                end
                            end
                        end
                    end
                end
            end
        end
    end
    table.sort(cands, function(p, q) return p.cost < q.cost end)

    -- checked every metre, the one taken again every STEP. Candidates
    -- share their first moves: where a move runs into something, those
    -- with the same moves before it and that move as long or longer are
    -- passed over.
    local best = nil
    local failAt = {}      -- moves before + the move's kind and way -> how far into it it fails
    local function keyOf(moves, mi)
        local t = {}
        for i = 1, mi - 1 do
            local mv = moves[i]
            t[#t + 1] = string.format("%s%d%.3f", mv[1], mv[2], mv[3])
        end
        t[#t + 1] = moves[mi][1] .. moves[mi][2]
        return table.concat(t, "|")
    end
    for _, c in ipairs(cands) do
        local skip = false
        for mi = 2, min(3, #c.moves) do
            local lim = failAt[keyOf(c.moves, mi)]
            if lim ~= nil and c.moves[mi][3] >= lim then skip = true; break end
        end
        if not skip then
            local pieces, x, y = trace(c.moves, true, 1.0)
            if pieces ~= nil and abs(x - sl) < 0.05 and abs(y - yB) < 0.05 then
                pieces = trace(c.moves, true)
                if pieces ~= nil then
                    best = { pieces = pieces, shifts = c.shifts, cost = c.cost }
                    break
                end
            elseif pieces == nil and x ~= nil and x >= 2 and x <= 3 then
                -- (x, y: the move it failed in, how far into it)
                local k = keyOf(c.moves, x)
                if failAt[k] == nil or y < failAt[k] then failAt[k] = y end
            end
        end
    end
    return best
    end
    -- Where to fold: after the full run out (STRIP_RUN_OUT), or a shorter one
    -- (the boom still right off the crop: fbLow) where that lets it turn, or
    -- lets it turn without backing up; where nothing fits at all, as soon as
    -- the boom has left the crop where the tramline runs (fbMin: not right
    -- off it at one end). A coarse search first, then a finer one.
    local bb = boomBackOf(opts)
    local fbMin = bb + TramlineGenerator.STRIP_FOLD_EXTRA
    local fbLow = max(fbMin, bb + fold.delta + TramlineGenerator.STRIP_RUN_OUT_MIN)
    local function startsFrom(y, ys)
        local list = { y }
        if opts.reverse then
            for _, f in ipairs(ys) do if y * f < y - 0.25 then list[#list + 1] = y * f end end
        end
        return list
    end
    local function coarse(fwdOnly) return search(7.5, 0, fwdOnly and { fb } or startsFrom(fb, { 0.5, 0 }), { 0 }, fwdOnly) end
    local function fine()
        local more = startsFrom(fb, { 0.75, 0.5, 0.25, 0 })
        return search(3.75, 3.75, more, { 0 }) or search(7.5, 0, more, { 0, 15, 30, 45, 60, 75 })
    end
    -- (fb: the checks and the turns start from it)
    local function from(y, find, ...)
        if y > fb - 0.25 then return nil end
        local was = fb
        fb = y
        local r = find(...)
        if r == nil then fb = was end
        return r
    end
    local best = coarse()
    if best == nil then best = from(fbLow, coarse) end
    if best == nil then best = fine() end
    if best == nil then best = from(fbMin, coarse) end
    if best ~= nil and best.shifts > 0 then
        local fwd = from(fbLow, coarse, true)
        if fwd == nil and fb > fbLow + 0.25 then
            fwd = from(fbLow, function() return search(3.75, 3.75, { fb }, { 0 }, true) end)
        end
        if fwd ~= nil then best = fwd end
    end
    if best ~= nil and fb < fold.fb - 0.01 then
        fold = { fb = fb, workEnd = max(0, fb - bb - 0.3), clear = fb - TramlineGenerator.STRIP_STOP_TOL - bb - fold.delta, delta = fold.delta }
    end
    if best == nil then return nil end
    local len = 0
    for _, pc in ipairs(best.pieces) do len = len + G.polyLength(pc.pts, false) end
    local fx, fz = W2(0, fb)
    local bx, bz = W2(sl, yB)
    -- into the next tramline: unfold, back up out of it to the end of the
    -- turn (or further where the crop starts further out across the boom)
    local start = TramlineGenerator.stripStartAt(geo, p2x, p2z, ux2, uz2, opts, lead)
    return { P1 = { p1x, p1z }, F = { fx, fz }, B = { bx, bz }, P2 = { p2x, p2z },
             S = { p2x + ux2 * (start.yS or 0), p2z + uz2 * (start.yS or 0) }, pieces = best.pieces,
             len = len, shifts = best.shifts, foldBack = fb, fold = fold, start = start, from = la.id, to = lb.id, side = sA }
end

-- the same turn when no turn fits: forwards round, inside the field if it
-- can be (over the crop), else wherever it goes
function TramlineGenerator.forcedStripTurn(geo, la, sA, lb, sB, opts)
    local dx, dz = geo.dirX, geo.dirZ
    local R = (opts and opts.R) or geo.radius or 7
    local function endOf(l, side)
        if side == "A" then return l[1], l[2], -dx, -dz end
        return l[3], l[4], dx, dz
    end
    local p1x, p1z, ux, uz = endOf(la, sA)
    local p2x, p2z, ux2, uz2 = endOf(lb, sB)
    local fold = geo.strip ~= nil and TramlineGenerator.stripFoldAt(geo, p1x, p1z, ux, uz, opts) or nil
    if fold == nil then
        local bb = boomBackOf(opts)
        fold = { fb = bb + TramlineGenerator.STRIP_FOLD_EXTRA, workEnd = 0, clear = TramlineGenerator.STRIP_FOLD_EXTRA, delta = 0 }
    end
    local fb = fold.fb
    local fx, fz = p1x + ux * fb, p1z + uz * fb
    local bx, bz = p2x + ux * TramlineGenerator.STRIP_LEAD, p2z + uz * TramlineGenerator.STRIP_LEAD
    local ctx = geo.strip
    local function inField(pts)
        for _, p in ipairs(pts) do
            if not G.pointInPolygon(ctx.poly, p[1], p[2])
                or G.nearestEdgeDist(ctx.edgeIdx, p[1], p[2], 4) < TramlineGenerator.STRIP_EDGE_MARGIN + 0.5 then return false end
        end
        return true
    end
    local path = (ctx ~= nil) and G.dubins(fx, fz, G.atan2(uz, ux), bx, bz, G.atan2(-uz, -ux), R, inField, 1.0) or nil
    if path == nil then path = G.dubins(fx, fz, G.atan2(uz, ux), bx, bz, G.atan2(-uz, -ux), R) end
    if path == nil then return nil end
    local pts = G.dubinsPoints(path, TramlineGenerator.STRIP_STEP)
    local start = TramlineGenerator.stripStartAt(geo, p2x, p2z, ux2, uz2, opts, TramlineGenerator.STRIP_LEAD)
    return { P1 = { p1x, p1z }, F = { fx, fz }, B = { bx, bz }, P2 = { p2x, p2z },
             S = { p2x + ux2 * (start.yS or 0), p2z + uz2 * (start.yS or 0) }, pieces = { { pts = pts, rev = false } },
             len = path.length, shifts = 0, foldBack = fb, fold = fold, start = start, from = la.id, to = lb.id, side = sA, forced = true }
end

-- The turn the worker drives from line 'from' (side) to line 'to' for a
-- vehicle (opts.foldBack, opts.reverse), worked out once per layout and
-- vehicle. Falls back to a turn that doesn't fit (forced = true).
function TramlineGenerator.stripTurnFor(geo, fromId, toId, side, opts)
    geo.stripTurns = geo.stripTurns or {}
    local function f(v) return v ~= nil and string.format("%.1f", v) or "-" end
    local key = string.format("%d>%d%s/%s/%s/%s/%s/%s/%s", fromId, toId, side, f(opts.foldBack), f(opts.wheelbase), f(opts.front),
        f(opts.rear), tostring(opts.reverse ~= false), f(opts.rearAxle))
    local t = geo.stripTurns[key]
    if t == nil then
        local la, lb = geo.lines[fromId], geo.lines[toId]
        t = TramlineGenerator.planStripTurn(geo, la, side, lb, side, opts)
            or TramlineGenerator.forcedStripTurn(geo, la, side, lb, side, opts) or false
        geo.stripTurns[key] = t
    end
    return t or nil
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
    local W = (tonumber(set.strip) or 0) > 0 and set.strip or nil      -- soil strip at the ends
    if W ~= nil then H = 0 end
    geo.spacing, geo.headlands, geo.radius, geo.stripWidth = s, H, R, W

    local dx, dz = G.normalize(set.dx or 1, set.dz or 0)
    if dx == 0 and dz == 0 then dx, dz = 1, 0 end
    local nx, nz = -dz, dx
    geo.dirX, geo.dirZ = dx, dz

    local edgeIdx = G.buildEdgeIndex(poly, 8)
    if W ~= nil then geo.strip = TramlineGenerator.stripContext(poly, dx, dz, W, edgeIdx) end

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
    if W ~= nil then
        -- soil strip: the lines run to the strip (trimmed below)
        clipLoops = { poly }
    elseif H == 0 then
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
        if W ~= nil then
            local cut = {}
            for _, v in ipairs(iv) do
                local a, b = TramlineGenerator.trimToCrop(geo.strip, ox, oz, dx, dz, v[1], v[2])
                if a ~= nil then cut[#cut + 1] = { a, b } end
            end
            iv = cut
        end
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
    local dir = set.headlandDir
    if (dir ~= "cw" and dir ~= "acw") or inner == nil then dir = nil end
    geo.headlandDir = dir
    local function keepAt(l, side)
        local x, z = endPoint(l, side)
        local ri = nearestRing(x, z)
        return TramlineGenerator.dirSense(inner[ri or 1], dir)
    end
    local loopsUsed, ringUsed, forcedUsed, bothWays = 0, 0, 0, 0
    local stripUsed, stripForced, stripShifts, stripBack, stripShort = 0, 0, 0, 0, 0
    local stripOpts = set.stripVehicle or { foldBack = TramlineGenerator.STRIP_BOOM_BACK + TramlineGenerator.STRIP_FOLD_EXTRA, reverse = true }
    for _, blk in ipairs(geo.blocks) do
        blk.conns = {}
        local m = #blk.lines
        for j = 1, m - 1 do
            local la, lb = geo.lines[blk.lines[j]], geo.lines[blk.lines[j + 1]]
            local side = (j % 2 == 1) and "B" or "A"
            local conn
            if W ~= nil then
                -- soil strip: fold, turn round on the strip, unfold
                local turn = TramlineGenerator.stripTurnFor(geo, la.id, lb.id, side, stripOpts)
                conn = { kind = "strip", from = la.id, to = lb.id, side = side, turn = turn }
                if turn ~= nil then
                    if turn.forced then stripForced = stripForced + 1 else stripUsed = stripUsed + 1 end
                    if not turn.forced and turn.fold.clear < -0.05 then stripShort = stripShort + 1 end
                    stripShifts = stripShifts + (turn.shifts or 0)
                    if (turn.shifts or 0) > 0 then stripBack = stripBack + 1 end
                    -- drawn (not cut into the ground: the strip is bare soil anyway)
                    segmentsOf({ turn.P1, turn.F }, false, geo.extras)
                    for _, pc in ipairs(turn.pieces) do segmentsOf(pc.pts, false, geo.extras) end
                    segmentsOf({ turn.B, turn.P2 }, false, geo.extras)
                    if (turn.start.yS or 0) > TramlineGenerator.STRIP_LEAD + 0.05 then segmentsOf({ turn.S, turn.B }, false, geo.extras) end
                end
            elseif style == 2 then
                conn = loopTurn(la, lb, side) or ringTurn(la, lb, side)
            else
                conn = ringTurn(la, lb, side) or loopTurn(la, lb, side)
            end
            if conn == nil then
                conn = loopTurn(la, lb, side, true)
                forcedUsed = forcedUsed + 1
            end
            if conn.kind == "loop" then loopsUsed = loopsUsed + 1 elseif conn.kind == "ring" then ringUsed = ringUsed + 1 end
            blk.conns[j] = conn
            if conn.pts ~= nil then drawn(conn.pts) end
            if conn.jEx ~= nil then
                drawJ(conn.jEx); drawJ(conn.jEn)
                recordExit(la, side, conn.sense, conn.jEx)
                recordExit(lb, side, -conn.sense, conn.jEn)
            end
        end
        -- free ends: line 1 at side A, line m at the side the serpentine leaves it
        -- entry[sense]: ring (travelling sense) -> line;  exit[sense]: line -> ring, then sense
        local first, last = geo.lines[blk.lines[1]], geo.lines[blk.lines[m]]
        local lastSide = (m % 2 == 1) and "B" or "A"
        local specs = { { first, "A" }, { last, lastSide } }
        local function newEnd(l, side)
            local ex, ez, ux, uz = endPoint(l, side)
            return { line = l.id, side = side, x = ex, z = ez, ux = ux, uz = uz, entry = {}, exit = {} }
        end
        blk.ends = nil
        if dir ~= nil then
            -- one way round the headland: in at one end of the block and out at
            -- the other, each with the one curve that goes that way round (in
            -- at line 1 if both curves fit, else in at line m)
            for _, order in ipairs({ { 1, 2 }, { 2, 1 } }) do
                local sIn, sOut = specs[order[1]], specs[order[2]]
                local kIn, kOut = keepAt(sIn[1], sIn[2]), keepAt(sOut[1], sOut[2])
                local jIn = toRing(sIn[1], sIn[2], -kIn)     -- reversed: in from the ring going kIn
                local jOut = toRing(sOut[1], sOut[2], kOut)
                if jIn ~= nil and jOut ~= nil then
                    local Ein, Eout = newEnd(sIn[1], sIn[2]), newEnd(sOut[1], sOut[2])
                    Ein.entry[kIn] = { pts = G.reverse(jIn.pts), s = jIn.s, trim = jIn.a }
                    Ein.ring = jIn.ring
                    Eout.exit[kOut] = { pts = jOut.pts, s = jOut.s, trim = jOut.a }
                    Eout.ring = jOut.ring
                    drawJ(jIn); drawJ(jOut)
                    recordExit(sIn[1], sIn[2], -kIn, jIn)
                    recordExit(sOut[1], sOut[2], kOut, jOut)
                    blk.ends = { [order[1]] = Ein, [order[2]] = Eout }
                    break
                end
            end
            if blk.ends == nil then bothWays = bothWays + 1 end
        end
        if blk.ends == nil then
            -- either way round: both curves at both ends
            blk.ends = {}
            for e, spec in ipairs(specs) do
                local l, side = spec[1], spec[2]
                local E = newEnd(l, side)
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
        if W ~= nil and blk.ends[2] ~= nil then
            -- soil strip: out of the crop at the end of the part (until the
            -- boom has left it), and backing out of the first tramline to start
            local E = blk.ends[2]
            local fold = TramlineGenerator.stripFoldAt(geo, E.x, E.z, E.ux, E.uz, stripOpts)
            local fb = fold ~= nil and fold.fb or (boomBackOf(stripOpts) + TramlineGenerator.STRIP_FOLD_EXTRA)
            if fold == nil or fold.clear < -0.05 then stripShort = stripShort + 1 end
            segmentsOf({ { E.x, E.z }, { E.x + E.ux * fb, E.z + E.uz * fb } }, false, geo.extras)
            local E1 = blk.ends[1]
            local start = TramlineGenerator.stripStartAt(geo, E1.x, E1.z, E1.ux, E1.uz, stripOpts, TramlineGenerator.STRIP_LEAD)
            if (start.yS or 0) > 0.3 then
                segmentsOf({ { E1.x, E1.z }, { E1.x + E1.ux * start.yS, E1.z + E1.uz * start.yS } }, false, geo.extras)
            end
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
            -- one way round the headland (dir): every corner is reached from
            -- the same side, so only that way of backing into it - and its
            -- one corner tramline - is kept
            if dir ~= nil then
                local keep = TramlineGenerator.dirSense(lp, dir)
                for _, c in ipairs(list) do c.m[-keep] = nil end
            end
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
    if stripForced > 0 then
        geo.warnings[#geo.warnings + 1] = string.format("%d of %d turn(s) do not fit on the soil strip - make the strip wider or the turn radius smaller",
            stripForced, stripForced + stripUsed)
    end
    if W ~= nil and #geo.blocks > 1 then
        geo.warnings[#geo.warnings + 1] = string.format("the field splits into %d parts - between them the worker drives over the crop (folded)", #geo.blocks)
    end
    if stripShort > 0 then
        geo.warnings[#geo.warnings + 1] = string.format("%d tramline end(s): no room on the soil strip to drive the whole boom off the crop before folding (where the end of the field is at an angle, or meets a side)",
            stripShort)
    end
    geo.stripTurnCount, geo.stripForcedCount, geo.stripShiftCount, geo.stripBackCount = stripUsed, stripForced, stripShifts, stripBack
    geo.stripShortCount = stripShort
    if bothWays > 0 then
        geo.warnings[#geo.warnings + 1] = string.format("%d of %d part(s) of the field have no room to turn on and off the headland only %s - the worker can turn either way there",
            bothWays, #geo.blocks, dir == "cw" and "clockwise" or "anticlockwise")
    end
    geo.loopCount, geo.ringTurnCount, geo.forcedCount, geo.bothWaysCount = loopsUsed, ringUsed, forcedUsed, bothWays
    TramlineLog.info("[TramlinePilot] field %d: %d tramlines in %d block(s), %d headland ring(s), %d loop turns, %d headland turns%s%s",
        set.fieldId or -1, #geo.lines, #geo.blocks, H, loopsUsed, ringUsed,
        dir ~= nil and string.format(", headland %s", dir) or "",
        W ~= nil and string.format(", soil strip %g m: %d turns on it (%d stops to back up), %d that don't fit, %d end(s) with no room to get the boom right off the crop",
            W, stripUsed, stripShifts, stripForced, stripShort) or "")
    return geo
end
