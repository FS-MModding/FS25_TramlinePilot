--[[
    TramlineRoute.lua
    1. buildJob: turns the layout of a field (TramlineGenerator) plus the
       vehicle's position into the route the worker drives. Every point of the
       route lies on the layout (rings, lines, loops, junction curves).
    2. PPC: pure pursuit controller (after Courseplay's PurePursuitController /
       Campbell's paper).
    3. workAt: is the route a working stretch at a given position?

    Route waypoints: { x, z, work = bool, turn = bool, kind = string,
                       rl, ri, rd = headland tramline (level, index, direction
                                    of travel) the segment runs along, or nil,
                       rev = true: driven in reverse (backing into a corner) }
      'work', 'turn' and the ring tags describe the segment ENDING at the
      waypoint.
]]

TramlineRoute = {}

local G = TramlineGeom
local sqrt, abs, min, max = math.sqrt, math.abs, math.min, math.max

TramlineRoute.MAX_START_DIST = 8      -- metres from a headland tramline to be allowed to start
TramlineRoute.GATE_MAX_DIST  = 60     -- metres from the field gate marker to the headland tramline

-------------------------------------------------------------------------------
-- route assembly helpers
-------------------------------------------------------------------------------

local function newRoute() return { list = {} } end

-- append points with the given flags; a point within 0.3 m of the previous
-- route point is skipped (pieces share their end points). Ring tags come
-- from the point itself or the flags.
local function add(route, pts, flags)
    local list = route.list
    for _, p in ipairs(pts) do
        local last = list[#list]
        if last == nil or (p[1] - last[1]) ^ 2 + (p[2] - last[2]) ^ 2 >= 0.09 then
            list[#list + 1] = { p[1], p[2], work = flags.work, turn = flags.turn, kind = flags.kind,
                                rl = p.rl or flags.rl, ri = p.ri or flags.ri, rd = p.rd or flags.rd,
                                rev = p.rev or flags.rev }
        end
    end
end

local function lastPoint(route) return route.list[#route.list] end

-- drive one line from 'fromSide' to the other end; boom only on the working
-- stretch. startTrim/endTrim: metres left out at the start/end, where a
-- junction curve joins the line further in than its end point.
local function driveLine(route, l, fromSide, startTrim, endTrim)
    local ax, az = l[1], l[2]
    local dx, dz = G.normalize(l[3] - ax, l[4] - az)
    local len = l.len
    local w0, w1 = l.w0 or 0, l.w1 or len
    startTrim, endTrim = startTrim or 0, endTrim or 0
    local ts, tws, twe, te
    if fromSide == "A" then
        ts, te = startTrim, len - endTrim
        tws, twe = max(w0, ts), min(w1, te)
    else
        ts, te = len - startTrim, endTrim
        tws, twe = min(w1, ts), max(w0, te)
    end
    local function P(t) return { ax + dx * t, az + dz * t } end
    add(route, { P(ts) }, { work = false, kind = "line" })
    add(route, { P(tws) }, { work = false, kind = "line" })
    add(route, { P(twe) }, { work = true, kind = "line", lineId = l.id })
    add(route, { P(te) }, { work = false, kind = "line" })
end

local function endOf(l, side)
    if side == "A" then return l[1], l[2] end
    return l[3], l[4]
end

local function dist2(p, x, z) return sqrt((p[1] - x) ^ 2 + (p[2] - z) ^ 2) end

local function otherSide(side) return side == "A" and "B" or "A" end

-- the side line j of a block is entered from when the block is driven forward
local function entrySideForward(j) return (j % 2 == 1) and "A" or "B" end

-------------------------------------------------------------------------------
-- buildJob
-- opts: { headlandOrder = "first" | "last" | "off",
--         corners = true: back into the headland corners (geo.corners),
--         boomBack = metres the boom is behind the vehicle (how far to back up) }

TramlineRoute.BOOM_BACK = 4      -- m, if the vehicle does not say

-- the stretch backed into a corner and the stretch driven out of it, for a
-- boom 'boomBack' metres behind the vehicle (generator maneuver m)
function TramlineRoute.cornerPieces(m, boomBack)
    local e = m.u - (boomBack or TramlineRoute.BOOM_BACK)
    if e > m.emax then e = m.emax end
    if e < 0.5 - m.back then e = 0.5 - m.back end          -- never back to before the curve
    local K = { m.P[1] + m.dir[1] * e, m.P[2] + m.dir[2] * e }
    local rev, fwd = { m.Q, m.R0 }, { K }
    if e > 0.3 then
        rev[#rev + 1] = m.P
        fwd[#fwd + 1] = m.P
    end
    rev[#rev + 1] = K
    fwd[#fwd + 1] = m.R0
    return rev, fwd
end
-- returns route (list of waypoints) or nil, reason
-------------------------------------------------------------------------------

function TramlineRoute.buildJob(geo, vx, vz, hx, hz, opts)
    opts = opts or {}
    if geo == nil or geo.lines == nil or #geo.lines == 0 then return nil, "no tramlines on this field" end
    local order = opts.headlandOrder or "first"
    local dirWay = geo.headlandDir           -- "cw" / "acw": one way round the headland
    local R = geo.radius or 7
    local H = geo.headlands or 0
    local spokes = geo.spokes
    local GAP = 3 * R + 6       -- a lap may stop this short of closing when it leaves via a crossing
    local route = newRoute()

    -- ring paths per level
    local ringPaths = {}
    for level = 1, H do
        ringPaths[level] = {}
        for i, lp in ipairs(geo.rings[level] or {}) do ringPaths[level][i] = G.makePath(lp, true) end
    end
    local function nearestOnRings(x, z, onlyLevel)
        local best = { d = math.huge }
        for level = 1, H do
            if onlyLevel == nil or onlyLevel == level then
                for i, rp in ipairs(ringPaths[level]) do
                    local s, px, pz, d = G.project(rp, x, z)
                    if d < best.d then best = { d = d, level = level, idx = i, s = s, x = px, z = pz } end
                end
            end
        end
        return best
    end
    -- forward ring distance from s0 to s1 travelling 'sense'
    local function ahead(rp, s0, s1, sense)
        if sense > 0 then return (s1 - s0) % rp.length end
        return (s0 - s1) % rp.length
    end
    -- along ring (level, idx) from s0 to s1 (short way forward; nothing if s1
    -- is just behind)
    local function along(level, idx, s0, s1, sense, flags)
        local rp = ringPaths[level][idx]
        local d = ahead(rp, s0, s1, sense)
        if d > rp.length - 0.3 then return end
        flags.rl, flags.ri, flags.rd = level, idx, sense
        add(route, G.walk(rp, s0, s1, sense, false), flags)
    end

    local cur = {}   -- position on a ring: level, idx, s, sense

    local function transit(s1)
        along(cur.level, cur.idx, cur.s, s1, cur.sense, { work = false, kind = "ring" })
        cur.s = s1
    end

    -- fallback where the layout has no crossing: shortest path (off the layout)
    local function connectToRing(level)
        local p = lastPoint(route)
        local prev = route.list[#route.list - 1] or p
        local ux, uz = G.normalize(p[1] - prev[1], p[2] - prev[2])
        local tx, tz = p[1] + ux * R * 2.5, p[2] + uz * R * 2.5
        local tgt = nearestOnRings(tx, tz, level)
        if tgt.level == nil then return false end
        local rp = ringPaths[level][tgt.idx]
        local _, _, rtx, rtz = G.pointAt(rp, tgt.s)
        local sense = (rtx * ux + rtz * uz) >= 0 and 1 or -1
        local path = G.dubins(p[1], p[2], G.atan2(uz, ux), tgt.x, tgt.z, G.atan2(rtz * sense, rtx * sense), R)
        if path ~= nil then add(route, (G.dubinsPoints(path, 0.5)), { work = false, turn = true, kind = "connect" }) end
        cur = { level = level, idx = tgt.idx, s = tgt.s, sense = sense }
        return true
    end

    -- the crossing from the current ring to another level, or nil
    local function crossing(level)
        if spokes == nil or level == cur.level then return nil end
        local a, b = math.min(cur.level, level), math.max(cur.level, level)
        local t = spokes.trans[a] and spokes.trans[a][b]
        if t == nil then return nil end
        if level > cur.level then
            local x = t[cur.sense]
            if x == nil then return nil end
            return { pts = x.pts, sFrom = x.sFrom, sTo = x.sTo, n1 = x.n1 }
        end
        local x = t[-cur.sense]
        if x == nil then return nil end
        return { pts = G.reverse(x.pts), sFrom = x.sTo, sTo = x.sFrom, n1 = #x.pts - x.n1 + 1 }
    end

    -- go to another ring level; 'sweep' = keep the boom on through the first
    -- curve (it covers the bit of ring the lap left out)
    local function moveTo(level, sweep)
        if level == cur.level then return end
        local c = crossing(level)
        if c == nil then connectToRing(level); return end
        transit(c.sFrom)
        local first, rest = {}, {}
        for i, p in ipairs(c.pts) do
            if i <= c.n1 then first[#first + 1] = p end
            if i >= c.n1 then rest[#rest + 1] = p end
        end
        add(route, first, { work = sweep == true, turn = true, kind = "crossing" })
        add(route, rest, { work = false, turn = true, kind = "crossing" })
        cur = { level = level, idx = 1, s = c.sTo, sense = cur.sense }
    end

    -- one working lap of the current ring; if the way on to 'nextLevel' leaves
    -- the ring just before the lap closes, stop there (returns true)
    -- a working stretch of 'dist' metres along the current ring from cur.s;
    -- with opts.corners, backing into each corner on the way: boom off round
    -- the curve, run out, back along the next side into the corner, spray out
    local useCorners = opts.corners == true and geo.corners ~= nil
    local function workAlong(dist)
        local rp = ringPaths[cur.level][cur.idx]
        local level, idx, sense, s0 = cur.level, cur.idx, cur.sense, cur.s
        local tags = { rl = level, ri = idx, rd = sense }
        local function flags(work, turn) return { work = work, turn = turn, kind = "headland", rl = tags.rl, ri = tags.ri, rd = tags.rd } end
        local list = {}
        if useCorners then
            for _, c in ipairs((geo.corners[level] or {})[idx] or {}) do
                local m = c.m and c.m[sense]
                if m ~= nil then
                    local d1, d2 = ahead(rp, s0, m.sStart, sense), ahead(rp, s0, m.sQ, sense)
                    if d1 > 0.5 and d2 > d1 and d2 < dist - 0.5 then list[#list + 1] = { m = m, d = d1 } end
                end
            end
            table.sort(list, function(p, q) return p.d < q.d end)
        end
        local s = s0
        for _, e in ipairs(list) do
            local m = e.m
            local rev, fwd = TramlineRoute.cornerPieces(m, opts.boomBack)
            add(route, G.walk(rp, s, m.sStart, sense, false), flags(true))
            add(route, G.walk(rp, m.sStart, m.sQ, sense, false), flags(false, true))
            add(route, rev, { work = false, turn = true, kind = "corner", rev = true })
            add(route, fwd, { work = true, kind = "corner" })
            s = m.sResume
        end
        local sEnd = (s0 + sense * dist) % rp.length
        if #list == 0 and dist >= rp.length - 1e-6 then
            add(route, G.walk(rp, s0, s0, sense, true), flags(true))
        else
            add(route, G.walk(rp, s, sEnd, sense, false), flags(true))
        end
        cur.s = sEnd
    end

    local function lap(nextLevel)
        local rp = ringPaths[cur.level][cur.idx]
        local c = nextLevel and crossing(nextLevel)
        if c ~= nil then
            local D = ahead(rp, cur.s, c.sFrom, cur.sense)
            if D >= rp.length - GAP then
                workAlong(D)
                cur.s = c.sFrom
                return true
            end
        end
        local s0 = cur.s
        workAlong(rp.length)
        cur.s = s0
        return false
    end

    local function headlandLaps()
        if H == 0 then return end
        -- the ring we are on first, then outwards, then inwards
        local levels = { cur.level }
        for l = cur.level - 1, 1, -1 do levels[#levels + 1] = l end
        for l = cur.level + 1, H do levels[#levels + 1] = l end
        for i, level in ipairs(levels) do
            local nextLevel = levels[i + 1]
            if nextLevel == nil and order == "first" and level ~= H then nextLevel = H end
            local short = lap(nextLevel)
            if nextLevel ~= nil then moveTo(nextLevel, short) end
        end
    end

    ---------------------------------------------------------------------------
    -- start
    ---------------------------------------------------------------------------
    if H > 0 then
        local st = nearestOnRings(vx, vz)
        if st.level == nil or st.d > TramlineRoute.MAX_START_DIST then
            return nil, "drive onto the headland tramline to start"
        end
        local rp = ringPaths[st.level][st.idx]
        local _, _, tx, tz = G.pointAt(rp, st.s)
        local sense = ((hx or 1) * tx + (hz or 0) * tz) >= 0 and 1 or -1
        if dirWay ~= nil then
            -- the field's headland direction: start facing that way round
            local want = TramlineGenerator.dirSense(geo.rings[st.level][st.idx], dirWay)
            if ((hx or 1) * tx + (hz or 0) * tz) * want < -0.3 then
                return nil, string.format("face %s along the headland tramline to start (this field goes %s)",
                    dirWay == "cw" and "clockwise" or "anticlockwise", dirWay == "cw" and "clockwise" or "anticlockwise")
            end
            sense = want
        end
        cur = { level = st.level, idx = st.idx, s = st.s, sense = sense }
        add(route, { { st.x, st.z } }, { work = false, kind = "start" })
        if order == "first" then
            headlandLaps()
        else
            moveTo(H)
        end
    end

    ---------------------------------------------------------------------------
    -- blocks
    ---------------------------------------------------------------------------
    local done = {}
    local remaining = #geo.blocks
    local guard = 0
    while remaining > 0 and guard < #geo.blocks + 2 do
        guard = guard + 1
        -- the block end to enter next: first one ahead on the ring
        local bestB, bestE, bestDist = nil, nil, math.huge
        for b, blk in ipairs(geo.blocks) do
            if not done[b] then
                for e = 1, 2 do
                    local E = blk.ends[e]
                    if H > 0 then
                        local en = E.entry[cur.sense]
                        if en ~= nil and (E.ring or 1) == cur.idx then
                            local rp = ringPaths[H][cur.idx]
                            local d = ahead(rp, cur.s, en.s, cur.sense)
                            if d > rp.length - 0.3 then d = 0 end
                            if d < bestDist then bestB, bestE, bestDist = b, e, d end
                        end
                    else
                        local p = lastPoint(route)
                        local d = p and sqrt((E.x - p[1]) ^ 2 + (E.z - p[2]) ^ 2) or sqrt((E.x - vx) ^ 2 + (E.z - vz) ^ 2)
                        if d < bestDist then bestB, bestE, bestDist = b, e, d end
                    end
                end
            end
        end
        if bestB == nil then break end
        local blk = geo.blocks[bestB]
        local E = blk.ends[bestE]
        local reverse = (bestE == 2)
        local X = blk.ends[reverse and 1 or 2]
        done[bestB] = true
        remaining = remaining - 1

        -- how we leave the block (needed now: the exit curve shortens the last line):
        -- the direction that reaches the next block soonest
        local exitSense, exitJ = cur.sense, nil
        if H > 0 then
            local bestD = math.huge
            local senses = { cur.sense, -cur.sense }
            if dirWay ~= nil and X.exit[cur.sense] ~= nil then senses = { cur.sense } end    -- one way round
            for _, sense in ipairs(senses) do
                local xj = X.exit[sense]
                if xj ~= nil then
                    local d = 0
                    if remaining > 0 then
                        d = math.huge
                        local rp = ringPaths[H][X.ring or cur.idx]
                        for b2, blk2 in ipairs(geo.blocks) do
                            if not done[b2] then
                                for e2 = 1, 2 do
                                    local en2 = blk2.ends[e2].entry[sense]
                                    if en2 ~= nil then d = min(d, ahead(rp, xj.s, en2.s, sense)) end
                                end
                            end
                        end
                    end
                    if d < bestD - 0.5 then bestD, exitSense, exitJ = d, sense, xj end
                end
            end
        end

        -- get to the block
        local startTrim = 0
        if H > 0 then
            local en = E.entry[cur.sense]
            transit(en.s)
            add(route, en.pts, { work = false, turn = true, kind = "junction" })
            startTrim = en.trim
        else
            local p = lastPoint(route)
            local px, pz, ph
            if p == nil then
                px, pz, ph = vx, vz, G.atan2(hz or 0, hx or 1)
                if (E.x - vx) ^ 2 + (E.z - vz) ^ 2 > 40 * 40 then return nil, "drive to the end of a tramline to start" end
                add(route, { { vx, vz } }, { work = false, kind = "start" })
            else
                local prev = route.list[#route.list - 1] or p
                local ux, uz = G.normalize(p[1] - prev[1], p[2] - prev[2])
                px, pz, ph = p[1], p[2], G.atan2(uz, ux)
            end
            local path = G.dubins(px, pz, ph, E.x, E.z, G.atan2(-E.uz, -E.ux), R)
            if path ~= nil then add(route, (G.dubinsPoints(path, 0.5)), { work = false, turn = true, kind = "approach" }) end
        end

        -- drive the block
        local m = #blk.lines
        local seq = {}
        if reverse then for j = m, 1, -1 do seq[#seq + 1] = j end else for j = 1, m do seq[#seq + 1] = j end end
        for qi, j in ipairs(seq) do
            local l = geo.lines[blk.lines[j]]
            local side = entrySideForward(j)
            if reverse then side = otherSide(side) end
            local conn = nil
            if qi < #seq then conn = blk.conns[reverse and (j - 1) or j] end
            local rc = nil     -- ring turn in driving order
            if conn ~= nil and conn.kind == "ring" then
                if not reverse then
                    rc = { exitArc = conn.exitArc, exitTrim = conn.exitTrim, s0 = conn.s0,
                           entryArc = conn.entryArc, entryTrim = conn.entryTrim, s1 = conn.s1, sense = conn.sense }
                else
                    rc = { exitArc = G.reverse(conn.entryArc), exitTrim = conn.entryTrim, s0 = conn.s1,
                           entryArc = G.reverse(conn.exitArc), entryTrim = conn.exitTrim, s1 = conn.s0, sense = -conn.sense }
                end
            end
            local endTrim = 0
            if rc ~= nil then
                endTrim = rc.exitTrim
            elseif qi == #seq and exitJ ~= nil then
                endTrim = exitJ.trim
            end
            driveLine(route, l, side, startTrim, endTrim)
            startTrim = 0
            if conn ~= nil then
                if conn.kind == "loop" then
                    local pts = conn.pts
                    if reverse then pts = G.reverse(pts) end
                    add(route, pts, { work = false, turn = true, kind = "loop" })
                else
                    add(route, rc.exitArc, { work = false, turn = true, kind = "junction" })
                    along(H, conn.ringIndex, rc.s0, rc.s1, rc.sense, { work = false, turn = true, kind = "ring" })
                    add(route, rc.entryArc, { work = false, turn = true, kind = "junction" })
                    startTrim = rc.entryTrim
                end
            end
        end

        -- leave the block
        if H > 0 and exitJ ~= nil then
            add(route, exitJ.pts, { work = false, turn = true, kind = "junction" })
            cur = { level = H, idx = X.ring or cur.idx, s = exitJ.s, sense = exitSense }
        end
    end

    ---------------------------------------------------------------------------
    -- finish
    ---------------------------------------------------------------------------
    if H > 0 then
        if order == "last" then headlandLaps() end
        -- run out a little along the ring so the vehicle ends on the tramline
        transit((cur.s + cur.sense * 2 * R) % ringPaths[cur.level][cur.idx].length)
    end

    local list = route.list
    if #list < 2 then return nil, "nothing to do" end
    return list
end

-------------------------------------------------------------------------------
-- Carrying on after an interruption
-------------------------------------------------------------------------------

-- fingerprint of a layout: a stored job only fits the layout it was made on
function TramlineRoute.layoutSignature(geo)
    if geo == nil or geo.lines == nil or #geo.lines == 0 then return "" end
    local n, tot = #geo.lines, 0
    for _, l in ipairs(geo.lines) do tot = tot + (l.len or 0) end
    local a, b = geo.lines[1], geo.lines[n]
    return string.format("%d/%d/%.1f/%.0f/%.0f,%.0f/%.0f,%.0f%s", n, geo.headlands or 0, geo.spacing or 0, tot,
        a[1], a[2], b[3], b[4], geo.headlandDir ~= nil and ("/" .. geo.headlandDir) or "")
end

local function segLen(a, b) return sqrt((b[1] - a[1]) ^ 2 + (b[2] - a[2]) ^ 2) end

-- Where a vehicle at (vx,vz) facing (hx,hz) can (re)join a job route: the
-- first segment from a little before 'fromIx' on that is within reach and
-- points the way the vehicle faces.
-- Returns { ix (segment start), skipped (metres of spraying passed over
-- since fromIx), x, z, dirX, dirZ (where it joins) } or nil.
-- metres of spraying along the route from (segment i1, fraction t1) to
-- (segment i2, fraction t2); segment k runs from route[k] to route[k + 1]
local function workBetween(route, i1, t1, i2, t2)
    if i2 < i1 or (i2 == i1 and t2 <= t1) then return 0 end
    local total = 0
    for k = i1, i2 do
        local a, b = route[k], route[k + 1]
        if b ~= nil and b.work then
            local ta = (k == i1) and t1 or 0
            local tb = (k == i2) and t2 or 1
            total = total + max(0, tb - ta) * segLen(a, b)
        end
    end
    return total
end

-- stopX/stopZ (optional): where the job stopped, on segment fromIx
function TramlineRoute.resumeIndex(route, fromIx, vx, vz, hx, hz, maxDist, stopX, stopZ)
    maxDist = maxDist or TramlineRoute.MAX_START_DIST
    local n = #route
    if n < 2 then return nil end
    fromIx = max(1, min(fromIx or 1, n - 1))
    local i0, back = fromIx, 0
    while i0 > 1 and back < 30 do
        back = back + segLen(route[i0 - 1], route[i0])
        i0 = i0 - 1
    end
    -- the first stretch of route within reach (pointing our way), and in it
    -- the piece the vehicle is actually beside
    local best = nil
    for i = i0, n - 1 do
        local a, b = route[i], route[i + 1]
        local d, t = G.distPointSeg(vx, vz, a[1], a[2], b[1], b[2])
        local dx, dz, l = G.normalize(b[1] - a[1], b[2] - a[2])
        local ok = d <= maxDist and l > 0.05 and dx * (hx or 0) + dz * (hz or 0) > 0.5
        if ok then
            if best == nil or d < best.d - 0.01 then
                best = { i = i, d = d, t = t, dx = dx, dz = dz, l = l }
            end
        elseif best ~= nil and l > 0.05 then
            break
        end
    end
    if best == nil then return nil end
    local i, a, b = best.i, route[best.i], route[best.i + 1]
    local tStop = 0
    if stopX ~= nil and route[fromIx + 1] ~= nil then
        local p, q = route[fromIx], route[fromIx + 1]
        local _, ts = G.distPointSeg(stopX, stopZ, p[1], p[2], q[1], q[2])
        tStop = ts or 0
    end
    local skipped = workBetween(route, fromIx, tStop, i, best.t)
    return { ix = i, skipped = skipped, x = a[1] + (b[1] - a[1]) * best.t, z = a[2] + (b[2] - a[2]) * best.t,
             dirX = best.dx, dirZ = best.dz }
end

-- Job routes a new job can join mid-field: one for each block end and ring
-- direction, each starting straight into that block. Starting mid-field
-- means the headland was done first, so it is left out - unless the set
-- sprays it last. Cached on the layout.
function TramlineRoute.joinCandidates(geo, order, corners, boomBack)
    local key = (order == "last") and "last" or "off"
    local cacheKey = key .. (corners and string.format("+corners%.1f", boomBack or TramlineRoute.BOOM_BACK) or "")
    geo.joinCache = geo.joinCache or {}
    if geo.joinCache[cacheKey] ~= nil then return geo.joinCache[cacheKey] end
    local list = {}
    local H = geo.headlands or 0
    for _, blk in ipairs(geo.blocks or {}) do
        for _, E in ipairs(blk.ends or {}) do
            local starts = {}
            if H > 0 then
                local lp = geo.rings[H] and geo.rings[H][E.ring or 1]
                if lp ~= nil then
                    local rp = lp.path
                    if rp == nil then rp = G.makePath(lp, true); lp.path = rp end
                    for _, sense in ipairs({ 1, -1 }) do
                        local en = E.entry and E.entry[sense]
                        if en ~= nil then
                            local px, pz, tx, tz = G.pointAt(rp, en.s)
                            starts[#starts + 1] = { px, pz, tx * sense, tz * sense }
                        end
                    end
                end
            else
                starts[#starts + 1] = { E.x + E.ux * 3, E.z + E.uz * 3, -E.ux, -E.uz }
            end
            for _, st in ipairs(starts) do
                local route = TramlineRoute.tagIndices(TramlineRoute.buildJob(geo, st[1], st[2], st[3], st[4],
                    { headlandOrder = key, corners = corners, boomBack = boomBack }))
                if route ~= nil then
                    list[#list + 1] = { route = route, sx = st[1], sz = st[2], hx = st[3], hz = st[4], order = key,
                                        corners = corners, boomBack = boomBack }
                end
            end
        end
    end
    geo.joinCache[cacheKey] = list
    return list
end

-- A new job joining the layout where the vehicle is (on a main tramline or
-- a turn): the candidate that passes over the least spraying before it.
-- Returns { cand, at = resumeIndex result } or nil
function TramlineRoute.findJoin(geo, order, vx, vz, hx, hz, corners, boomBack)
    local best = nil
    for _, c in ipairs(TramlineRoute.joinCandidates(geo, order, corners, boomBack)) do
        local at = TramlineRoute.resumeIndex(c.route, 1, vx, vz, hx, hz)
        if at ~= nil and (best == nil or at.skipped < best.at.skipped) then
            best = { cand = c, at = at }
        end
    end
    return best
end

-------------------------------------------------------------------------------
-- Getting about on the headland tramlines (refill trips)
-------------------------------------------------------------------------------

-- job route waypoints remember their index (progress is kept in these)
function TramlineRoute.tagIndices(route)
    if route ~= nil then for i, w in ipairs(route) do w.jix = i end end
    return route
end

-- ring paths of a layout, [level][idx], cached on it
function TramlineRoute.ringPathsOf(geo)
    if geo.ringPathCache == nil then
        local all = {}
        for level = 1, geo.headlands or 0 do
            all[level] = {}
            for i, lp in ipairs(geo.rings[level] or {}) do
                if lp.path == nil then lp.path = G.makePath(lp, true) end
                all[level][i] = lp.path
            end
        end
        geo.ringPathCache = all
    end
    return geo.ringPathCache
end

-- the headland tramline a point lies on: { level, idx, s, d } or nil
local function ringAt(geo, x, z, tol)
    local best = nil
    for level, list in pairs(TramlineRoute.ringPathsOf(geo)) do
        for idx, rp in ipairs(list) do
            local s, _, _, d = G.project(rp, x, z)
            if d <= (tol or 0.3) and (best == nil or d < best.d) then
                best = { level = level, idx = idx, s = s, d = d }
            end
        end
    end
    return best
end

-- direction of travel (+1 / -1) on ring path rp at s when heading (dx, dz)
local function senseAt(rp, s, dx, dz)
    local _, _, tx, tz = G.pointAt(rp, s)
    return (tx * dx + tz * dz) >= 0 and 1 or -1
end

-- segment i -> i + 1 of a route runs along a headland tramline: ring position or nil
local function ringSegment(geo, route, i)
    local a, b = route[i], route[i + 1]
    if a == nil or b == nil then return nil end
    if b.rl ~= nil then
        local list = TramlineRoute.ringPathsOf(geo)[b.rl]
        local rp = list and list[b.ri]
        if rp == nil then return nil end
        local s, _, _, d = G.project(rp, a[1], a[2])
        if d > 0.5 then return nil end
        return { level = b.rl, idx = b.ri, s = s, d = d, sense = b.rd }
    end
    if b.kind ~= "ring" and b.kind ~= "headland" then return nil end
    local ra = ringAt(geo, a[1], a[2], 0.3)
    if ra == nil then return nil end
    local rb = ringAt(geo, b[1], b[2], 0.3)
    if rb == nil or rb.level ~= ra.level or rb.idx ~= ra.idx then return nil end
    local rp = TramlineRoute.ringPathsOf(geo)[ra.level][ra.idx]
    ra.sense = senseAt(rp, ra.s, b[1] - a[1], b[2] - a[2])
    return ra
end

-- the crossing from one headland pass to another, travelling 'sense'
local function crossingOf(geo, fromLevel, toLevel, sense)
    local spokes = geo.spokes
    if spokes == nil or fromLevel == toLevel then return nil end
    local a, b = min(fromLevel, toLevel), max(fromLevel, toLevel)
    local t = spokes.trans[a] and spokes.trans[a][b]
    if t == nil then return nil end
    if toLevel > fromLevel then
        local x = t[sense]
        if x == nil then return nil end
        return { pts = x.pts, sFrom = x.sFrom, sTo = x.sTo }
    end
    local x = t[-sense]
    if x == nil then return nil end
    return { pts = G.reverse(x.pts), sFrom = x.sTo, sTo = x.sFrom }
end

-- Points along the headland tramlines from 'from' = { level, idx, s, sense }
-- to s1 on ring (toLevel, toIdx), keeping the direction of travel (changing
-- pass at the crossing). nil if there is no way.
function TramlineRoute.navRings(geo, from, toLevel, toIdx, s1)
    local paths = TramlineRoute.ringPathsOf(geo)
    local pts = {}
    local cur = { level = from.level, idx = from.idx, s = from.s, sense = from.sense }
    local function walkTo(target)
        local rp = paths[cur.level] and paths[cur.level][cur.idx]
        if rp == nil then return false end
        local d = (cur.sense > 0) and ((target - cur.s) % rp.length) or ((cur.s - target) % rp.length)
        if d > 0.3 and d < rp.length - 0.3 then
            for _, p in ipairs(G.walk(rp, cur.s, target, cur.sense, false)) do
                p.rl, p.ri, p.rd = cur.level, cur.idx, cur.sense
                pts[#pts + 1] = p
            end
        end
        cur.s = target
        return true
    end
    if cur.level ~= toLevel then
        local c = crossingOf(geo, cur.level, toLevel, cur.sense)
        if c == nil then return nil end
        if not walkTo(c.sFrom) then return nil end
        for _, p in ipairs(c.pts) do pts[#pts + 1] = p end
        cur.level, cur.idx, cur.s = toLevel, 1, c.sTo
    end
    if cur.idx ~= toIdx then return nil end
    if not walkTo(s1) then return nil end
    return pts, cur
end

-- The field gate: an AutoDrive marker at the field edge. The way in and out
-- is straight between the gate G and the nearest point J of the outer
-- headland tramline, curving onto it either way round.
--   { x, z (gate), jx, jz, idx, sJ, dist, onRing, inArc = { [sense] = junction } }
--   inArc[sense]: from the straight (heading in) onto the ring, then 'sense'
-- nil, reason if the gate is too far from the headland tramline
function TramlineRoute.gate(geo, gx, gz)
    if (geo.headlands or 0) < 1 then return nil, "the field needs a headland tramline" end
    local paths = TramlineRoute.ringPathsOf(geo)
    local best = nil
    for idx, rp in ipairs(paths[1] or {}) do
        local s, px, pz, d = G.project(rp, gx, gz)
        if best == nil or d < best.d then best = { idx = idx, s = s, x = px, z = pz, d = d } end
    end
    if best == nil then return nil, "the field has no headland tramline" end
    if best.d > TramlineRoute.GATE_MAX_DIST then
        return nil, string.format("the AutoDrive target is %.0f m from this field's headland tramline", best.d)
    end
    local g = { x = gx, z = gz, jx = best.x, jz = best.z, idx = best.idx, sJ = best.s, dist = best.d, inArc = {} }
    if best.d < 1.5 then
        g.onRing = true        -- the marker sits on the headland tramline itself
        return g
    end
    local ux, uz = G.normalize(best.x - gx, best.z - gz)
    for _, sense in ipairs({ 1, -1 }) do
        g.inArc[sense] = G.junction(paths[1][best.idx], best.s, best.x, best.z, ux, uz, sense, geo.radius or 7, 0.5)
    end
    return g
end

-- polylines to show the gate on the overlay
function TramlineRoute.gateLines(g)
    if g == nil then return {} end
    local out = { { { g.x, g.z }, { g.jx, g.jz } } }
    for _, j in pairs(g.inArc) do out[#out + 1] = j.pts end
    return out
end

-- From where the vehicle is on a job route (segment fromIx) out to the field
-- gate, on the tramlines all the way: on along the job until it runs on a
-- headland tramline, round to the gate, out to the gate. Nothing sprayed.
-- Returns the route or nil, reason
function TramlineRoute.exitRoute(geo, route, fromIx, vx, vz, gate)
    local out = newRoute()
    add(out, { { vx, vz } }, { work = false, kind = "exit" })
    local ring = nil
    for i = max(1, fromIx), #route - 1 do
        local r = ringSegment(geo, route, i)
        if r ~= nil then
            if i == fromIx then
                -- on it already: from a metre ahead (a sideways hop to the
                -- line under the vehicle could never be passed)
                local rp = TramlineRoute.ringPathsOf(geo)[r.level][r.idx]
                r.s = ((G.project(rp, vx, vz)) + r.sense * 1.0) % rp.length
            end
            ring = r
            break
        end
        local b = route[i + 1]
        add(out, { b }, { work = false, turn = b.turn, kind = "exit", rev = b.rev })
    end
    if ring == nil then return nil, "no way to the headland tramline from here" end
    if gate.onRing then
        local pts = TramlineRoute.navRings(geo, ring, 1, gate.idx, gate.sJ)
        if pts == nil then return nil, "no way round the headland tramline to the gate" end
        add(out, pts, { work = false, kind = "exit" })
        return out.list
    end
    local arc = gate.inArc[-ring.sense]            -- reversed: off the ring towards the gate
    if arc == nil then return nil, "no curve off the headland tramline to the gate" end
    local pts = TramlineRoute.navRings(geo, ring, 1, gate.idx, arc.s)
    if pts == nil then return nil, "no way round the headland tramline to the gate" end
    add(out, pts, { work = false, kind = "exit" })
    add(out, G.reverse(arc.pts), { work = false, turn = true, kind = "exit" })
    add(out, { { gate.x, gate.z } }, { work = false, kind = "exit" })
    return out.list
end

-- Back in from the field gate (vehicle at vx,vz facing hx,hz) to where a job
-- stopped (segment stopIx of the job route, at stopX/Z), on the tramlines:
-- in from the gate onto the headland tramline, round to where the job last
-- ran along a headland tramline before it stopped, and on along the job.
-- Nothing is sprayed before the stop point; from there on the job's own
-- waypoints are used (they keep their 'jix'). Returns the route or nil, reason
function TramlineRoute.returnRoute(geo, route, stopIx, stopX, stopZ, gate, vx, vz, hx, hz)
    local paths = TramlineRoute.ringPathsOf(geo)
    -- gate on the ring itself: we can only go the way the vehicle faces
    local need = nil
    if gate.onRing then need = senseAt(paths[1][gate.idx], gate.sJ, hx, hz) end
    local k, ring = nil, nil
    for i = min(stopIx, #route - 1), 1, -1 do
        local r = ringSegment(geo, route, i)
        if r ~= nil and (need == nil or r.sense == need) then k, ring = i, r; break end
    end
    if k == nil then return nil, "no way back along the tramlines" end
    local out = newRoute()
    add(out, { { vx, vz } }, { work = false, kind = "return" })
    local entry = { level = 1, idx = gate.idx, sense = ring.sense }
    if gate.onRing then
        entry.s = (G.project(paths[1][gate.idx], vx, vz))
    else
        local arc = gate.inArc[ring.sense]
        if arc == nil then return nil, "no curve from the gate onto the headland tramline" end
        add(out, arc.pts, { work = false, turn = true, kind = "return" })
        entry.s = arc.s
    end
    local pts = TramlineRoute.navRings(geo, entry, ring.level, ring.idx, ring.s)
    if pts == nil then return nil, "no way round the headland tramline" end
    add(out, pts, { work = false, kind = "return" })
    for i = k + 1, stopIx do
        local w = route[i]
        add(out, { w }, { work = false, turn = w.turn, kind = w.kind })
    end
    add(out, { { stopX or route[stopIx][1], stopZ or route[stopIx][2] } }, { work = false, kind = "line" })
    local list = out.list
    for i = stopIx + 1, #route do list[#list + 1] = route[i] end
    return list
end

-------------------------------------------------------------------------------
-- Back to where a job stopped, from wherever the vehicle is
--   on the job itself before the stop -> along the job
--   on a main tramline                -> to its end, onto the headland tramline
--   on a headland tramline            -> along it the way the vehicle faces
--   at the field edge / gate          -> straight in onto the nearest one
-- then round the headland tramlines to where the job last ran along one
-- before it stopped, and on along the job to the stop point. Nothing is
-- sprayed before the stop point; from there the job's own waypoints follow.
-------------------------------------------------------------------------------

TramlineRoute.BACK_LINE_DIST = 4      -- m beside a main tramline to count as on it
TramlineRoute.BACK_RING_DIST = 8      -- m beside a headland tramline to count as on it
TramlineRoute.BACK_LINK_DIST = 60     -- m: longest straight in onto a headland tramline
TramlineRoute.BACK_GATE_DIST = 25     -- m from the refill gate: may come in from off the field
TramlineRoute.BACK_COS       = math.cos(math.rad(35))

local function polyLen(pts, x, z)
    local t = 0
    for _, p in ipairs(pts) do
        if x ~= nil then t = t + sqrt((p[1] - x) ^ 2 + (p[2] - z) ^ 2) end
        x, z = p[1], p[2]
    end
    return t
end

-- metres of spraying on a job route before waypoint ix, and in all
function TramlineRoute.progress(route, ix)
    if route == nil then return 0, 0 end
    if route.workTotal == nil then
        local t = 0
        for i = 2, #route do if route[i].work then t = t + segLen(route[i - 1], route[i]) end end
        route.workTotal = t
    end
    local done = 0
    for i = 2, min(ix or 1, #route) do if route[i].work then done = done + segLen(route[i - 1], route[i]) end end
    return done, route.workTotal
end

-- metres of route from waypoint ix to the end (cumulative lengths kept on the route)
function TramlineRoute.lengthFrom(route, ix)
    if route == nil or #route < 2 then return 0 end
    local cum = route.cumLen
    if cum == nil or #cum ~= #route then
        cum = { 0 }
        for i = 2, #route do cum[i] = cum[i - 1] + segLen(route[i - 1], route[i]) end
        route.cumLen = cum
    end
    ix = max(1, min(ix or 1, #route))
    return cum[#route] - cum[ix]
end

-- length of navRings(geo, from, toLevel, toIdx, s1) without building it
-- (nil = no way); xc: cache for the crossings between passes
local function navCost(geo, from, toLevel, toIdx, s1, xc)
    local paths = TramlineRoute.ringPathsOf(geo)
    local level, idx, s, sense = from.level, from.idx, from.s, from.sense
    local function aheadOn(rp, a, b)
        local d = (sense > 0) and ((b - a) % rp.length) or ((a - b) % rp.length)
        if d <= 0.3 or d >= rp.length - 0.3 then return 0 end
        return d
    end
    local total = 0
    if level ~= toLevel then
        local key = level .. ">" .. toLevel .. ":" .. sense
        local c = xc and xc[key]
        if c == nil then
            local x = crossingOf(geo, level, toLevel, sense)
            c = x and { sFrom = x.sFrom, sTo = x.sTo, len = polyLen(x.pts) } or false
            if xc then xc[key] = c end
        end
        local rp = paths[level] and paths[level][idx]
        if not c or rp == nil then return nil end
        total = aheadOn(rp, s, c.sFrom) + c.len
        level, idx, s = toLevel, 1, c.sTo
    end
    if idx ~= toIdx then return nil end
    local rp = paths[level] and paths[level][idx]
    if rp == nil then return nil end
    return total + aheadOn(rp, s, s1)
end

-- Turning round on a main tramline: in at one end from the headland tramline
-- (travelling sIn), across, and out at the other end travelling sOut.
-- { l, jin (exit curve at the entry end, reversed when driven), jout, sIn, sOut, len }
local function lineTurns(geo)
    if geo.turnCache ~= nil then return geo.turnCache end
    local list = {}
    for _, l in ipairs(geo.lines or {}) do
        if l.exits ~= nil then
            for _, X in ipairs({ "A", "B" }) do
                local X2 = (X == "A") and "B" or "A"
                for _, sIn in ipairs({ 1, -1 }) do
                    local jin = l.exits[X][-sIn]
                    if jin ~= nil then
                        for _, sOut in ipairs({ 1, -1 }) do
                            local jout = l.exits[X2][sOut]
                            if jout ~= nil and l.len - jin.a - jout.a > 1 then
                                list[#list + 1] = { l = l, jin = jin, jout = jout, sIn = sIn, sOut = sOut,
                                                    len = polyLen(jin.pts) + (l.len - jin.a - jout.a) + polyLen(jout.pts) }
                            end
                        end
                    end
                end
            end
        end
    end
    geo.turnCache = list
    return list
end

-- inside the innermost headland tramline (the cropped main area)?
local function insideMain(geo, x, z)
    local count = 0
    for _, lp in ipairs(geo.rings[geo.headlands or 0] or {}) do
        if G.pointInPolygon(lp, x, z) then count = count + 1 end
    end
    return count % 2 == 1
end

-- Ways onto the headland tramlines from (vx, vz) facing (hx, hz):
-- { level, idx, s, sense, pts = how to get there, len, how }
-- opts.strict: the vehicle is on the layout (driving a route): only from the
-- tramline it is exactly on, no links across the field
local function backEntries(geo, vx, vz, hx, hz, opts)
    local out = {}
    local strict = opts.strict == true
    local lineDist = strict and 0.6 or TramlineRoute.BACK_LINE_DIST
    local lineCos = strict and math.cos(math.rad(15)) or TramlineRoute.BACK_COS
    local H = geo.headlands or 0
    if H < 1 then return out end
    local paths = TramlineRoute.ringPathsOf(geo)
    local R = geo.radius or 7

    -- on a main tramline, facing along it: on to its end and the curve onto the ring
    for _, l in ipairs(geo.lines or {}) do
        if l.exits ~= nil then
            local d, t = G.distPointSeg(vx, vz, l[1], l[2], l[3], l[4])
            if d <= lineDist then
                local ux, uz = G.normalize(l[3] - l[1], l[4] - l[2])
                local c = ux * hx + uz * hz
                if abs(c) >= lineCos then
                    local side = c > 0 and "B" or "A"
                    local toEnd = (side == "B") and (1 - t) * l.len or t * l.len
                    for _, sense in ipairs({ 1, -1 }) do
                        local j = l.exits[side][sense]
                        local idx = j and j.ring or 1
                        if j ~= nil and toEnd >= j.a + 1 and paths[H][idx] ~= nil then
                            local p0 = j.pts[1]
                            out[#out + 1] = { level = H, idx = idx, s = j.s, sense = sense, pts = j.pts, turn = true, how = "line",
                                              len = sqrt((p0[1] - vx) ^ 2 + (p0[2] - vz) ^ 2) + polyLen(j.pts) }
                        end
                    end
                end
            end
        end
    end

    -- the nearest headland tramline
    local near = nil
    for level = 1, H do
        for idx, rp in ipairs(paths[level] or {}) do
            local s, px, pz, d = G.project(rp, vx, vz)
            if near == nil or d < near.d then near = { level = level, idx = idx, s = s, x = px, z = pz, d = d, rp = rp } end
        end
    end
    if near == nil then return out end
    local _, _, tx, tz = G.pointAt(near.rp, near.s)
    local c = tx * hx + tz * hz

    -- on it (or right beside it), facing along it: on the way we face
    if strict then
        if near.d <= 0.6 and abs(c) >= 0.9 then
            local sense = c >= 0 and 1 or -1
            out[#out + 1] = { level = near.level, idx = near.idx, s = (near.s + sense * 1) % near.rp.length, sense = sense,
                              pts = {}, len = 1 + near.d, how = "ring" }
        end
        return out
    end
    if near.d <= TramlineRoute.BACK_RING_DIST and (abs(c) >= 0.5 or near.d < 1.5) then
        local sense = c >= 0 and 1 or -1
        local ahead = max(3, near.d * 1.5)
        out[#out + 1] = { level = near.level, idx = near.idx, s = (near.s + sense * ahead) % near.rp.length, sense = sense,
                          pts = {}, len = ahead + near.d, how = "ring" }
    end

    -- off the layout (field edge, gate): straight in, curving onto it either way.
    -- Only from the field (or near the refill gate), not through the crop.
    local inField = geo.polygon ~= nil and (G.pointInPolygon(geo.polygon, vx, vz) or G.distToPolyline(geo.polygon, true, vx, vz) <= 3)
    local g = opts.gate
    local nearGate = g ~= nil and (vx - g.x) ^ 2 + (vz - g.z) ^ 2 <= TramlineRoute.BACK_GATE_DIST ^ 2
    local linkOk = (inField or nearGate) and not (insideMain(geo, vx, vz) and near.d > TramlineRoute.BACK_RING_DIST)
    if linkOk and near.d >= 1.5 and near.d <= TramlineRoute.BACK_LINK_DIST then
        local ux, uz = G.normalize(near.x - vx, near.z - vz)
        if ux * hx + uz * hz >= 0.3 then
            for _, sense in ipairs({ 1, -1 }) do
                local j = G.junction(near.rp, near.s, near.x, near.z, ux, uz, sense, R, 0.5)
                if j ~= nil and j.a <= near.d + 1 then
                    local p0 = j.pts[1]
                    out[#out + 1] = { level = near.level, idx = near.idx, s = j.s, sense = sense, pts = j.pts, turn = true, how = "edge",
                                      len = sqrt((p0[1] - vx) ^ 2 + (p0[2] - vz) ^ 2) + polyLen(j.pts) }
                end
            end
        end
    end

    -- last resort, close by: the shortest drivable curve onto it, ahead
    if #out == 0 and linkOk and near.d <= 20 then
        local s2, px, pz = G.project(near.rp, vx + hx * R * 2.5, vz + hz * R * 2.5)
        local _, _, rtx, rtz = G.pointAt(near.rp, s2)
        local sense = (rtx * hx + rtz * hz) >= 0 and 1 or -1
        local path = G.dubins(vx, vz, G.atan2(hz, hx), px, pz, G.atan2(rtz * sense, rtx * sense), R)
        if path ~= nil then
            local pts = G.dubinsPoints(path, 0.5)
            out[#out + 1] = { level = near.level, idx = near.idx, s = s2, sense = sense, pts = pts, turn = true, how = "connect",
                              len = polyLen(pts, vx, vz) }
        end
    end
    return out
end

-- The way back from (vx, vz) facing (hx, hz) to where the job stopped
-- (segment stopIx of the job route, at stopX/Z), and on with the job.
-- opts.gate = { x, z }: the refill gate (may come in from off the field there)
-- Returns route, { total = metres to the stop point, how, n = waypoints
-- before the job's own } or nil, reason
-- The shortest way from one of 'entries' (ways onto the headland tramlines,
-- see backEntries) to one of 'targets' ({ level, idx, s, sense, cost = metres
-- still to go from there }) round the headland tramlines - or turning round
-- on a main tramline (across the field and back onto the headland tramline
-- going the other way) where that is shorter or the only way.
-- Returns { entry, target, lineTurn, total } or nil
local function bestWay(geo, entries, targets)
    local best = nil
    local xc = {}
    for _, e in ipairs(entries) do
        for _, tg in ipairs(targets) do
            if tg.sense == e.sense then
                local nav = navCost(geo, e, tg.level, tg.idx, tg.s, xc)
                if nav ~= nil then
                    local total = e.len + nav + tg.cost
                    if best == nil or total < best.total then best = { entry = e, target = tg, total = total } end
                end
            end
        end
    end
    local H = geo.headlands
    if #targets == 0 or #entries == 0 then return best end
    for _, t in ipairs(lineTurns(geo)) do
        local bestIn, inE = nil, nil
        for _, e in ipairs(entries) do
            if e.sense == t.sIn then
                local c = navCost(geo, e, H, t.jin.ring or 1, t.jin.s, xc)
                if c ~= nil and (bestIn == nil or e.len + c < bestIn) then bestIn, inE = e.len + c, e end
            end
        end
        if bestIn ~= nil and (best == nil or bestIn + t.len < best.total) then
            local from = { level = H, idx = t.jout.ring or 1, s = t.jout.s, sense = t.sOut }
            for _, tg in ipairs(targets) do
                if tg.sense == t.sOut then
                    local c = navCost(geo, from, tg.level, tg.idx, tg.s, xc)
                    if c ~= nil then
                        local total = bestIn + t.len + c + tg.cost
                        if best == nil or total < best.total then
                            best = { entry = inE, target = tg, lineTurn = t, total = total }
                        end
                    end
                end
            end
        end
    end
    return best
end

-- a curve: the straight to its first point is not part of the turn
local function addCurve(out, pts, kind)
    if pts == nil or #pts == 0 then return end
    add(out, { pts[1] }, { work = false, kind = kind })
    add(out, pts, { work = false, turn = true, kind = kind })
end

-- add the way 'best' (bestWay) to route 'out': how the entry gets onto the
-- headland tramline, then round (and across) to the target point. false if
-- the way cannot be built
local function addWay(out, geo, best)
    local e, tg, t = best.entry, best.target, best.lineTurn
    if e.jobPts ~= nil then
        for _, w in ipairs(e.jobPts) do add(out, { w }, { work = false, turn = w.turn, kind = "exit", rev = w.rev }) end
    else
        addCurve(out, e.pts, "back")
    end
    if t ~= nil then
        local H = geo.headlands
        local nav1 = TramlineRoute.navRings(geo, e, H, t.jin.ring or 1, t.jin.s)
        local nav2 = TramlineRoute.navRings(geo, { level = H, idx = t.jout.ring or 1, s = t.jout.s, sense = t.sOut },
            tg.level, tg.idx, tg.s)
        if nav1 == nil or nav2 == nil then return false end
        add(out, nav1, { work = false, kind = "ring" })
        add(out, G.reverse(t.jin.pts), { work = false, turn = true, kind = "junction" })
        addCurve(out, t.jout.pts, "junction")
        add(out, nav2, { work = false, kind = "ring" })
    else
        local nav = TramlineRoute.navRings(geo, e, tg.level, tg.idx, tg.s)
        if nav == nil then return false end
        add(out, nav, { work = false, kind = "ring" })
    end
    return true
end

function TramlineRoute.backToJob(geo, route, stopIx, stopX, stopZ, vx, vz, hx, hz, opts)
    opts = opts or {}
    local n = route ~= nil and #route or 0
    if geo == nil or n < 2 then return nil, "there is no job to go back to" end
    stopIx = max(1, min(stopIx or 1, n - 1))
    local sa, sb = route[stopIx], route[stopIx + 1]
    if stopX == nil then stopX, stopZ = sa[1], sa[2] end
    hx, hz = G.normalize(hx or 1, hz or 0)
    if hx == 0 and hz == 0 then hx = 1 end
    local _, tStop = G.distPointSeg(stopX, stopZ, sa[1], sa[2], sb[1], sb[2])

    -- metres along the job from waypoint i to the stop point: stopLen - cum[i]
    local cum = { 0 }
    for i = 2, stopIx do cum[i] = cum[i - 1] + segLen(route[i - 1], route[i]) end
    local stopLen = cum[stopIx] + sqrt((stopX - sa[1]) ^ 2 + (stopZ - sa[2]) ^ 2)

    local best = nil

    -- a. on the job itself before the stop (the latest such stretch): along the job
    for i = stopIx, 1, -1 do
        local a, b = route[i], route[i + 1]
        local d, t = G.distPointSeg(vx, vz, a[1], a[2], b[1], b[2])
        if d <= 3 and (i < stopIx or t < tStop) then
            local dx, dz, l = G.normalize(b[1] - a[1], b[2] - a[2])
            if l > 0.05 and dx * hx + dz * hz >= TramlineRoute.BACK_COS then
                best = { k = i, total = d + stopLen - cum[i] - t * l, how = "job" }
                break
            end
        end
    end

    -- b. round the headland tramlines to where the job last ran along one
    --    before it stopped (for each pass and direction of travel)
    local paths = TramlineRoute.ringPathsOf(geo)
    local nEntries, nTargets = 0, 0
    if (geo.headlands or 0) > 0 then
        local targets, seen = {}, {}
        for i = stopIx, 1, -1 do
            local b = route[i + 1]
            if b.rl ~= nil then
                local key = b.rl .. "/" .. b.ri .. "/" .. b.rd
                if not seen[key] then
                    seen[key] = true
                    local rp = paths[b.rl] and paths[b.rl][b.ri]
                    if rp ~= nil then
                        local sTo, _, _, d = G.project(rp, route[i][1], route[i][2])
                        if d <= 0.5 then
                            targets[#targets + 1] = { k = i, level = b.rl, idx = b.ri, sense = b.rd, s = sTo,
                                                      cost = stopLen - cum[i] }
                        end
                    end
                end
            end
        end
        local entries = backEntries(geo, vx, vz, hx, hz, opts)
        nEntries, nTargets = #entries, #targets
        local w = bestWay(geo, entries, targets)
        if w ~= nil and (best == nil or w.total < best.total) then
            best = w
            best.k = w.target.k
            best.how = w.entry.how .. (w.lineTurn and "+across" or "")
        end
    end
    if best == nil then
        if nEntries > 0 and nTargets > 0 then
            return nil, "no way back on the tramlines facing this way - turn the sprayer round"
        end
        return nil, "no way back on the tramlines from here - drive onto a tramline or the headland tramline"
    end

    local out = newRoute()
    add(out, { { vx, vz } }, { work = false, kind = "back" })
    if best.entry ~= nil and not addWay(out, geo, best) then return nil, "no way round the headland tramlines" end
    for i = best.k + 1, stopIx do
        local w = route[i]
        add(out, { w }, { work = false, turn = w.turn, kind = w.kind })
    end
    add(out, { { stopX, stopZ } }, { work = false, turn = sb.turn, kind = sb.kind, rl = sb.rl, ri = sb.ri, rd = sb.rd, rev = sb.rev })
    local list = out.list
    local backN = #list
    for i = stopIx + 1, n do list[#list + 1] = route[i] end
    return list, { total = best.total, how = best.how, n = backN }
end

-- The shortest way on the tramlines from where the vehicle is (on a job
-- route at segment fromIx, at vx/vz facing hx/hz) out to the field gate: on
-- along the job to the headland tramline, or from the end of the tramline it
-- is on either way round, or across the field if that is shorter. Nothing
-- sprayed. Returns route, metres or nil, reason (exitRoute: the old way)
function TramlineRoute.toGate(geo, route, fromIx, vx, vz, hx, hz, gate)
    if gate == nil or (geo.headlands or 0) < 1 then return nil, "the field needs a headland tramline" end
    local paths = TramlineRoute.ringPathsOf(geo)
    hx, hz = G.normalize(hx or 1, hz or 0)
    if hx == 0 and hz == 0 then hx = 1 end
    -- where to leave the headland tramline for the gate, either way round
    local targets = {}
    for _, sense in ipairs({ 1, -1 }) do
        if gate.onRing then
            targets[#targets + 1] = { level = 1, idx = gate.idx, s = gate.sJ, sense = sense, cost = 0, pts = {} }
        else
            local arc = gate.inArc[-sense]
            if arc ~= nil then
                local pts = G.reverse(arc.pts)
                local last = pts[#pts]
                targets[#targets + 1] = { level = 1, idx = gate.idx, s = arc.s, sense = sense, pts = pts,
                                          cost = polyLen(pts) + sqrt((last[1] - gate.x) ^ 2 + (last[2] - gate.z) ^ 2) }
            end
        end
    end
    -- ways onto the headland tramline: on along the job, or from where we are
    local entries = {}
    if route ~= nil then
        local pts = {}
        for i = max(1, fromIx or 1), #route - 1 do
            local r = ringSegment(geo, route, i)
            if r ~= nil then
                if i == fromIx then
                    -- on it already: from a metre ahead (see exitRoute)
                    local rp = paths[r.level][r.idx]
                    r.s = ((G.project(rp, vx, vz)) + r.sense * 1.0) % rp.length
                end
                entries[#entries + 1] = { level = r.level, idx = r.idx, s = r.s, sense = r.sense, jobPts = pts,
                                          len = polyLen(pts, vx, vz), how = "job" }
                break
            end
            pts[#pts + 1] = route[i + 1]
        end
    end
    for _, e in ipairs(backEntries(geo, vx, vz, hx, hz, { gate = gate, strict = true })) do entries[#entries + 1] = e end
    local best = bestWay(geo, entries, targets)
    if best == nil then return nil, "no way to the field gate on the tramlines" end
    local out = newRoute()
    add(out, { { vx, vz } }, { work = false, kind = "exit" })
    if not addWay(out, geo, best) then return nil, "no way round the headland tramlines to the gate" end
    add(out, best.target.pts, { work = false, turn = true, kind = "exit" })
    add(out, { { gate.x, gate.z } }, { work = false, kind = "exit" })
    return out.list, best.total, best.entry.how .. (best.lineTurn and "+across" or "")
end

-------------------------------------------------------------------------------
-- Where the worker would set off from: the nearest point of any headland
-- tramline (or, without headland, the nearest free tramline end) and the
-- direction it would drive. { x, z, dirX, dirZ, d, ready } or nil
-------------------------------------------------------------------------------

function TramlineRoute.startPoint(geo, vx, vz, hx, hz)
    if geo == nil or geo.lines == nil or #geo.lines == 0 then return nil end
    local H = geo.headlands or 0
    local best = nil
    if H > 0 then
        for level = 1, H do
            for _, lp in ipairs(geo.rings[level] or {}) do
                local rp = lp.path
                if rp == nil then rp = G.makePath(lp, true); lp.path = rp end
                local s, px, pz, d = G.project(rp, vx, vz)
                if best == nil or d < best.d then
                    local _, _, tx, tz = G.pointAt(rp, s)
                    local facing = (hx or 1) * tx + (hz or 0) * tz
                    local sense = facing >= 0 and 1 or -1
                    local wrongWay = false
                    if geo.headlandDir ~= nil then
                        -- the field's way round; facing the other way it can't set off
                        sense = TramlineGenerator.dirSense(lp, geo.headlandDir)
                        wrongWay = facing * sense < -0.3
                    end
                    best = { x = px, z = pz, dirX = tx * sense, dirZ = tz * sense, d = d, wrongWay = wrongWay, way = geo.headlandDir }
                end
            end
        end
        if best ~= nil then best.ready = best.d <= TramlineRoute.MAX_START_DIST end
    else
        for _, blk in ipairs(geo.blocks or {}) do
            for _, E in ipairs(blk.ends or {}) do
                local d = sqrt((E.x - vx) ^ 2 + (E.z - vz) ^ 2)
                if best == nil or d < best.d then
                    best = { x = E.x, z = E.z, dirX = -E.ux, dirZ = -E.uz, d = d }
                end
            end
        end
        if best ~= nil then best.ready = best.d <= 40 end
    end
    return best
end

-------------------------------------------------------------------------------
-- Pure pursuit controller
--
--   ppc = TramlineRoute.PPC.new(route, turningRadius, startIx)
--   ppc:update(vx, vz)
--   ppc.goalX, ppc.goalZ   point to steer towards
--   ppc.currentIx          waypoint being driven towards
--   ppc.finished           past the last waypoint
--   ppc.cte                signed cross-track error (m)
--   ppc.offTrack           distance from the current segment (m)
-------------------------------------------------------------------------------

TramlineRoute.PPC = {}
local PPC = TramlineRoute.PPC
PPC.__index = PPC

function PPC.new(route, turningRadius, startIx)
    local self = setmetatable({}, PPC)
    self.route = route
    self.radius = turningRadius or 6
    self.baseLookahead = min(self.radius, 6)
    self.lookahead = self.baseLookahead
    self.relevantIx = max(1, min(startIx or 1, #route - 1))
    self.currentIx = self.relevantIx + 1
    self.finished = (#route < 2)
    self.cte, self.offTrack = 0, 0
    self.goalX, self.goalZ = route[self.relevantIx][1], route[self.relevantIx][2]
    self.initializing = true
    self:setSection()
    return self
end

-- the stretch driven in one direction (forwards / in reverse) from relevantIx:
-- it ends at waypoint sectionEnd, where the vehicle stops and changes direction
function PPC:setSection()
    local route = self.route
    local i = self.relevantIx
    local rev = route[i + 1] ~= nil and route[i + 1].rev == true
    local e = i + 1
    while route[e + 1] ~= nil and (route[e + 1].rev == true) == rev do e = e + 1 end
    self.sectionStart = i
    self.sectionEnd = min(e, #route)
    self.reversing = rev
end

local function segDir(route, ix)
    local a, b = route[ix - 1], route[ix]
    if a == nil then a = route[ix]; b = route[ix + 1] end
    if b == nil then return 0, 1 end
    local dx, dz, l = G.normalize(b[1] - a[1], b[2] - a[2])
    if l < 1e-6 then return 0, 1 end
    return dx, dz
end

-- passed waypoint ix: ahead of it in its own direction, and reasonably close
function PPC:passed(ix, vx, vz)
    local wp = self.route[ix]
    if wp == nil then return false end
    local dx, dz = segDir(self.route, ix)
    local rx, rz = vx - wp[1], vz - wp[2]
    return (rx * dx + rz * dz) >= 0 and (rx * rx + rz * rz) < (self.radius * 4) ^ 2
end

function PPC:update(vx, vz)
    local route = self.route
    if self.finished then return end

    if self.initializing and self:passed(self.relevantIx, vx, vz) then
        self.initializing = false
    end
    -- at the end of a stretch: change direction once there
    local se = self.sectionEnd or #route
    if se < #route and self.relevantIx >= se - 1 then
        local p = route[se]
        if self:passed(se, vx, vz) or (vx - p[1]) ^ 2 + (vz - p[2]) ^ 2 < 0.8 * 0.8 then
            self.relevantIx = se
            self.initializing = false
            self.wpBeforeGoalIx = nil
            self:setSection()
            se = self.sectionEnd
        end
    end
    -- advance to the last waypoint passed between here and the goal point
    local limit = min(se < #route and se - 1 or #route, max(self.wpBeforeGoalIx or (self.relevantIx + 1), self.relevantIx + 1))
    local newRel = self.relevantIx
    for ix = self.relevantIx + 1, limit do
        if self:passed(ix, vx, vz) then newRel = ix end
    end
    if newRel ~= self.relevantIx then
        self.relevantIx = newRel
        self.initializing = false
    end
    if self.relevantIx >= #route then
        self.finished = true
        return
    end
    self.currentIx = self.relevantIx + 1

    -- cross-track error / projection on the relevant segment
    local rel, nxt = route[self.relevantIx], route[self.relevantIx + 1]
    local dx, dz = G.normalize(nxt[1] - rel[1], nxt[2] - rel[2])
    if dx == 0 and dz == 0 then dx, dz = segDir(route, self.relevantIx) end
    local rx, rz = vx - rel[1], vz - rel[2]
    local along = rx * dx + rz * dz
    self.cte = rx * (-dz) + rz * dx
    self.projX, self.projZ = rel[1] + dx * along, rel[2] + dz * along
    self.offTrack = (G.distPointSeg(vx, vz, rel[1], rel[2], nxt[1], nxt[2]))

    -- lookahead: shorter in turns and in reverse, longer when off the line (Courseplay)
    local base = self.baseLookahead
    if nxt.turn then base = max(2, base * 0.6) end
    if self.reversing then base = min(base, 3) end
    do
        local p = route[se]
        self.toSectionEnd = sqrt((vx - p[1]) ^ 2 + (vz - p[2]) ^ 2)
    end
    self.lookahead = min(base + abs(self.cte), base * 2)
    local la = self.lookahead

    local ix = self.relevantIx
    while ix < se do
        local p1, p2 = route[ix], route[ix + 1]
        local q1 = sqrt((p1[1] - vx) ^ 2 + (p1[2] - vz) ^ 2)
        local q2 = sqrt((p2[1] - vx) ^ 2 + (p2[2] - vz) ^ 2)
        local l = sqrt((p2[1] - p1[1]) ^ 2 + (p2[2] - p1[2]) ^ 2)

        if self.initializing and ix == self.relevantIx and q1 >= la and q2 >= la then
            self.goalX, self.goalZ = p1[1], p1[2]
            self.wpBeforeGoalIx = ix
            return
        end
        if q1 <= la and q2 >= la and l > 1e-3 then
            if q1 < 1e-4 then q1 = 0.1 end
            local cosG = (q2 * q2 - q1 * q1 - l * l) / (-2 * l * q1)
            local p = q1 * cosG + sqrt(max(0, q1 * q1 * (cosG * cosG - 1) + la * la))
            local sx, sz = G.normalize(p2[1] - p1[1], p2[2] - p1[2])
            self.goalX, self.goalZ = p1[1] + sx * p, p1[2] + sz * p
            self.wpBeforeGoalIx = ix
            return
        end
        if ix == self.relevantIx and q1 >= la and q2 >= la then
            if abs(self.cte) <= la then
                local p = sqrt(la * la - self.cte * self.cte)
                self.goalX, self.goalZ = self.projX + dx * p, self.projZ + dz * p
            else
                self.goalX, self.goalZ = self.projX, self.projZ
            end
            self.wpBeforeGoalIx = ix
            return
        end
        ix = ix + 1
    end
    local last = route[se]
    self.goalX, self.goalZ = last[1], last[2]
    self.wpBeforeGoalIx = se - 1
end

-- work flag of the route at (px,pz), looked up around waypoint ix (within
-- waypoints lo..hi if given: the stretch driven in the current direction)
function TramlineRoute.workAt(route, ix, px, pz, lo, hi)
    local bestD, bestWork = math.huge, false
    for i = max(2, ix - 40, (lo or 1) + 1), min(#route, ix + 30, hi or #route) do
        local a, b = route[i - 1], route[i]
        local d = G.distPointSeg(px, pz, a[1], a[2], b[1], b[2])
        if d < bestD then bestD, bestWork = d, (b.work == true) end
    end
    return bestWork
end

-- index of the route waypoint nearest to (x,z) at or after 'fromIx' (resume)
function TramlineRoute.nearestIndex(route, x, z, fromIx)
    local best, bi = math.huge, nil
    for i = max(1, fromIx or 1), #route do
        local d = (route[i][1] - x) ^ 2 + (route[i][2] - z) ^ 2
        if d < best then best, bi = d, i end
    end
    return bi, sqrt(best)
end
