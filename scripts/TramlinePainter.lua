--[[
    TramlinePainter.lua
    Marks the tramline layout on the ground right away, so you don't have to
    wait for the next sowing: every part the worker drives on (main
    tramlines, headland tramlines, the curves between them and any loops or
    crossings) is applied as a chain of short parallelograms written to the
    density maps: crop cleared and ground set to cultivated, only on field
    ground.

    Modes:
      PAINT_NONE    - overlay only, ground untouched
      PAINT_TRACKS  - two wheel tracks the field's wheel track width apart
                      (set in the menu, to match the sprayer's wheels), each
                      ONE ground cell wide: the ground and crop maps are a
                      grid (0.5 m on a 2 km map with 4096 maps), so a track
                      is the chain of cells its centre line passes through -
                      0.5 m across where it runs along the grid, ~0.35 m
                      where it runs diagonally
      PAINT_FULL    - one lane down the centre, the wheel track + 1 m wide
]]

TramlinePainter = {}

TramlinePainter.PAINT_NONE   = 1
TramlinePainter.PAINT_TRACKS = 2
TramlinePainter.PAINT_FULL   = 3

TramlinePainter.TRACK_WIDTH   = 2.0   -- metres centre-to-centre between the two wheel tracks (default)
TramlinePainter.TRACK_MIN     = 1.5   -- metres, the range the menu offers (5 cm steps)
TramlinePainter.TRACK_MAX     = 4.0
TramlinePainter.STRIP_WIDTH   = 0.5   -- metres, width of each wheel track (one density-map pixel)
TramlinePainter.LANE_EXTRA    = 1.0   -- metres a full lane is wider than the wheel track (half a tyre each side)

-- a wheel track width within the range, to the nearest 5 cm (default if nil)
function TramlinePainter.clampTrack(w)
    w = tonumber(w) or TramlinePainter.TRACK_WIDTH
    w = math.max(TramlinePainter.TRACK_MIN, math.min(TramlinePainter.TRACK_MAX, w))
    return math.floor(w * 20 + 0.5) / 20
end

-- width of a full lane for a wheel track width
function TramlinePainter.laneWidth(trackWidth)
    return TramlinePainter.clampTrack(trackWidth) + TramlinePainter.LANE_EXTRA
end
TramlinePainter.CHUNK_LEN     = 10    -- metres per density-map call on straights
TramlinePainter.CHUNK_OVERLAP = 0.6   -- metres each chunk runs past the next, so there are no gaps
TramlinePainter.CURVE_TOL     = 0.1   -- metres a straight piece may deviate from a curve

-- size of one ground cell in metres (terrain size / density map size)
function TramlinePainter.cellSize()
    if TramlinePainter.cell == nil then
        local size = 0.5
        local ok, res = pcall(function()
            local mapId = TramlineFieldUtil.getGroundMapId()
            local ts = g_currentMission.terrainSize
            if mapId ~= nil and ts ~= nil and ts > 0 and getDensityMapSize ~= nil then
                local n = getDensityMapSize(mapId)
                if n ~= nil and n > 0 then return ts / n end
            end
            return nil
        end)
        if ok and res ~= nil and res > 0.05 and res < 4 then size = res end
        TramlinePainter.cell = size
        TramlineLog.info("[TramlinePilot] ground cell size %.3f m", size)
    end
    return TramlinePainter.cell
end

-- Where the cell grid starts (0 or half a cell): a square half a cell across
-- in the middle of a cell must cover exactly one map pixel. Checked once.
function TramlinePainter.gridOffset(x, z, c)
    if TramlinePainter.offset == nil then
        local found = nil
        for _, o in ipairs({ 0, c * 0.5 }) do
            local i, j = math.floor((x - o) / c), math.floor((z - o) / c)
            local x0, z0 = i * c + o + c * 0.25, j * c + o + c * 0.25
            local n = TramlineFieldUtil.countPixels(x0, z0, x0 + c * 0.5, z0, x0, z0 + c * 0.5)
            if n == nil then break end
            TramlineLog.info("[TramlinePilot] ground grid check: offset %.2f -> %s pixel(s)", o, tostring(n))
            if n == 1 and found == nil then found = o end
        end
        TramlinePainter.offset = found or 0
    end
    return TramlinePainter.offset
end

local OFF, MUL = 1048576, 2097152           -- cell keys: (row + OFF) * MUL + (col + OFF)
local floor = math.floor

-- Every cell the straight (x1,z1)-(x2,z2) passes through. Where a line that
-- runs at an angle to the grid steps across a row (or column), the two cells
-- on either side of the step share an edge, so a track one cell wide stays
-- one unbroken strip at any angle. (Taking just one cell per column joined
-- those steps only at a corner: the crop grew through and an angled track
-- showed as a row of short slanted dashes.)
local function rasterize(cells, x1, z1, x2, z2, c)
    local i, j = floor(x1 / c), floor(z1 / c)
    local dx, dz = x2 - x1, z2 - z1
    local si = (dx > 0) and 1 or -1
    local sj = (dz > 0) and 1 or -1
    local huge = math.huge
    local tdx = (dx ~= 0) and math.abs(c / dx) or huge
    local tdz = (dz ~= 0) and math.abs(c / dz) or huge
    local tx = (dx ~= 0) and ((((si > 0) and (i + 1) or i) * c - x1) / dx) or huge
    local tz = (dz ~= 0) and ((((sj > 0) and (j + 1) or j) * c - z1) / dz) or huge
    cells[(j + OFF) * MUL + (i + OFF)] = true
    while true do
        if (tx < tz and tx or tz) > 1 then break end
        if tx < tz then
            i = i + si; tx = tx + tdx
        elseif tz < tx then
            j = j + sj; tz = tz + tdz
        else
            -- exactly through a grid corner: take a side cell too, edge to edge
            cells[(j + OFF) * MUL + (i + si + OFF)] = true
            i = i + si; j = j + sj
            tx = tx + tdx; tz = tz + tdz
        end
        cells[(j + OFF) * MUL + (i + OFF)] = true
    end
end

-- the cells of a polyline shifted sideways by 'lat' (each piece shifted along
-- its own normal, the pieces joined up at the bends)
local function rasterizeOffset(cells, pts, closed, lat, c)
    local n = #pts
    local last = closed and n or n - 1
    local px, pz = nil, nil
    local fx, fz = nil, nil
    for k = 1, last do
        local a, b = pts[k], pts[(k % n) + 1]
        local dx, dz = b[1] - a[1], b[2] - a[2]
        local l = math.sqrt(dx * dx + dz * dz)
        if l > 0.01 then
            local nx, nz = -dz / l * lat, dx / l * lat
            local ax, az, bx, bz = a[1] + nx, a[2] + nz, b[1] + nx, b[2] + nz
            if px ~= nil then rasterize(cells, px, pz, ax, az, c) end
            if fx == nil then fx, fz = ax, az end
            rasterize(cells, ax, az, bx, bz, c)
            px, pz = bx, bz
        end
    end
    if closed and px ~= nil then rasterize(cells, px, pz, fx, fz, c) end
end

-- Write the cells: runs along rows, then runs along columns, then single
-- cells, each as a parallelogram inside its cells (half a cell across), so
-- exactly those cells change. Returns the number of runs, ok
function TramlinePainter.paintCells(cells, c, o)
    o = o or 0
    local keys = {}
    for k in pairs(cells) do keys[#keys + 1] = k end
    table.sort(keys)
    local q = 0.25 * c
    local runs = 0
    local singles = {}
    local i = 1
    while i <= #keys do
        local k0 = keys[i]
        local j = i
        while j < #keys and keys[j + 1] == keys[j] + 1 do j = j + 1 end
        local row, col = floor(k0 / MUL) - OFF, (k0 % MUL) - OFF
        if j > i then
            local x0, z0 = col * c + q + o, row * c + q + o
            local len = (j - i) * c + 2 * q
            if not TramlineFieldUtil.cultivateArea(x0, z0, x0 + len, z0, x0, z0 + 2 * q) then return runs, false end
            runs = runs + 1
        else
            singles[#singles + 1] = (col + OFF) * MUL + (row + OFF)
        end
        i = j + 1
    end
    table.sort(singles)
    i = 1
    while i <= #singles do
        local k0 = singles[i]
        local j = i
        while j < #singles and singles[j + 1] == singles[j] + 1 do j = j + 1 end
        local col, row = floor(k0 / MUL) - OFF, (k0 % MUL) - OFF
        local x0, z0 = col * c + q + o, row * c + q + o
        local len = (j - i) * c + 2 * q
        if not TramlineFieldUtil.cultivateArea(x0, z0, x0 + 2 * q, z0, x0, z0 + len) then return runs, false end
        runs = runs + 1
        i = j + 1
    end
    return runs, true
end

-- Apply a strip of 'width' along the segment (x1,z1)-(x2,z2), shifted
-- sideways by 'lateral' metres. Returns false if the ground write failed.
local function paintStrip(x1, z1, x2, z2, width, lateral)
    local dx, dz = x2 - x1, z2 - z1
    local len = math.sqrt(dx * dx + dz * dz)
    if len < 0.02 then return true end
    dx, dz = dx / len, dz / len
    local nx, nz = -dz, dx
    x1, z1 = x1 + nx * lateral, z1 + nz * lateral

    local half = width * 0.5
    local chunks = math.max(1, math.ceil(len / TramlinePainter.CHUNK_LEN))
    local step = len / chunks
    -- short pieces of a curve: overlap a little less so they don't stick out
    local overlap = math.min(TramlinePainter.CHUNK_OVERLAP, step * 0.5 + 0.1)
    for i = 0, chunks - 1 do
        local sx, sz = x1 + dx * step * i, z1 + dz * step * i
        local cx, cz = sx - nx * half, sz - nz * half          -- start corner
        local wx, wz = sx + nx * half, sz + nz * half          -- across the strip
        local runLen = step + overlap
        local hx, hz = cx + dx * runLen, cz + dz * runLen      -- along the strip
        if not TramlineFieldUtil.cultivateArea(cx, cz, wx, wz, hx, hz) then
            return false
        end
    end
    return true
end

-- a polyline (open or closed) as straight pieces that follow it within CURVE_TOL
local function paintPolyline(pts, closed, width, offsets)
    if pts == nil or #pts < 2 then return 0, true end
    local simple = TramlineGeom.simplify(pts, TramlinePainter.CURVE_TOL, closed)
    local n = #simple
    local last = closed and n or n - 1
    local count = 0
    for i = 1, last do
        local a, b = simple[i], simple[(i % n) + 1]
        for _, lat in ipairs(offsets) do
            if not paintStrip(a[1], a[2], b[1], b[2], width, lat) then return count, false end
        end
        count = count + 1
    end
    return count, true
end

-- geo: output of TramlineGenerator.generate; mode: one of PAINT_*;
-- trackWidth: metres between the wheel centres (nil = TRACK_WIDTH).
-- Returns the number of pieces painted.
function TramlinePainter.paint(geo, mode, trackWidth)
    if geo == nil or mode == nil or mode == TramlinePainter.PAINT_NONE then return 0 end
    trackWidth = TramlinePainter.clampTrack(trackWidth)

    if mode == TramlinePainter.PAINT_TRACKS then
        -- two wheel tracks, one ground cell wide each
        local c = TramlinePainter.cellSize()
        local h = trackWidth * 0.5
        local o = nil
        local cells = {}
        local function track(pts, closed)
            if pts == nil or #pts < 2 then return end
            if o == nil then o = TramlinePainter.gridOffset(pts[1][1], pts[1][2], c) end
            local simple = TramlineGeom.simplify(pts, TramlinePainter.CURVE_TOL, closed)
            local shifted = {}
            for k, p in ipairs(simple) do shifted[k] = { p[1] - o, p[2] - o } end
            rasterizeOffset(cells, shifted, closed, -h, c)
            rasterizeOffset(cells, shifted, closed, h, c)
        end
        for _, l in ipairs(geo.lines or {}) do track({ { l[1], l[2] }, { l[3], l[4] } }, false) end
        for _, loops in pairs(geo.rings or {}) do
            for _, lp in ipairs(loops) do track(lp, true) end
        end
        for _, cv in ipairs(geo.curves or {}) do track(cv, false) end
        local n = 0
        for _ in pairs(cells) do n = n + 1 end
        local runs, ok = TramlinePainter.paintCells(cells, c, o or 0)
        if not ok then
            TramlineLog.error("[TramlinePilot] could not write the ground for the tramlines")
        end
        TramlineLog.info("[TramlinePilot] marked the wheel tracks %.2f m apart: %d ground cells (%.2f m) in %d pieces", trackWidth, n, c, runs)
        return runs
    end

    local offsets, width = { 0 }, TramlinePainter.laneWidth(trackWidth)

    local pieces, ok = 0, true
    local function add(n, good)
        pieces = pieces + n
        if not good then ok = false end
    end
    for _, l in ipairs(geo.lines or {}) do
        if ok then add(paintPolyline({ { l[1], l[2] }, { l[3], l[4] } }, false, width, offsets)) end
    end
    for _, loops in pairs(geo.rings or {}) do
        for _, lp in ipairs(loops) do
            if ok then add(paintPolyline(lp, true, width, offsets)) end
        end
    end
    for _, c in ipairs(geo.curves or {}) do
        if ok then add(paintPolyline(c, false, width, offsets)) end
    end
    if not ok then
        TramlineLog.error("[TramlinePilot] could not write the ground for the tramlines")
    end
    TramlineLog.info("[TramlinePilot] marked %d tramline pieces on the ground (mode %d)", pieces, mode)
    return pieces
end
