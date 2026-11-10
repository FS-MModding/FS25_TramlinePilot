--[[
    TramlineDraw.lua
    Drawing for the field overlay and the menu preview, done the way the
    game's own map and HUD are drawn: textured screen overlays. Lines are
    anti-aliased (soft-edged texture), any width, with a dark outline so
    they read on crop and soil; labels are tags with a dark panel, an accent
    bar and a bold title.

    Field lines are drawn in pieces that follow the terrain, thinner and
    fainter with distance, faded out right in front of the camera so the
    cab view stays clear. All lines of a frame are batched: outlines first,
    then the lines on top, so crossings and joints stay clean.
]]

TramlineDraw = {}
TramlineDraw.modDir = g_currentModDirectory

local atan2 = math.atan2 or function(y, x) return math.atan(y, x) end
local sqrt, max, min, floor = math.sqrt, math.max, math.min, math.floor

local ov = nil       -- overlay ids by name

local function overlays()
    if ov == nil then
        ov = {}
        for _, n in ipairs({ "line", "dot", "ring", "pointer", "disc", "rim", "flagpole", "flagcloth", "chequer" }) do
            local ok, id = pcall(createImageOverlay, (TramlineDraw.modDir or "") .. "textures/tl_" .. n .. ".dds")
            if ok and id ~= nil and id ~= 0 then ov[n] = id end
        end
    end
    return ov
end

function TramlineDraw.delete()
    if ov ~= nil then
        for _, id in pairs(ov) do pcall(delete, id) end
        ov = nil
    end
end

-- pixels -> screen units
local function pxX(p) return p / (g_screenWidth or 1920) end
local function pxY(p) return p / (g_screenHeight or 1080) end
TramlineDraw.pxX, TramlineDraw.pxY = pxX, pxY

local function smooth(e0, e1, x)
    local t = (x - e0) / (e1 - e0)
    if t <= 0 then return 0 elseif t >= 1 then return 1 end
    return t * t * (3 - 2 * t)
end
TramlineDraw.smooth = smooth

-------------------------------------------------------------------------------
-- Screen primitives (screen units, sizes in pixels)
-------------------------------------------------------------------------------

-- a soft line from (x1,y1) to (x2,y2), w pixels wide
function TramlineDraw.line(x1, y1, x2, y2, w, r, g, b, a)
    local o = overlays().line
    if o == nil or a <= 0.003 then return end
    local aspect = g_screenAspectRatio or 1.7778
    local dx, dy = x2 - x1, (y2 - y1) / aspect
    local len = sqrt(dx * dx + dy * dy)
    if len < 1e-6 then return end
    -- the texture has soft edges: draw it a little wider than the solid part
    local th = pxY(w * 1.35 + 1)
    setOverlayColor(o, r, g, b, a)
    setOverlayRotation(o, atan2(dy, dx), 0, th * 0.5)
    renderOverlay(o, x1, y1 - th * 0.5, len, th)
    setOverlayRotation(o, 0, 0, 0)
end

-- a line with a dark outline under it
function TramlineDraw.outlined(x1, y1, x2, y2, w, c, a)
    TramlineDraw.line(x1, y1, x2, y2, w + 2.4, 0, 0, 0, 0.45 * a)
    TramlineDraw.line(x1, y1, x2, y2, w, c[1], c[2], c[3], a)
end

local function sprite(name, x, y, d, r, g, b, a)
    local o = overlays()[name]
    if o == nil or a <= 0.003 then return end
    local w, h = pxX(d), pxY(d)
    setOverlayColor(o, r, g, b, a)
    renderOverlay(o, x - w * 0.5, y - h * 0.5, w, h)
end

-- filled soft dot / ring, d pixels across, centred on (x, y)
function TramlineDraw.dot(x, y, d, r, g, b, a) sprite("dot", x, y, d, r, g, b, a) end
function TramlineDraw.ring(x, y, d, r, g, b, a) sprite("ring", x, y, d, r, g, b, a) end

-- a small triangle pointing down with its tip at (x, y)
function TramlineDraw.pointer(x, y, wpx, hpx, r, g, b, a)
    local o = overlays().pointer
    if o == nil or a <= 0.003 then return end
    local w, h = pxX(wpx), pxY(hpx)
    setOverlayColor(o, r, g, b, a)
    renderOverlay(o, x - w * 0.5, y, w, h)
end

-- leave the text settings as other mods expect them
local function resetText()
    setTextBold(false)
    setTextColor(1, 1, 1, 1)
    setTextAlignment(RenderText.ALIGN_LEFT)
end

TramlineDraw.PANEL = { 0.055, 0.065, 0.055 }

-- text sizes and widths of a label (screen units)
local function textMetrics(title, sub)
    local ts = getCorrectTextSize(0.0160)
    local ss = getCorrectTextSize(0.0128)
    setTextBold(true)
    local tw = getTextWidth(ts, title)
    setTextBold(false)
    local sw = (sub ~= nil) and getTextWidth(ss, sub) or 0
    return ts, ss, tw, sw
end

-- A label tag: dark panel, accent bar on the left, bold title and an
-- optional second line. (x, y) = bottom centre (screen units). Returns w, h.
function TramlineDraw.tag(x, y, title, sub, accent, alpha)
    alpha = alpha or 1
    local ts, ss, tw, sw = textMetrics(title, sub)
    local padX, padY, gap, bar = pxX(11), pxY(7), pxY(3), pxX(4)
    local w = max(tw, sw) + 2 * padX + bar
    local h = ts + ((sub ~= nil) and (ss + gap) or 0) + 2 * padY
    local x0 = x - w * 0.5
    local p = TramlineDraw.PANEL
    drawFilledRect(x0, y, w, h, p[1], p[2], p[3], 0.84 * alpha)
    drawFilledRect(x0, y, bar, h, accent[1], accent[2], accent[3], alpha)
    drawFilledRect(x0 + bar, y + h - pxY(1), w - bar, pxY(1), 1, 1, 1, 0.06 * alpha)      -- top edge light
    setTextAlignment(RenderText.ALIGN_LEFT)
    local tx = x0 + bar + padX
    if sub ~= nil then
        setTextBold(false)
        setTextColor(0.80, 0.82, 0.78, alpha)
        renderText(tx, y + padY + ss * 0.22, ss, sub)
    end
    setTextBold(true)
    setTextColor(1, 1, 1, alpha)
    renderText(tx, y + padY + ((sub ~= nil) and (ss + gap) or 0) + ts * 0.22, ts, title)
    resetText()
    return w, h
end

-- the screen position of a world point, or nil when it is behind the camera
function TramlineDraw.toScreen(x, y, z)
    local sx, sy, sz = project(x, y, z)
    if sx == nil or sz == nil or sz < 0 or sz > 1 then return nil end
    return sx, sy
end

-------------------------------------------------------------------------------
-- Course symbols: a flag in a ringed badge - green flag at the start of the
-- course, chequered flag at the finish - at the left end of a pill-shaped
-- label (a pin on a stem, like a flag on a pole).
-------------------------------------------------------------------------------

TramlineDraw.CHIP_MIN = 40             -- px: smallest pill height (= badge size)

-- a badge d pixels across, centred on (x, y): dark disc, coloured rim, flag
function TramlineDraw.badge(x, y, d, icon, rim, alpha)
    local p = TramlineDraw.PANEL
    sprite("disc", x, y, d, p[1], p[2], p[3], alpha)
    local inner = d * 0.84
    sprite("rim", x, y, inner, rim[1], rim[2], rim[3], alpha)
    local g = inner * 0.80
    sprite("flagpole", x, y, g, 0.92, 0.93, 0.90, alpha)
    if icon == "finish" then
        sprite("chequer", x, y, g, 1, 1, 1, alpha)
    else
        sprite("flagcloth", x, y, g, rim[1], rim[2], rim[3], alpha)
    end
end

-- pill height (px) and the text of a label with a badge
local function chipMetrics(title, sub)
    local ts, ss, tw, sw = textMetrics(title, sub)
    local hText = ts + ((sub ~= nil) and (ss + pxY(3)) or 0)
    local hpx = max(TramlineDraw.CHIP_MIN, (hText + pxY(14)) * (g_screenHeight or 1080))
    return hpx, ts, ss, max(tw, sw)
end

-- the pill: badge centred on x, bottom at y (screen units); opaque, so the
-- pieces it is made of do not show where they overlap
function TramlineDraw.chip(x, y, icon, title, sub, accent)
    local hpx, ts, ss, tw = chipMetrics(title, sub)
    local p = TramlineDraw.PANEL
    local R = hpx * 0.5
    local cy = y + pxY(R)
    local tx = x + pxX(R + 7)
    local rx = tx + tw + pxX(12) - pxX(R)      -- centre of the round right end
    if rx > x then
        drawFilledRect(x, y, rx - x, pxY(hpx), p[1], p[2], p[3], 1)
        sprite("disc", rx, cy, hpx, p[1], p[2], p[3], 1)
    end
    TramlineDraw.badge(x, cy, hpx, icon, accent, 1)
    local gap = pxY(3)
    local by = cy - (ts + ((sub ~= nil) and (ss + gap) or 0)) * 0.5
    setTextAlignment(RenderText.ALIGN_LEFT)
    if sub ~= nil then
        setTextBold(false)
        setTextColor(0.80, 0.82, 0.78, 1)
        renderText(tx, by + ss * 0.22, ss, sub)
    end
    setTextBold(true)
    setTextColor(1, 1, 1, 1)
    renderText(tx, by + ((sub ~= nil) and (ss + gap) or 0) + ts * 0.22, ts, title)
    resetText()
end

TramlineDraw.WHITE = { 0.97, 0.97, 0.95 }
TramlineDraw.BLACK = { 0.015, 0.015, 0.015 }

-- A marker in the field: a ring on the ground (world radius 'rad' m), a dot,
-- a leader line up and a tag on top; with an icon ("start" / "finish") the
-- tag is a pill with the course symbol. ring = "chequer": black and white
-- ring (finish). alpha fades with distance (the pill stays solid).
function TramlineDraw.callout(wx, wz, rad, title, sub, accent, cam, icon, ring)
    local gy = TramlineDraw.groundY(wx, wz)
    local d = sqrt((wx - cam[1]) ^ 2 + (gy - cam[2]) ^ 2 + (wz - cam[3]) ^ 2)
    local alpha = 1 - 0.45 * smooth(150, 600, d)
    -- ring on the ground
    if rad ~= nil and rad > 0 then
        local n = 28
        local px, pz = wx + rad, wz
        for i = 1, n do
            local t = i / n * 2 * math.pi
            local nx, nz = wx + math.cos(t) * rad, wz + math.sin(t) * rad
            local c = accent
            if ring == "chequer" then
                c = (floor((i - 1) / 2) % 2 == 0) and TramlineDraw.WHITE or TramlineDraw.BLACK
            end
            TramlineDraw.groundSegment(px, pz, nx, nz, c, 3.2, alpha, nil, cam)
            px, pz = nx, nz
        end
    end
    local sx, sy = TramlineDraw.toScreen(wx, gy, wz)
    if sx == nil or sx < -0.2 or sx > 1.2 or sy < -0.2 or sy > 1.2 then return end
    return { sx = sx, sy = sy, d = d, title = title, sub = sub, accent = accent, alpha = alpha, icon = icon }
end

TramlineDraw.LEAD = 54                 -- px: leader line / stem

-- the label's box relative to the top of the leader: x from sx + x0, w, h
local function labelBox(c)
    if c.icon ~= nil then
        local hpx, _, _, tw = chipMetrics(c.title, c.sub)
        return -pxX(hpx * 0.5), pxX(hpx + 19) + tw, pxY(hpx), 0
    end
    local ts, ss, tw, sw = textMetrics(c.title, c.sub)
    local w = max(tw, sw) + pxX(26)
    local h = ts + ((c.sub ~= nil) and (ss + pxY(3)) or 0) + pxY(14)
    return -w * 0.5, w, h, pxY(8)
end

-- Place the labels so they do not cover each other: the nearest keeps its
-- place, labels further away move up out of its way. Leaves the list in
-- drawing order (furthest first, nearest on top).
function TramlineDraw.layoutCallouts(list)
    table.sort(list, function(a, b) return a.d < b.d end)
    local placed = {}
    local gap = pxY(5)
    for _, c in ipairs(list) do
        local x0, w, h, above = labelBox(c)
        c.lead = pxY(TramlineDraw.LEAD)
        local bx0, bx1 = c.sx + x0, c.sx + x0 + w
        for _ = 1, 8 do
            local by0 = c.sy + c.lead + above
            local by1 = by0 + h
            local hit = nil
            for _, p in ipairs(placed) do
                if bx0 < p[2] and bx1 > p[1] and by0 < p[4] + gap and by1 > p[3] - gap then
                    hit = p
                    break
                end
            end
            if hit == nil then break end
            c.lead = c.lead + (hit[4] + gap - by0)
        end
        local by0 = c.sy + c.lead + above
        placed[#placed + 1] = { bx0, bx1, by0, by0 + h }
    end
    table.sort(list, function(a, b) return a.d > b.d end)
end

-- draw a callout's dot, leader line and tag (after the lines, on top of them)
function TramlineDraw.drawCallout(c)
    local lead = c.lead or pxY(TramlineDraw.LEAD)
    local a, ac = c.alpha, c.accent
    TramlineDraw.line(c.sx, c.sy, c.sx, c.sy + lead, 3.4, 0, 0, 0, 0.45 * a)
    TramlineDraw.line(c.sx, c.sy, c.sx, c.sy + lead, 1.6, ac[1], ac[2], ac[3], a)
    TramlineDraw.dot(c.sx, c.sy, 12, 0, 0, 0, 0.45 * a)
    TramlineDraw.dot(c.sx, c.sy, 8, ac[1], ac[2], ac[3], a)
    if c.icon ~= nil then
        TramlineDraw.chip(c.sx, c.sy + lead, c.icon, c.title, c.sub, ac)
        return
    end
    local p = TramlineDraw.PANEL
    TramlineDraw.pointer(c.sx, c.sy + lead, 14, 8, p[1], p[2], p[3], 0.84 * a)
    TramlineDraw.tag(c.sx, c.sy + lead + pxY(8), c.title, c.sub, ac, a)
end

-------------------------------------------------------------------------------
-- Lines in the field
-------------------------------------------------------------------------------

TramlineDraw.LIFT = 0.12               -- m above the ground

function TramlineDraw.groundY(x, z)
    return getTerrainHeightAtWorldPos(g_currentMission.terrainRootNode, x, 0, z) + TramlineDraw.LIFT
end

-- the frame's lines: outlines are drawn first, then the lines on top
local batch, nb = {}, 0

function TramlineDraw.beginFrame()
    nb = 0
end

local function push(x1, y1, x2, y2, w, r, g, b, a)
    local i = nb * 9
    batch[i + 1], batch[i + 2], batch[i + 3], batch[i + 4] = x1, y1, x2, y2
    batch[i + 5], batch[i + 6], batch[i + 7], batch[i + 8], batch[i + 9] = w, r, g, b, a
    nb = nb + 1
end

function TramlineDraw.endFrame()
    for k = 0, nb - 1 do
        local i = k * 9
        TramlineDraw.line(batch[i + 1], batch[i + 2], batch[i + 3], batch[i + 4], batch[i + 5] + 2.4, 0, 0, 0, 0.42 * batch[i + 9])
    end
    for k = 0, nb - 1 do
        local i = k * 9
        TramlineDraw.line(batch[i + 1], batch[i + 2], batch[i + 3], batch[i + 4], batch[i + 5],
            batch[i + 6], batch[i + 7], batch[i + 8], batch[i + 9])
    end
    nb = 0
end

TramlineDraw.NEAR_FADE = { 5, 14 }     -- m: invisible below, fully there above
TramlineDraw.FAR_FADE  = { 380, 650 }  -- m: fading out between

-- width (px) and opacity of a line piece at distance d
local function look(d, w40, alpha)
    local w = w40 * 40 / max(d, 1)
    w = max(1.3, min(w, w40 * 1.6))
    local a = alpha * smooth(TramlineDraw.NEAR_FADE[1], TramlineDraw.NEAR_FADE[2], d)
        * (1 - smooth(TramlineDraw.FAR_FADE[1], TramlineDraw.FAR_FADE[2], d))
    return w, a
end

-- A straight piece of ground line from (x1,z1) to (x2,z2): split into short
-- pieces near the camera (they follow the terrain), longer ones far away.
-- c = colour, w40 = width in px at 40 m, dash = { on, off } metres or nil.
-- cam = { x, y, z }
function TramlineDraw.groundSegment(x1, z1, x2, z2, c, w40, alpha, dash, cam)
    local dx, dz = x2 - x1, z2 - z1
    local len = sqrt(dx * dx + dz * dz)
    if len < 0.01 then return end
    local ux, uz = dx / len, dz / len
    local s = 0
    local ax, az = x1, z1
    local ay = TramlineDraw.groundY(ax, az)
    local s1x, s1y, s1z = project(ax, ay, az)
    local period = dash and (dash[1] + dash[2]) or nil
    while s < len - 1e-6 do
        local d = sqrt((ax - cam[1]) ^ 2 + (ay - cam[2]) ^ 2 + (az - cam[3]) ^ 2)
        local step = max(2.5, min(25, d * 0.07))
        if period ~= nil then
            -- stop at the next dash edge
            local ph = s % period
            local edge = (ph < dash[1]) and (dash[1] - ph) or (period - ph)
            step = min(step, max(edge, 0.05))
        end
        local s2 = min(len, s + step)
        local bx, bz = x1 + ux * s2, z1 + uz * s2
        local by = TramlineDraw.groundY(bx, bz)
        local s2x, s2y, s2z = project(bx, by, bz)
        local on = period == nil or ((s + 1e-4) % period) < dash[1]
        if on and s1z ~= nil and s2z ~= nil and s1z >= 0 and s1z <= 1 and s2z >= 0 and s2z <= 1
            and not ((s1x < -0.05 and s2x < -0.05) or (s1x > 1.05 and s2x > 1.05)
                or (s1y < -0.05 and s2y < -0.05) or (s1y > 1.05 and s2y > 1.05)) then
            local w, a = look(d, w40, alpha)
            if a > 0.01 then push(s1x, s1y, s2x, s2y, w, c[1], c[2], c[3], a) end
        end
        s = s2
        ax, ay, az = bx, by, bz
        s1x, s1y, s1z = s2x, s2y, s2z
    end
end

-- a polyline { {x,z}, ... } on the ground (closed: back to the start)
function TramlineDraw.groundPath(pts, closed, c, w40, alpha, dash, cam, maxDist)
    local n = #pts
    if n < 2 then return end
    local r2 = (maxDist or 700) ^ 2
    local last = closed and n or n - 1
    for i = 1, last do
        local a, b = pts[i], pts[(i % n) + 1]
        if (a[1] - cam[1]) ^ 2 + (a[2] - cam[3]) ^ 2 < r2 or (b[1] - cam[1]) ^ 2 + (b[2] - cam[3]) ^ 2 < r2 then
            TramlineDraw.groundSegment(a[1], a[2], b[1], b[2], c, w40, alpha, dash, cam)
        end
    end
end

-- chevrons along the ground: 'count' arrow heads from (x,z) in direction
-- (dx,dz), 'gap' metres apart, pulsing in turn
function TramlineDraw.chevrons(x, z, dx, dz, count, gap, c, cam)
    local l = sqrt(dx * dx + dz * dz)
    if l < 1e-6 then return end
    dx, dz = dx / l, dz / l
    local nx, nz = -dz, dx
    local t = ((g_time or 0) % 1400) / 1400
    for i = 1, count do
        local px, pz = x + dx * gap * i, z + dz * gap * i
        local phase = (t * count - (i - 1)) % count
        local a = 0.45 + 0.55 * (1 - smooth(0, 1.2, phase))
        TramlineDraw.groundSegment(px - dx * 2.2 + nx * 2.4, pz - dz * 2.2 + nz * 2.4, px, pz, c, 4.2, a, nil, cam)
        TramlineDraw.groundSegment(px - dx * 2.2 - nx * 2.4, pz - dz * 2.2 - nz * 2.4, px, pz, c, 4.2, a, nil, cam)
    end
end

-------------------------------------------------------------------------------
-- The layout as polylines for drawing (cached on the layout)
-------------------------------------------------------------------------------

-- consecutive segments { x1, z1, x2, z2 } joined into polylines
local function joinSegments(segs)
    local out, cur = {}, nil
    for _, s in ipairs(segs or {}) do
        if cur ~= nil then
            local e = cur[#cur]
            if math.abs(e[1] - s[1]) < 1e-3 and math.abs(e[2] - s[2]) < 1e-3 then
                cur[#cur + 1] = { s[3], s[4] }
            else
                cur = nil
            end
        end
        if cur == nil then
            cur = { { s[1], s[2] }, { s[3], s[4] } }
            out[#out + 1] = cur
        end
    end
    return out
end

function TramlineDraw.layoutPaths(geo)
    if geo.drawPaths ~= nil then return geo.drawPaths end
    local G = TramlineGeom
    local p = { rings = {}, curves = {} }
    for _, loops in pairs(geo.rings or {}) do
        for _, lp in ipairs(loops) do
            p.rings[#p.rings + 1] = G.simplify(lp, 0.08, true)
        end
    end
    for _, pl in ipairs(joinSegments(geo.extras)) do
        p.curves[#p.curves + 1] = (#pl > 2) and G.simplify(pl, 0.08, false) or pl
    end
    geo.drawPaths = p
    return p
end
