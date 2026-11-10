--[[
    TramlineAD.lua
    Bridge to AutoDrive (FS25_AutoDrive) for refill trips. Uses only
    AutoDrive's public parts, the same ones Courseplay uses:
      * the vehicle's AutoDrive destinations (vehicle.ad.stateModule):
          target      = the field gate (an AutoDrive marker at the field edge)
          2nd target  = the refill point
      * Load mode + AutoDrive:StartDriving - AutoDrive drives to the refill
        point, fills up and drives back to the target, then stops
      * the vehicle event onStopAutoDrive (TramlineDriver listens to it)
]]

TramlineAD = {}

-- the AutoDrive table, or nil when AutoDrive is not installed
function TramlineAD.get()
    local ok, ad = pcall(function()
        return FS25_AutoDrive ~= nil and FS25_AutoDrive.AutoDrive or nil
    end)
    if ok then return ad end
    return nil
end

local function graph()
    local ok, gm = pcall(function()
        return FS25_AutoDrive ~= nil and FS25_AutoDrive.ADGraphManager or nil
    end)
    if ok then return gm end
    return nil
end

local function stateOf(vehicle)
    if vehicle ~= nil and vehicle.ad ~= nil then return vehicle.ad.stateModule end
    return nil
end

-- is AutoDrive driving this vehicle?
function TramlineAD.isDriving(vehicle)
    local sm = stateOf(vehicle)
    if sm == nil or sm.isActive == nil then return false end
    local ok, a = pcall(sm.isActive, sm)
    return ok and a == true
end

local function markerPos(m)
    local gm = graph()
    if type(m) ~= "table" or m.id == nil or gm == nil then return nil end
    local ok, wp = pcall(gm.getWayPointById, gm, m.id)
    if ok and type(wp) == "table" and wp.x ~= nil then return wp.x, wp.z end
    return nil
end

-- The vehicle's AutoDrive destinations:
-- { gateName, gx, gz, refillName, firstId, secondId } or nil, reason
function TramlineAD.destinations(vehicle)
    if TramlineAD.get() == nil then return nil, "AutoDrive is not installed" end
    local sm = stateOf(vehicle)
    if sm == nil then return nil, "this vehicle has no AutoDrive" end
    local first = sm.getFirstMarker ~= nil and sm:getFirstMarker() or nil
    local second = sm.getSecondMarker ~= nil and sm:getSecondMarker() or nil
    if type(first) ~= "table" then return nil, "set the AutoDrive target of this vehicle to the field gate" end
    if type(second) ~= "table" then return nil, "set the 2nd AutoDrive target of this vehicle to the refill point" end
    local gx, gz = markerPos(first)
    if gx == nil then return nil, "the AutoDrive target has no position" end
    return { gateName = tostring(first.name), gx = gx, gz = gz, refillName = tostring(second.name),
             firstId = first.markerIndex, secondId = second.markerIndex }
end

-- AutoDrive in Load mode: to the refill point, fill up with 'fillType', back
-- to the gate. Returns true, destinations or false, reason
function TramlineAD.startRefillTrip(vehicle, fillType)
    local AD = TramlineAD.get()
    local d, why = TramlineAD.destinations(vehicle)
    if d == nil then return false, why end
    if TramlineAD.isDriving(vehicle) then return false, "AutoDrive is already driving" end
    if d.firstId == nil or d.secondId == nil then return false, "AutoDrive destinations not found" end
    local sm = stateOf(vehicle)
    local ok, err = pcall(function()
        if AD.MODE_LOAD ~= nil then sm:setMode(AD.MODE_LOAD) end
        if fillType ~= nil and fillType > 0 and sm.setFillType ~= nil then sm:setFillType(fillType) end
        AD:StartDriving(vehicle, d.firstId, d.secondId)
    end)
    if not ok then return false, tostring(err) end
    if not TramlineAD.isDriving(vehicle) then return false, "AutoDrive did not start" end
    TramlineLog.info("[TramlinePilot][AD] refill trip: '%s' -> '%s' (fill type %s)", d.gateName, d.refillName, tostring(fillType))
    return true, d
end
