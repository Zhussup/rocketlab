-- A Hohmann transfer flown by a script.
--
-- Raises the apoapsis to a target altitude, coasts to it, then circularises,
-- dropping a stage when the one it is riding runs dry. This is the smallest
-- program that exercises everything the flight computer offers: reading the
-- orbit, choosing an attitude, working the throttle, staging, and deciding when
-- a phase is over.
--
-- It is written the way a person would write it, with no thought for the
-- sandbox, because the sandbox is not something a script author should have to
-- think about. What it cannot do — open a file, read a clock, roll a die — it
-- simply has no way to say.

-- Metres above the surface. The whole point of the exercise.
local target_altitude = 1200e3

-- How close to apoapsis to start the circularisation burn [s]. A real autopilot
-- would work this out from the thrust and the mass; a demo can be told.
local lead_time = 25

-- A stage with less than this left is spent. Not zero, because a tank that has
-- been drawn down in floating point lands a few grams short of empty.
local dry = 1.0

local phase = "raise"
local target_radius = 0

function update()
  -- Resolved on the first tick rather than at load time, because at load time
  -- there is no telemetry yet: `ship` is empty until the vessel is flying.
  if target_radius == 0 then
    target_radius = ship.body_radius + target_altitude
    ship.log(string.format("raising apoapsis to %.0f km", target_altitude / 1000))
  end

  -- A spent stage is dropped without being asked, wherever we are in the
  -- transfer. When the last stage runs dry this does nothing at all, which is
  -- why it needs no guard.
  if ship.has_parts and ship.stage_propellant < dry then
    ship.jettison()
  end

  if not ship.has_parts then
    ship.abort("nothing to fly")
    return
  end

  if phase == "raise" then
    ship.point_prograde()
    ship.set_throttle(1)
    if ship.apoapsis >= target_radius then
      phase = "coast"
      ship.set_throttle(0)
      ship.log("apoapsis reached, coasting")
    end

  elseif phase == "coast" then
    -- Point prograde while coasting so the burn starts already aligned.
    ship.point_prograde()
    ship.set_throttle(0)
    if ship.time_to_apoapsis < lead_time then
      phase = "circularise"
      ship.log("at apoapsis, circularising")
    end

  elseif phase == "circularise" then
    ship.point_prograde()
    ship.set_throttle(1)
    -- Within a kilometre of circular is circular for a demo.
    if ship.periapsis >= target_radius - 1000 then
      ship.set_throttle(0)
      ship.log(string.format("circular at %.1f km, %.0f m/s left",
                             (ship.periapsis - ship.body_radius) / 1000, ship.delta_v))
      ship.abort("transfer complete")
    end
  end
end
