-- Paraglider Mixer for ArduPlane SITL (and real hardware)
--
-- Custom control law per user spec (NOT realistic paraglider, but matches user's mental model):
--   * Throttle UP        -> motor RPM. Canopy lift + low CG creates natural pitch-up moment -> climb.
--   * Both arms DOWN     -> pitch DOWN  (commanded via elevator stick)
--   * Both arms NEUTRAL  -> pitch UP    (default trim, no brake)
--   * Right arm DOWN     -> roll RIGHT  (commanded via aileron stick)
--   * Left  arm DOWN     -> roll LEFT
--
-- Mapping in this script (ArduPlane internal demand -> servo PWM):
--   pitch_demand  in -1..+1 (negative = AP wants to pitch down):
--     pitch_demand <= 0   -> brake_common = -pitch_demand     (0..1 brake pull)
--     pitch_demand  > 0   -> brake_common = 0                 (release)
--   roll_demand   in -1..+1:
--     brake_left  = clamp(brake_common + roll_demand, 0, BRAKE_MAX)
--     brake_right = clamp(brake_common - roll_demand, 0, BRAKE_MAX)
--   throttle pass-through.
--
-- Servo function assignments (SERVOx_FUNCTION):
--   K_SCRIPTING1  (94) = left  brake servo -> bridge maps to /brake/left
--   K_SCRIPTING2  (95) = right brake servo -> bridge maps to /brake/right
--   K_THROTTLE    (70) = motor               -> bridge maps to /paraglider_uav/command/motor_speed
--
-- BRAKE_MAX clipped at 0.85 PWM-norm to avoid full stall.

local K_SCRIPTING1 = 94
local K_SCRIPTING2 = 95
local BRAKE_MAX    = 0.85
local LOOP_RATE_MS = 20    -- 50 Hz

local function clamp(x, lo, hi)
  if x < lo then return lo end
  if x > hi then return hi end
  return x
end

-- vehicle:get_control_output(N) returns -1..+1 normalized demand.
-- Enum: 1=Roll 2=Pitch 3=Throttle 4=Yaw
local function get_demands()
  local roll_in  = vehicle:get_control_output(1) or 0.0
  local pitch_in = vehicle:get_control_output(2) or 0.0
  return roll_in, pitch_in
end

local debug_count = 0
function update()
  local roll_d, pitch_d = get_demands()
  debug_count = debug_count + 1
  if debug_count % 100 == 0 then
    gcs:send_text(6, string.format("PG mixer: roll=%.2f pitch=%.2f", roll_d, pitch_d))
  end

  -- Custom paraglider mixing:
  --   pitch_d > 0 (AP wants nose up) -> release brakes  (brake_common = 0)
  --   pitch_d < 0 (AP wants nose dn) -> pull brakes     (brake_common = -pitch_d)
  local brake_common = 0.0
  if pitch_d < 0 then
    brake_common = -pitch_d
  end

  -- Roll: differential
  local brake_left  = clamp(brake_common + roll_d, 0, BRAKE_MAX)
  local brake_right = clamp(brake_common - roll_d, 0, BRAKE_MAX)

  -- Convert 0..1 norm to PWM 1000..2000 (servo neutral = 1500 not used; 1000 = released, 2000 = full pull)
  local pwm_l = math.floor(1000 + brake_left  * 1000)
  local pwm_r = math.floor(1000 + brake_right * 1000)

  SRV_Channels:set_output_pwm(K_SCRIPTING1, pwm_l)
  SRV_Channels:set_output_pwm(K_SCRIPTING2, pwm_r)

  return update, LOOP_RATE_MS
end

gcs:send_text(6, "paraglider_mixer.lua: roll/pitch -> differential brake, custom paraglider law")
return update, 1000
