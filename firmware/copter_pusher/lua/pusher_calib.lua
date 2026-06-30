-- pusher_calib.lua  -  calibration aid for ATC_PUSH_* feed-forward gains
--
-- Lua cannot inject control moments, so the COMPENSATION itself lives in C++
-- (see ../ardupilot_patch/). This script only MEASURES the disturbance so you
-- can choose the gains.
--
-- Method: while the pilot's PITCH stick is neutral (so desired pitch ~ 0), the
-- actual pitch angle IS the pusher-induced disturbance. We bin the steady-state
-- pitch (and roll, and yaw rate) by pusher level and report the averages.
--
--   * A negative mean pitch at high pusher  => nose-down disturbance
--     => set ATC_PUSH_PIT POSITIVE (commands nose-up) until the mean -> ~0.
--   * Residual mean roll      => trim ATC_PUSH_RLL.
--   * Residual mean yaw-rate  => trim ATC_PUSH_YAW.
--
-- Put this file in the autopilot's APM/scripts/ folder, set SCR_ENABLE=1.
-- Pusher assumed on RC8 (edit PUSHER_CH below if different).

local PUSHER_CH    = 8        -- RC channel that drives the pusher
local PITCH_CH     = 2        -- RC channel for pilot pitch (elevator)
local STICK_DZ     = 40       -- PWM deadband around 1500 = "stick neutral"
local PUSHER_MIN   = 1095     -- RC8 idle pwm
local PUSHER_MAX   = 1935     -- RC8 full pwm
local NBINS        = 10       -- 0..100% pusher in 10 bins
local REPORT_MS    = 10000    -- print table every 10 s
local LOOP_MS      = 100      -- 10 Hz sampling

-- per-bin accumulators: sum_pitch, sum_roll, sum_yawrate, count
local sum_pitch = {}
local sum_roll  = {}
local sum_yawr  = {}
local cnt       = {}
for i = 1, NBINS do sum_pitch[i]=0; sum_roll[i]=0; sum_yawr[i]=0; cnt[i]=0 end

local last_report = 0
local was_armed   = false

local function bin_of(pwm)
  local n = (pwm - PUSHER_MIN) / (PUSHER_MAX - PUSHER_MIN)   -- 0..1
  if n < 0 then n = 0 end
  if n > 1 then n = 1 end
  local b = math.floor(n * NBINS) + 1
  if b > NBINS then b = NBINS end
  return b, n
end

local function report()
  gcs:send_text(6, "PUSH-CAL  bin  pusher%   meanPitch  meanRoll  meanYawRate(deg/s)  n")
  for i = 1, NBINS do
    if cnt[i] > 0 then
      local p = sum_pitch[i]/cnt[i]
      local r = sum_roll[i]/cnt[i]
      local y = sum_yawr[i]/cnt[i]
      gcs:send_text(6, string.format("PUSH-CAL  %2d   %3d-%3d   %+7.2f   %+7.2f   %+7.2f          %d",
        i, (i-1)*100//NBINS, i*100//NBINS, p, r, y, cnt[i]))
    end
  end
end

function update()
  local armed = arming:is_armed()

  -- on disarm edge, dump the table once
  if was_armed and not armed then
    gcs:send_text(6, "PUSH-CAL  === flight summary ===")
    report()
  end
  was_armed = armed

  if armed then
    local pusher_pwm = rc:get_pwm(PUSHER_CH) or PUSHER_MIN
    local pitch_pwm  = rc:get_pwm(PITCH_CH)  or 1500
    -- only sample when pilot pitch stick is neutral (desired pitch ~ 0)
    if math.abs(pitch_pwm - 1500) <= STICK_DZ then
      local b = bin_of(pusher_pwm)
      local pitch_deg = math.deg(ahrs:get_pitch())
      local roll_deg  = math.deg(ahrs:get_roll())
      local gyro = ahrs:get_gyro()                 -- rad/s, body
      local yawrate_deg = gyro and math.deg(gyro:z()) or 0.0
      sum_pitch[b] = sum_pitch[b] + pitch_deg
      sum_roll[b]  = sum_roll[b]  + roll_deg
      sum_yawr[b]  = sum_yawr[b]  + yawrate_deg
      cnt[b]       = cnt[b] + 1
    end

    local now = millis():toint()
    if now - last_report >= REPORT_MS then
      last_report = now
      report()
    end
  end

  return update, LOOP_MS
end

gcs:send_text(6, "pusher_calib.lua loaded: keep PITCH stick neutral; read PUSH-CAL table to tune ATC_PUSH_*")
return update, 1000
