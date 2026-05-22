-- A script for controlling the roll and pitch of 6DoF vehicles
-- The script sets the target roll and pitch to the current value when arming or when E-stop is removed
-- this allows the vehicle to be placed upside-down on the ground and to take off as normal
-- its is also possible to setup a switch with option 300 to use either 4 or 6DoF control

-- make sure the vehicle is capable of 6DoF control
assert(param:get('FRAME_CLASS') == 16, "script requires a 6DoF vehicle")

local PARAM_TABLE_KEY = 74
local PARAM_TABLE_PREFIX = "NHO_"

local function bind_add_param(name, idx, default_value)
  assert(param:add_param(PARAM_TABLE_KEY, idx, name, default_value), string.format("could not add param %s", name))
  return Parameter(PARAM_TABLE_PREFIX .. name)
end

assert(param:add_table(PARAM_TABLE_KEY, PARAM_TABLE_PREFIX, 2), "could not add 6DoF offset param table")

-- Roll/pitch offsets in degrees, added to the attitude captured when arming.
local ROLL_OFFSET = bind_add_param("ROLL_OFF", 1, 0)
local PITCH_OFFSET = bind_add_param("PIT_OFF", 2, 0)

local e_stop = rc:find_channel_for_option(31)
local sw = rc:find_channel_for_option(300)
local motors_spinning = false
local base_roll = 0
local base_pitch = 0
local last_roll_offset = nil
local last_pitch_offset = nil

local function apply_offsets(report)
  local roll_offset = ROLL_OFFSET:get()
  local pitch_offset = PITCH_OFFSET:get()
  local roll = base_roll + roll_offset
  local pitch = base_pitch + pitch_offset

  attitude_control:set_offset_roll_pitch(roll,pitch)
  last_roll_offset = roll_offset
  last_pitch_offset = pitch_offset

  if report then
    gcs:send_text(0, string.format("Set Offsets Roll: %0.1f, Pitch: %0.1f", roll, pitch))
  end
end

function update() -- this is the loop which periodically runs

  -- motor state
  local current_motors_spinning = false

  if arming:is_armed() then
    -- if armed then motors are spinning
    current_motors_spinning = true
  end

  if e_stop then
    -- if E-stop switch is setup
    if e_stop:get_aux_switch_pos() == 2 then
      -- E-stop on, motors stopped
      current_motors_spinning = false
    end
  end

  if not motors_spinning and current_motors_spinning then
    -- Just armed or removed E-stop
    -- capture the current attitude as the base, then add configured offsets
    base_roll = math.deg(ahrs:get_roll())
    base_pitch = math.deg(ahrs:get_pitch())
    apply_offsets(true)

    if sw then
      if sw:get_aux_switch_pos() == 2 then
        -- switch to 'normal' 4 DoF attitude control
        gcs:send_text(0, "4 DoF attitude control")
        attitude_control:set_lateral_enable(false)
        attitude_control:set_forward_enable(false)

      else
        -- 6DoF attutude control
        gcs:send_text(0, "6 DoF attitude control")
        attitude_control:set_lateral_enable(true)
        attitude_control:set_forward_enable(true)
      end
    end

  end

  if current_motors_spinning then
    local roll_offset = ROLL_OFFSET:get()
    local pitch_offset = PITCH_OFFSET:get()
    if roll_offset ~= last_roll_offset or pitch_offset ~= last_pitch_offset then
      apply_offsets(true)
    end
  end

  motors_spinning = current_motors_spinning

  return update, 100 -- 10hz
end

return update()
