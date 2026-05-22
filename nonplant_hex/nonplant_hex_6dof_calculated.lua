-- Geometry-calculated fixed 6DoF motor matrix for a non-planar tilted hexacopter.
-- Use with: SCR_ENABLE=1, FRAME_CLASS=16, then reboot.
--
-- Fill MOTOR_GEOMETRY with measured motor positions and thrust tilt directions.
-- The script calculates the 6x6 allocation matrix at boot and loads Motors_6DoF.

assert(param:get("FRAME_CLASS") == 16, "set FRAME_CLASS=16 for Motors_6DoF")

local MOTOR_COUNT = 6
local YAW_DRAG_FACTOR = 0.05
local NORMALISE_COLUMNS = true
local FACTOR_SCALE = 0.5
local PRINT_FACTORS = true

local PARAM_TABLE_KEY = 73
local PARAM_TABLE_PREFIX = "NH_"

local function bind_add_param(name, idx, default_value)
    assert(param:add_param(PARAM_TABLE_KEY, idx, name, default_value), string.format("could not add param %s", name))
    return Parameter(PARAM_TABLE_PREFIX .. name)
end

assert(param:add_table(PARAM_TABLE_KEY, PARAM_TABLE_PREFIX, 2), "could not add non-planar hex param table")

-- Ground-station adjustable factors. Reboot or restart scripting after changing them
-- because this script loads the motor matrix once during startup.
local FORWARD_FACTOR_SCALE = bind_add_param("FWD_SCL", 1, FACTOR_SCALE)
local RIGHT_FACTOR_SCALE = bind_add_param("RGT_SCL", 2, FACTOR_SCALE)

local function param_value(p, default_value)
    local value = p:get()
    if value == nil then
        return default_value
    end
    return value
end

-- Body axes used by this script:
--   x: forward, y: right, z: up
-- arm_angle_deg is the motor arm direction around the body z-axis:
--   0=forward, 90=right, 180=back, 270=left
-- arm_tilt_deg is the signed rotor side tilt about the outward arm axis.
--
-- Replace arm_length/arm_angle_deg/z/arm_tilt_deg/spin with your actual frame.
-- Distances can be in any consistent unit because factors are normalised.
-- spin: +1 and -1 are opposite propeller directions. Swap signs if yaw test is reversed.
local MOTOR_GEOMETRY = {
    { num = 0, order = 1, arm_length = 0.5, arm_angle_deg =  30, z = 0.00, arm_tilt_deg = -20, spin = 1, reversible = false },
    { num = 1, order = 2, arm_length = 0.5, arm_angle_deg =  90, z = 0.00, arm_tilt_deg =  20, spin = -1, reversible = false },
    { num = 2, order = 3, arm_length = 0.5, arm_angle_deg = 150, z = 0.00, arm_tilt_deg = -20, spin = 1, reversible = false },
    { num = 3, order = 4, arm_length = 0.5, arm_angle_deg = 210, z = 0.00, arm_tilt_deg =  20, spin = -1, reversible = false },
    { num = 4, order = 5, arm_length = 0.5, arm_angle_deg = 270, z = 0.00, arm_tilt_deg = -20, spin = 1, reversible = false },
    { num = 5, order = 6, arm_length = 0.5, arm_angle_deg = 330, z = 0.00, arm_tilt_deg =  20, spin = -1, reversible = false },
}

-- Flip these if bench testing shows an axis moves the wrong way.
local AXIS_SIGN = {
    roll = -1,
    pitch = -1,
    yaw = -1,
    throttle = 1,
    forward = -1,
    right = -1,
}

local AXES = { "roll", "pitch", "yaw", "throttle", "forward", "right" }

local function thrust_vector(arm_angle_deg, arm_tilt_deg)
    local arm_angle = math.rad(arm_angle_deg)
    local arm_tilt = math.rad(arm_tilt_deg)
    local horizontal = math.sin(arm_tilt)

    return {
        x = horizontal * math.sin(arm_angle),
        y = -horizontal * math.cos(arm_angle),
        z = math.cos(arm_tilt),
    }
end

local function motor_effect(m)
    local t = thrust_vector(m.arm_angle_deg, m.arm_tilt_deg)
    local arm_angle = math.rad(m.arm_angle_deg)
    local x = m.arm_length * math.cos(arm_angle)
    local y = m.arm_length * math.sin(arm_angle)

    -- Torque = position cross thrust, plus propeller drag torque around yaw.
    local roll = y * t.z - m.z * t.y
    local pitch = m.z * t.x - x * t.z
    local yaw = x * t.y - y * t.x + m.spin * YAW_DRAG_FACTOR

    return {
        roll = roll * AXIS_SIGN.roll,
        pitch = pitch * AXIS_SIGN.pitch,
        yaw = yaw * AXIS_SIGN.yaw,
        throttle = t.z * AXIS_SIGN.throttle,
        forward = t.x * AXIS_SIGN.forward,
        right = t.y * AXIS_SIGN.right,
    }
end

local function invert_matrix(a)
    local n = #a
    local aug = {}

    for r = 1, n do
        aug[r] = {}
        for c = 1, n do
            aug[r][c] = a[r][c]
        end
        for c = 1, n do
            aug[r][n + c] = (r == c) and 1.0 or 0.0
        end
    end

    for col = 1, n do
        local pivot_row = col
        local pivot_abs = math.abs(aug[col][col])

        for row = col + 1, n do
            local candidate_abs = math.abs(aug[row][col])
            if candidate_abs > pivot_abs then
                pivot_abs = candidate_abs
                pivot_row = row
            end
        end

        assert(pivot_abs > 1.0e-6, "motor allocation matrix is singular")

        if pivot_row ~= col then
            aug[col], aug[pivot_row] = aug[pivot_row], aug[col]
        end

        local pivot = aug[col][col]
        for c = 1, 2 * n do
            aug[col][c] = aug[col][c] / pivot
        end

        for row = 1, n do
            if row ~= col then
                local factor = aug[row][col]
                if math.abs(factor) > 1.0e-12 then
                    for c = 1, 2 * n do
                        aug[row][c] = aug[row][c] - factor * aug[col][c]
                    end
                end
            end
        end
    end

    local inv = {}
    for r = 1, n do
        inv[r] = {}
        for c = 1, n do
            inv[r][c] = aug[r][n + c]
        end
    end

    return inv
end

local function build_factor_rows()
    assert(#MOTOR_GEOMETRY == MOTOR_COUNT, "expected exactly 6 motors")

    local effects = {}
    for i = 1, MOTOR_COUNT do
        effects[i] = motor_effect(MOTOR_GEOMETRY[i])
    end

    -- effect_matrix maps motor outputs to [roll, pitch, yaw, throttle, forward, right].
    local effect_matrix = {}
    for axis_index = 1, MOTOR_COUNT do
        local axis = AXES[axis_index]
        effect_matrix[axis_index] = {}
        for motor_index = 1, MOTOR_COUNT do
            effect_matrix[axis_index][motor_index] = effects[motor_index][axis]
        end
    end

    local factor_rows = invert_matrix(effect_matrix)

    if NORMALISE_COLUMNS then
        for axis_index = 1, MOTOR_COUNT do
            local max_abs = 0.0
            for motor_index = 1, MOTOR_COUNT do
                max_abs = math.max(max_abs, math.abs(factor_rows[motor_index][axis_index]))
            end

            assert(max_abs > 1.0e-6, "zero factor column")
            for motor_index = 1, MOTOR_COUNT do
                factor_rows[motor_index][axis_index] = factor_rows[motor_index][axis_index] / max_abs
            end
        end
    end

    return factor_rows
end

local function add_motors(factor_rows)
    local forward_scale = param_value(FORWARD_FACTOR_SCALE, FACTOR_SCALE)
    local right_scale = param_value(RIGHT_FACTOR_SCALE, FACTOR_SCALE)

    for i = 1, MOTOR_COUNT do
        local m = MOTOR_GEOMETRY[i]
        local f = factor_rows[i]

        Motors_6DoF:add_motor(
            m.num,
            f[1] * FACTOR_SCALE, -- roll
            f[2] * FACTOR_SCALE, -- pitch
            f[3] * FACTOR_SCALE, -- yaw
            f[4] * FACTOR_SCALE, -- throttle
            f[5] * forward_scale, -- forward
            f[6] * right_scale, -- right
            m.reversible,
            m.order
        )

        if PRINT_FACTORS then
            gcs:send_text(6, string.format(
                "M%u r%.2f p%.2f y%.2f t%.2f f%.2f s%.2f",
                m.num,
                f[1] * FACTOR_SCALE,
                f[2] * FACTOR_SCALE,
                f[3] * FACTOR_SCALE,
                f[4] * FACTOR_SCALE,
                f[5] * forward_scale,
                f[6] * right_scale))
        end
    end
end

local factor_rows = build_factor_rows()
add_motors(factor_rows)

assert(Motors_6DoF:init(MOTOR_COUNT), "failed to init calculated non-planar hex")
motors:set_frame_string("NonPlanar Hex Calc")
gcs:send_text(6, "Calculated non-planar hex 6DoF matrix loaded")
