#include "Copter.h"

namespace {

constexpr uint16_t TSDT4_TRIGGER_PWM = 1800U;
constexpr uint32_t TSDT4_SWITCH_LOW_CONFIRM_MS = 500U;

constexpr float TSDT4_TAKEOFF_HEIGHT_CM = 200.0f;
// [Cybernetics Ch.4] Keep XY closed by the native Guided position controller
// while the vertical reference follows a smooth, zero-slope profile.
constexpr uint32_t TSDT4_TAKEOFF_PROFILE_MS = 25000U;
constexpr uint32_t TSDT4_TAKEOFF_LEVEL_MS = 1000U;
constexpr uint32_t TSDT4_TAKEOFF_TIMEOUT_MS = 30000U;

// Keep the pre-contact descent slow and finish with native landing detection.
constexpr float TSDT4_LAND_CLIMB_RATE_CMS = -5.0f;
constexpr uint32_t TSDT4_LAND_BLEND_MS = 1000U;
constexpr uint32_t TSDT4_CONTACT_ARM_DELAY_MS = 1500U;
constexpr float TSDT4_CONTACT_MIN_DESCENT_CM = 20.0f;
constexpr float TSDT4_CONTACT_ACCEL_Z_MSS = 0.18f;
constexpr uint8_t TSDT4_CONTACT_CONFIRM_SAMPLES = 2U;
constexpr float TSDT4_AIRFRAME_LAND_GATE_Z_CM = 60.0f;
constexpr uint32_t TSDT4_LAND_TIMEOUT_MS = 90000U;
constexpr float TSDT4_TOOL_CONTACT_RISE_CM = 146.7f;
constexpr uint32_t TSDT4_TROCHOID_PROFILE_MS = 30000U;
constexpr float TSDT4_TROCHOID_END_DEG = -22.85f;
constexpr uint8_t TSDT4_TROCHOID_POINTS = 24U;
// Let the passive hinge respond before commanding the horizontal component
// that otherwise drags the tool across the ground.
constexpr float TSDT4_FOLD_X_GATE_DROP_CM = 3.0f;

// Mesh-derived support envelope.  The offsets are relative to the 90 deg
// tool-contact pose and follow the measured finite support, rather than an
// ideal circle.  x is along the launch heading and z is NEU-up height.
constexpr float TSDT4_TROCHOID_ANGLE_DEG[TSDT4_TROCHOID_POINTS] = {
    90.0f, 85.0f, 80.0f, 75.0f, 70.0f, 65.0f, 60.0f, 55.0f,
    50.0f, 45.0f, 40.0f, 35.0f, 30.0f, 25.0f, 20.0f, 15.0f,
    10.0f, 5.0f, 0.0f, -5.0f, -10.0f, -15.0f, -20.0f, -22.85f};
constexpr float TSDT4_TROCHOID_X_CM[TSDT4_TROCHOID_POINTS] = {
       0.0000f,  -14.4999f,  -28.8920f,  -43.0939f,  -57.1424f,  -70.9780f,
     -84.5225f,  -97.7009f, -110.4416f, -122.6739f, -134.3340f, -145.3646f,
    -155.7204f, -165.3662f, -174.2699f, -182.4040f, -189.7502f, -196.2932f,
    -202.0244f, -206.9438f, -211.0529f, -214.3642f, -216.8941f, -217.9940f};
constexpr float TSDT4_TROCHOID_Z_CM[TSDT4_TROCHOID_POINTS] = {
       0.0000f,   -0.6233f,   -2.4815f,   -4.4584f,   -6.4561f,   -9.3414f,
     -13.1252f,  -17.7236f,  -23.1603f,  -29.3489f,  -36.2700f,  -43.7524f,
     -51.7046f,  -60.0285f,  -68.7170f,  -77.6313f,  -86.7777f,  -96.0259f,
     -105.3496f, -114.6542f, -123.8707f, -132.9505f, -141.7792f, -146.6969f};

enum class TsdT4State : uint8_t {
    IDLE,
    TAKEOFF_ANGLE,
    TAKEOFF_LEVEL,
    HOLD,
    LAND_DESCEND,
    LAND_ANGLE,
    ABORT,
};

TsdT4State tsdt4_state = TsdT4State::IDLE;
Vector3f tsdt4_start_neu_cm;
Vector3f tsdt4_land_hold_neu_cm;
Vector3f tsdt4_fold_start_neu_cm;
float tsdt4_start_yaw_rad;
uint32_t tsdt4_takeoff_start_ms;
uint32_t tsdt4_level_start_ms;
uint32_t tsdt4_land_start_ms;
uint32_t tsdt4_fold_start_ms;
float tsdt4_fold_x_gate_s;
bool tsdt4_fold_x_gate_open;
float tsdt4_land_start_z_cm;
uint8_t tsdt4_contact_accel_count;
bool tsdt4_trigger_high;
bool tsdt4_trigger_seen_low;
uint32_t tsdt4_switch_low_since_ms;
bool tsdt4_force_flying_owned;

float smoothstep5(const float value)
{
    const float s = constrain_float(value, 0.0f, 1.0f);
    return 10.0f * s * s * s - 15.0f * s * s * s * s +
           6.0f * s * s * s * s * s;
}

float smoothstep5_integral(const float value)
{
    const float s = constrain_float(value, 0.0f, 1.0f);
    const float s2 = s * s;
    const float s4 = s2 * s2;
    return 2.5f * s4 - 3.0f * s4 * s + s4 * s2;
}

float smoothstep5_derivative(const float value)
{
    const float s = constrain_float(value, 0.0f, 1.0f);
    return 30.0f * s * s * (1.0f - s) * (1.0f - s);
}

void tsdt4_trochoid_offsets(const float angle_deg, float &x_cm,
                            float &z_cm)
{
    const float q = constrain_float(angle_deg, TSDT4_TROCHOID_END_DEG, 90.0f);
    if (q >= TSDT4_TROCHOID_ANGLE_DEG[0]) {
        x_cm = TSDT4_TROCHOID_X_CM[0];
        z_cm = TSDT4_TROCHOID_Z_CM[0];
        return;
    }
    for (uint8_t i = 0U; i + 1U < TSDT4_TROCHOID_POINTS; i++) {
        if (q <= TSDT4_TROCHOID_ANGLE_DEG[i] &&
            q >= TSDT4_TROCHOID_ANGLE_DEG[i + 1U]) {
            const float span = TSDT4_TROCHOID_ANGLE_DEG[i] -
                               TSDT4_TROCHOID_ANGLE_DEG[i + 1U];
            const float blend =
                (TSDT4_TROCHOID_ANGLE_DEG[i] - q) / span;
            x_cm = TSDT4_TROCHOID_X_CM[i] +
                   (TSDT4_TROCHOID_X_CM[i + 1U] -
                    TSDT4_TROCHOID_X_CM[i]) * blend;
            z_cm = TSDT4_TROCHOID_Z_CM[i] +
                   (TSDT4_TROCHOID_Z_CM[i + 1U] -
                    TSDT4_TROCHOID_Z_CM[i]) * blend;
            return;
        }
    }
    x_cm = TSDT4_TROCHOID_X_CM[TSDT4_TROCHOID_POINTS - 1U];
    z_cm = TSDT4_TROCHOID_Z_CM[TSDT4_TROCHOID_POINTS - 1U];
}

void tsdt4_trochoid_reference(const float profile_s, float &x_cm, float &z_cm,
                              float &vx_cms, float &vz_cms)
{
    const float s = constrain_float(profile_s, 0.0f, 1.0f);
    const float lambda = smoothstep5(s);
    const float angle_deg =
        90.0f + (TSDT4_TROCHOID_END_DEG - 90.0f) * lambda;
    tsdt4_trochoid_offsets(angle_deg, x_cm, z_cm);

    const float eps = 0.001f;
    float x_plus, z_plus, x_minus, z_minus;
    tsdt4_trochoid_offsets(
        90.0f + (TSDT4_TROCHOID_END_DEG - 90.0f) *
                     smoothstep5(s + eps),
        x_plus, z_plus);
    tsdt4_trochoid_offsets(
        90.0f + (TSDT4_TROCHOID_END_DEG - 90.0f) *
                     smoothstep5(s - eps),
        x_minus, z_minus);
    // The finite difference already includes the quintic phase derivative.
    const float ds_dt = 1.0f / (TSDT4_TROCHOID_PROFILE_MS * 0.001f);
    vx_cms = (x_plus - x_minus) / (2.0f * eps) * ds_dt;
    vz_cms = (z_plus - z_minus) / (2.0f * eps) * ds_dt;
}

} // namespace

#ifdef USERHOOK_INIT
void Copter::userhook_init()
{
    // put your initialisation code here
    // this will be called once at start-up
}
#endif

#ifdef USERHOOK_FASTLOOP
void Copter::userhook_FastLoop()
{
    // put your 100Hz code here
}
#endif

#ifdef USERHOOK_50HZLOOP
void Copter::userhook_50Hz()
{
    // put your 50Hz code here
}
#endif

#ifdef USERHOOK_MEDIUMLOOP
void Copter::userhook_MediumLoop()
{
    const uint32_t now_ms = AP_HAL::millis();
    const bool trigger_high =
        RC_Channels::get_radio_in(CH_13) >= TSDT4_TRIGGER_PWM;

    // A boot-time high value is never accepted as an operator edge.  CH13
    // must first remain low for a full debounce interval.
    if (!trigger_high) {
        if (tsdt4_switch_low_since_ms == 0U) {
            tsdt4_switch_low_since_ms = now_ms;
        } else if (now_ms - tsdt4_switch_low_since_ms >=
                   TSDT4_SWITCH_LOW_CONFIRM_MS) {
            tsdt4_trigger_seen_low = true;
        }
    } else {
        tsdt4_switch_low_since_ms = 0U;
    }

    const bool rising_edge =
        trigger_high && tsdt4_trigger_seen_low && !tsdt4_trigger_high;
    const bool falling_edge = !trigger_high && tsdt4_trigger_high;
    tsdt4_trigger_high = trigger_high;

    if (!trigger_high && tsdt4_state == TsdT4State::ABORT) {
        tsdt4_state = TsdT4State::IDLE;
    }

    if (rising_edge && tsdt4_state == TsdT4State::IDLE) {
        // Consume the qualified edge even if a safety gate rejects the run.
        tsdt4_trigger_seen_low = false;
        if (!position_ok() || !ap.land_complete || any_failsafe_triggered()) {
            tsdt4_state = TsdT4State::ABORT;
            gcs().send_text(MAV_SEVERITY_WARNING,
                            "TSDT4: takeoff gate failed");
        } else if (!set_mode(Mode::Number::GUIDED,
                             ModeReason::GCS_COMMAND)) {
            tsdt4_state = TsdT4State::ABORT;
            gcs().send_text(MAV_SEVERITY_WARNING,
                            "TSDT4: GUIDED rejected");
        } else if (!motors->armed() &&
                   !arming.arm(AP_Arming::Method::MAVLINK)) {
            tsdt4_state = TsdT4State::ABORT;
            gcs().send_text(MAV_SEVERITY_WARNING,
                            "TSDT4: arming rejected");
        } else {
            tsdt4_start_neu_cm = inertial_nav.get_position_neu_cm();
            tsdt4_start_yaw_rad = ahrs.get_yaw();
            tsdt4_takeoff_start_ms = now_ms;

            // [Cybernetics Ch.4] Close the ground-to-flight transition once;
            // subsequent landing state is owned by the native detector.
            set_land_complete(false);
            set_land_complete_maybe(false);
            ap.auto_armed = true;

            tsdt4_state = TsdT4State::TAKEOFF_ANGLE;
            gcs().send_text(MAV_SEVERITY_INFO,
                            "TSDT4: takeoff angle started");
        }
    }

    const bool active =
        tsdt4_state != TsdT4State::IDLE &&
        tsdt4_state != TsdT4State::ABORT;
    if (active &&
        (any_failsafe_triggered() ||
         flightmode->mode_number() != Mode::Number::GUIDED)) {
        if (tsdt4_force_flying_owned) {
            force_flying = false;
            tsdt4_force_flying_owned = false;
        }
        tsdt4_state = TsdT4State::ABORT;
        gcs().send_text(MAV_SEVERITY_WARNING,
                        "TSDT4: external mode or failsafe abort");
        return;
    }

    if (tsdt4_state == TsdT4State::TAKEOFF_ANGLE) {
        const uint32_t elapsed_ms = now_ms - tsdt4_takeoff_start_ms;
        const float profile = smoothstep5(
            elapsed_ms / float(TSDT4_TAKEOFF_PROFILE_MS));
        const float profile_rate = smoothstep5_derivative(
            elapsed_ms / float(TSDT4_TAKEOFF_PROFILE_MS));
        Vector3f takeoff_target_neu_cm = tsdt4_start_neu_cm;
        takeoff_target_neu_cm.z += TSDT4_TAKEOFF_HEIGHT_CM * profile;
        Vector3f takeoff_velocity_cms{};
        takeoff_velocity_cms.z =
            (TSDT4_TAKEOFF_HEIGHT_CM * 1000.0f /
             float(TSDT4_TAKEOFF_PROFILE_MS)) * profile_rate;

        // [Cybernetics Ch.4] Position/velocity feedback retains XY at the
        // launch point while the quintic Z reference avoids a step in climb.
        if (!mode_guided.set_destination_posvel(
                takeoff_target_neu_cm, takeoff_velocity_cms, true,
                degrees(tsdt4_start_yaw_rad) * 100.0f, false, 0.0f, false)) {
            tsdt4_state = TsdT4State::ABORT;
            gcs().send_text(MAV_SEVERITY_WARNING,
                            "TSDT4: takeoff position profile rejected");
            return;
        }

        if (elapsed_ms >= TSDT4_TAKEOFF_PROFILE_MS) {
            tsdt4_level_start_ms = now_ms;
            tsdt4_state = TsdT4State::TAKEOFF_LEVEL;
        } else if (elapsed_ms >= TSDT4_TAKEOFF_TIMEOUT_MS) {
            tsdt4_state = TsdT4State::ABORT;
            gcs().send_text(MAV_SEVERITY_WARNING,
                            "TSDT4: takeoff timeout, position abort");
        }
        return;
    }

    if (tsdt4_state == TsdT4State::TAKEOFF_LEVEL) {
        const uint32_t elapsed_ms = now_ms - tsdt4_level_start_ms;
        Vector3f hold_neu_cm = tsdt4_start_neu_cm;
        hold_neu_cm.z += TSDT4_TAKEOFF_HEIGHT_CM;
        Vector3f hold_velocity_cms{};
        // [Cybernetics Ch.4] Re-enter the zero-velocity position loop with a
        // continuous target after the takeoff profile reaches its endpoint.
        mode_guided.set_destination_posvel(
            hold_neu_cm, hold_velocity_cms, true,
            degrees(tsdt4_start_yaw_rad) * 100.0f, false, 0.0f, false);

        if (elapsed_ms >= TSDT4_TAKEOFF_LEVEL_MS) {
            if (mode_guided.set_destination(
                    hold_neu_cm, true,
                    degrees(tsdt4_start_yaw_rad) * 100.0f,
                    false, 0.0f, false, false)) {
                tsdt4_state = TsdT4State::HOLD;
                gcs().send_text(MAV_SEVERITY_INFO,
                                "TSDT4: takeoff position hold");
            } else {
                tsdt4_state = TsdT4State::ABORT;
                gcs().send_text(MAV_SEVERITY_WARNING,
                                "TSDT4: position hold rejected");
            }
        }
        return;
    }

    if (falling_edge && tsdt4_state == TsdT4State::HOLD) {
        tsdt4_land_hold_neu_cm = inertial_nav.get_position_neu_cm();
        tsdt4_land_start_ms = now_ms;
        tsdt4_land_start_z_cm = tsdt4_land_hold_neu_cm.z;
        tsdt4_contact_accel_count = 0U;
        tsdt4_state = TsdT4State::LAND_DESCEND;
        gcs().send_text(MAV_SEVERITY_INFO,
                        "TSDT4: landing descent started");
    }

    const bool landing_active =
        tsdt4_state == TsdT4State::LAND_DESCEND ||
        tsdt4_state == TsdT4State::LAND_ANGLE;
    if (landing_active) {
        const bool airframe_above_land_gate =
            inertial_nav.get_position_neu_cm().z >
            tsdt4_start_neu_cm.z + TSDT4_AIRFRAME_LAND_GATE_Z_CM;
        if (airframe_above_land_gate) {
            force_flying = true;
            tsdt4_force_flying_owned = true;
        } else if (tsdt4_force_flying_owned) {
            // This gate only filters premature tool-contact detection.  It
            // never changes the attitude or vertical command.
            force_flying = false;
            tsdt4_force_flying_owned = false;
        }

        if (ap.land_complete) {
            if (tsdt4_force_flying_owned) {
                force_flying = false;
                tsdt4_force_flying_owned = false;
            }
            arming.disarm(AP_Arming::Method::LANDED);
            tsdt4_state = TsdT4State::IDLE;
            gcs().send_text(MAV_SEVERITY_INFO,
                            "TSDT4: landing complete");
            return;
        }
    }

    if (tsdt4_state == TsdT4State::LAND_DESCEND) {
        const uint32_t elapsed_ms = now_ms - tsdt4_land_start_ms;
        const float blend_time_s = TSDT4_LAND_BLEND_MS * 0.001f;
        const float elapsed_s = elapsed_ms * 0.001f;
        const float normalized_time = elapsed_s / blend_time_s;
        const float ramp_distance_s =
            blend_time_s * smoothstep5_integral(normalized_time);
        const float constant_distance_s =
            MAX(elapsed_s - blend_time_s, 0.0f);
        Vector3f descent_target_neu_cm = tsdt4_land_hold_neu_cm;
        descent_target_neu_cm.z += TSDT4_LAND_CLIMB_RATE_CMS *
                                   (ramp_distance_s + constant_distance_s);

        // [Cybernetics Ch.4] Keep native Guided Position ownership of XY
        // during the slow pre-contact descent.  Only the Z destination moves;
        // no XY error or velocity is converted into a custom pitch command.
        if (!mode_guided.set_destination(
                descent_target_neu_cm, true,
                degrees(tsdt4_start_yaw_rad) * 100.0f,
                false, 0.0f, false, false)) {
            tsdt4_state = TsdT4State::ABORT;
            gcs().send_text(MAV_SEVERITY_WARNING,
                            "TSDT4: descent position rejected");
            return;
        }

        const bool contact_detection_armed =
            elapsed_ms >= TSDT4_CONTACT_ARM_DELAY_MS &&
            inertial_nav.get_position_neu_cm().z <=
                tsdt4_land_start_z_cm - TSDT4_CONTACT_MIN_DESCENT_CM;
        const float accel_z_mss = ahrs.get_accel_ef().z + GRAVITY_MSS;
        if (contact_detection_armed &&
            fabsf(accel_z_mss) >= TSDT4_CONTACT_ACCEL_Z_MSS) {
            tsdt4_contact_accel_count++;
        } else {
            tsdt4_contact_accel_count = 0U;
        }

        // [Cybernetics Ch.4] Geometry starts folding before a slow contact can
        // accumulate a large vertical tracking error; no contact force is used.
        if (tsdt4_contact_accel_count >= TSDT4_CONTACT_CONFIRM_SAMPLES ||
            descent_target_neu_cm.z <=
                tsdt4_start_neu_cm.z + TSDT4_TOOL_CONTACT_RISE_CM + 2.0f) {
            // [Cybernetics Ch.4] Anchor the geometric fold to the measured
            // body pose at the phase boundary so the first command is
            // continuous even when the slow descent has tracking error.
            tsdt4_fold_start_neu_cm = inertial_nav.get_position_neu_cm();
            tsdt4_fold_start_ms = now_ms;
            tsdt4_fold_x_gate_s = 0.0f;
            tsdt4_fold_x_gate_open = false;
            tsdt4_state = TsdT4State::LAND_ANGLE;
            gcs().send_text(MAV_SEVERITY_INFO,
                            "TSDT4: landing fold started");
        } else if (elapsed_ms >= TSDT4_LAND_TIMEOUT_MS) {
            if (tsdt4_force_flying_owned) {
                force_flying = false;
                tsdt4_force_flying_owned = false;
            }
            set_mode(Mode::Number::LAND, ModeReason::GCS_COMMAND);
            tsdt4_state = TsdT4State::ABORT;
            gcs().send_text(MAV_SEVERITY_WARNING,
                            "TSDT4: contact timeout, LAND fallback");
        }
        return;
    }

    if (tsdt4_state == TsdT4State::LAND_ANGLE) {
        const uint32_t total_landing_ms = now_ms - tsdt4_land_start_ms;
        const uint32_t fold_elapsed_ms = now_ms - tsdt4_fold_start_ms;
        const float profile_s = fold_elapsed_ms * 0.001f /
                                (TSDT4_TROCHOID_PROFILE_MS * 0.001f);
        float x_offset_cm;
        float z_height_offset_cm;
        float vx_heading_cms;
        float vz_height_cms;
        tsdt4_trochoid_reference(profile_s, x_offset_cm,
                                 z_height_offset_cm, vx_heading_cms,
                                 vz_height_cms);

        // Hold the horizontal reference while the passive hinge is still
        // locked at its hanging pose.  Once the body has descended through
        // the measured geometric gate, remap the horizontal quintic so it
        // starts at zero and still reaches the complete endpoint.
        const float current_z_cm = inertial_nav.get_position_neu_cm().z;
        if (!tsdt4_fold_x_gate_open &&
            current_z_cm <=
                tsdt4_fold_start_neu_cm.z - TSDT4_FOLD_X_GATE_DROP_CM) {
            tsdt4_fold_x_gate_open = true;
            tsdt4_fold_x_gate_s = profile_s;
            gcs().send_text(MAV_SEVERITY_INFO,
                            "TSDT4: fold translation gate open");
        }
        if (!tsdt4_fold_x_gate_open) {
            x_offset_cm = 0.0f;
            vx_heading_cms = 0.0f;
        } else {
            const float gate_lambda =
                smoothstep5(tsdt4_fold_x_gate_s);
            const float path_lambda = smoothstep5(profile_s);
            const float remaining = MAX(1.0f - gate_lambda, 0.001f);
            const float remapped = constrain_float(
                (path_lambda - gate_lambda) / remaining, 0.0f, 1.0f);
            const float remapped_rate =
                smoothstep5_derivative(remapped) *
                smoothstep5_derivative(profile_s) / remaining;
            const float final_x_cm =
                TSDT4_TROCHOID_X_CM[TSDT4_TROCHOID_POINTS - 1U];
            x_offset_cm = final_x_cm * smoothstep5(remapped);
            vx_heading_cms = final_x_cm * remapped_rate /
                             (TSDT4_TROCHOID_PROFILE_MS * 0.001f);
        }

        const float yaw_cos = cosf(tsdt4_start_yaw_rad);
        const float yaw_sin = sinf(tsdt4_start_yaw_rad);
        Vector3f fold_target_neu_cm = tsdt4_fold_start_neu_cm;
        fold_target_neu_cm.x += x_offset_cm * yaw_cos;
        fold_target_neu_cm.y += x_offset_cm * yaw_sin;
        // The envelope is expressed as a decrease in body height.  NEU z is
        // positive up, so the negative envelope offset is used directly.
        fold_target_neu_cm.z += z_height_offset_cm;
        Vector3f fold_velocity_cms{};
        fold_velocity_cms.x = vx_heading_cms * yaw_cos;
        fold_velocity_cms.y = vx_heading_cms * yaw_sin;
        fold_velocity_cms.z = vz_height_cms;

        // [Cybernetics Ch.4] A fixed endpoint can hold thrust above the floor.
        // Continue a smooth slow descent until the native detector disarms.
        if (fold_elapsed_ms > TSDT4_TROCHOID_PROFILE_MS) {
            const float terminal_s =
                (fold_elapsed_ms - TSDT4_TROCHOID_PROFILE_MS) * 0.001f;
            const float blend_s = TSDT4_LAND_BLEND_MS * 0.001f;
            fold_target_neu_cm.z += TSDT4_LAND_CLIMB_RATE_CMS *
                (blend_s * smoothstep5_integral(terminal_s / blend_s) +
                 MAX(terminal_s - blend_s, 0.0f));
            fold_velocity_cms.z = TSDT4_LAND_CLIMB_RATE_CMS *
                smoothstep5(terminal_s / blend_s);
        }

        // [Cybernetics Ch.4] Keep the landing loop closed around the measured
        // trochoid reference.  Position and velocity share the same smooth
        // quintic phase, so the passive joint is driven without a command
        // step or a fixed-pitch impulse.
        if (!mode_guided.set_destination_posvel(
                fold_target_neu_cm, fold_velocity_cms, true,
                degrees(tsdt4_start_yaw_rad) * 100.0f, false, 0.0f, false)) {
            tsdt4_state = TsdT4State::ABORT;
            gcs().send_text(MAV_SEVERITY_WARNING,
                            "TSDT4: fold position rejected");
            return;
        }

        if (total_landing_ms >= TSDT4_LAND_TIMEOUT_MS) {
            if (tsdt4_force_flying_owned) {
                force_flying = false;
                tsdt4_force_flying_owned = false;
            }
            // [Cybernetics Ch.17] This is a failed-run safety fallback only;
            // a passing run never leaves GUIDED before disarm.
            set_mode(Mode::Number::LAND, ModeReason::GCS_COMMAND);
            tsdt4_state = TsdT4State::ABORT;
            gcs().send_text(MAV_SEVERITY_WARNING,
                            "TSDT4: landing timeout, LAND fallback");
        }
    }
}
#endif

#ifdef USERHOOK_SLOWLOOP
void Copter::userhook_SlowLoop()
{
    // put your 3.3Hz code here
}
#endif

#ifdef USERHOOK_SUPERSLOWLOOP
void Copter::userhook_SuperSlowLoop()
{
    // put your 1Hz code here
}
#endif

#ifdef USERHOOK_AUXSWITCH
void Copter::userhook_auxSwitch1(const RC_Channel::AuxSwitchPos ch_flag)
{
    // put your aux switch #1 handler here (CHx_OPT = 47)
}

void Copter::userhook_auxSwitch2(const RC_Channel::AuxSwitchPos ch_flag)
{
    // put your aux switch #2 handler here (CHx_OPT = 48)
}

void Copter::userhook_auxSwitch3(const RC_Channel::AuxSwitchPos ch_flag)
{
    // put your aux switch #3 handler here (CHx_OPT = 49)
}
#endif
