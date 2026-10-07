/* Project-specific DroneCAN standby management. GPL-3.0-or-later. */
#include "AP_DroneCAN.h"

#if HAL_ENABLE_DRONECAN_DRIVERS
#include <AP_Arming/AP_Arming.h>
#include <AP_Logger/AP_Logger.h>

extern const AP_HAL::HAL& hal;

void AP_DroneCAN::standby_update(bool &standby)
{
    if (_standby_node.get() <= 0 || _standby.boot_id == 0) {
        return;
    }
    // Refuse ambiguous management across two input drivers.
    for (uint8_t i = 0; i < AP::can().get_num_drivers(); i++) {
        const auto *other = get_dronecan(i);
        if (other != nullptr && other != this && other->_standby_node.get() > 0) {
            return;
        }
    }
    WITH_SEMAPHORE(_standby_sem);
    const uint32_t now = AP_HAL::millis();
    _standby.supported = true;
    _standby.observed_ms = now;
    if (_standby.pending) {
        _standby.pending = false;
        if (int32_t(_standby.deadline_ms - now) <= 0) {
            _standby.result = POGO_REDUNDANCY_STANDBY_RESPONSE_EXPIRED;
        } else {
            standby = _standby.action == POGO_REDUNDANCY_STANDBY_REQUEST_STANDBY;
            _standby.applied_sequence = _standby.sequence;
            _standby.result = POGO_REDUNDANCY_STANDBY_RESPONSE_APPLIED;
            AP::logger().Write("SBCA", "TimeUS,Seq,State,Driver", "QIBB",
                               AP_HAL::micros64(), _standby.applied_sequence,
                               uint8_t(standby ? 1 : 2), _driver_index);
        }
    }
    _standby.state = standby ? POGO_REDUNDANCY_STANDBY_REQUEST_STANDBY : POGO_REDUNDANCY_STANDBY_REQUEST_ACTIVE;
}

void AP_DroneCAN::process_standby_command(const CanardRxTransfer& transfer,
                                        const pogo_redundancy_StandbyRequest& req,
                                        pogo_redundancy_StandbyResponse& rsp)
{
    {
        WITH_SEMAPHORE(_standby_sem);
        const uint32_t now = AP_HAL::millis();
        uint8_t result = POGO_REDUNDANCY_STANDBY_RESPONSE_OK;
        if (_standby_node.get() <= 0 || _standby.boot_id == 0) {
            result = POGO_REDUNDANCY_STANDBY_RESPONSE_DISABLED;
        } else if (transfer.source_node_id != _standby_node.get()) {
            result = POGO_REDUNDANCY_STANDBY_RESPONSE_UNAUTHORIZED;
        } else if (!_standby.supported || now - _standby.observed_ms > 100) {
            result = POGO_REDUNDANCY_STANDBY_RESPONSE_UNSUPPORTED;
        } else if (req.action == POGO_REDUNDANCY_STANDBY_REQUEST_QUERY) {
            // Read-only: does not bind a session or refresh a command deadline.
        } else if (req.action != POGO_REDUNDANCY_STANDBY_REQUEST_STANDBY &&
                   req.action != POGO_REDUNDANCY_STANDBY_REQUEST_ACTIVE) {
            result = POGO_REDUNDANCY_STANDBY_RESPONSE_INVALID;
        } else if (req.boot_id != _standby.boot_id) {
            result = POGO_REDUNDANCY_STANDBY_RESPONSE_BAD_BOOT;
        } else if (req.session_id == 0 ||
                   (req.session_id != _standby.session_id && AP::arming().is_armed())) {
            result = POGO_REDUNDANCY_STANDBY_RESPONSE_BAD_SESSION;
        } else if (req.sequence < _standby.sequence) {
            result = POGO_REDUNDANCY_STANDBY_RESPONSE_STALE;
        } else if (req.sequence == _standby.sequence) {
            // Retrying the same application transaction cannot reapply it or extend it.
            result = (req.session_id == _standby.session_id && req.action == _standby.action && req.valid_until_ms == _standby.deadline_ms)
                ? _standby.result : POGO_REDUNDANCY_STANDBY_RESPONSE_CONFLICT;
        } else if (_standby.pending) {
            result = POGO_REDUNDANCY_STANDBY_RESPONSE_BUSY;
        } else if (req.sequence == 0 || int32_t(req.valid_until_ms - now) > 500) {
            result = POGO_REDUNDANCY_STANDBY_RESPONSE_INVALID;
        } else if (int32_t(req.valid_until_ms - now) <= 0) {
            result = POGO_REDUNDANCY_STANDBY_RESPONSE_EXPIRED;
        } else {
            if (req.session_id != _standby.session_id) {
                _standby.applied_sequence = 0;
            }
            _standby.session_id = req.session_id;
            _standby.sequence = req.sequence;
            _standby.deadline_ms = req.valid_until_ms;
            _standby.action = req.action;
            _standby.pending = true;
            result = _standby.result = POGO_REDUNDANCY_STANDBY_RESPONSE_PENDING;
        }
        rsp.result = result;
        rsp.last_result = _standby.result;
        rsp.boot_id = _standby.boot_id;
        rsp.session_id = _standby.session_id;
        rsp.sequence = _standby.sequence;
        rsp.applied_sequence = _standby.applied_sequence;
        rsp.state = _standby.state;
        rsp.uptime_ms = now;
        const uint32_t age = now - _standby.observed_ms;
        rsp.state_age_ms = _standby.supported && age < UINT16_MAX ? age : UINT16_MAX;
    }
}

void AP_DroneCAN::handle_standby_request(const CanardRxTransfer& transfer,
                                       const pogo_redundancy_StandbyRequest& req)
{
    // Compatibility diagnostics only. Never allow two independent write paths.
    pogo_redundancy_StandbyRequest query {};
    pogo_redundancy_StandbyResponse rsp {};
    process_standby_command(transfer, query, rsp);
    if (req.action != POGO_REDUNDANCY_STANDBY_REQUEST_QUERY) {
        rsp.result = POGO_REDUNDANCY_STANDBY_RESPONSE_UNSUPPORTED;
    }
    standby_server.respond(transfer, rsp);
}

void AP_DroneCAN::handle_standby_command(const CanardRxTransfer& transfer,
                                       const pogo_redundancy_StandbyCommand& msg)
{
    if (msg.target_node_id != _dronecan_node.get() ||
        _standby_node.get() <= 0 || transfer.source_node_id != _standby_node.get()) {
        return;
    }
    pogo_redundancy_StandbyRequest req {};
    req.action = msg.action;
    req.boot_id = msg.boot_id;
    req.session_id = msg.session_id;
    req.sequence = msg.sequence;
    req.valid_until_ms = msg.valid_until_ms;
    pogo_redundancy_StandbyResponse rsp {};
    process_standby_command(transfer, req, rsp);
    WITH_SEMAPHORE(_standby_sem);
    _standby.rx_result = rsp.result;
    _standby.rx_sequence = msg.sequence;
}

void AP_DroneCAN::send_standby_status()
{
    pogo_redundancy_StandbyStatus msg {};
    {
        WITH_SEMAPHORE(_standby_sem);
        const uint32_t now = AP_HAL::millis();
        if (now - _standby.status_ms < 40) {
            return;
        }
        _standby.status_ms = now;
        msg.result = _standby_node.get() > 0 ? _standby.rx_result : POGO_REDUNDANCY_STANDBY_RESPONSE_DISABLED;
        msg.last_result = _standby.result;
        msg.state = _standby.state;
        msg.boot_id = _standby.boot_id;
        msg.session_id = _standby.session_id;
        msg.sequence = _standby.sequence;
        msg.applied_sequence = _standby.applied_sequence;
        msg.uptime_ms = now;
        const uint32_t age = now - _standby.observed_ms;
        msg.state_age_ms = _standby.supported && age < UINT16_MAX ? age : UINT16_MAX;
        msg.status_sequence = ++_standby.status_sequence;
        msg.command_sequence = _standby.rx_sequence;
    }
    standby_status_pub.broadcast(msg);
}
#endif
