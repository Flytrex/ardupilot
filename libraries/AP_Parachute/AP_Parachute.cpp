#include "AP_Parachute.h"
#include <AP_Relay/AP_Relay.h>
#include <AP_Math/AP_Math.h>
#include <RC_Channel/RC_Channel.h>
#include <SRV_Channel/SRV_Channel.h>
#include <AP_Notify/AP_Notify.h>
#include <AP_HAL/AP_HAL.h>
#include <AP_Logger/AP_Logger.h>
#include <GCS_MAVLink/GCS.h>

extern const AP_HAL::HAL& hal;

const AP_Param::GroupInfo AP_Parachute::var_info[] = {

    // @Param: ENABLED
    // @DisplayName: Parachute release enabled or disabled
    // @Description: Parachute release enabled or disabled
    // @Values: 0:Disabled,1:Enabled
    // @User: Standard
    AP_GROUPINFO_FLAGS("ENABLED", 0, AP_Parachute, _enabled, 0, AP_PARAM_FLAG_ENABLE),

    // @Param: TYPE
    // @DisplayName: Parachute release mechanism type (relay or servo)
    // @Description: Parachute release mechanism type (relay or servo)
    // @Values: 0:First Relay,1:Second Relay,2:Third Relay,3:Fourth Relay,10:Servo
    // @User: Standard
    AP_GROUPINFO("TYPE", 1, AP_Parachute, _release_type, AP_PARACHUTE_TRIGGER_TYPE_RELAY_0),

    // @Param: SERVO_ON
    // @DisplayName: Parachute Servo ON PWM value
    // @Description: Parachute Servo PWM value in microseconds when parachute is released
    // @Range: 1000 2000
    // @Units: PWM
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("SERVO_ON", 2, AP_Parachute, _servo_on_pwm, AP_PARACHUTE_SERVO_ON_PWM_DEFAULT),

    // @Param: SERVO_OFF
    // @DisplayName: Servo OFF PWM value
    // @Description: Parachute Servo PWM value in microseconds when parachute is not released
    // @Range: 1000 2000
    // @Units: PWM
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("SERVO_OFF", 3, AP_Parachute, _servo_off_pwm, AP_PARACHUTE_SERVO_OFF_PWM_DEFAULT),

    // @Param: ALT_MIN
    // @DisplayName: Parachute min altitude in meters above home
    // @Description: Parachute min altitude above home.  Parachute will not be released below this altitude.  0 to disable alt check.
    // @Range: 0 32000
    // @Units: m
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("ALT_MIN", 4, AP_Parachute, _alt_min, AP_PARACHUTE_ALT_MIN_DEFAULT),

    // @Param: DELAY_MS
    // @DisplayName: Parachute release delay
    // @Description: Delay in millseconds between motor stop and chute release
    // @Range: 0 5000
    // @Units: ms
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("DELAY_MS", 5, AP_Parachute, _delay_ms, AP_PARACHUTE_RELEASE_DELAY_MS),
    
    // @Param: SINK_CRT
    // @DisplayName: Critical sink speed rate in m/s to trigger emergency parachute
    // @Description: Release parachute when critical sink rate is reached
    // @Range: 0 15
    // @Units: m/s
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("SINK_CRT", 6, AP_Parachute, _critical_sink, AP_PARACHUTE_CRITICAL_SINK_DEFAULT),
    
    // @Param: SINK_TIME
    // @DisplayName: When critical sink speed rate for more than critical sink time - trigger emergency parachute
    // @Description: Release parachute when critical sink rate is reached for more than critical sink time
    // @Range: 0 5000
    // @Units: ms
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("SINK_TIME", 7, AP_Parachute, _critical_sink_time, AP_PARACHUTE_CRITICAL_SINK_TIME_DEFAULT),
    
    // @Param: FLIP_CRT
    // @DisplayName: Critical flip degree to trigger emergency parachute
    // @Description: Release parachute when critical flip rate is reached
    // @Range: 0 180
    // @Units: degrees
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("FLIP_CRT", 8, AP_Parachute, _critical_flip, AP_PARACHUTE_CRITICAL_FLIP_DEFAULT),
    
    // @Param: FLIP_TIME
    // @DisplayName: When in critical flip for more than critical flip time - trigger emergency parachute
    // @Description: Release parachute when critical flip rate is reached for more than critical flip time
    // @Range: 0 5000
    // @Units: ms
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("FLIP_TIME", 9, AP_Parachute, _critical_flip_time, AP_PARACHUTE_CRITICAL_FLIP_TIME_DEFAULT),

    // @Param: YAW_CRT
    // @DisplayName: Critical yaw rate in degrees/second to trigger emergency parachute
    // @Description: Release parachute when critical yaw rate is reached
    // @Range: 0 720
    // @Units: degrees/second
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("YAW_CRT", 10, AP_Parachute, _critical_yaw, AP_PARACHUTE_CRITICAL_YAW_DEFAULT),
    
    // @Param: YAW_TIME
    // @DisplayName: When in critical yaw for more than critical yaw time - trigger emergency parachute
    // @Description: Release parachute when critical yaw rate is reached for more than critical yaw time
    // @Range: 0 5000
    // @Units: ms
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("YAW_TIME", 11, AP_Parachute, _critical_yaw_time, AP_PARACHUTE_CRITICAL_YAW_TIME_DEFAULT),

    AP_GROUPEND
};

/// enabled - enable or disable parachute release
void AP_Parachute::enabled(bool on_off)
{
    _enabled = on_off;

    // clear release_time
    _release_time = 0;
    _release_initiated = false;

    AP::logger().Write_Event(_enabled ? LogEvent::PARACHUTE_ENABLED : LogEvent::PARACHUTE_DISABLED);
}

/// release - release parachute
void AP_Parachute::release()
{
    // exit immediately if not enabled
    if (_enabled <= 0) {
        return;
    }

    gcs().send_text(MAV_SEVERITY_CRITICAL,"Parachute: Released");
    AP::logger().Write_Event(LogEvent::PARACHUTE_RELEASED);

    // set release time to current system time
    if (_release_time == 0) {
        _release_time = AP_HAL::millis();
    }

    _release_initiated = true;

    // update AP_Notify
    AP_Notify::flags.parachute_release = 1;
}

// move servo to off position
void AP_Parachute::servo_off()
{
    if (_release_type == AP_PARACHUTE_TRIGGER_TYPE_SERVO) {
        SRV_Channels::set_output_pwm(SRV_Channel::k_parachute_release, _servo_off_pwm);
    }
}

bool AP_Parachute::_release_by_condition(const AP_Float& critical_condition, const AP_Float& critical_condition_time, const float& current_condition_value, uint32_t& condition_time)
{
    uint32_t time = AP_HAL::millis();
    if ((critical_condition > 0) && (current_condition_value > critical_condition) && !_release_initiated) {
        if (condition_time == 0) {
            condition_time = AP_HAL::millis();
        }
        if ((time - condition_time) >= critical_condition_time) {
            release();
            return true;
        }
    } else {
        condition_time = 0;
    }
    return false;
}

/// update - shuts off the trigger should be called at about 10hz
void AP_Parachute::update()
{
    // exit immediately if not enabled or parachute not to be released
    if (_enabled <= 0) {
        return;
    }

    const uint32_t time = AP_HAL::millis();
    const AP_AHRS &ahrs_yb = AP::ahrs();

    const int32_t yaw_rate = labs(roundf(ToDeg(ahrs_yb.get_yaw_rate_earth())));
    if (_release_by_condition(_critical_yaw, _critical_yaw_time, yaw_rate, _yaw_time)) {
        gcs().send_text(MAV_SEVERITY_INFO, "yaw_rate %ld, time %ld ms", yaw_rate, time - _yaw_time);
    }

    const int32_t pitch = labs(roundf(ahrs_yb.pitch_sensor / 100.0)); // attitude pitch in degrees
    if (_release_by_condition(_critical_flip, _critical_flip_time, pitch, _flip_time)) {
        gcs().send_text(MAV_SEVERITY_INFO, "pitch %ld, critical angle %d, time %lu ms", pitch, (int)_critical_flip, time - _flip_time);
    }

    const int32_t roll = labs(roundf(ahrs_yb.roll_sensor / 100.0));   // attitude roll in degrees
    if (_release_by_condition(_critical_flip, _critical_flip_time, roll, _flip_time)) {
        gcs().send_text(MAV_SEVERITY_INFO, "roll %ld, critical angle %d, time %lu ms", roll, (int)_critical_flip, time - _flip_time);
    }

    if (_release_by_condition(_critical_sink, _critical_sink_time, _sink_rate, _sink_time)) {
        gcs().send_text(MAV_SEVERITY_INFO, "sink %f, time %ld ms", _sink_rate, time - _sink_time);
    }
    
    // calc time since release
    uint32_t time_diff = AP_HAL::millis() - _release_time;
    uint32_t delay_ms = _delay_ms<=0 ? 0: (uint32_t)_delay_ms;
    
    // check if we should release parachute
    if ((_release_time != 0) && !_release_in_progress) {
        if (time_diff >= delay_ms) {
            if (_release_type == AP_PARACHUTE_TRIGGER_TYPE_SERVO) {
                // move servo
                SRV_Channels::set_output_pwm(SRV_Channel::k_parachute_release, _servo_on_pwm);
            }else if (_release_type <= AP_PARACHUTE_TRIGGER_TYPE_RELAY_3) {
                // set relay
                _relay.on(_release_type);
            }
            _release_in_progress = true;
            _released = true;
        }
    }else if ((_release_time == 0) || time_diff >= delay_ms + AP_PARACHUTE_RELEASE_DURATION_MS) {
        if (_release_type == AP_PARACHUTE_TRIGGER_TYPE_SERVO) {
            // move servo back to off position
            SRV_Channels::set_output_pwm(SRV_Channel::k_parachute_release, _servo_off_pwm);
        }else if (_release_type <= AP_PARACHUTE_TRIGGER_TYPE_RELAY_3) {
            // set relay back to zero volts
            _relay.off(_release_type);
        }
        // reset released flag and release_time
        _release_in_progress = false;
        _release_time = 0;
        // update AP_Notify
        AP_Notify::flags.parachute_release = 0;
    }
}

// singleton instance
AP_Parachute *AP_Parachute::_singleton;

namespace AP {

AP_Parachute *parachute()
{
    return AP_Parachute::get_singleton();
}

}
