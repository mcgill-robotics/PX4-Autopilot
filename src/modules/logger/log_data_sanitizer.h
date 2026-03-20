#pragma once

#include <uORB/topics/uORBTopics.hpp>
#include <uORB/topics/vehicle_global_position.h>
#include <uORB/topics/vehicle_local_position.h>
// ... other position topic headers

typedef enum {
	UNRESTRICTED = 0,
	NO_GLOBAL    = 1,
	NO_LOCAL     = 2
} log_restriction;

template<typename T>
inline void clear_global(T &value, int r) { if (r >= NO_GLOBAL) value *= T{0}; }

template<typename T>
inline void clear_local(T &value, int r)  { if (r >= NO_LOCAL)  value *= T{0}; }

// --- Per-topic sanitize functions ---

inline void sanitize_vehicle_global_position(void *buffer, int r)
{
	auto *msg = static_cast<vehicle_global_position_s *>(buffer);
	clear_global(msg->lat, r);
	clear_global(msg->lon, r);
}

inline void sanitize_vehicle_local_position(void *buffer, int r)
{
	auto *msg = static_cast<vehicle_local_position_s *>(buffer);
	clear_local(msg->x, r);
	clear_local(msg->y, r);
	clear_local(msg->vx, r);
	clear_local(msg->vy, r);
	clear_global(msg->ref_lat, r);
	clear_global(msg->ref_lon, r);
}

// ... other position topics


// --- Lookup table ---

// Any way to make this more specific? void* are scary
using SanitizeFn = void (*)(void *, int);

// Explicit no-op for topics we have reviewed and confirmed contain no position data
inline void sanitize_noop(void *buffer, int r) {}


struct SanitizerEntry {
	ORB_ID      id;
	SanitizeFn  fn;
};

// TODO:
//
// 1. This still does not build for all targets because some builds don't have all the topics.
//
//   a) switch to a string -> function pointer lookup table here (still
//      populating the runtime ORB_ID -> function pointer table the same)
//   b) gate the relevant topics by whatever gates their inclusion in the build
//
// 2. Classify the topics, write the redactions
//
// 3. Ideally add some CI that complains if something is in logged_topics but not here?
//    Might be pretty simple regex based.
//
// 4. from logged_topics throw out the ones that apparently don't exist anymore

static const SanitizerEntry known_sanitizers[] = {
	{ ORB_ID::vehicle_global_position,             sanitize_vehicle_global_position },
	{ ORB_ID::vehicle_global_position_groundtruth, sanitize_vehicle_global_position },
	{ ORB_ID::estimator_global_position,           sanitize_vehicle_global_position },
	{ ORB_ID::vehicle_local_position,              sanitize_vehicle_local_position  },
	{ ORB_ID::actuator_armed,                      sanitize_noop },
	{ ORB_ID::action_request, sanitize_noop },
	{ ORB_ID::actuator_armed, sanitize_noop },
	{ ORB_ID::airspeed, sanitize_noop },
	{ ORB_ID::battery_info, sanitize_noop },
	{ ORB_ID::cellular_status, sanitize_noop },
	// { ORB_ID::commander_state, sanitize_noop },  // does not exist anymore I think
	{ ORB_ID::config_overrides, sanitize_noop },
	{ ORB_ID::cpuload, sanitize_noop },
	{ ORB_ID::distance_sensor_mode_change_request, sanitize_noop },
	{ ORB_ID::device_information, sanitize_noop },
	{ ORB_ID::dronecan_node_status, sanitize_noop },
	// { ORB_ID::esc_status, sanitize_noop }, CONFIG_UAV_CANNODE_ESC_STATUS
	{ ORB_ID::failure_detector_status, sanitize_noop },
	{ ORB_ID::failsafe_flags, sanitize_noop },
	{ ORB_ID::gimbal_manager_set_attitude, sanitize_noop },
	{ ORB_ID::gps_dump, sanitize_noop },
	{ ORB_ID::hover_thrust_estimate, sanitize_noop },
	{ ORB_ID::input_rc, sanitize_noop },
	{ ORB_ID::log_message_incoming, sanitize_noop },
	{ ORB_ID::logger_status, sanitize_noop },
	{ ORB_ID::manual_control_setpoint, sanitize_noop },
	{ ORB_ID::manual_control_switches, sanitize_noop },
	{ ORB_ID::mission_result, sanitize_noop },
	{ ORB_ID::navigator_status, sanitize_noop },
	{ ORB_ID::offboard_control_mode, sanitize_noop },
	{ ORB_ID::onboard_computer_status, sanitize_noop },
	{ ORB_ID::parameter_update, sanitize_noop },
	{ ORB_ID::position_controller_status, sanitize_noop },
	{ ORB_ID::position_controller_landing_status, sanitize_noop },
	{ ORB_ID::goto_setpoint, sanitize_noop },
	{ ORB_ID::radio_status, sanitize_noop },
	{ ORB_ID::rtl_time_estimate, sanitize_noop },
	{ ORB_ID::rtl_status, sanitize_noop },
	{ ORB_ID::sensor_combined, sanitize_noop },
	{ ORB_ID::sensor_selection, sanitize_noop },
	{ ORB_ID::sensors_status_imu, sanitize_noop },
	{ ORB_ID::system_power, sanitize_noop },
	{ ORB_ID::trajectory_setpoint, sanitize_noop },
	{ ORB_ID::vehicle_acceleration, sanitize_noop },
	{ ORB_ID::vehicle_air_data, sanitize_noop },
	{ ORB_ID::vehicle_angular_velocity, sanitize_noop },
	{ ORB_ID::vehicle_attitude, sanitize_noop },
	{ ORB_ID::vehicle_attitude_setpoint, sanitize_noop },
	{ ORB_ID::vehicle_command_ack, sanitize_noop },
	{ ORB_ID::vehicle_constraints, sanitize_noop },
	{ ORB_ID::vehicle_control_mode, sanitize_noop },
	{ ORB_ID::vehicle_land_detected, sanitize_noop },
	{ ORB_ID::vehicle_local_position_setpoint, sanitize_noop },
	{ ORB_ID::vehicle_magnetometer, sanitize_noop },
	{ ORB_ID::vehicle_rates_setpoint, sanitize_noop },
	{ ORB_ID::vehicle_status, sanitize_noop },
	{ ORB_ID::vtx, sanitize_noop },
	{ ORB_ID::wind, sanitize_noop },
	{ ORB_ID::fixed_wing_lateral_setpoint, sanitize_noop },
	{ ORB_ID::fixed_wing_longitudinal_setpoint, sanitize_noop },
	{ ORB_ID::longitudinal_control_configuration, sanitize_noop },
	{ ORB_ID::lateral_control_configuration, sanitize_noop },
	{ ORB_ID::follow_target_status, sanitize_noop },
	{ ORB_ID::follow_target_estimator, sanitize_noop },
	{ ORB_ID::home_position, sanitize_noop },
	{ ORB_ID::navigator_mission_item, sanitize_noop },
	{ ORB_ID::position_setpoint_triplet, sanitize_noop },
	{ ORB_ID::transponder_report, sanitize_noop },
	{ ORB_ID::vehicle_command, sanitize_noop },
	{ ORB_ID::vehicle_global_position, sanitize_noop },
	{ ORB_ID::vehicle_gps_position, sanitize_noop },
	{ ORB_ID::vehicle_local_position, sanitize_noop },
	{ ORB_ID::vehicle_roi, sanitize_noop },
	{ ORB_ID::timesync_status, sanitize_noop },
	{ ORB_ID::battery_status, sanitize_noop },
	{ ORB_ID::differential_pressure, sanitize_noop },
	{ ORB_ID::distance_sensor, sanitize_noop },
	// { ORB_ID::optical_flow, sanitize_noop },  is this not vehicle_optical_flow sensor_optical_flow etc? -> also stale...
	{ ORB_ID::sensor_baro, sanitize_noop },
	{ ORB_ID::sensor_gps, sanitize_noop },
	{ ORB_ID::aux_global_position, sanitize_noop },
	{ ORB_ID::sensor_gnss_relative, sanitize_noop },
	{ ORB_ID::sensor_mag, sanitize_noop },
	{ ORB_ID::sensor_optical_flow, sanitize_noop },
	{ ORB_ID::vehicle_imu, sanitize_noop },
	{ ORB_ID::vehicle_imu_status, sanitize_noop },
	{ ORB_ID::vehicle_optical_flow, sanitize_noop },
	{ ORB_ID::aux_global_position, sanitize_noop },
	{ ORB_ID::actuator_motors, sanitize_noop },
	{ ORB_ID::actuator_servos, sanitize_noop },
	{ ORB_ID::vehicle_thrust_setpoint, sanitize_noop },
	{ ORB_ID::vehicle_torque_setpoint, sanitize_noop },
	{ ORB_ID::vehicle_angular_velocity_groundtruth, sanitize_noop },
	{ ORB_ID::vehicle_attitude_groundtruth, sanitize_noop },
	{ ORB_ID::vehicle_global_position_groundtruth, sanitize_noop },
	{ ORB_ID::vehicle_local_position_groundtruth, sanitize_noop },
	{ ORB_ID::fw_virtual_attitude_setpoint, sanitize_noop },
	{ ORB_ID::mc_virtual_attitude_setpoint, sanitize_noop },
	// { ORB_ID::time_offset, sanitize_noop }, - does not exist anymore
	{ ORB_ID::vehicle_angular_velocity, sanitize_noop },
	{ ORB_ID::vehicle_angular_velocity_groundtruth, sanitize_noop },
	{ ORB_ID::vehicle_attitude_groundtruth, sanitize_noop },
	{ ORB_ID::vehicle_global_position_groundtruth, sanitize_noop },
	{ ORB_ID::vehicle_local_position_groundtruth, sanitize_noop },
	{ ORB_ID::vehicle_attitude, sanitize_noop },
	{ ORB_ID::vehicle_global_position, sanitize_noop },
	{ ORB_ID::vehicle_local_position, sanitize_noop },
	{ ORB_ID::wind, sanitize_noop },
	{ ORB_ID::can_interface_status, sanitize_noop },
	{ ORB_ID::manual_control_setpoint, sanitize_noop },
	{ ORB_ID::rate_ctrl_status, sanitize_noop },
	{ ORB_ID::sensor_combined, sanitize_noop },
	{ ORB_ID::vehicle_angular_velocity, sanitize_noop },
	{ ORB_ID::vehicle_attitude, sanitize_noop },
	{ ORB_ID::vehicle_attitude_setpoint, sanitize_noop },
	{ ORB_ID::vehicle_rates_setpoint, sanitize_noop },
	{ ORB_ID::esc_status, sanitize_noop },
	{ ORB_ID::actuator_motors, sanitize_noop },
	{ ORB_ID::actuator_outputs_debug, sanitize_noop },
	{ ORB_ID::actuator_servos, sanitize_noop },
	{ ORB_ID::vehicle_thrust_setpoint, sanitize_noop },
	{ ORB_ID::vehicle_torque_setpoint, sanitize_noop },
	{ ORB_ID::debug_array, sanitize_noop },
	{ ORB_ID::debug_key_value, sanitize_noop },
	{ ORB_ID::debug_value, sanitize_noop },
	{ ORB_ID::debug_vect, sanitize_noop },
	{ ORB_ID::satellite_info, sanitize_noop },
	{ ORB_ID::mag_worker_data, sanitize_noop },
	{ ORB_ID::sensor_preflight_mag, sanitize_noop },
	{ ORB_ID::actuator_test, sanitize_noop },
	{ ORB_ID::neural_control, sanitize_noop },
	{ ORB_ID::ekf2_timestamps, sanitize_noop },
	{ ORB_ID::airspeed, sanitize_noop },
	{ ORB_ID::airspeed_validated, sanitize_noop },
	{ ORB_ID::vehicle_optical_flow, sanitize_noop },
	{ ORB_ID::sensor_combined, sanitize_noop },
	{ ORB_ID::sensor_selection, sanitize_noop },
	{ ORB_ID::vehicle_air_data, sanitize_noop },
	{ ORB_ID::vehicle_gps_position, sanitize_noop },
	{ ORB_ID::aux_global_position, sanitize_noop },
	{ ORB_ID::vehicle_land_detected, sanitize_noop },
	{ ORB_ID::vehicle_magnetometer, sanitize_noop },
	{ ORB_ID::vehicle_status, sanitize_noop },
	{ ORB_ID::vehicle_visual_odometry, sanitize_noop },
	{ ORB_ID::aux_global_position, sanitize_noop },
	{ ORB_ID::distance_sensor, sanitize_noop },
	{ ORB_ID::sensor_accel, sanitize_noop },
	{ ORB_ID::sensor_baro, sanitize_noop },
	{ ORB_ID::sensor_gyro, sanitize_noop },
	{ ORB_ID::sensor_mag, sanitize_noop },
	{ ORB_ID::sensor_accel, sanitize_noop },
	{ ORB_ID::sensor_baro, sanitize_noop },
	{ ORB_ID::sensor_gyro, sanitize_noop },
	{ ORB_ID::sensor_mag, sanitize_noop },
	{ ORB_ID::collision_constraints, sanitize_noop },
	{ ORB_ID::distance_sensor, sanitize_noop },
	{ ORB_ID::obstacle_distance_fused, sanitize_noop },
	{ ORB_ID::obstacle_distance, sanitize_noop },
	{ ORB_ID::vehicle_mocap_odometry, sanitize_noop },
	{ ORB_ID::vehicle_visual_odometry, sanitize_noop },
	{ ORB_ID::sensor_gyro_fifo, sanitize_noop },
	{ ORB_ID::sensor_accel_fifo, sanitize_noop },
	{ ORB_ID::sensor_combined, sanitize_noop },
	{ ORB_ID::vehicle_angular_velocity, sanitize_noop },
	{ ORB_ID::vehicle_torque_setpoint, sanitize_noop },
	{ ORB_ID::vehicle_acceleration, sanitize_noop },
	{ ORB_ID::actuator_motors, sanitize_noop },
	{ ORB_ID::distance_sensor, sanitize_noop },
	{ ORB_ID::sensor_optical_flow, sanitize_noop },
	{ ORB_ID::sensor_gps, sanitize_noop },
	{ ORB_ID::sensor_gnss_relative, sanitize_noop },
	{ ORB_ID::sensor_mag, sanitize_noop },
	{ ORB_ID::mavlink_tunnel, sanitize_noop },
};

class LogDataSanitizer
{
public:
	void init()
	{
		// const orb_metadata *const *topics = orb_get_topics();

		// Default: zero the whole message for any topic not in known_sanitizers.
		// Stored as nullptr here; the call site checks and memsets using o_size.
		for (size_t i = 0; i < ORB_TOPICS_COUNT; i++) {
			_lookup[i] = nullptr;  // nullptr == "unknown, redact entirely"
		}

		for (const auto &entry : known_sanitizers) {
			_lookup[static_cast<orb_id_size_t>(entry.id)] = entry.fn;
		}
	}

	void sanitize(const orb_id_size_t o_id, void *buffer, int restriction) const
	{
		if (restriction == UNRESTRICTED) { return; }

		// Maybe too paranoid to guard agains this? should be guaranteed
		if (o_id >= ORB_TOPICS_COUNT) { return; }

		const orb_id_t meta = get_orb_meta(static_cast<ORB_ID>(o_id));

		SanitizeFn fn = _lookup[o_id];

		if (fn == nullptr) {
			// Unknown topic: zero entire message to be safe
			// TODO but warn once this happens the first time, telling users to add the redaction
			memset(buffer, 0, meta->o_size);

		} else {
			fn(buffer, restriction);
		}
	}

private:
	SanitizeFn _lookup[ORB_TOPICS_COUNT] {};
};
