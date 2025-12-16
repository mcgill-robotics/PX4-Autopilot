// TODO the copyright copypasta

#include <uORB/uORB.h>
#include <uORB/topics/uORBTopics.hpp>
#include <uORB/topics/vehicle_global_position.h>
#include <uORB/topics/vehicle_local_position.h>
#include <uORB/topics/aux_global_position.h>
#include <uORB/topics/camera_capture.h>
#include <uORB/topics/estimator_aid_source2d.h>
#include <uORB/topics/actuator_armed.h>

// Ordered by increasing severity: Each restriction redacts a strict superset of
// the information redacted by the preceding restriction.
typedef enum {
	UNRESTRICTED = 0,
	NO_GLOBAL = 1,
	NO_LOCAL = 2
} log_restriction;

template<typename T>
inline void clear_global(T &value, int restriction)
{
	if (restriction >= log_restriction::NO_GLOBAL) {
		value *= T{0};
	}
}

template<typename T>
inline void clear_local(T &value, int restriction)
{
	if (restriction >= log_restriction::NO_LOCAL) {
		value *= T{0};
	}
}

void remove_position_from_message(const orb_id_size_t orb_id, void *buffer, int restriction)
{

	// Is this ok performance wise???
	const orb_metadata *metadata = get_orb_meta((ORB_ID) orb_id);

	if (static_cast<log_restriction>(restriction) == log_restriction::UNRESTRICTED) {
		return;
	}


	/**
	 * There is by design not a default here; DO NOT add one.
	 *
	 * When adding a new topic there is the risk of introducing position
	 * data, which if automatically default-ed here would break the position
	 * restricted logging feature.
	 *
	 * If you add a new topic that contains local or global position
	 * information, follow the pattern and provide the appropriate redaction
	 * below.
	 *
	 * If you add a new topic without any position information, add it to
	 * the last block explicitly to confirm this.
	 *
	 * Until that is done, we want the build to fail with -Werror=switch.
	 *
	 * Rules:
	 *  - Redact all horizontal velocities with clear_local -- whether from
	 *    sensors, estimate, or setpoints [*]. Airspeed and heading is imprecise
	 *    enough to log.
	 *  - Horizontal acceleration is okay to log - it is useless without
	 *    position reference and can only be accurately integrated (twice) over
	 *    very short time
	 *  - Global _altitude_ is always okay to log
	 *
	 *  * Possibly in the future we will add the functionality to add some noise
	 *    to the velocities, to make them useless for integrating up to position
	 *    but still useful for debugging.
	 *
	 */


	// TODO: putting higher rate topics first would be a bit of a perf gain
	// (if the compiler keeps the order)

	// TODO: add CI to check obvious things like not restricting fields
	// called "lat" "lon" "global" etc?

	switch (static_cast<ORB_ID>(metadata->o_id)) {
	case ORB_ID::vehicle_global_position:
	case ORB_ID::vehicle_global_position_groundtruth:
	case ORB_ID::estimator_global_position:
	case ORB_ID::external_ins_global_position: {
			auto *msg = static_cast<vehicle_global_position_s *>(buffer);
			clear_global(msg->lat, restriction);
			clear_global(msg->lon, restriction);
			break;
		}

	case ORB_ID::vehicle_local_position:
	case ORB_ID::vehicle_local_position_groundtruth:
	case ORB_ID::estimator_local_position:
	case ORB_ID::external_ins_local_position: {
			auto *msg = static_cast<vehicle_local_position_s *>(buffer);
			clear_local(msg->x, restriction);
			clear_local(msg->y, restriction);
			clear_local(msg->vx, restriction);
			clear_local(msg->vy, restriction);
			clear_global(msg->ref_lat, restriction);
			clear_global(msg->ref_lon, restriction);
			break;
		}

	case ORB_ID::aux_global_position: {
			auto *msg = static_cast<aux_global_position_s *>(buffer);
			clear_global(msg->lat, restriction);
			clear_global(msg->lon, restriction);
			break;
		}

	case ORB_ID::camera_capture: {
			auto *msg = static_cast<camera_capture_s *>(buffer);
			clear_global(msg->lat, restriction);
			clear_global(msg->lon, restriction);
			break;
		}

	case ORB_ID::estimator_aid_src_aux_global_position: {
			auto *msg = static_cast<estimator_aid_source2d_s *>(buffer);
			clear_global(msg->observation[0], restriction);
			clear_global(msg->observation[1], restriction);
			break;
		}

	case ORB_ID::estimator_aid_src_aux_vel:
	case ORB_ID::estimator_aid_src_ev_vel: {
			auto *msg = static_cast<estimator_aid_source2d_s *>(buffer);
			clear_local(msg->observation[0], restriction);
			clear_local(msg->observation[1], restriction);
			break;
		}

	case ORB_ID::estimator_aid_src_ev_pos:

	// All of these topics contain no local or global positon -- do not redact anything
	case ORB_ID::action_request:
	case ORB_ID::actuator_armed:
	case ORB_ID::actuator_controls_status_0:
	case ORB_ID::actuator_controls_status_1:
	case ORB_ID::actuator_motors:
	case ORB_ID::actuator_outputs:
	case ORB_ID::actuator_outputs_debug:
	case ORB_ID::actuator_outputs_sim:
	case ORB_ID::actuator_servos:
	case ORB_ID::actuator_servos_trim:
	case ORB_ID::actuator_test:
	case ORB_ID::airspeed:
	case ORB_ID::airspeed_validated:
	case ORB_ID::airspeed_wind: // TODO revisit if we can really log this one - no position but heavily correlated with position estimation
	case ORB_ID::arming_check_reply:
	case ORB_ID::arming_check_request:
	case ORB_ID::autotune_attitude_control_status:
	case ORB_ID::battery_info:
	case ORB_ID::battery_status:
	case ORB_ID::button_event:
	case ORB_ID::camera_status:
	case ORB_ID::camera_trigger:
	case ORB_ID::config_control_setpoints: // VehicleControlMode.msg
	case ORB_ID::vehicle_control_mode:
	case ORB_ID::config_overrides:
	case ORB_ID::config_overrides_request:
	case ORB_ID::adc_report: // TODO check again
	case ORB_ID::can_interface_status:
	case ORB_ID::cellular_status:
	case ORB_ID::collision_constraints: // TODO: revisit in more detail, only 90% sure this is no position
	case ORB_ID::control_allocator_status:
	case ORB_ID::cpuload:
	case ORB_ID::dataman_request:
	case ORB_ID::dataman_response:
	case ORB_ID::debug_array: // TODO: what is the float32[58] data field exactly?
	case ORB_ID::debug_key_value: // TODO: also this could basically have any data
	case ORB_ID::debug_value: // TODO same as above
	case ORB_ID::debug_vect: // TODO same as above
	case ORB_ID::device_information:
	case ORB_ID::differential_pressure:
	case ORB_ID::distance_sensor:
	case ORB_ID::distance_sensor_mode_change_request:
	case ORB_ID::dronecan_node_status:
	case ORB_ID::ekf2_timestamps: // timing only, ok to log
	case ORB_ID::esc_report:
	case ORB_ID::esc_serial_passthru:
	case ORB_ID::esc_status:
	case ORB_ID::estimator_aid_src_airspeed: // Related to velocity but too imprecise to reconstruct flight with inverse airspeed deadreckoning
	case ORB_ID::estimator_aid_src_baro_hgt:
	case ORB_ID::estimator_aid_src_drag:
	case ORB_ID::estimator_aid_src_ev_hgt:
		break;

	case ORB_ID::estimator_aid_src_ev_yaw:
	case ORB_ID::estimator_aid_src_fake_hgt:
	case ORB_ID::estimator_aid_src_fake_pos:
	case ORB_ID::estimator_aid_src_gnss_hgt:
	case ORB_ID::estimator_aid_src_gnss_pos:
	case ORB_ID::estimator_aid_src_gnss_vel:
	case ORB_ID::estimator_aid_src_gnss_yaw:
	case ORB_ID::estimator_aid_src_gravity:
	case ORB_ID::estimator_aid_src_mag:
	case ORB_ID::estimator_aid_src_optical_flow:
	case ORB_ID::estimator_aid_src_rng_hgt:
	case ORB_ID::estimator_aid_src_sideslip:
	case ORB_ID::estimator_attitude:
	case ORB_ID::estimator_baro_bias:
	case ORB_ID::estimator_bias3d:
	case ORB_ID::estimator_ev_pos_bias:
	case ORB_ID::estimator_event_flags:
	case ORB_ID::estimator_gnss_hgt_bias:
	case ORB_ID::estimator_gps_status:
	case ORB_ID::estimator_innovation_test_ratios:
	case ORB_ID::estimator_innovation_variances:
	case ORB_ID::estimator_innovations:
	case ORB_ID::estimator_odometry:
	case ORB_ID::estimator_optical_flow_vel:
	case ORB_ID::estimator_selector_status:
	case ORB_ID::estimator_sensor_bias:
	case ORB_ID::estimator_states:
	case ORB_ID::estimator_status:
	case ORB_ID::estimator_status_flags:
	case ORB_ID::estimator_wind:
	case ORB_ID::event:
	case ORB_ID::external_ins_attitude:
	case ORB_ID::failsafe_flags:
	case ORB_ID::failure_detector_status:
	case ORB_ID::figure_eight_status:
	case ORB_ID::fixed_wing_lateral_guidance_status:
	case ORB_ID::fixed_wing_lateral_setpoint:
	case ORB_ID::fixed_wing_lateral_status:
	case ORB_ID::fixed_wing_longitudinal_setpoint:
	case ORB_ID::fixed_wing_runway_control:
	case ORB_ID::flaps_setpoint:
	case ORB_ID::flight_phase_estimation:
	case ORB_ID::follow_target:
	case ORB_ID::follow_target_estimator:
	case ORB_ID::follow_target_status:
	case ORB_ID::fuel_tank_status:
	case ORB_ID::fw_virtual_attitude_setpoint:
	case ORB_ID::gain_compression:
	case ORB_ID::generator_status:
	case ORB_ID::geofence_result:
	case ORB_ID::geofence_status:
	case ORB_ID::gimbal_controls:
	case ORB_ID::gimbal_device_attitude_status:
	case ORB_ID::gimbal_device_information:
	case ORB_ID::gimbal_device_set_attitude:
	case ORB_ID::gimbal_manager_information:
	case ORB_ID::gimbal_manager_set_attitude:
	case ORB_ID::gimbal_manager_set_manual_control:
	case ORB_ID::gimbal_manager_status:
	case ORB_ID::gimbal_v1_command:
	case ORB_ID::goto_setpoint:
	case ORB_ID::gpio_config:
	case ORB_ID::gpio_in:
	case ORB_ID::gpio_out:
	case ORB_ID::gpio_request:
	case ORB_ID::gps_dump:
	case ORB_ID::gps_inject_data:
	case ORB_ID::gripper:
	case ORB_ID::health_report:
	case ORB_ID::heater_status:
	case ORB_ID::home_position:
	case ORB_ID::hover_thrust_estimate:
	case ORB_ID::input_rc:
	case ORB_ID::internal_combustion_engine_control:
	case ORB_ID::internal_combustion_engine_status:
	case ORB_ID::io_serial_passthru:
	case ORB_ID::iridiumsbd_status:
	case ORB_ID::irlock_report:
	case ORB_ID::landing_gear:
	case ORB_ID::landing_gear_wheel:
	case ORB_ID::landing_target_innovations:
	case ORB_ID::landing_target_pose:
	case ORB_ID::lateral_control_configuration:
	case ORB_ID::launch_detection_status:
	case ORB_ID::led_control:
	case ORB_ID::log_message:
	case ORB_ID::logger_status:
	case ORB_ID::longitudinal_control_configuration:
	case ORB_ID::mag_worker_data:
	case ORB_ID::magnetometer_bias_estimate:
	case ORB_ID::manual_control_input:
	case ORB_ID::manual_control_setpoint:
	case ORB_ID::manual_control_switches:
	case ORB_ID::mavlink_log:
	case ORB_ID::mavlink_tunnel:
	case ORB_ID::mc_virtual_attitude_setpoint:
	case ORB_ID::message_format_request:
	case ORB_ID::message_format_response:
	case ORB_ID::mission:
	case ORB_ID::mission_result:
	case ORB_ID::mode_completed:
	case ORB_ID::mount_orientation:
	case ORB_ID::navigator_mission_item:
	case ORB_ID::navigator_status:
	case ORB_ID::neural_control:
	case ORB_ID::obstacle_distance:
	case ORB_ID::obstacle_distance_fused:
	case ORB_ID::offboard_control_mode:
	case ORB_ID::onboard_computer_status:
	case ORB_ID::open_drone_id_arm_status:
	case ORB_ID::open_drone_id_operator_id:
	case ORB_ID::open_drone_id_self_id:
	case ORB_ID::open_drone_id_system:
	case ORB_ID::orb_multitest:
	case ORB_ID::orb_test:
	case ORB_ID::orb_test_large:
	case ORB_ID::orb_test_medium:
	case ORB_ID::orb_test_medium_multi:
	case ORB_ID::orb_test_medium_queue:
	case ORB_ID::orb_test_medium_queue_poll:
	case ORB_ID::orb_test_medium_wrap_around:
	case ORB_ID::orbit_status:
	case ORB_ID::parameter_primary_set_value_request:
	case ORB_ID::parameter_primary_set_value_response:
	case ORB_ID::parameter_remote_set_value_request:
	case ORB_ID::parameter_remote_set_value_response:
	case ORB_ID::parameter_reset_request:
	case ORB_ID::parameter_set_used_request:
	case ORB_ID::parameter_set_value_request:
	case ORB_ID::parameter_set_value_response:
	case ORB_ID::parameter_update:
	case ORB_ID::ping:
	case ORB_ID::position_controller_landing_status:
	case ORB_ID::position_controller_status:
	case ORB_ID::position_setpoint:
	case ORB_ID::position_setpoint_triplet:
	case ORB_ID::power_button_state:
	case ORB_ID::power_monitor:
	case ORB_ID::pps_capture:
	case ORB_ID::pure_pursuit_status:
	case ORB_ID::pwm_input:
	case ORB_ID::px4io_status:
	case ORB_ID::qshell_req:
	case ORB_ID::qshell_retval:
	case ORB_ID::radio_status:
	case ORB_ID::raptor_input:
	case ORB_ID::raptor_status:
	case ORB_ID::rate_ctrl_status:
	case ORB_ID::rc_channels:
	case ORB_ID::rc_parameter_map:
	case ORB_ID::register_ext_component_reply:
	case ORB_ID::register_ext_component_request:
	case ORB_ID::rover_attitude_setpoint:
	case ORB_ID::rover_attitude_status:
	case ORB_ID::rover_position_setpoint:
	case ORB_ID::rover_rate_setpoint:
	case ORB_ID::rover_rate_status:
	case ORB_ID::rover_speed_setpoint:
	case ORB_ID::rover_speed_status:
	case ORB_ID::rover_steering_setpoint:
	case ORB_ID::rover_throttle_setpoint:
	case ORB_ID::rpm:
	case ORB_ID::rtl_status:
	case ORB_ID::rtl_time_estimate:
	case ORB_ID::safety_button:
	case ORB_ID::satellite_info:
	case ORB_ID::sensor_accel:
	case ORB_ID::sensor_accel_fifo:
	case ORB_ID::sensor_airflow:
	case ORB_ID::sensor_baro:
	case ORB_ID::sensor_combined:
	case ORB_ID::sensor_correction:
	case ORB_ID::sensor_gnss_relative:
	case ORB_ID::sensor_gnss_status:
	case ORB_ID::sensor_gps:
	case ORB_ID::sensor_gyro:
	case ORB_ID::sensor_gyro_fft:
	case ORB_ID::sensor_gyro_fifo:
	case ORB_ID::sensor_hygrometer:
	case ORB_ID::sensor_mag:
	case ORB_ID::sensor_optical_flow:
	case ORB_ID::sensor_preflight_mag:
	case ORB_ID::sensor_selection:
	case ORB_ID::sensor_temp:
	case ORB_ID::sensor_uwb:
	case ORB_ID::sensors_status_baro:
	case ORB_ID::sensors_status_imu:
	case ORB_ID::sensors_status_mag:
	case ORB_ID::spoilers_setpoint:
	case ORB_ID::system_power:
	case ORB_ID::takeoff_status:
	case ORB_ID::task_stack_info:
	case ORB_ID::tecs_status:
	case ORB_ID::telemetry_status:
	case ORB_ID::tiltrotor_extra_controls:
	case ORB_ID::timesync_status:
	case ORB_ID::trajectory_setpoint:
	case ORB_ID::trajectory_setpoint6dof:
	case ORB_ID::transponder_report:
	case ORB_ID::tune_control:
	case ORB_ID::uavcan_parameter_request:
	case ORB_ID::uavcan_parameter_value:
	case ORB_ID::ulog_stream:
	case ORB_ID::ulog_stream_ack:
	case ORB_ID::unregister_ext_component:
	case ORB_ID::vehicle_acceleration:
	case ORB_ID::vehicle_air_data:
	case ORB_ID::vehicle_angular_acceleration_setpoint:
	case ORB_ID::vehicle_angular_velocity:
	case ORB_ID::vehicle_angular_velocity_groundtruth:
	case ORB_ID::vehicle_attitude:
	case ORB_ID::vehicle_attitude_groundtruth:
	case ORB_ID::vehicle_attitude_setpoint:
	case ORB_ID::vehicle_command:
	case ORB_ID::vehicle_command_ack:
	case ORB_ID::vehicle_command_mode_executor:
	case ORB_ID::vehicle_constraints:
	case ORB_ID::vehicle_gps_position:
	case ORB_ID::vehicle_imu:
	case ORB_ID::vehicle_imu_status:
	case ORB_ID::vehicle_land_detected:
	case ORB_ID::vehicle_local_position_setpoint:
	case ORB_ID::vehicle_magnetometer:
	case ORB_ID::vehicle_mocap_odometry:
	case ORB_ID::vehicle_odometry:
	case ORB_ID::vehicle_optical_flow:
	case ORB_ID::vehicle_optical_flow_vel:
	case ORB_ID::vehicle_rates_setpoint:
	case ORB_ID::vehicle_roi:
	case ORB_ID::vehicle_status:
	case ORB_ID::vehicle_thrust_setpoint:
	case ORB_ID::vehicle_thrust_setpoint_virtual_fw:
	case ORB_ID::vehicle_thrust_setpoint_virtual_mc:
	case ORB_ID::vehicle_torque_setpoint:
	case ORB_ID::vehicle_torque_setpoint_virtual_fw:
	case ORB_ID::vehicle_torque_setpoint_virtual_mc:
	case ORB_ID::vehicle_visual_odometry:
	case ORB_ID::velocity_limits:
	case ORB_ID::vtol_vehicle_status:
	case ORB_ID::vtx:
	case ORB_ID::wheel_encoders:
	case ORB_ID::wind:
	case ORB_ID::yaw_estimator_status:

	case ORB_ID::INVALID:

		PX4_ERR("unhandled case");
		break;

	}
}
