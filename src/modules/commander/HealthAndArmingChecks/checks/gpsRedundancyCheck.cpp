/****************************************************************************
 *
 *   Copyright (c) 2024 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

#include "gpsRedundancyCheck.hpp"

using namespace time_literals;

void GpsRedundancyChecks::checkAndReport(const Context &context, Report &reporter)
{
	// Always reset — will be set below only when the condition is active
	reporter.failsafeFlags().gps_redundancy_lost = false;

	// Count GPS instances that are present and providing fresh data
	int active_count = 0;

	for (int i = 0; i < _sensor_gps_sub.size(); i++) {
		if (!_sensor_gps_sub[i].advertised()) {
			continue;
		}

		sensor_gps_s gps;

		if (_sensor_gps_sub[i].copy(&gps)
		    && (gps.device_id != 0)
		    && (gps.timestamp != 0)
		    && (hrt_elapsed_time(&gps.timestamp) < 2_s)) {
			active_count++;
		}
	}

	// Track the highest count seen — used to detect any GPS loss regardless of SYS_HAS_NUM_GPS
	if (active_count > _peak_active_count) {
		_peak_active_count = active_count;
	}

	const int required = _param_sys_has_num_gps.get();

	// Always warn if count dropped below peak (any GPS lost), unless SYS_HAS_NUM_GPS
	// already covers this case (to avoid sending two warnings for the same event)
	const bool redundancy_check_covers_loss = (required > 0 && active_count < required);

	if (_peak_active_count > 1 && active_count < _peak_active_count && !redundancy_check_covers_loss) {
		if (context.isArmed()) {
			/* EVENT
			 * @description
			 * A GPS receiver was lost during flight.
			 */
			reporter.healthFailure<uint8_t, uint8_t>(NavModes::All, health_component_t::gps,
					events::ID("check_gps_count_dropped"),
					events::Log::Warning,
					"GPS receiver lost: {1} of {2} previously active",
					(uint8_t)active_count, (uint8_t)_peak_active_count);
		}
	}

	// Enforce SYS_HAS_NUM_GPS requirement
	if (required <= 0 || active_count >= required) {
		return;
	}

	reporter.failsafeFlags().gps_redundancy_lost = true;

	if (!context.isArmed()) {
		/* EVENT
		 * @description
		 * <profile name="dev">
		 * Configure the minimum required GPS count with <param>SYS_HAS_NUM_GPS</param>.
		 * </profile>
		 */
		reporter.armingCheckFailure<uint8_t, uint8_t>(NavModes::All, health_component_t::gps,
				events::ID("check_gps_redundancy_insufficient"),
				events::Log::Error,
				"GPS redundancy check failed: {1} of {2} required receivers active",
				(uint8_t)active_count, (uint8_t)required);

		if (reporter.mavlink_log_pub()) {
			mavlink_log_critical(reporter.mavlink_log_pub(),
					     "Preflight Fail: GPS redundancy (%u of %u)", (unsigned)active_count, (unsigned)required);
		}

	} else {
		const events::Log log_level = (_param_com_gps_loss_act.get() == 0)
					      ? events::Log::Warning : events::Log::Error;

		/* EVENT
		 * @description
		 * A GPS receiver was lost during flight.
		 *
		 * <profile name="dev">
		 * Configure the minimum required GPS count with <param>SYS_HAS_NUM_GPS</param>.
		 * </profile>
		 */
		reporter.healthFailure<uint8_t, uint8_t>(NavModes::All, health_component_t::gps,
				events::ID("check_gps_redundancy_lost_inflight"),
				log_level,
				"GPS receiver lost: {1} of {2} required receivers active",
				(uint8_t)active_count, (uint8_t)required);
	}
}
