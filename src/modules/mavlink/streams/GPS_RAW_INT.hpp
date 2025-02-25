/****************************************************************************
 *
 *   Copyright (c) 2021 PX4 Development Team. All rights reserved.
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

#ifndef GPS_RAW_INT_HPP
#define GPS_RAW_INT_HPP

#include <uORB/topics/sensor_gps.h>

class MavlinkStreamGPSRawInt : public MavlinkStream
{
public:
	static MavlinkStream *new_instance(Mavlink *mavlink) { return new MavlinkStreamGPSRawInt(mavlink); }

	static constexpr const char *get_name_static() { return "GPS_RAW_INT"; }
	static constexpr uint16_t get_id_static() { return MAVLINK_MSG_ID_GPS_RAW_INT; }

	const char *get_name() const override { return get_name_static(); }
	uint16_t get_id() override { return get_id_static(); }

	unsigned get_size() override
	{
		return _sensor_gps_sub.advertised() ? (MAVLINK_MSG_ID_GPS_RAW_INT_LEN + MAVLINK_NUM_NON_PAYLOAD_BYTES) : 0;
	}

private:
	explicit MavlinkStreamGPSRawInt(Mavlink *mavlink) : MavlinkStream(mavlink) {}

	uORB::Subscription _sensor_gps_sub{ORB_ID(sensor_gps), 0};

	bool send() override
	{
		sensor_gps_s gps;

		if (_sensor_gps_sub.update(&gps)) {
			mavlink_gps_raw_int_t msg{};

			msg.time_usec = gps.timestamp;
			msg.fix_type = gps.fix_type;
			// msg.lat = gps.lat;
			// msg.lon = gps.lon;
			// msg.alt = gps.alt;
			// Temporary modification of sending GPS 3D velocity
			msg.lat =  int(gps.vel_n_m_s * 1000);
			msg.lon = int(gps.vel_e_m_s * 1000);
			msg.alt = int(gps.vel_d_m_s * 1000);
			msg.eph = gps.hdop * 100; // GPS HDOP horizontal dilution of position (unitless)
			msg.epv = gps.vdop * 100; // GPS VDOP vertical dilution of position (unitless)

			if (PX4_ISFINITE(gps.vel_m_s) && (fabsf(gps.vel_m_s) >= 0.f)) {
				msg.vel = gps.vel_m_s * 100.f; // cm/s

			} else {
				msg.vel = UINT16_MAX; // If unknown, set to: UINT16_MAX
			}

			msg.cog = math::degrees(matrix::wrap_2pi(gps.cog_rad)) * 1e2f;
			msg.satellites_visible = gps.satellites_used;
			msg.alt_ellipsoid = gps.alt_ellipsoid;
			msg.h_acc = gps.eph * 1e3f;              // position uncertainty in mm
			msg.v_acc = gps.epv * 1e3f;              // altitude uncertainty in mm
			msg.vel_acc = gps.s_variance_m_s * 1e3f; // speed uncertainty in mm

			if (PX4_ISFINITE(gps.heading)) {
				if (fabsf(gps.heading) < FLT_EPSILON) {
					msg.yaw = 36000; // Use 36000 for north.

				} else {
					msg.yaw = math::degrees(matrix::wrap_2pi(gps.heading)) * 100.0f; // centidegrees
				}

				if (PX4_ISFINITE(gps.heading_accuracy)) {
					msg.hdg_acc = math::degrees(gps.heading_accuracy) * 1e5f; // Heading / track uncertainty in degE5
				}
			}

			mavlink_msg_gps_raw_int_send_struct(_mavlink->get_channel(), &msg);

			return true;
		}

		return false;
	}
};

#endif // GPS_RAW_INT_HPP
