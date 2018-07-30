/*
 * File      : gps_sensor.h
 *
 *
 * Change Logs:
 * Date			  Author       	Notes
 * 2016-06-30	  zoujiachi   	the first version
 */
 
#ifndef __GPS_SENSOR_H__
#define __GPS_SENSOR_H__

#include "stm32f4xx.h"
#include <rtthread.h>
#include "gps_ubx.h"

#define CONFIGURE_RETRY_MAX		2
#define RD_ONLY_POSLLH			1
#define RD_ONLY_VELNED			2
#define RD_COMPLETED_REPORT		3
#define RD_SVINFO				4

struct vehicle_gps_position_s {
	uint64_t timestamp_position;		/**< Timestamp for position information */
	int32_t lat;						/**< Latitude in 1E-7 degrees */
	int32_t lon;						/**< Longitude in 1E-7 degrees */
	int32_t alt;						/**< Altitude in 1E-3 meters (millimeters) above MSL  */

	uint64_t timestamp_variance;
	float s_variance_m_s;				/**< speed accuracy estimate m/s */
	float c_variance_rad;				/**< course accuracy estimate rad */
	uint8_t fix_type; 					/**< 0-1: no fix, 2: 2D fix, 3: 3D fix. Some applications will not use the value of this field unless it is at least two, so always correctly fill in the fix.   */

	float eph;							/**< GPS HDOP horizontal dilution of position in m */
	float epv;							/**< GPS VDOP horizontal dilution of position in m */

	unsigned noise_per_ms;				/**< */
	unsigned jamming_indicator;			/**< */

	uint64_t timestamp_velocity;		/**< Timestamp for velocity informations */
	float vel_m_s;						/**< GPS ground speed (m/s) */
	float vel_n_m_s;					/**< North velocity in m/s */
	float vel_e_m_s;					/**< East velocity in m/s */
	float vel_d_m_s;					/**< Down velocity in m/s */
	float cog_rad;						/**< Course over ground (NOT heading, but direction of movement) in rad, -PI..PI */
	rt_bool_t vel_ned_valid;			/**< Flag to indicate if NED speed is valid */

	uint64_t timestamp_time;			/**< Timestamp for time information */
	uint64_t time_gps_usec;				/**< Timestamp (microseconds in GPS format), this is the timestamp which comes from the gps module   */

	uint8_t satellites_used;			/**< Number of satellites used */
	float hdop;
	float ndop;
	float edop;
	float vdop;
	float tdop;
	uint32_t alt_ellipsoid;
};

rt_err_t rt_gps_init(char* serial_device_name , struct vehicle_gps_position_s *gps_position, struct satellite_info_s *satellite_info);

#endif

