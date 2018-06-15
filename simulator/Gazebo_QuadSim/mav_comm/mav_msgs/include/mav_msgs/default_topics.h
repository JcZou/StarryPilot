/*
 * Copyright 2015 Fadri Furrer, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Michael Burri, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Markus Achtelik, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Helen Oleynikova, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Mina Kamel, ASL, ETH Zurich, Switzerland
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0

 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#ifndef DEFAULT_TOPICS_H_
#define DEFAULT_TOPICS_H_

namespace mav_msgs {
namespace default_topics {

static constexpr char IMU[] = "imu";
static constexpr char MOTOR_MEASUREMENT[] = "motor_speed";
static constexpr char MAGNETIC_FIELD[] = "magnetic_field";
static constexpr char GPS[] = "gps";
static constexpr char RC[] = "rc";
static constexpr char STATUS[] = "status";
static constexpr char FILTERED_SENSOR_DATA[] = "filtered_sensor_data";
static constexpr char AIR_SPEED[] = "air_speed";
static constexpr char GROUND_SPEED[] = "ground_speed";

static constexpr char COMMAND_ACTUATORS[] = "command/motor_speed";
static constexpr char COMMAND_RATE_THRUST[] = "command/rate_thrust";
static constexpr char COMMAND_ROLL_PITCH_YAWRATE_THRUST[] =
    "command/roll_pitch_yawrate_thrust";
static constexpr char COMMAND_ATTITUDE_THRUST[] = "command/attitude_thrust";
static constexpr char COMMAND_TRAJECTORY[] = "command/trajectory";
static constexpr char COMMAND_POSE[] = "command/pose";
static constexpr char COMMAND_GPS_WAYPOINT[] = "command/gps_waypoint";

static constexpr char POSE[] = "pose";
static constexpr char POSE_WITH_COVARIANCE[] = "pose_with_covariance";
static constexpr char TRANSFORM[] = "transform";
static constexpr char ODOMETRY[] = "odometry";
static constexpr char POSITION[] = "position";

// Simulation-specific topic names.
static constexpr char WRENCH[] = "wrench";
static constexpr char WIND_SPEED[] = "wind_speed";
static constexpr char EXTERNAL_FORCE[] = "external_force";

static constexpr char GROUND_TRUTH_POSE[] = "ground_truth/pose";
static constexpr char GROUND_TRUTH_TWIST[] = "ground_truth/twist";

}  // end namespace default_topics
}  // end namespace mav_msgs

#endif /* DEFAULT_TOPICS_H_ */
