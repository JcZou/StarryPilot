/*
 * Copyright 2015 Fadri Furrer, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Michael Burri, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Markus Achtelik, ASL, ETH Zurich, Switzerland
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

#ifndef PLANNING_MSGS_EIGEN_MAV_MSGS_H
#define PLANNING_MSGS_EIGEN_MAV_MSGS_H

#include <Eigen/Eigen>
#include <vector>

namespace planning_msgs {

struct EigenPolynomialSegment {
  EigenPolynomialSegment() : segment_time_ns(0), num_coeffs(0) {};

  Eigen::VectorXd x;
  Eigen::VectorXd y;
  Eigen::VectorXd z;
  Eigen::VectorXd yaw;
  uint64_t segment_time_ns;
  int num_coeffs;
};

typedef std::vector<EigenPolynomialSegment> EigenPolynomialTrajectory;

}

#endif
