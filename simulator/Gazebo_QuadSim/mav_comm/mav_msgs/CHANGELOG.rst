^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package mav_msgs
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
3.2.0 (2017-03-02)
------------------
* Access covariance in eigen odometry
* External force default topic
* External wind speed default topic

3.1.0 (2016-12-01)
------------------
* Add getEulerAngles method to EigenOdometry message.
* Improved quaternionFromMsg unit quaternion checking.
* Add EigenMavState to eigen_mav_msgs.
* Add EigenMavStateFromEigenTrajectoryPoint conversion.
* Add `timestamp_ns` to EigenTrajectoryPoint.
* Add default values in a seperate header.
* Add in_air bool to Status.msg.
* Add many helper function to calculate earth gravitational field based on hight and latitude, get euler angles from quaternion and shortest distance between two yaw angles.
* Contributors: Mina Kamel, Helen Oleynikova, Michael Burri

3.0.0 (2015-08-09)
------------------
* Dropped "Command" from the names of all messages.
* Converted CommandPositionYawTrajectory message to MultiDOFJointTrajectory (with an additional option to use a geometry_msgs/PoseStamped instead) as the two ways to send trajectory/waypoint commands.
* Added conversions between the Eigen and ROS message types.
* Switched to using full orientation instead of just yaw where appropriate.
* Documented reference frame in the Eigen messages where possible.
* Contributors: Helen Oleynikova, Markus Achtelik

2.0.3 (2015-05-22)
------------------
* added install target for include
  Headers can be included outside of this package.
* Contributors: Fadri Furrer
