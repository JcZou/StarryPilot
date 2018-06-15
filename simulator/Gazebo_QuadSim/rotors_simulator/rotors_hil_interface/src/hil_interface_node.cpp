/*
 * Copyright 2016 Pavel Vechersky, ASL, ETH Zurich, Switzerland
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

#include "rotors_hil_interface/hil_interface_node.h"
#include <sys/time.h>

namespace rotors_hil {

  HilInterfaceNode::HilInterfaceNode() :
      rate_(kDefaultHilFrequency) {
    ros::NodeHandle pnh("~");

    bool sensor_level_hil;
    double hil_frequency;
    double S_B_roll;
    double S_B_pitch;
    double S_B_yaw;
    std::string actuators_pub_topic;
    std::string mavlink_pub_topic;
    std::string hil_controls_sub_topic;
    std::string hil_actuator_control_sub_topic;

    /* read parameters from private namespace */
    pnh.param("sensor_level_hil", sensor_level_hil, kDefaultSensorLevelHil);
    pnh.param("hil_frequency", hil_frequency, kDefaultHilFrequency);
    pnh.param("body_to_sensor_roll", S_B_roll, kDefaultBodyToSensorsRoll);
    pnh.param("body_to_sensor_pitch", S_B_pitch, kDefaultBodyToSensorsPitch);
    pnh.param("body_to_sensor_yaw", S_B_yaw, kDefaultBodyToSensorsYaw);
    pnh.param("actuators_pub_topic", actuators_pub_topic, std::string(mav_msgs::default_topics::COMMAND_ACTUATORS));
    pnh.param("mavlink_pub_topic", mavlink_pub_topic, kDefaultMavlinkPubTopic);
    pnh.param("hil_controls_sub_topic", hil_controls_sub_topic, kDefaultHilControlsSubTopic);
    pnh.param("hil_actuator_control_sub_topic", hil_actuator_control_sub_topic, kDefaultActuatorControlSubTopic);


    // Create the quaternion and rotation matrix to rotate data into NED frame.
    Eigen::AngleAxisd roll_angle(S_B_roll, Eigen::Vector3d::UnitX());
    Eigen::AngleAxisd pitch_angle(S_B_pitch, Eigen::Vector3d::UnitY());
    Eigen::AngleAxisd yaw_angle(S_B_yaw, Eigen::Vector3d::UnitZ());

    const Eigen::Quaterniond q_S_B = roll_angle * pitch_angle * yaw_angle;

    ROS_INFO("hil freq:%d", hil_frequency);
    /* it is not hard real-time */
    rate_ = ros::Rate(hil_frequency);

    /* publisher to publish motor throttle */
    actuators_pub_ = nh_.advertise<mav_msgs::Actuators>(actuators_pub_topic, 1);
    /* publisher to publish mavlink topics */
    mavlink_pub_ = nh_.advertise<mavros_msgs::Mavlink>(mavlink_pub_topic, 5);
    /* hil_controls for fixwing */
    hil_controls_sub_ = nh_.subscribe(hil_controls_sub_topic, 1,
                                           &HilInterfaceNode::HilControlsCallback, this);
    /* hil_actuator_controls for multirotors */
    hil_actuator_control_sub_ = nh_.subscribe(hil_actuator_control_sub_topic, 1,
                                           &HilInterfaceNode::ActuatorControlsCallback, this);

    if (sensor_level_hil)
      hil_interface_ = std::auto_ptr<HilSensorLevelInterface>(new HilSensorLevelInterface(q_S_B, mavlink_pub_));
    else
      ROS_INFO("state level simulation, TODO");
      //hil_interface_ = std::auto_ptr<HilStateLevelInterface>(new HilStateLevelInterface(q_S_B,  mavlink_pub_));
  }

  HilInterfaceNode::~HilInterfaceNode() {
  }

  void HilInterfaceNode::MainTask() {
    //static double last_msec = 0;
    /* while node is not shutdown */
    // while (ros::ok()) {
    //   ros::spinOnce();
    //   /* collect sensor data */
    //   std::vector<mavros_msgs::Mavlink> hil_msgs = hil_interface_->CollectData();
    //   while (!hil_msgs.empty()) {
    //     /* publish mavlink msg */
    //     mavlink_pub_.publish(hil_msgs.back());
    //     hil_msgs.pop_back();
    //   }
    //   //ros::spinOnce();
    //   rate_.sleep();
    // }

    ros::spin();
  }

  void HilInterfaceNode::HilControlsCallback(const mavros_msgs::HilControlsConstPtr& hil_controls_msg) {
    mav_msgs::Actuators act_msg;

    ros::Time current_time = ros::Time::now();

    ROS_INFO("control cb");

    // act_msg.normalized.push_back(hil_controls_msg->roll_ailerons);
    // act_msg.normalized.push_back(hil_controls_msg->pitch_elevator);
    // act_msg.normalized.push_back(hil_controls_msg->yaw_rudder);
    // act_msg.normalized.push_back(hil_controls_msg->aux1);
    // act_msg.normalized.push_back(hil_controls_msg->aux2);
    // act_msg.normalized.push_back(hil_controls_msg->throttle);

    act_msg.header.stamp.sec = current_time.sec;
    act_msg.header.stamp.nsec = current_time.nsec;

    actuators_pub_.publish(act_msg);
  }

  void HilInterfaceNode::ActuatorControlsCallback(const mavros_msgs::ActuatorControlConstPtr& actuator_control_msg) {
    int motor_num = actuator_control_msg->group_mix;
    mav_msgs::ActuatorsPtr actuator_msg(new mav_msgs::Actuators);
    ros::Time current_time = ros::Time::now();

    actuator_msg->angular_velocities.clear();
    float motor_speed;
    for (int i = 0; i < motor_num; i++){
      if(actuator_control_msg->controls[i] < 0.06){
        // dead-zone throttle
        motor_speed = 0.0f;
      }else{
        motor_speed = 718.078 * actuator_control_msg->controls[i] + 88.448;
      }
      actuator_msg->angular_velocities.push_back(motor_speed);
    }


    actuator_msg->header.stamp.sec = current_time.sec;
    actuator_msg->header.stamp.nsec = current_time.nsec;

    actuators_pub_.publish(actuator_msg);
  }

}

int main(int argc, char** argv) {
  ros::init(argc, argv, "rotors_hil_interface_node");
  rotors_hil::HilInterfaceNode hil_interface_node;

  hil_interface_node.MainTask();

  return 0;
}
