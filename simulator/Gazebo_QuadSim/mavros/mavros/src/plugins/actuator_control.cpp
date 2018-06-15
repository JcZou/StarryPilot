/**
 * @brief ActuatorControl plugin
 * @file actuator_control.cpp
 * @author Marcel Stüttgen <stuettgen@fh-aachen.de>
 *
 * @addtogroup plugin
 * @{
 */
/*
 * Copyright 2015 Marcel Stüttgen <stuettgen@fh-aachen.de>
 *
 * This file is part of the mavros package and subject to the license terms
 * in the top-level LICENSE file of the mavros repository.
 * https://github.com/mavlink/mavros/tree/master/LICENSE.md
 */

/* change log: by Jiachi, publish HIL_Actuator from autopilot to simulator */

#include <mavros/mavros_plugin.h>
#include <pluginlib/class_list_macros.h>

#include <mavros_msgs/ActuatorControl.h>

namespace mavplugin {
/**
 * @brief ActuatorControl plugin
 *
 * Sends actuator controls to FCU controller.
 */
class ActuatorControlPlugin : public MavRosPlugin {
public:
	ActuatorControlPlugin() :
		nh("~"),
		uas(nullptr)
	{ };

	void initialize(UAS &uas_)
	{
		uas = &uas_;

		//actuator_control_sub = nh.subscribe("actuator_control", 10, &ActuatorControlPlugin::actuator_control_cb, this);
		actuator_control_pub = nh.advertise<mavros_msgs::ActuatorControl>("actuator_control", 10);
	}

	const message_map get_rx_handlers() {
		return {
			       MESSAGE_HANDLER(MAVLINK_MSG_ID_HIL_ACTUATOR_CONTROLS, &ActuatorControlPlugin::handle_hil_actuator_controls),
		};
	}

private:
	ros::NodeHandle nh;
	UAS *uas;
	ros::Subscriber actuator_control_sub;
	ros::Publisher actuator_control_pub;

	/* -*- low-level send -*- */

	//! message definiton here: @p http://mavlink.org/messages/common#SET_ACTUATOR_CONTROL_TARGET
	void set_actuator_control_target(const uint64_t time_usec,
			const uint8_t group_mix,
			const float controls[8])
	{
		mavlink_message_t msg;

		mavlink_msg_set_actuator_control_target_pack_chan(UAS_PACK_CHAN(uas), &msg,
				time_usec,
				group_mix,
				UAS_PACK_TGT(uas),
				controls);
		UAS_FCU(uas)->send_message(&msg);
	}

	/* -*- callbacks -*- */

	void actuator_control_cb(const mavros_msgs::ActuatorControl::ConstPtr &req) {
		//! about groups, mixing and channels: @p https://pixhawk.org/dev/mixing
		set_actuator_control_target(ros::Time::now().toNSec() / 1000,
				req->group_mix,
				req->controls.data());
	}

	/* -*- rx handlers -*- */

	void handle_hil_actuator_controls(const mavlink_message_t *msg, uint8_t sysid, uint8_t compid) {
		mavlink_hil_actuator_controls_t hil_actuator_controls;
		mavlink_msg_hil_actuator_controls_decode(msg, &hil_actuator_controls);

		auto hil_actuator_controls_msg = boost::make_shared<mavros_msgs::ActuatorControl>();

		hil_actuator_controls_msg->header.stamp = uas->synchronise_stamp(hil_actuator_controls.time_usec);
		hil_actuator_controls_msg->group_mix = hil_actuator_controls.flags;	// motor number
		for(int i = 0 ; i < hil_actuator_controls_msg->group_mix ; i++)
			hil_actuator_controls_msg->controls[i] = hil_actuator_controls.controls[i];


		//ROS_INFO("num:%d control:%f %f %f %f", hil_actuator_controls_msg->group_mix, hil_actuator_controls_msg->controls[0], hil_actuator_controls_msg->controls[1],
		//		hil_actuator_controls_msg->controls[2], hil_actuator_controls_msg->controls[3]);
		actuator_control_pub.publish(hil_actuator_controls_msg);
	}
};
};	// namespace mavplugin

PLUGINLIB_EXPORT_CLASS(mavplugin::ActuatorControlPlugin, mavplugin::MavRosPlugin)
