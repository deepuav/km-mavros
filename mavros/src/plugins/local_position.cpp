/**
 * @brief LocalPosition plugin
 * @file local_position.cpp
 * @author Vladimir Ermakov <vooon341@gmail.com>
 * @author Glenn Gregory
 * @author Eddy Scott <scott.edward@aurora.aero>
 *
 * @addtogroup plugin
 * @{
 */
/*
 * Copyright 2014,2016 Vladimir Ermakov.
 *
 * This file is part of the mavros package and subject to the license terms
 * in the top-level LICENSE file of the mavros repository.
 * https://github.com/mavlink/mavros/tree/master/LICENSE.md
 */

#include <mavros/mavros_plugin.h>
#include <eigen_conversions/eigen_msg.h>

#include <geometry_msgs/AccelWithCovarianceStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/TwistWithCovarianceStamped.h>
#include <geometry_msgs/TransformStamped.h>

#include <nav_msgs/Odometry.h>

namespace mavros {
namespace std_plugins {
/**
 * @brief Local position plugin.
 * Publish local position to TF, PositionStamped, TwistStamped
 * and Odometry
 */
class LocalPositionPlugin : public plugin::PluginBase {
public:
	LocalPositionPlugin() : PluginBase(),
		lp_nh("~local_position"),
		tf_send(false),
		has_local_position_ned(false),
		has_local_position_ned_cov(false),
		has_odometry(false)
	{ }

	void initialize(UAS &uas_) override
	{
		PluginBase::initialize(uas_);

		// header frame_id.
		// default to map (world-fixed,ENU as per REP-105).
		lp_nh.param<std::string>("frame_id", frame_id, "map");
		// Important tf subsection
		// Report the transform from world to base_link here.
		lp_nh.param("tf/send", tf_send, false);
		lp_nh.param<std::string>("tf/frame_id", tf_frame_id, "map");
		lp_nh.param<std::string>("tf/child_frame_id", tf_child_frame_id, "base_link");
		
		// Whether to use ODOMETRY message for odom
		lp_nh.param("use_odometry", use_odometry, true);
		
		// Whether to process ODOMETRY messages at all
		lp_nh.param("enable_odometry_handler", enable_odometry_handler, false);

		local_position = lp_nh.advertise<geometry_msgs::PoseStamped>("pose", 10);
		local_position_cov = lp_nh.advertise<geometry_msgs::PoseWithCovarianceStamped>("pose_cov", 10);
		local_velocity_local = lp_nh.advertise<geometry_msgs::TwistStamped>("velocity_local", 10);
		local_velocity_body = lp_nh.advertise<geometry_msgs::TwistStamped>("velocity_body", 10);
		local_velocity_cov = lp_nh.advertise<geometry_msgs::TwistWithCovarianceStamped>("velocity_body_cov", 10);
		local_accel = lp_nh.advertise<geometry_msgs::AccelWithCovarianceStamped>("accel", 10);
		local_odom = lp_nh.advertise<nav_msgs::Odometry>("odom",10);
		// New topic for external odometry from MAVLINK ODOMETRY message
		external_odom = lp_nh.advertise<nav_msgs::Odometry>("external_odom", 10);
	}

	Subscriptions get_subscriptions() override {
		return {
			       make_handler(&LocalPositionPlugin::handle_local_position_ned),
			       make_handler(&LocalPositionPlugin::handle_local_position_ned_cov),
			       make_handler(&LocalPositionPlugin::handle_odometry)
		};
	}

private:
	ros::NodeHandle lp_nh;

	ros::Publisher local_position;
	ros::Publisher local_position_cov;
	ros::Publisher local_velocity_local;
	ros::Publisher local_velocity_body;
	ros::Publisher local_velocity_cov;
	ros::Publisher local_accel;
	ros::Publisher local_odom;
	ros::Publisher external_odom;

	std::string frame_id;		//!< frame for Pose
	std::string tf_frame_id;	//!< origin for TF
	std::string tf_child_frame_id;	//!< frame for TF
	bool tf_send;
	bool has_local_position_ned;
	bool has_local_position_ned_cov;
	bool has_odometry;
	bool use_odometry;		//!< whether to use ODOMETRY message for odom
	bool enable_odometry_handler;	//!< whether to process ODOMETRY messages at all
	
	void publish_tf(boost::shared_ptr<nav_msgs::Odometry> &odom)
	{
		if (tf_send) {
			geometry_msgs::TransformStamped transform;
			transform.header.stamp = odom->header.stamp;
			transform.header.frame_id = tf_frame_id;
			transform.child_frame_id = tf_child_frame_id;
			transform.transform.translation.x = odom->pose.pose.position.x;
			transform.transform.translation.y = odom->pose.pose.position.y;
			transform.transform.translation.z = odom->pose.pose.position.z;
			transform.transform.rotation = odom->pose.pose.orientation;
			m_uas->tf2_broadcaster.sendTransform(transform);
		}
	}

	void handle_local_position_ned(const mavlink::mavlink_message_t *msg, mavlink::common::msg::LOCAL_POSITION_NED &pos_ned)
	{
		has_local_position_ned = true;

		//--------------- Transform FCU position and Velocity Data ---------------//
		// 特殊转换方案：x和z取反、y不变
		Eigen::Vector3d enu_position(-pos_ned.x, pos_ned.y, -pos_ned.z);
		Eigen::Vector3d enu_velocity(-pos_ned.vx, pos_ned.vy, -pos_ned.vz);

		//--------------- Get Odom Information ---------------//
		// Note this orientation describes baselink->ENU transform
		auto enu_orientation_msg = m_uas->get_attitude_orientation_enu();
		auto baselink_angular_msg = m_uas->get_attitude_angular_velocity_enu();
		Eigen::Quaterniond enu_orientation;
		tf::quaternionMsgToEigen(enu_orientation_msg, enu_orientation);
		
		// 调整baselink_linear的计算，以适应新的坐标转换方案
		// 使用直接转换，不调用可能使用标准转换的ftf函数
		Eigen::Vector3d baselink_linear = enu_orientation.inverse() * enu_velocity;

		auto odom = boost::make_shared<nav_msgs::Odometry>();
		odom->header = m_uas->synchronized_header(frame_id, pos_ned.time_boot_ms);
		odom->child_frame_id = tf_child_frame_id;

		tf::pointEigenToMsg(enu_position, odom->pose.pose.position);
		odom->pose.pose.orientation = enu_orientation_msg;
		tf::vectorEigenToMsg(baselink_linear, odom->twist.twist.linear);
		odom->twist.twist.angular = baselink_angular_msg;

		// publish odom if we don't have LOCAL_POSITION_NED_COV
		if (!has_local_position_ned_cov) {
			local_odom.publish(odom);
		}

		// publish pose always
		auto pose = boost::make_shared<geometry_msgs::PoseStamped>();
		pose->header = odom->header;
		pose->pose = odom->pose.pose;
		local_position.publish(pose);

		// publish velocity always
		// velocity in the body frame
		auto twist_body = boost::make_shared<geometry_msgs::TwistStamped>();
		twist_body->header.stamp = odom->header.stamp;
		twist_body->header.frame_id = tf_child_frame_id;
		twist_body->twist.linear = odom->twist.twist.linear;
		twist_body->twist.angular = baselink_angular_msg;
		local_velocity_body.publish(twist_body);

		// velocity in the local frame
		auto twist_local = boost::make_shared<geometry_msgs::TwistStamped>();
		twist_local->header.stamp = twist_body->header.stamp;
		twist_local->header.frame_id = tf_child_frame_id;
		tf::vectorEigenToMsg(enu_velocity, twist_local->twist.linear);
		tf::vectorEigenToMsg(ftf::transform_frame_baselink_enu(ftf::to_eigen(baselink_angular_msg), enu_orientation),
						twist_local->twist.angular);

		local_velocity_local.publish(twist_local);

		// publish tf
		publish_tf(odom);
	}

	void handle_local_position_ned_cov(const mavlink::mavlink_message_t *msg, mavlink::common::msg::LOCAL_POSITION_NED_COV &pos_ned)
	{
		has_local_position_ned_cov = true;

		// 特殊转换方案：x和z取反、y不变
		Eigen::Vector3d enu_position(-pos_ned.x, pos_ned.y, -pos_ned.z);
		Eigen::Vector3d enu_velocity(-pos_ned.vx, pos_ned.vy, -pos_ned.vz);

		auto enu_orientation_msg = m_uas->get_attitude_orientation_enu();
		auto baselink_angular_msg = m_uas->get_attitude_angular_velocity_enu();
		Eigen::Quaterniond enu_orientation;
		tf::quaternionMsgToEigen(enu_orientation_msg, enu_orientation);
		
		// 使用直接转换，不调用可能使用标准转换的ftf函数
		Eigen::Vector3d baselink_linear = enu_orientation.inverse() * enu_velocity;

		auto odom = boost::make_shared<nav_msgs::Odometry>();
		odom->header = m_uas->synchronized_header(frame_id, pos_ned.time_usec);
		odom->child_frame_id = tf_child_frame_id;

		tf::pointEigenToMsg(enu_position, odom->pose.pose.position);
		odom->pose.pose.orientation = enu_orientation_msg;
		tf::vectorEigenToMsg(baselink_linear, odom->twist.twist.linear);
		odom->twist.twist.angular = baselink_angular_msg;

		odom->pose.covariance[0] = pos_ned.covariance[0];	// x
		odom->pose.covariance[7] = pos_ned.covariance[9];	// y
		odom->pose.covariance[14] = pos_ned.covariance[17];	// z

		odom->twist.covariance[0] = pos_ned.covariance[24];	// vx
		odom->twist.covariance[7] = pos_ned.covariance[30];	// vy
		odom->twist.covariance[14] = pos_ned.covariance[35];	// vz
		// TODO: orientation + angular velocity covariances from ATTITUDE_QUATERION_COV

		// publish odom always
		local_odom.publish(odom);

		// publish pose_cov always
		auto pose_cov = boost::make_shared<geometry_msgs::PoseWithCovarianceStamped>();
		pose_cov->header = odom->header;
		pose_cov->pose = odom->pose;
		local_position_cov.publish(pose_cov);

		// publish velocity_cov always
		auto twist_cov = boost::make_shared<geometry_msgs::TwistWithCovarianceStamped>();
		twist_cov->header.stamp = odom->header.stamp;
		twist_cov->header.frame_id = odom->child_frame_id;
		twist_cov->twist = odom->twist;
		local_velocity_cov.publish(twist_cov);

		// publish pose, velocity, tf if we don't have LOCAL_POSITION_NED
		if (!has_local_position_ned) {
			auto pose = boost::make_shared<geometry_msgs::PoseStamped>();
			pose->header = odom->header;
			pose->pose = odom->pose.pose;
			local_position.publish(pose);

			auto twist = boost::make_shared<geometry_msgs::TwistStamped>();
			twist->header.stamp = odom->header.stamp;
			twist->header.frame_id = odom->child_frame_id;
			twist->twist = odom->twist.twist;
			local_velocity_body.publish(twist);

			// publish tf
			publish_tf(odom);
		}

		// publish accelerations
		auto accel = boost::make_shared<geometry_msgs::AccelWithCovarianceStamped>();
		accel->header = odom->header;

		// 特殊转换方案：x和z取反、y不变
		Eigen::Vector3d enu_accel(-pos_ned.ax, pos_ned.ay, -pos_ned.az);
		tf::vectorEigenToMsg(enu_accel, accel->accel.accel.linear);

		accel->accel.covariance[0] = pos_ned.covariance[39];	// ax
		accel->accel.covariance[7] = pos_ned.covariance[42];	// ay
		accel->accel.covariance[14] = pos_ned.covariance[44];	// az

		local_accel.publish(accel);
	}

	void handle_odometry(const mavlink::mavlink_message_t *msg, mavlink::common::msg::ODOMETRY &odom_msg)
	{
		// 如果禁用了处理ODOMETRY消息，则立即返回
		if (!enable_odometry_handler)
			return;
			
		has_odometry = true;

		// 创建ROS Odometry消息并设置基本信息
		auto odom = boost::make_shared<nav_msgs::Odometry>();
		odom->header = m_uas->synchronized_header(frame_id, odom_msg.time_usec);
		odom->child_frame_id = tf_child_frame_id;
		
		// NED到ENU坐标系转换 - 使用特殊转换方案：x和z取反、y不变
		// 原来的标准转换: auto enu_position = ftf::transform_frame_ned_enu(Eigen::Vector3d(odom_msg.x, odom_msg.y, odom_msg.z));
		Eigen::Vector3d enu_position(-odom_msg.x, odom_msg.y, -odom_msg.z);
		Eigen::Vector3d enu_velocity(-odom_msg.vx, odom_msg.vy, -odom_msg.vz);
		Eigen::Vector3d enu_angular_velocity(-odom_msg.rollspeed, odom_msg.pitchspeed, -odom_msg.yawspeed);
		
		// 填充位置和速度数据
		tf::pointEigenToMsg(enu_position, odom->pose.pose.position);
		tf::vectorEigenToMsg(enu_velocity, odom->twist.twist.linear);
		tf::vectorEigenToMsg(enu_angular_velocity, odom->twist.twist.angular);
		
		// 转换四元数：NED -> 特殊ENU (x和z取反、y不变)
		Eigen::Quaterniond q(odom_msg.q[0], odom_msg.q[1], odom_msg.q[2], odom_msg.q[3]);
		// 确保四元数是单位四元数
		if (std::abs(q.norm() - 1.0) > 1e-3) {
			ROS_WARN_THROTTLE(1.0, "接收到的四元数不是单位四元数，已进行归一化");
			q.normalize();
		}
		
		// 不使用标准转换：q = ftf::transform_orientation_ned_enu(q);
		// 自定义四元数转换 - x和z轴翻转相当于绕y轴旋转180度
		Eigen::Quaterniond q_rotation = Eigen::Quaterniond(0, 0, 1, 0); // 绕y轴旋转180度的四元数
		Eigen::Quaterniond q_enu = q_rotation * q * q_rotation;
		
		tf::quaternionEigenToMsg(q_enu, odom->pose.pose.orientation);
		
		// 高效转换协方差 - 位置和姿态
		fill_covariance_matrix(odom_msg.pose_covariance, odom->pose.covariance);
		
		// 高效转换协方差 - 线速度和角速度
		fill_covariance_matrix(odom_msg.velocity_covariance, odom->twist.covariance);
		
		// 发布外部里程计消息
		external_odom.publish(odom);
		
		// 如果启用了ODOMETRY消息作为主要来源，且其他来源不可用
		if (use_odometry && !has_local_position_ned_cov && !has_local_position_ned) {
			// 发布到主要的里程计话题
			local_odom.publish(odom);
			
			// 发布姿态和速度
			publish_pose_and_velocity(odom);
			
			// 发布TF变换
			publish_tf(odom);
		}
	}
	
	// 高效填充协方差矩阵
	void fill_covariance_matrix(const std::array<float, 21> &covariance_in, boost::array<double, 36> &covariance_out)
	{
		// MAVLINK ODOMETRY协方差是21个浮点数的下三角矩阵
		// ROS Odometry协方差是6x6的全矩阵
		for (int i = 0; i < 6; i++) {
			for (int j = 0; j <= i; j++) {
				// 计算下三角矩阵中的索引
				int k = i * (i + 1) / 2 + j;
				if (k < 21) { // 确保索引有效
					double value = static_cast<double>(covariance_in[k]);
					// 填充对称位置
					covariance_out[i * 6 + j] = value;
					covariance_out[j * 6 + i] = value;
				}
			}
		}
	}
	
	// 发布姿态和速度信息
	void publish_pose_and_velocity(const nav_msgs::OdometryConstPtr &odom)
	{
		// 发布姿态
		auto pose = boost::make_shared<geometry_msgs::PoseStamped>();
		pose->header = odom->header;
		pose->pose = odom->pose.pose;
		local_position.publish(pose);
		
		// 发布速度
		auto twist = boost::make_shared<geometry_msgs::TwistStamped>();
		twist->header = odom->header;
		twist->header.frame_id = odom->child_frame_id;
		twist->twist = odom->twist.twist;
		local_velocity_body.publish(twist);
	}
};
}	// namespace std_plugins
}	// namespace mavros

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(mavros::std_plugins::LocalPositionPlugin, mavros::plugin::PluginBase)
