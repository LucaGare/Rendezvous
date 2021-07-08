/*
Forward the odometry message from a motion capture
system, renaming the frames so that it can be used
by the mavros odom module.
*/

#include <string>
#include <math.h>

#include <ros/time.h>
#include <ros/ros.h>
#include <ros/console.h>

#include <nav_msgs/Odometry.h>

ros::Publisher mavros_odom_pub;
nav_msgs::Odometry mavros_odom_msg;

void odom_callback(const nav_msgs::OdometryConstPtr& msg){
  mavros_odom_msg.pose = msg->pose;
  mavros_odom_msg.twist = msg->twist;
}

void publish()
{
  static unsigned int sequenceNumber = 0;

  mavros_odom_msg.header.seq = sequenceNumber;
  mavros_odom_msg.header.stamp = ros::Time::now();
  mavros_odom_pub.publish(mavros_odom_msg);
  
  sequenceNumber++;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "mocap_odom_to_fcu");
  ros::NodeHandle nh;

  //Run at 100hz
  ros::Rate loop_rate(100);

  mavros_odom_msg.header.frame_id = "odom";
  mavros_odom_msg.child_frame_id = "base_link";

  ros::TransportHints transportHints;
  transportHints.tcpNoDelay();

  ros::Subscriber odom_sub = nh.subscribe("mocap_odom", 2, odom_callback, transportHints);
  mavros_odom_pub = nh.advertise<nav_msgs::Odometry>("mavros/odometry/out", 10);

  while(ros::ok()) {
    publish();
    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}
