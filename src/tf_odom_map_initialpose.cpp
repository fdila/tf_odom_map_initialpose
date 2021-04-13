#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include <nav_msgs/Odometry.h>
#include <tf/transform_datatypes.h>



geometry_msgs::TransformStamped odom_trans;
tf::Transform firstodom_trans;
tf::Transform intialpose_trans;

double first_x;
double first_y;
double first_th;

int first_odom_flag = 1;
nav_msgs::Odometry odometry;

void initialposeCallback(const geometry_msgs::PoseWithCovarianceStamped msg)
{
  
  float init_x;
  float init_y;
  float init_th;
  
  
  init_th = tf::getYaw(msg.pose.pose.orientation);
  init_x = msg.pose.pose.position.x;
  init_y = msg.pose.pose.position.y;
  
  intialpose_trans.setOrigin(tf::Vector3(init_x, init_y, 0.0));
  
  tf::Quaternion init_quat = tf::createQuaternionFromYaw(init_th).normalize();
  intialpose_trans.setRotation(init_quat);
  
  intialpose_trans = intialpose_trans.inverse();
  
  firstodom_trans.setOrigin(tf::Vector3(first_x, first_y, 0.0));
  tf::Quaternion first_quat = tf::createQuaternionFromYaw(first_th).normalize();
  firstodom_trans.setRotation(first_quat);
  
  
}

void odometryCallback(const nav_msgs::Odometry msg)
{
  if(first_odom_flag){
    first_x = msg.pose.pose.position.x;
    first_y = msg.pose.pose.position.y;
    first_th = tf::getYaw(msg.pose.pose.orientation);
    first_odom_flag = 0; 
  }
  
  odometry = msg;
  odometry.pose.pose.position.x = msg.pose.pose.position.x;
  odometry.pose.pose.position.y = msg.pose.pose.position.y;
  
  double received_th = tf::getYaw(odometry.pose.pose.orientation);
  
  double real_th = received_th;
  tf::Quaternion odom_quat = tf::createQuaternionFromYaw(real_th);
  geometry_msgs::Quaternion odom_quat_geo;
  tf::quaternionTFToMsg(odom_quat, odom_quat_geo); 
  odometry.pose.pose.orientation = odom_quat_geo;
  
  odometry.header.frame_id = "odom_noamcl";
  odometry.child_frame_id = "base_link_noamcl";
  
}

int main(int argc, char **argv)
{
 
  ros::init(argc, argv, "tf_odom_map_initialpose");
  ros::NodeHandle n;
  ros::Subscriber initial_pose_sub = n.subscribe("/initialpose", 1000, initialposeCallback);
  ros::Subscriber odometry_sub = n.subscribe("/odom_ackermann", 1000, odometryCallback);
  ros::Publisher odom_pub = n.advertise<nav_msgs::Odometry>("odometry_no_amcl", 1000);
  odometry.header.frame_id = "odom_noamcl";
  odometry.child_frame_id = "base_link_noamcl";
 

  intialpose_trans.setOrigin(tf::Vector3(0.0, 0.0, 0.0)); 
  tf::Quaternion zero_quat = tf::createQuaternionFromYaw(0.0).normalize();
  intialpose_trans.setRotation(zero_quat);
  firstodom_trans.setRotation(zero_quat);
  
  
  geometry_msgs::Quaternion odom_quat_geo;
  tf::quaternionTFToMsg(zero_quat, odom_quat_geo);
  odom_trans.transform.rotation = odom_quat_geo;
  
  ros::Rate loop_rate(30);
  
  

  static tf::TransformBroadcaster br;
  
  while(n.ok()){
    //send transform init_pose -> map
    br.sendTransform(tf::StampedTransform(intialpose_trans, ros::Time::now(), "/init_pose", "/map"));
    
    //send transform odom_noamcl -> init_pose
    br.sendTransform(tf::StampedTransform(firstodom_trans, ros::Time::now(), "/odom_noamcl", "/init_pose"));

    //send odom transform
    geometry_msgs::TransformStamped odom_trans;
    odom_trans.header.stamp = ros::Time::now();
    odom_trans.header.frame_id = "odom_noamcl";
    odom_trans.child_frame_id = "base_link_noamcl";

    odom_trans.transform.translation.x = odometry.pose.pose.position.x;
    odom_trans.transform.translation.y = odometry.pose.pose.position.y;
    odom_trans.transform.translation.z = 0.0;
    odom_trans.transform.rotation = odometry.pose.pose.orientation;
    
    br.sendTransform(odom_trans);

    //next, we'll publish the odometry message over ROS
    odometry.header.stamp = ros::Time::now();
    odom_pub.publish(odometry);
    loop_rate.sleep();
    ros::spinOnce();
  }
  


  return 0;
}


