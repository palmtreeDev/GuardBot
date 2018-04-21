#include "ros/ros.h"
#include <nodelet/nodelet.h>
#include <sensor_msgs/Image.h>
#include <depth_image_proc/depth_traits.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/QuaternionStamped.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>


ros::Publisher cmd_vel_pub;
ros::Subscriber depth_sub;
ros::Subscriber landmark_sub;

tf::TransformListener listener_;

bool follow_state = true, tracking_home=false;
double yaw_angle,cur_x,cur_y,dist;
double PI = 3.14159265;

int landmark_depths[6];
float landmark_bearings[6];

// ROS_INFO();


// for when theta exceeds 3.14 or -3.14
double recast_angle(double theta){
	if(theta < -PI){
		return 2*PI+theta;
	} else if(theta > PI){
		return theta-2*PI;
	} else {
		return theta;
	}
}

// for deciding the direction must turn
double sign(double d){
	if(d > 0){
		return 1;
	} else if(d < 0){
		return -1;
	} else {
		return 0;
	}
}

// for the follow state and the return state of the robot
void DepthCallback(const nav_msgs::Odometry depth_msg){
	int depth = depth_msg.twist.twist.linear.x; // distance component of depth_msg in mm
	float turn_angle = depth_msg.twist.twist.angular.z;

	geometry_msgs::Twist base_cmd;
	//
	base_cmd.linear.x = 0;
	base_cmd.linear.y = 0;
	base_cmd.angular.z = 0;

	if(depth > 500 && turn != 0){

		base_cmd.linear.x = 0.15;
		if(turn_angle > -0.05 && turn_angle < 0.05){
			base_cmd.angular.z = 0;
		} else if(turn_angle > 0) {
			if(turn_angle > 0.25){
				base_cmd.angular.z = 2*sqrt(turn_angle);
			} else {
				base_cmd.angular.z = 2*turn_angle;
			}
		} else {
			if(turn_angle < -0.25){
				base_cmd.angular.z = -2*sqrt(-turn_angle);
			} else {
				base_cmd.angular.z = 2*turn_angle;
			}
		}
		base_cmd.linear.x = 0.15;

	} else {
		base_cmd.linear.x = 0;
		base_cmd.angular.z = 0;
	}

	// when in return state
	if(!follow_state) {
		double home_angle = atan2(cur_y,cur_x);
		if(home_angle<=0){
			home_angle = PI + home_angle;
		} else {
			home_angle = home_angle - PI;
		}

		double angle_diff = recast_angle(home_angle - yaw_angle);
		double angle_sign = sign(recast_angle(home_angle-yaw_angle));

		// rotate if we aren't facing the home destination yet
		if(!tracking_home) {
			base_cmd.linear.x = 0;
			base_cmd.linear.y = 0;
			base_cmd.angular.z = angle_sign*0.4;

			// if we are facing the home destination, stop rotating
			if(sign(angle_diff)*angle_diff < 0.05){
				tracking_home = true;
			}
	// turn to correct for error
	} else {
			if(sign(angle_diff)*angle_diff < 0.025){ // no correction for small angle error
				base_cmd.linear.x = 0.2;
				base_cmd.linear.y = 0;
				base_cmd.angular.z = 0;
			} else { // correct for larger angle error
				base_cmd.linear.x = 0.2;
				base_cmd.linear.y = 0;
				base_cmd.angular.z = angle_sign*0.2;
			}
		}
		// stop
		if(dist < 0.05) {
			base_cmd.linear.x = 0;
			base_cmd.linear.y = 0;
			base_cmd.angular.z = 0;
		}
	}

	cmd_vel_pub.publish(base_cmd);
}

void LandmarkCallback(double turn_angle) {

	if(turn_angle > -0.05 && turn_angle < 0.05){
		base_cmd.angular.z = 0; // stop turning
	} else if(turn_angle > 0) {
		if(turn_angle > 0.25){
			// small turn rate
			base_cmd.angular.z = 2*sqrt(turn_angle);
		} else { // large turn rate
			base_cmd.angular.z = 2*turn_angle;
	} else {
		if(turn_angle < -0.25){
			base_cmd.angular.z = -2*sqrt(-turn_angle);
		} else {
			base_cmd.angular.z = 2*turn_angle;
		}
	}
	base_cmd.linear.x = 0.15;
}

bool turnOdom(bool clockwise, double radians){
  while(radians < 0) radians += 2*M_PI;
  while(radians > 2*M_PI) radians -= 2*M_PI;

  //wait for the listener to get the first message
  listener_.waitForTransform("base_footprint", "odom_combined",
                             ros::Time(0), ros::Duration(1.0));

  //we will record transforms here
  tf::StampedTransform start_transform;
  tf::StampedTransform current_transform;

  //record the starting transform from the odometry to the base frame
  listener_.lookupTransform("base_footprint", "odom_combined",
                            ros::Time(0), start_transform);

  //we will be sending commands of type "twist"
  geometry_msgs::Twist base_cmd;
  //the command will be to turn at 0.75 rad/s
  base_cmd.linear.x = base_cmd.linear.y = 0.0;
  base_cmd.angular.z = 0.75;
  if (clockwise) base_cmd.angular.z = -base_cmd.angular.z;

  //the axis we want to be rotating by
  tf::Vector3 desired_turn_axis(0,0,1);
  if (!clockwise) desired_turn_axis = -desired_turn_axis;

  ros::Rate rate(10.0);
  bool done = false;
  while (!done && nh_.ok())
  {
    //send the drive command
    cmd_vel_pub.publish(base_cmd);
    rate.sleep();
    //get the current transform
    try
    {
      listener_.lookupTransform("base_footprint", "odom_combined",
                                ros::Time(0), current_transform);
    }
    catch (tf::TransformException ex)
    {
      ROS_ERROR("%s",ex.what());
      break;
    }
    tf::Transform relative_transform =
      start_transform.inverse() * current_transform;
    tf::Vector3 actual_turn_axis =
      relative_transform.getRotation().getAxis();
    double angle_turned = relative_transform.getRotation().getAngle();
    if ( fabs(angle_turned) < 1.0e-2) continue;

    if ( actual_turn_axis.dot( desired_turn_axis ) < 0 )
      angle_turned = 2 * M_PI - angle_turned;

    if (angle_turned > radians) done = true;
  }
  if (done) return true;
  return false;
}




// finds distance that the robot has traveled from odom data and detects when the robot travels more than 2m
void chatterCallback(const nav_msgs::Odometry::ConstPtr& msg) {
	tf::Pose pose;
	tf::poseMsgToTF(msg->pose.pose, pose);
	yaw_angle = tf::getYaw(pose.getRotation());
	cur_x = msg->pose.pose.position.x;
	cur_y = msg->pose.pose.position.y;
	dist = sqrt(msg->pose.pose.position.x*msg->pose.pose.position.x + msg->pose.pose.position.y*msg->pose.pose.position.y);
	if(dist > 2){
		follow_state = false;
	}
	double dd = atan2(cur_y,cur_x);
}

int main(int argc, char** argv){
	ros::init(argc,argv, "depth_follower_node");
	ros::NodeHandle nh;
	cmd_vel_pub = nh.advertise<geometry_msgs::Twist>("/mobile_base/commands/velocity", 1);
	depth_sub = nh.subscribe("Depth", 1, DepthCallback);
	ros::Subscriber sub = nh.subscribe("odom", 1000, chatterCallback);

	ros::spin();
	return 0;
}
