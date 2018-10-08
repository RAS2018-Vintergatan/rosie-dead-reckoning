#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <phidgets/motor_encoder.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose2D.h>
#include <sensor_msgs/Imu.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>

double offsetIn = 0; // In direction of driving
double offsetCr = 0; // Cross direction of driving

double wlR = 1.0; //wheel left radius
double wrR = 1.0;

double wheelSeparation = 1.0;

double errorSumLeft = 0;
double errorSumRight = 0;

double est_w_left = 0;
double est_w_right = 0;

double pose_x = 0;
double pose_y = 0;
double pose_theta = 0;

double est_v_right = 0;
double est_v_left = 0;

double dt_left = 0;
double dt_right = 0;


ros::Time t_last_l;
ros::Time t_last_r;

ros::Publisher imu_pub;

void encoderLeftCallback(const phidgets::motor_encoder& msg){
    //ROS_INFO("-----------");
    //ROS_INFO("LEFT:: E1: %d, E2: %d", msg.count,msg.count_change);

    ros::Time t_now = msg.header.stamp;
    dt_left = (t_now-t_last_l).toSec();
    est_w_left = ((double)msg.count_change* 10.0 * 2.0 * 3.1415)/(360.0);
    est_v_right = est_w_left/wlR;

    //ROS_INFO("-----------");

    t_last_l = t_now;
}

void encoderRightCallback(const phidgets::motor_encoder& msg){
    //ROS_INFO("-----------");
    //ROS_INFO("LEFT:: E1: %d, E2: %d", msg.count,msg.count_change);

    ros::Time t_now = msg.header.stamp;
    dt_right = (t_now-t_last_r).toSec();

    est_w_right = ((double)msg.count_change * 10.0 * 2.0 * 3.1415)/(360.0);
    est_v_left = est_w_right/wrR;

    //ROS_INFO("-----------");

    t_last_r = t_now;
}

void imuCallback(const sensor_msgs::Imu& msg){
    imu_pub.publish(msg);
}

int main(int argc, char **argv){
    ros::init(argc, argv, "rosie_reckoning_ekf");

    ros::NodeHandle n;

    n.getParam("wheel_left_radius", wlR);
    n.getParam("wheel_right_radius", wrR);
    n.getParam("wheel_separation", wheelSeparation);

    ros::Subscriber encoderLeft_sub = n.subscribe("/motorLeft/encoder", 1, encoderLeftCallback); //double
    ros::Subscriber encoderRight_sub = n.subscribe("/motorRight/encoder", 1, encoderRightCallback); //double
    ros::Subscriber imu_sub = n.subscribe("/imu/data", 1, imuCallback); 
    ros::Publisher odom_pub = n.advertise<nav_msgs::Odometry>("/odom",1);
    imu_pub = n.advertise<sensor_msgs::Imu>("/imu_data",1);
    
    nav_msgs::Odometry odom;
	
    odom.pose.pose.position.x = 0;
    odom.pose.pose.position.y = 0;
    odom.pose.pose.orientation.z = 0;

    odom.twist.twist.linear.x = 0;
    odom.twist.twist.linear.y = 0;
    odom.twist.twist.linear.z = 0;
    odom.twist.twist.angular.x = 0;
    odom.twist.twist.angular.y = 0;
    odom.twist.twist.angular.z = 0;
	
    ros::Rate loop_rate(10);

    static tf::TransformBroadcaster br;
    tf::Transform transform;

    while(ros::ok()){

        transform.setOrigin( tf::Vector3(0.0, 0.0, 0.0) );
        transform.setRotation( tf::Quaternion(0, 0, 0) );
        br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "odom", "wheelodom"));

	// orientation
	double dist_l = est_v_left*dt_left;
	double dist_r = est_v_right*dt_right;
	double abs_wheel_dist = abs(dist_l-dist_r);
	double phi = abs_wheel_dist/wheelSeparation;

	// distance
	double center_dist = (dist_l+dist_r)/2;	
	
	

	// odometry
	odom.header.stamp = ros::Time::now();
	odom.header.frame_id = "wheelodom";

	odom.pose.pose.orientation.z += phi;
	odom.pose.pose.position.x += center_dist*cos(odom.pose.pose.orientation.z);
	odom.pose.pose.position.y += center_dist*sin(odom.pose.pose.orientation.z);
	float sF = 0.000001;
	odom.pose.covariance = {sF,0,0,0,0,0,
				0,sF,0,0,0,0,
				0,0,sF,0,0,0,
				0,0,0,sF,0,0,
				0,0,0,0,sF,0,
				0,0,0,0,0,sF};

        odom.twist.twist.linear.x = (est_v_left+est_v_right)/2;
        odom.twist.twist.angular.z = (est_v_left-est_v_right)/((double) 2*wheelSeparation);
	odom.twist.covariance = {sF,0,0,0,0,0,
				0,0,0,0,0,0,
				0,0,0,0,0,0,
				0,0,0,0,0,0,
				0,0,0,0,0,0,
				0,0,0,0,0,sF};

 	//ROS_INFO("x: %f, y: %f, theta: %f", pose.x, pose.y, pose.theta);
	odom_pub.publish(odom);
        ros::spinOnce();
        loop_rate.sleep();

    }
}
