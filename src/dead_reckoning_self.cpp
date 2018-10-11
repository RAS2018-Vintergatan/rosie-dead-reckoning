#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <phidgets/motor_encoder.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose2D.h>
#include <sensor_msgs/Imu.h>
#include <nav_msgs/Odometry.h>
#include <math.h>
#include <tf/transform_broadcaster.h>

double offsetIn = 0; // In direction of driving
double offsetCr = 0; // Cross direction of driving

double wlR = 1.0; //wheel left radius
double wrR = 1.0; //wheel right radius

double wheelSeparation = 1.0;

double errorSumLeft = 0.0;
double errorSumRight = 00.0;

double est_w_left = 0.0;
double est_w_right = 0.0;

double pose_x = 0.0;
double pose_y = 0.0;
double pose_z = 0.0;
double pose_theta = 0.0;

double est_v_right = 0.0;
double est_v_left = 0.0;

ros::Time t_last_l;
ros::Time t_last_r;

ros::Publisher imu_pub;
ros::Publisher odom_pub;

void encoderLeftCallback(const phidgets::motor_encoder& msg){
    ros::Time t_now = msg.header.stamp;
    double dt = (t_now-t_last_l).toSec();
    
    est_w_left = ((double)msg.count_change * dt * 2.0 * 3.1415)/(360.0);
    est_v_left = -est_w_left/wlR;

    t_last_l = t_now;
}

void encoderRightCallback(const phidgets::motor_encoder& msg){
    ros::Time t_now = msg.header.stamp;
    double dt = (t_now-t_last_r).toSec();
    
    est_w_right = ((double)msg.count_change * dt * 2.0 * 3.1415)/(360.0);
    est_v_right = est_w_right/wrR;

    t_last_r = t_now;
}

void odomCallback(const nav_msgs::Odometry& odom){
    static tf::TransformBroadcaster br;
	tf::Transform transform;
    transform.setOrigin( tf::Vector3(odom.pose.pose.position.x, odom.pose.pose.position.y, odom.pose.pose.position.z) );
    tf::Quaternion q;
    q[0] = odom.pose.pose.orientation.x;
	q[1] = odom.pose.pose.orientation.y;
	q[2] = odom.pose.pose.orientation.z;
	q[3] = odom.pose.pose.orientation.w;
    transform.setRotation( q );
    br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", "rosie"));
}

void imuCallback(const sensor_msgs::Imu& msg){
    imu_pub.publish(msg);
}

ros::Time last_odom_time;
nav_msgs::Odometry lastOdom;

void updateOdom(){
	ros::Time current_odom_time = ros::Time::now();
	float dt = (current_odom_time - last_odom_time).toSec();

	//Velocities
	double velLinear = (est_v_left+est_v_right)/2.0;
	double velAngular = (est_v_right-est_v_left)/(2.0*wheelSeparation);

    //Odometry (Position of rosie in relation to world)
    lastOdom.header.stamp = current_odom_time;
    lastOdom.header.frame_id = "world";
    lastOdom.child_frame_id = "rosie";

	pose_theta += velAngular*dt;
	pose_x += velLinear*cos(pose_theta)*dt;
    pose_y += velLinear*sin(pose_theta)*dt;

    lastOdom.pose.pose.orientation.x = 0;
    lastOdom.pose.pose.orientation.y = 0;
    lastOdom.pose.pose.orientation.z = sin(pose_theta/2);
    lastOdom.pose.pose.orientation.w = cos(pose_theta/2);

    lastOdom.pose.pose.position.x = pose_x;
    lastOdom.pose.pose.position.y = pose_y;

    float sF = 0.000001;
    lastOdom.pose.covariance = {sF,0,0,0,0,0,
    						0,sF,0,0,0,0,
							0,0,sF,0,0,0,
							0,0,0,sF,0,0,
							0,0,0,0,sF,0,
							0,0,0,0,0,sF};

    lastOdom.twist.twist.linear.x = velLinear;
    lastOdom.twist.twist.angular.z = velAngular;
    lastOdom.twist.covariance = {sF,0,0,0,0,0,
							0,0,0,0,0,0,
							0,0,0,0,0,0,
							0,0,0,0,0,0,
							0,0,0,0,0,0,
							0,0,0,0,0,sF};

	odomCallback(lastOdom);
	odom_pub.publish(lastOdom);

	tf::Transform transform;
    static tf::TransformBroadcaster br;
    transform.setOrigin( tf::Vector3(0, 0, 0) );
    tf::Quaternion qtf;
    qtf.setRPY(0, 0, 0);
    transform.setRotation( qtf );
    br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "rosie", "wheelodom"));

	last_odom_time = current_odom_time;
}

int main(int argc, char **argv){
    ros::init(argc, argv, "rosie_reckoning_self");

    ros::NodeHandle n;

    n.getParam("wheel_left_radius", wlR);
    n.getParam("wheel_right_radius", wrR);
    n.getParam("wheel_separation", wheelSeparation);

    ros::Subscriber encoderLeft_sub = n.subscribe("/motorLeft/encoder", 1, encoderLeftCallback); //double
    ros::Subscriber encoderRight_sub = n.subscribe("/motorRight/encoder", 1, encoderRightCallback); //double
    ros::Subscriber imu_sub = n.subscribe("/imu/data", 1, imuCallback); 
    odom_pub = n.advertise<nav_msgs::Odometry>("/odom",1);
    imu_pub = n.advertise<sensor_msgs::Imu>("/imu_data",1);
	
    ros::Rate loop_rate(10);

	last_odom_time = ros::Time::now();
    while(ros::ok()){
		updateOdom();
        ros::spinOnce();
        loop_rate.sleep();
    }
}
