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
double wrR = 1.0;

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

double dt_left = 0.0;
double dt_right = 0.0;

ros::Time t_last_l;
ros::Time t_last_r;

ros::Publisher imu_pub;
ros::Publisher odom_pub;

void encoderLeftCallback(const phidgets::motor_encoder& msg){
    //ROS_INFO("-----------");
    //ROS_INFO("LEFT:: E1: %d, E2: %d", msg.count,msg.count_change);

    ros::Time t_now = msg.header.stamp;
    dt_left = (t_now-t_last_l).toSec();
    est_w_left = -((double)msg.count_change * dt_left * 2.0 * 3.1415)/(360.0);
    est_v_left = est_w_left/wlR;

    //ROS_INFO("-----------");

    t_last_l = t_now;
}

void encoderRightCallback(const phidgets::motor_encoder& msg){
    //ROS_INFO("-----------");
    //ROS_INFO("LEFT:: E1: %d, E2: %d", msg.count,msg.count_change);

    ros::Time t_now = msg.header.stamp;
    dt_right = (t_now-t_last_r).toSec();

    est_w_right = ((double)msg.count_change * dt_right * 2.0 * 3.1415)/(360.0);
    est_v_right = est_w_right/wrR;

    //ROS_INFO("-----------");

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
    br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", "base_line"));
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
	double velLinear = (est_v_left+est_v_right)/2;
	double velAngular = (est_v_left-est_v_right)/((double) 2*wheelSeparation);

    //Odometry
    lastOdom.header.stamp = current_odom_time;
    lastOdom.header.frame_id = "world";
    lastOdom.child_frame_id = "base_line";

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
    br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "base_line", "wheelodom"));

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
	
    /*odom.pose.pose.position.x = 0;
    odom.pose.pose.position.y = 0;
    odom.pose.pose.orientation.z = 0;

	odom.twist.twist.linear.x = 0;
	odom.twist.twist.linear.y = 0;
	odom.twist.twist.linear.z = 0;
	odom.twist.twist.angular.x = 0;
	odom.twist.twist.angular.y = 0;
	odom.twist.twist.angular.z = 0;
	*/
    ros::Rate loop_rate(10);

 
    //tf::TransformBroadcaster br;

	last_odom_time = ros::Time::now();
    while(ros::ok()){
		updateOdom();
/*
        transform.setOrigin( tf::Vector3(0.0, 0.0, 0.0) );
        transform.setRotation( tf::Quaternion(0, 0, 0) );
        br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "odom", "wheelodom"));

	// orientation
	double dist_l = est_v_left*dt_left;
	double dist_r = est_v_right*dt_right;
	double abs_wheel_dist = abs(dist_l-dist_r);
	double phi = abs_wheel_dist/(double)wheelSeparation;

	// distance
	double center_dist = (dist_l+dist_r)/2;	
	
	//velocities
	double velx = (est_v_left+est_v_right)/2;
	double veltheta = (est_v_left-est_v_right)/((double) 2*wheelSeparation);

	//pose
	pose_theta += phi;
	pose_x += velx*cos(pose_theta);
	pose_y += velx*sin(pose_theta);
	pose_z += 0.0;

*/




 	//ROS_INFO("x: %f, y: %f, theta: %f", pose.x, pose.y, pose.theta);
/*
	odomCallback(odom);
	odom_pub.publish(odom);
*/
        ros::spinOnce();
        loop_rate.sleep();

    }
}
