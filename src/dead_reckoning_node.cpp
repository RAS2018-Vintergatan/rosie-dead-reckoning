#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <phidgets/motor_encoder.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Imu.h>

double offsetIn = 0; // In direction of driving
double offsetCr = 0; // Cross direction of driving

double wlR = 1.0; //wheel left radius
double wrR = 1.0;

double wheelSeparation = 1.0;

double errorSumLeft = 0;
double errorSumRight = 0;

double ang_x = 0.0;
double ang_y = 0.0;
double ang_z = 0.0;
double lin_x = 0.0;
double lin_y = 0.0;
double lin_z = 0.0;
double orient_w = 0.0;
double orient_x = 0.0;
double orient_y = 0.0;
double orient_z = 0.0;

void encoderLeftCallback(const phidgets::motor_encoder& msg){
    ROS_INFO("-----------");
    ROS_INFO("LEFT:: E1: %d, E2: %d", msg.count,msg.count_change);

    double est_w = ((double)msg.count_change * 10.0 * 2.0 * 3.1415)/(360.0);


    ROS_INFO("-----------");
}

void encoderRightCallback(const phidgets::motor_encoder& msg){
    ROS_INFO("-----------");


    double est_w = ((double)msg.count_change * 10.0 * 2.0 * 3.1415)/(360.0);


    ROS_INFO("-----------");
}

void imuCallback(const sensor_msgs::Imu& msg){

    ang_x = msg.angular_velocity.x;
    ang_y = msg.angular_velocity.y;
    ang_z = msg.angular_velocity.z;
    lin_x = msg.linear_acceleration.x;
    lin_y = msg.linear_acceleration.y;
    lin_z = msg.linear_acceleration.z;
    orient_w = msg.orientation.w;
    orient_x = msg.orientation.x;
    orient_y = msg.orientation.y;
    orient_z = msg.orientation.z;

  //  ROS_INFO("LinX: %f, AngX: %f",
  //  lin_x, ang_x);

}

int main(int argc, char **argv){
    ros::init(argc, argv, "rosie_dead_reckoning");

    ros::NodeHandle n;

    ros::Subscriber encoderLeft_sub = n.subscribe("/motorLeft/encoder", 1, encoderLeftCallback); //double
    ros::Subscriber encoderRight_sub = n.subscribe("/motorRight/encoder", 1, encoderRightCallback); //double
    ros::Subscriber imu_sub = n.subscribe("/imu/data", 1, imuCallback); //float;

    ros::Rate loop_rate(10);

    while(ros::ok()){

        ros::spinOnce();
        loop_rate.sleep();

    }
}
