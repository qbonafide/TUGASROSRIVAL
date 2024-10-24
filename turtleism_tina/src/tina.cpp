#include "ros/ros.h"
#include "turtlesim/SetPen.h"
#include "geometry_msgs/Twist.h"
#include "turtlesim/Pose.h"
#include <cmath>

using namespace std;

ros::Publisher velocity_publisher;
ros::Subscriber pose_subscriber;
turtlesim::Pose turtlesim_pose;

const double Pi = 3.14159265359;
const double linear_speed = 2.0;

void setPen(bool is_down, int width = 3, int r = 255, int g = 20, int b = 147) {
    turtlesim::SetPen srv;
    srv.request.width = width;   
    srv.request.r = r;          
    srv.request.g = g;         
    srv.request.b = b; 

    srv.request.off = !is_down;
    if (!ros::service::call("/turtle1/set_pen", srv)) {
        ROS_ERROR("Failed to call service set_pen");
    }
}

double degrees2radians(double angle_in_degrees) {
    return angle_in_degrees * Pi / 180.0;
}

void move(double speed, double distance, bool isForward) {
    geometry_msgs::Twist vel_msg;
    vel_msg.linear.x = isForward ? speed : -speed;
    vel_msg.angular.z = 0;

    double t0 = ros::Time::now().toSec();
    double current_distance = 0.0;
    ros::Rate loop_rate(10);

    do {
        velocity_publisher.publish(vel_msg);
        double t1 = ros::Time::now().toSec();
        current_distance = speed * (t1 - t0);
        ros::spinOnce();
        loop_rate.sleep();
    } while (current_distance < distance);

    vel_msg.linear.x = 0;
    velocity_publisher.publish(vel_msg);
}

void rotate(double angle_in_degrees, bool clockwise) {
    geometry_msgs::Twist vel_msg;
    double angular_speed = degrees2radians(60);

    vel_msg.angular.z = clockwise ? -fabs(angular_speed) : fabs(angular_speed);

    double angle_in_radians = degrees2radians(angle_in_degrees);
    double t0 = ros::Time::now().toSec();
    double current_angle = 0.0;
    ros::Rate loop_rate(100);

    do {
        velocity_publisher.publish(vel_msg);
        double t1 = ros::Time::now().toSec();
        current_angle = angular_speed * (t1 - t0);
        ros::spinOnce();
        loop_rate.sleep();
    } while (current_angle < angle_in_radians);

    vel_msg.angular.z = 0;
    velocity_publisher.publish(vel_msg);
}

void drawT() {
    setPen(true);  

    move(linear_speed, 2.0, true);  
    rotate(180, true);
    move(linear_speed, 2.0, true);  
    rotate(180, true);
    move(linear_speed, 2.0, true);  
    rotate(180, true);
    move(linear_speed, 1.0, true); 
    rotate(90, true);  
    move(linear_speed, 2.0, true);

}

void drawI() {   
    setPen(true);

    move(linear_speed, 2.0, true);
    rotate(180, true); 
    move(linear_speed, 2.0, true); 
    
}

void drawN() {

    setPen(true); 

    move(linear_speed, 2.0, true); 
    rotate(135, true);
    move(linear_speed, 2.5, true); 
    rotate(135, false);
    move(linear_speed, 2.0, true);
    rotate(180, true);
    move(linear_speed, 2.0, true); 

}

void drawA() {
    setPen(true);

    rotate(30, true);
    move(linear_speed, 2.0, true);
    rotate(120, true); 
    move(linear_speed, 2.0, true); 
    rotate(180, true);
    move(linear_speed, 1.0, true); 
    rotate(60, false); 
    move(linear_speed, 1.0, true); 

    setPen(false);
}

void executeDrawing() {
    
    setPen(false);
    rotate(180, false);
    move(linear_speed, 3.5, true);
    drawT();

    setPen(false);
    rotate(90, false);
    move(linear_speed, 1.5, true);
    rotate(90, false);
    drawI();

    setPen(false);
    rotate(90, false);
    move(linear_speed, 0.5, true);
    rotate(90, false);
    drawN();

    setPen(false);
    rotate(90, false);
    move(linear_speed, 0.5, true);
    rotate(90, false);
    drawA();

}

void poseCallback(const turtlesim::Pose::ConstPtr &pose_message) {
    turtlesim_pose.x = pose_message->x;
    turtlesim_pose.y = pose_message->y;
    turtlesim_pose.theta = pose_message->theta;
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "turtlesim_draw_name");
    ros::NodeHandle n;

    velocity_publisher = n.advertise<geometry_msgs::Twist>("/turtle1/cmd_vel", 10);
    pose_subscriber = n.subscribe("/turtle1/pose", 10, poseCallback);

    ros::Rate loop_rate(0.5);
    ros::Duration(2).sleep();

    executeDrawing();

    ros::spin();
    return 0;
}
