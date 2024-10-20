#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "turtlesim/TeleportAbsolute.h"
#include "turtlesim/SetPen.h"

void setPen(bool is_down) {
    turtlesim::SetPen srv;
    srv.request.off = !is_down; // true = tidak gambar
    if (!ros::service::call("/turtle1/set_pen", srv)) {
        ROS_ERROR("Failed to call service set_pen");
    }
}

void moveTurtle(ros::Publisher pub, double linear, double angular, double duration) {
    geometry_msgs::Twist move_cmd;
    move_cmd.linear.x = linear;
    move_cmd.angular.z = angular;

    ros::Rate rate(10); // 10 Hz
    int ticks = duration * 10;
    for (int i = 0; i < ticks; i++) {
        pub.publish(move_cmd);
        rate.sleep();
    }
}

void drawT(ros::Publisher pub) {
    moveTurtle(pub, 2.0, 0.0, 1.0); 
    moveTurtle(pub, 0.0, 1.57, 0.5); 
    moveTurtle(pub, 1.0, 0.0, 1.0); 
    moveTurtle(pub, 0.0, -1.57, 0.5); 
    moveTurtle(pub, 2.0, 0.0, 1.0); 
}

void drawI(ros::Publisher pub) {
    moveTurtle(pub, 0.0, 1.57, 0.5); 
    moveTurtle(pub, 2.0, 0.0, 1.0); 
}

void drawN(ros::Publisher pub) {
    moveTurtle(pub, 2.0, 0.0, 1.0); 
    moveTurtle(pub, 0.0, 1.57, 0.5); 
    moveTurtle(pub, 2.5, 0.0, 1.5); 
    moveTurtle(pub, 0.0, -1.57, 0.5); 
    moveTurtle(pub, 2.0, 0.0, 1.0); 
}

void drawA(ros::Publisher pub) {
    moveTurtle(pub, 2.0, 1.0, 1.0); 
    moveTurtle(pub, 2.0, -1.0, 1.0); 
    moveTurtle(pub, 0.0, 3.14, 0.5); 
    moveTurtle(pub, 1.0, 0.0, 0.5); 
}

void teleportTurtle(ros::ServiceClient teleport_client, float x, float y, float theta) {
    turtlesim::TeleportAbsolute srv;
    srv.request.x = x;
    srv.request.y = y;
    srv.request.theta = theta;
    teleport_client.call(srv);
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "tina_node");
    ros::NodeHandle nh;

    ros::Publisher pub = nh.advertise<geometry_msgs::Twist>("/turtle1/cmd_vel", 10);

    ros::ServiceClient teleport_client = nh.serviceClient<turtlesim::TeleportAbsolute>("/turtle1/teleport_absolute");

    ros::Rate rate(1);
    
    rate.sleep(); 

    setPen(false);
    teleportTurtle(teleport_client, 2.0, 8.0, 0.0); 
    setPen(true);
    drawT(pub);

    setPen(false);
    teleportTurtle(teleport_client, 5.0, 8.0, 0.0); 
    setPen(true);
    drawI(pub);

    setPen(false);
    teleportTurtle(teleport_client, 7.0, 8.0, 0.0); 
    setPen(true);
    drawN(pub);

    setPen(false);
    teleportTurtle(teleport_client, 10.0, 8.0, 0.0); 
    setPen(true);
    drawA(pub);

    ros::spinOnce();
    return 0;
}