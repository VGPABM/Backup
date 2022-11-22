#include <ros/ros.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/UInt8.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>
#include "../include/control_its/pid.hpp"
#include "../include/control_its/soeroLog.hpp"
#include "../include/control_its/soeroYawUtils.hpp"
#include "../include/control_its/soeroCrypto.hpp"
#include "../../vision_its/include/vision_its/RotatedRect.h"

#include <stdexcept>
#include <string>

#define _CRT_SECURE_NO_WARNINGS

#define DEG_2_RAD 0.01745329251
#define pix_per_meter 0.01875
#define HALF_PI 1.57079632679

//=====================PID SETUP===================================//
double dt = 1/32.0;
double max_val = 1.0;
double min_val = -1.0;
double Kp = 0.45;
double Ki = 0.000;
double Kd = 0.0007;
double theta_max_val=2.0;
double theta_min_val=-2.0;

double camera_width=640;
double camera_height=360;

PID mypid_x(dt, max_val, min_val, Kp, Kd, Ki);
PID mypid_y(dt, max_val*0.5, min_val*0.5, 0.7, 0.001, 0.000);
PID mypid_z(dt, max_val, min_val, Kp+2.5, Kd+0.002, Ki);
PID mypid_z_hrz(dt, max_val+3.0, min_val-3.0, Kp+5.5, Kd+1, Ki+0.01);
PID mypid_yaw(dt, 2.5, -2.5, 2.5, 1.00, 0.00000);
/*----------------------del time, interval val, kp,    kd,    ki*/
PID mypid_z_vision_gate(dt, max_val, min_val, 0.0040, 0.0011, 0);
PID mypid_y_vision_gate(dt, max_val, min_val, 0.0060, 0.0020, 0);
PID mypid_y_hor_gate(dt, max_val, min_val, 0.500, 0.0011, 0);
PID mypid_x_hor_gate(dt, max_val, min_val, 0.500, 0.0021, 0);
PID mypid_y_landing(dt, max_val, min_val, 0.75, 0.0011, 0);
PID mypid_x_landing(dt, max_val, min_val, 0.75, 0.0011, 0);

//=====================CALLBACKs===================================//
geometry_msgs::Pose current_pose;
void pose_cb(const geometry_msgs::Pose::ConstPtr msg){
    current_pose = *msg;
}

vision::RotatedRect upperLine;
void uline_cb(const vision::RotatedRect::ConstPtr msg){
    upperLine = *msg;
}

vision::RotatedRect lowerLine;
void lline_cb(const vision::RotatedRect::ConstPtr msg){
    lowerLine = *msg;
}

vision::RotatedRect landingPad;
void lpad_cb(const vision::RotatedRect::ConstPtr msg){
    landingPad = *msg;
}

vision::RotatedRect verticalGate;
void vgate_cb(const vision::RotatedRect::ConstPtr msg){
    verticalGate = *msg;
}

vision::RotatedRect horizontalGate;
void hgate_cb(const vision::RotatedRect::ConstPtr msg){
    horizontalGate = *msg;
}

vision::RotatedRect horizontalGateHelper;
void hgate_help_cb(const vision::RotatedRect::ConstPtr msg){
    horizontalGateHelper = *msg;
}

std_msgs::Float32MultiArray verticalGateHelper;
void vgate_help1_cb(const std_msgs::Float32MultiArray::ConstPtr msg){
    verticalGateHelper = *msg;
}

std_msgs::UInt8 specialVGateCmd;
void vgate_help2_cb(const std_msgs::UInt8::ConstPtr msg){
    specialVGateCmd = *msg;
}


int main(int argc, char **argv){
    ros::init(argc, argv, "bmctrl");
    ros::NodeHandle nh;
    soero::Log::EnableFileOutput();

    //takeoff api
    ros::Publisher takeoff_pub      = nh.advertise<std_msgs::Empty>      ("/drone/takeoff", 10, true);
    //movement api
    ros::Publisher vel_pub          = nh.advertise<geometry_msgs::Twist> ("/cmd_vel", 10);
    ros::Subscriber pose_sub        = nh.subscribe<geometry_msgs::Pose>  ("/drone/gt_pose", 10, pose_cb);
    ros::Publisher land_pub         = nh.advertise<std_msgs::Empty>      ("/drone/land", 10);
    // vision datas
    ros::Subscriber landingpad_sub  = nh.subscribe<vision::RotatedRect>  ("/landingpad_data", 10, lpad_cb);
    ros::Subscriber upper_line_sub  = nh.subscribe<vision::RotatedRect>  ("/upper/line_data", 10, uline_cb);
    ros::Subscriber lower_line_sub  = nh.subscribe<vision::RotatedRect>  ("/lower/line_data", 10, lline_cb);
    ros::Subscriber vgate_sub       = nh.subscribe<vision::RotatedRect>  ("/gate/vertical/data", 10, vgate_cb);
    ros::Subscriber vgate_help1_sub = nh.subscribe<std_msgs::Float32MultiArray>  ("/gate/vertical/help/1", 10, vgate_help1_cb);
    // help2 ga kepake 
    ros::Subscriber vgate_help2_sub = nh.subscribe<std_msgs::UInt8>  ("/gate/vertical/help/2", 10, vgate_help2_cb);
    ros::Subscriber hgate_sub       = nh.subscribe<vision::RotatedRect>  ("/gate/horizontal/data", 10, hgate_cb);
    ros::Subscriber hgate_help1_sub = nh.subscribe<vision::RotatedRect>  ("/gate/horizontal/help", 10, hgate_help_cb);

    ros::Rate rate(1/dt);

    std_msgs::Empty empty;
    geometry_msgs::Twist vel;
    vel.linear.x = 0;
    vel.linear.y = 0;
    vel.linear.z = 0;
    vel.angular.x = 0;
    vel.angular.y = 0;
    vel.angular.z = 0;

    //send a few setpoints before starting
    for(int i = 10; ros::ok() && i > 0; --i){
        vel_pub.publish(vel);
        ros::spinOnce();
        rate.sleep();
    }

    // Change drone mode to takeoff
    soero::Log::Info("Set Drone to takeoff");
    takeoff_pub.publish(empty);
    soero::Log::Info("Done set to takeoff");

    bool early_off = true;
    double hold_z = 1.35;//-> 0.5-1.5m kaki, 0.7 gate, so (0.5+1.5)/2 + 0.7/2
    double hold_yaw = -1.57079632679;
    geometry_msgs::Pose lastGatePos;
    ros::Time existGuideTime = ros::Time::now();
    ros::Time start_flight_time = ros::Time::now();     // self itung berapa lama terbang (not exact but close enough)
    
    while(ros::ok() && early_off){
        vel.linear.z = mypid_z.calculate(hold_z, current_pose.position.z);
        vel.linear.x = 0.25;
        vel.angular.z = soero::CalcYawVelPID(hold_yaw, soero::getYawFromQuaternion(current_pose.orientation), mypid_yaw);
        
    }



}

