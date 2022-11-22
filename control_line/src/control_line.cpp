#include <ros/ros.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/UInt8.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include "control_line/pid.hpp"
#include "control_line/soerolog.hpp"
#include "control_line/soeroYawUtils.hpp"
#include "../../vision_line/include/vision_line/RotatedRect.h"

#include <stdexcept>
#include <string>

#define _CRT_SECURE_NO_WARNINGS

#define DEG_2_RAD 0.01745329251
#define pix_per_meter 0.01875
#define HALF_PI 1.57079632679

//=====================PID SETUP===================================//
double dt = 1/32.0;
// double max_val = 1.0;
// double min_val = -1.0;
// double Kp = 0.45;
// double Ki = 0.000;
// double Kd = 0.0007;
// double theta_max_val=2.0;
// double theta_min_val=-2.0;

PID mypid_x(0.03125, 0.1, -0.1, 0.5, 0.001, 0.001);
PID mypid_y(0.03125, 0.1, -0.1, 0.5, 0.001, 0.001);
PID mypid_z(0.03125, 0.1, -0.1, 0.5, 0.001, 0.001);
PID mypid_yaw(dt, 1.5, -1.5, 0.900, 0.100, 0.00000);
/*----------------------del time, interval val, kp,    kd,    ki*/
// PID mypid_z_vision_gate(dt, max_val, min_val, 0.0040, 0.0011, 0);
// PID mypid_y_vision_gate(dt, max_val, min_val, 0.0040, 0.0016, 0);
// PID mypid_y_hor_gate(dt, max_val, min_val, 0.500, 0.0011, 0);
// PID mypid_x_hor_gate(dt, max_val, min_val, 0.500, 0.0021, 0);

//=====================CALLBACKs===================================//

geometry_msgs::PoseStamped current_pose;
void pose_cb(const geometry_msgs::PoseStamped::ConstPtr msg){
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

mavros_msgs::State current_state;
void state_callback(const mavros_msgs::State::ConstPtr& msg){
    current_state = *msg;
}




// =====================HELPER =========================== //
void zerofied_main_vel(geometry_msgs::Twist &vel){
    vel.linear.x = 0;
    vel.linear.y = 0;
    vel.linear.z = 0;
    vel.angular.z = 0;
}


int convert_string_to_integer(std::string &arg){
    try {
    std::size_t pos;
    int x = std::stoi(arg, &pos);
    if (pos < arg.size()) {
        std::cerr << "Trailing characters after number: " << arg << '\n';
    }
    return x;
    } catch (std::invalid_argument const &ex) {
    std::cerr << "Invalid number: " << arg << '\n';
    } catch (std::out_of_range const &ex) {
    std::cerr << "Number out of range: " << arg << '\n';
    }
    return -1;// biar compiler ga ngasi warning
}

//=======================================================================//
//                    MATH STUFF                                        //
//=======================================================================
constexpr double vx_max = 0.64;                     // velocity maximum when line is align
constexpr double vx_min = 0.005;                    // minimum velocity when angular z exceed vaz_max
constexpr double vaz_max = 0.15;                    // angular velocity maximum to consider still moving/ do vmin
constexpr double _m_fx = (vx_max - vx_min)/vaz_max; // gradient for x that seamlessly transition between vmax and vmin
double line_activation_x(double vel_ang_z, double mult){
    double vel = -std::fabs(_m_fx*vel_ang_z) + vx_max;  // try this func in desmos to see visualization
    return std::max(vel, vx_min)*mult;                  // cap the min vel to vmin not negative and set to 0 if mult is 0
}

constexpr double area_min = 20.0;                   // minimum area to consider moving towards the gate, instead of centering only
constexpr double area_stop = 85.0;                  // maximum area to consider STOP, dah deket bang
constexpr double area_min_v = 0.5; // max velocity when gate area is == area_min | jangan 0.5 berbahaya suka gagal-->ditambal pakek if else room top h bottom h gate
constexpr double _m_avx = area_min_v/(area_min - area_stop);// calculate gradient for seamless transition of vel
constexpr double _b_avx = -area_stop*_m_avx;                // and the b too
double gate_activation_x(double gateArea, int mult=1){
    gateArea *= 0.001;// discaledown
    return std::max(_m_avx*gateArea + _b_avx, 0.0)*mult;// ensure min vel is 0 not negative and set 0 if mult is 0
}

/*
approximate pixel on down cam to meter using some data (udah tak ambil)
*/
double get_max_width_meter(){
/* ŷ = 1.09881X - 0.00682 -> x is Z a.k.a height/altitude, y is approximation range of camera
   <drone>
   /    \
  /      \
 /        \
a--ground--b --> nah berapa jarak a-b di camera? this is what function do/try to approximate
*/
    return 1.09881*current_pose.pose.position.z - 0.00682;
}

/*
then use that 'meter' to calculate velocity using PID in this function
*/
double calc_vel_based_vision_z(double vision_data, double cam_length, double max_meter,PID &thePID, char label, int mult=1){
    double cam_center_meter = max_meter * 0.5;                                      // center of cam is always half of the range
    double vision_data_meter = vision_data/cam_length * max_meter;                  // convert vision pixel to 'meter'
    double calc_vel = thePID.calculate(cam_center_meter, vision_data_meter)*mult;   // calc the vel using PID
    soero::Log::Info("[z: %f, cal_vel:%c]: cam center=%f | vis raw=%f, res: %f", current_pose.pose.position.z,label, cam_center_meter, vision_data, calc_vel);
    return calc_vel;

}

// return velocity needed to go from theta a to theta b with PID
double velAlignYawToLine(double line_rad){
    return soero::CalcYawVelPID(line_rad, HALF_PI, mypid_yaw);
}

// ============================================================================ //
//                        VISION HANDLER                                        //
// =============================================================================//

/*
calculate velocity for ang.z, lin.x, lin.y based on line
*/
void handleLine(geometry_msgs::Twist &vel, vision::RotatedRect &line_guide, double mult_x=1.0){
    double line_rad = line_guide.angle*DEG_2_RAD;
    vel.angular.z = velAlignYawToLine(line_rad);
    soero::Log::Info("target is: %f, yet we are here: %f, so ang.z: %f", HALF_PI, line_rad, vel.angular.z);
    // maju mundur
    vel.linear.x = line_activation_x(vel.angular.z, mult_x) + 0.0;
    // kiri kanan
    vel.linear.y = calc_vel_based_vision_z(line_guide.x, 640.0, get_max_width_meter(),mypid_y, 'y');
    soero::Log::Info("y garis: %f, yet we are here: %f, so vel y: %f", 320.0, line_guide.x, vel.linear.y);
}

bool near_equal(double a,double b,double precision){
    return std::fabs(a-b) < precision;
}

void takeoff(ros::Publisher vel_pub,ros::Rate rate){
    geometry_msgs::Twist vel;
    bool tookoff = false;
    ros::Time takeoff;

    while(ros::ok() && !tookoff){
        ros::spinOnce();
        rate.sleep();

        if(!near_equal(current_pose.pose.position.z,2,0.3)){
            vel.linear.x = mypid_x.calculate(current_pose.pose.position.x,current_pose.pose.position.x);
            vel.linear.y = mypid_y.calculate(current_pose.pose.position.y,current_pose.pose.position.y);
            vel.linear.z = mypid_z.calculate(1.5,current_pose.pose.position.z);

            vel_pub.publish(vel);
            continue;
        }

        if(takeoff.is_zero()){
            takeoff = ros::Time::now();
            continue;
        }

        vel_pub.publish(vel);

        if(ros::Time::now() - takeoff < ros::Duration(5.0)){
            continue;
        }

        tookoff == true;

    }
}

void forward(ros::Publisher vel_pub,ros::Rate rate){
    geometry_msgs::Twist vel;
    ros::Time existGuideTime = ros::Time::now();
    double hold_z = 1.5;//-> 0.5-1.5m kaki, 0.7 gate, so (0.5+1.5)/2 + 0.7/2
    double hold_yaw = 0;
    bool line = false;
    while(ros::ok() && !line){
        int counter = 0;
        while (counter <= 10) {ROS_INFO("Mulai masuk misi utama"); counter++;}
        vel.linear.z = mypid_z.calculate(hold_z, current_pose.pose.position.z);
        vel.linear.x = 0.25;
        vel.angular.z = soero::CalcYawVelPID(hold_yaw, soero::getYawFromQuaternion(current_pose.pose.orientation), mypid_yaw);
        if(upperLine.is_exist){
            ROS_INFO("Wah ada garis depan");
            handleLine(vel, upperLine);
        }else if(lowerLine.is_exist){
            ROS_INFO("Wah ada garis belakang");
            handleLine(vel, upperLine);
        }
        else{
            ROS_INFO("Garisnya mana ya");
            existGuideTime= ros::Time::now();
        }
     }
    vel_pub.publish(vel);
    ros::spinOnce();
    rate.sleep();
}

//=================================================MAIN-MAIN-MAIN===================================================================//
int main(int argc, char **argv){
    ros::init(argc, argv, "linecontrol");
    ros::NodeHandle nh;
    //takeoff api
    ros::Subscriber state_sub        = nh.subscribe<mavros_msgs::State>   ("/mavros/state",10,state_callback );
    ros::Subscriber current_pos_sub  = nh.subscribe<geometry_msgs::PoseStamped> ("/mavros/local_position/pose",20,pose_cb);
    ros::ServiceClient  arming_client   = nh.serviceClient<mavros_msgs::CommandBool>   ("/mavros/cmd/arming", 20);
    ros::ServiceClient  set_mode_client = nh.serviceClient<mavros_msgs::SetMode>       ("/mavros/set_mode", 20);

    // ros::Publisher takeoff_pub      = nh.advertise<std_msgs::Empty>      ("/drone/takeoff", 10, true);

    //movement api
    ros::Publisher vel_pub          = nh.advertise<geometry_msgs::Twist> ("/mavros/setpoint_velocity/cmd_vel_unstamped", 20);
    ros::Publisher local_pos_pub    = nh.advertise<geometry_msgs::Pose>  ("/mavros/setpoint_position/local", 10 );

    //ros::Publisher land_pub         = nh.advertise<std_msgs::Empty>      ("/drone/land", 10);

    // vision datas
    ros::Subscriber upper_line_sub  = nh.subscribe<vision::RotatedRect>  ("/upper/line_data", 10, uline_cb);
    ros::Subscriber lower_line_sub  = nh.subscribe<vision::RotatedRect>  ("/lower/line_data", 10, lline_cb);

    ros::Rate rate(32.0);

    std_msgs::Empty empty;
    geometry_msgs::Twist vel; //Publish kecepatan
    mavros_msgs::SetMode set_mode;   //variable service set mode UAV
    mavros_msgs::CommandBool arm_cmd; //Variable service arming
    vel.linear.x = 0;
    vel.linear.y = 0;
    vel.linear.z = 0;
    vel.angular.x = 0;
    vel.angular.y = 0;
    vel.angular.z = 0;
    
    ROS_INFO("UAV Inbound...");

    char wait_sir;
    std::cout << "\nReady to fly? \n";
    std::cin >> wait_sir;

    for(int i = 10; ros::ok() && i > 0; --i){
        vel_pub.publish(vel);
        ros::spinOnce();
        rate.sleep();
    }

    ROS_INFO("Taking off");
    takeoff(vel_pub,rate);


    ROS_INFO("Going forward");
    forward(vel_pub,rate);

     return 0;
}