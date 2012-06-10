/**
 * @file /src/qnode.cpp
 *
 * @brief Ros communication central!
 *
 * @date February 2011
 **/

/*****************************************************************************
** Includes
*****************************************************************************/

#include <ros/ros.h>
#include <ros/network.h>
#include <string>
#include <std_msgs/String.h>
#include <std_msgs/Int16.h>
#include <sstream>
#include "../include/tanky_gui/qnode.hpp"
#include <QtGui>

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace tanky_gui {

/*****************************************************************************
** Implementation
*****************************************************************************/
int right_motor_speed=0;
QNode::QNode(int argc, char** argv ) :
    init_argc(argc),
    init_argv(argv)
{}

QNode::~QNode() {
    if(ros::isStarted()) {
        ros::shutdown(); // explicitly needed since we use ros::start();
        ros::waitForShutdown();
    }
    wait();
}

bool QNode::init() {
    static int started = 0;
    if (started == 1){
        return true;}
    ros::init(init_argc,init_argv,"tanky_gui");
    if ( ! ros::master::check() ) {
        return false;
    }
    ros::start(); // explicitly needed since our nodehandle is going out of scope.
    ros::NodeHandle n;
    // Add your ros communications here.
    robot_motor_left_speed_set_pub =n.advertise<std_msgs::Int16>("/robot/motor/left/speed_set",1000);
    robot_motor_right_speed_set_pub =n.advertise<std_msgs::Int16>("/robot/motor/right/speed_set",1000);
    robot_mode_set_pub =  n.advertise<std_msgs::Int16>("/robot/mode_set",10);
    robot_motor_speed_cap_pub = n.advertise<std_msgs::Int16>("/robot/motor/speed_cap",10);

    robot_motor_left_current_sub= n.subscribe("robot/motor/left/current", 1, &QNode::robot_motor_left_current, this);
    robot_motor_right_current_sub=n.subscribe("robot/motor/right/current", 1, &QNode::robot_motor_right_current, this);
    robot_motor_left_speed_val_sub = n.subscribe("robot/motor/left/speed_val", 1, &QNode::robot_motor_left_speed_val, this);
    robot_motor_right_speed_val_sub = n.subscribe("robot/motor/right/speed_val", 1, &QNode::robot_motor_right_speed_val, this);
    robot_mode_sub = n.subscribe("robot/mode", 1, &QNode::robot_mode, this);
    robot_red_led_sub = n.subscribe("robot/LED/red", 1, &QNode::robot_red_led, this);
    robot_yellow_led_sub = n.subscribe("robot/LED/yellow", 1, &QNode::robot_yellow_led, this);
    robot_vert_sub = n.subscribe("/robot/RC/vertical_slider" , 100 , &QNode::robot_vert_slider, this);
    robot_horiz_sub = n.subscribe("/robot/RC/horizontal_slider" , 100 , &QNode::robot_horiz_slider, this);
    start();
    log(None,"Connected!!");
    started =1;
    return true;
}

bool QNode::init(const std::string &master_url, const std::string &host_url) {
    static int started = 0;
    if (started == 1)
        return true;
    std::map<std::string,std::string> remappings;
    remappings["__master"] = master_url;
    remappings["__hostname"] = host_url;
    ros::init(remappings,"tanky_gui");
    if ( ! ros::master::check() ) {
        return false;
    }
    ros::start(); // explicitly needed since our nodehandle is going out of scope.
    ros::NodeHandle n;
    // Add your ros communications here.
    //chatter_publisher = n.advertise<std_msgs::String>("chatter", 1000);
    robot_motor_left_speed_set_pub =n.advertise<std_msgs::Int16>("/robot/motor/left/speed_set",10);
    robot_motor_right_speed_set_pub =n.advertise<std_msgs::Int16>("/robot/motor/right/speed_set",10);
    robot_mode_set_pub =  n.advertise<std_msgs::Int16>("/robot/mode_set",10);
    robot_motor_speed_cap_pub = n.advertise<std_msgs::Int16>("/robot/motor/speed_cap",10);

    robot_motor_left_current_sub= n.subscribe("robot/motor/left/current", 1, &QNode::robot_motor_left_current, this);
    robot_motor_right_current_sub=n.subscribe("robot/motor/right/current", 1, &QNode::robot_motor_right_current, this);
    robot_motor_left_speed_val_sub = n.subscribe("robot/motor/left/speed_val", 1, &QNode::robot_motor_left_speed_val, this);
    robot_motor_right_speed_val_sub = n.subscribe("robot/motor/right/speed_val", 1, &QNode::robot_motor_right_speed_val, this);
    robot_mode_sub = n.subscribe("robot/mode", 1, &QNode::robot_mode, this);
    robot_red_led_sub = n.subscribe("robot/LED/red", 1000, &QNode::robot_red_led, this);
    robot_yellow_led_sub = n.subscribe("robot/LED/yellow", 1000, &QNode::robot_yellow_led, this);
    robot_vert_sub = n.subscribe("/robot/RC/vertical_slider" , 100 , &QNode::robot_vert_slider, this);
    robot_horiz_sub = n.subscribe("/robot/RC/horizontal_slider" , 100 , &QNode::robot_horiz_slider, this);

    start();
    started =1;
    return true;
}
void QNode::robot_vert_slider(const std_msgs::Int16::ConstPtr &msg)  {
    emit new_vert_val(msg->data);
}
void QNode::robot_horiz_slider(const std_msgs::Int16::ConstPtr &msg)  {
    emit new_horiz_val(msg->data);
}
void QNode::robot_red_led(const std_msgs::Int16::ConstPtr &msg)  {
    emit new_red_led_val(msg->data);
}
void QNode::robot_yellow_led(const std_msgs::Int16::ConstPtr &msg)  {
    emit new_yellow_led_val(msg->data);
}
void QNode::robot_mode(const std_msgs::Int16::ConstPtr &msg)  {
    emit new_mode_val(msg->data);
}
void QNode::robot_motor_left_current(const std_msgs::Int16::ConstPtr &msg)  {
    emit new_motor_left_current(msg->data);
}

void QNode::robot_motor_right_current(const std_msgs::Int16::ConstPtr &msg) {
    emit new_motor_right_current(msg->data);
}

void QNode::robot_motor_left_speed_val(const std_msgs::Int16::ConstPtr &msg){
    emit new_motor_left_speed_val(msg->data);
}

void QNode::robot_motor_right_speed_val(const std_msgs::Int16::ConstPtr &msg){
    emit new_motor_right_speed_val(msg->data);
}

void QNode::mode_set(int mode){
    std_msgs::Int16 modemsg;
    modemsg.data = mode;
    robot_mode_set_pub.publish(modemsg);
}

void QNode::motor_cap_set(int cap){
    std_msgs::Int16 capmsg;
    capmsg.data = cap;
    robot_motor_speed_cap_pub.publish(capmsg);
}

void QNode::robot_motors_speed_set(int L , int R){
    //int count = 0;
    if ( ros::ok() ){
        std_msgs::Int16 LM;
        std_msgs::Int16 RM;
        LM.data=L;
        RM.data=R;
        std::stringstream ss;
        ss << "Motors: Left Speed: " << LM.data<< " - Right Speed: " << RM.data;
        robot_motor_left_speed_set_pub.publish(LM);
        robot_motor_right_speed_set_pub.publish(RM);
        log(None,std::string(ss.str()));
    }
}

void QNode::run() {
    //ros::Rate loop_rate(1);
    //int count = 0;
    while ( ros::ok() ) {
        ros::spin();//was once
    }
    std::cout << "Ros shutdown, proceeding to close the gui." << std::endl;
    emit rosShutdown(); // used to signal the gui for a shutdown (useful to roslaunch)
}


void QNode::log( const LogLevel &level, const std::string &msg) {
    logging_model.insertRows(logging_model.rowCount(),1);
    std::stringstream logging_model_msg;
    switch ( level ) {
    case(Debug) : {
        ROS_DEBUG_STREAM(msg);
        logging_model_msg << "[DEBUG] [" << ros::Time::now() << "]: " << msg;
        break;
    }
    case(Info) : {
        ROS_INFO_STREAM(msg);
        logging_model_msg << "[INFO] [" << ros::Time::now() << "]: " << msg;
        break;
    }
    case(Warn) : {
        ROS_WARN_STREAM(msg);
        logging_model_msg << "[WARN] [" << ros::Time::now() << "]: " << msg;
        break;
    }
    case(Error) : {
        ROS_ERROR_STREAM(msg);
        logging_model_msg << "[ERROR] [" << ros::Time::now() << "]: " << msg;
        break;
    }
    case(Fatal) : {
        ROS_FATAL_STREAM(msg);
        logging_model_msg << "[FATAL] [" << ros::Time::now() << "]: " << msg;
        break;
    }
    default:                logging_model_msg << "[ GUI ] " << msg;
        break;
    }
    QVariant new_row(QString(logging_model_msg.str().c_str()));
    logging_model.setData(logging_model.index(logging_model.rowCount()-1),new_row);
    emit loggingUpdated(); // used to readjust the scrollbar
}




}  // namespace tanky_gui
