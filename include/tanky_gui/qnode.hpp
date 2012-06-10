/**
 * @file /include/tanky_gui/qnode.hpp
 *
 * @brief Communications central!
 *
 * @date February 2011
 **/
/*****************************************************************************
** Ifdefs
*****************************************************************************/

#ifndef tanky_gui_QNODE_HPP_
#define tanky_gui_QNODE_HPP_

/*****************************************************************************
** Includes
*****************************************************************************/

#include <ros/ros.h>
#include <string>
#include <QThread>
#include <QStringListModel>
#include <QProgressBar>
#include <std_msgs/String.h>
#include <std_msgs/Int16.h>

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace tanky_gui {

/*****************************************************************************
** Class
*****************************************************************************/

class QNode : public QThread {
    Q_OBJECT
public:
	QNode(int argc, char** argv );
	virtual ~QNode();
	bool init();
	bool init(const std::string &master_url, const std::string &host_url);
	void run();
        void robot_motors_speed_set(int,int);
        void mode_set(int );
        void motor_cap_set(int);
        void robot_motor_right_speed_val(const std_msgs::Int16::ConstPtr &msg);
        void robot_motor_left_speed_val(const std_msgs::Int16::ConstPtr &msg);
        void robot_motor_right_current(const std_msgs::Int16::ConstPtr &msg);
        void robot_motor_left_current(const std_msgs::Int16::ConstPtr &msg);
        void robot_mode(const std_msgs::Int16::ConstPtr &msg);
        void robot_red_led(const std_msgs::Int16::ConstPtr &msg);
        void robot_yellow_led(const std_msgs::Int16::ConstPtr &msg);
        void robot_vert_slider(const std_msgs::Int16::ConstPtr &msg);
        void robot_horiz_slider(const std_msgs::Int16::ConstPtr &msg);

        /*********************
	** Logging
	**********************/
	enum LogLevel {
	         Debug,
	         Info,
	         Warn,
	         Error,
                 Fatal,
                 None
         };

	QStringListModel* loggingModel() { return &logging_model; }
	void log( const LogLevel &level, const std::string &msg);
        QProgressBar current_bar;

signals:
	void loggingUpdated();
        void new_motor_right_speed_val(int speed);
        void new_motor_left_speed_val(int speed);
        void new_motor_right_current(int current);
        void new_motor_left_current(int current);
        void new_mode_val(int mode);
        void new_red_led_val(int);
        void new_yellow_led_val(int);
        void new_vert_val(int);
        void new_horiz_val(int);
        void rosShutdown();

private:
	int init_argc;
	char** init_argv;

        ros::Publisher  robot_motor_left_speed_set_pub;
        ros::Publisher  robot_motor_right_speed_set_pub;
        ros::Publisher  robot_mode_set_pub ;
        ros::Publisher  robot_motor_speed_cap_pub ;

        ros::Subscriber robot_motor_left_speed_val_sub;
        ros::Subscriber robot_motor_right_speed_val_sub;
        ros::Subscriber robot_motor_left_current_sub;
        ros::Subscriber robot_motor_right_current_sub;
        ros::Subscriber robot_mode_sub;
        ros::Subscriber robot_red_led_sub;
        ros::Subscriber robot_yellow_led_sub;
        ros::Subscriber robot_vert_sub;
        ros::Subscriber robot_horiz_sub;
        QStringListModel logging_model;

};

}  // namespace tanky_gui

#endif /* tanky_gui_QNODE_HPP_ */
