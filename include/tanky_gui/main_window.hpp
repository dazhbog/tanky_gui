/**
 * @file /include/tanky_gui/main_window.hpp
 *
 * @brief Qt based gui for tanky_gui.
 *
 * @date November 2010
 **/
#ifndef tanky_gui_MAIN_WINDOW_H
#define tanky_gui_MAIN_WINDOW_H

/*****************************************************************************
** Includes
*****************************************************************************/

#include <QtGui/QMainWindow>
#include "ui_main_window.h"
#include "qnode.hpp"

/*****************************************************************************
** Namespace
*****************************************************************************/

namespace tanky_gui {

/*****************************************************************************
** Interface [MainWindow]
*****************************************************************************/
/**
 * @brief Qt central, all operations relating to the view part here.
 */
class MainWindow : public QMainWindow {
Q_OBJECT

public:
	MainWindow(int argc, char** argv, QWidget *parent = 0);
	~MainWindow();

	void ReadSettings(); // Load up qt program settings at startup
	void WriteSettings(); // Save qt program settings when closing

	void closeEvent(QCloseEvent *event); // Overloaded function
	void showNoMasterMessage();

public slots:
	/******************************************
	** Auto-connections (connectSlotsByName())
	*******************************************/
	void on_actionAbout_triggered();
	void on_button_connect_clicked(bool check );
	void on_checkbox_use_environment_stateChanged(int state);
	void button1_Handler();
        void robot_boot_Handler();
        void vision_driver_open_Handler();
        void mode_Handler(int);
        void motor_left_speed_Handler(int);
        void motor_right_speed_Handler(int);
        void motor_left_current_Handler(int);
        void motor_right_current_Handler(int);
        void emergency_Handler();
        void manual_Handler();
        void cap_Handler();
        void red_led_Handler(int);
        void yellow_led_Handler(int);
        void vert_slider_Handler(int);
        void horiz_slider_Handler(int);
        void roscore_Handler();
        void slam_Handler();
        void object_recognition_Handler();
        void dump_map_Handler();
        void rviz_Handler();
    /******************************************
    ** Manual connections
    *******************************************/
    void updateLoggingView(); // no idea why this can't connect automatically

private:
	Ui::MainWindowDesign ui;
	QNode qnode;
};

}  // namespace tanky_gui

#endif // tanky_gui_MAIN_WINDOW_H
