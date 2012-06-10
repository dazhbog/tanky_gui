/*****************************************************************************
** Includes
*****************************************************************************/

#include <QtGui>
#include <QMessageBox>
#include <iostream>
#include "../include/tanky_gui/main_window.hpp"
//#include "../include/tanky_gui/qnode.hpp"
/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace tanky_gui {
extern int right_motor_speed;
//extern QNode::LogLevel bla;
//extern QNode::LogLevel log_type;
using namespace Qt;

/*****************************************************************************
** Implementation [MainWindow]
*****************************************************************************/

MainWindow::MainWindow(int argc, char** argv, QWidget *parent)
    : QMainWindow(parent)
    , qnode(argc,argv)
{
    ui.setupUi(this); // Calling this incidentally connects all ui's triggers to on_...() callbacks in this class.
    QObject::connect(ui.actionAbout_Qt, SIGNAL(triggered(bool)), qApp, SLOT(aboutQt())); // qApp is a global variable for the application

    ReadSettings();
    setWindowIcon(QIcon(":/images/icon.png"));
    //ui.tab_manager->setCurrentIndex(0); // ensure the first tab is showing - qt-designer should have this already hardwired, but often loses it (settings?).
    QObject::connect(&qnode, SIGNAL(rosShutdown()), this, SLOT(close()));

    /*********************
 ** Logging
 **********************/
    ui.view_logging->setModel(qnode.loggingModel());
    QObject::connect(&qnode, SIGNAL(loggingUpdated()), this, SLOT(updateLoggingView()));
    //barrrrrrr

    QObject::connect(&qnode,SIGNAL(new_motor_left_speed_val(int)),this,SLOT(motor_left_speed_Handler(int)));
    QObject::connect(&qnode,SIGNAL(new_motor_right_speed_val(int)),this,SLOT(motor_right_speed_Handler(int)));
    QObject::connect(&qnode,SIGNAL(new_motor_left_current(int)),this,SLOT(motor_left_current_Handler(int)));
    QObject::connect(&qnode,SIGNAL(new_motor_right_current(int)),this,SLOT(motor_right_current_Handler(int)));
    QObject::connect(&qnode,SIGNAL(new_mode_val(int)),this,SLOT(mode_Handler(int)));
    QObject::connect(&qnode,SIGNAL(new_red_led_val(int)),this,SLOT(red_led_Handler(int)));
    QObject::connect(&qnode,SIGNAL(new_yellow_led_val(int)),this,SLOT(yellow_led_Handler(int)));
    QObject::connect(&qnode,SIGNAL(new_vert_val(int)),this,SLOT(vert_slider_Handler(int)));
    QObject::connect(&qnode,SIGNAL(new_horiz_val(int)),this,SLOT(horiz_slider_Handler(int)));
    /*********************
    ** Auto Start
    **********************/
    if ( ui.checkbox_remember_settings->isChecked() ) {
        on_button_connect_clicked(true);
    }
}

MainWindow::~MainWindow() {}

/*****************************************************************************
** Implementation [Slots]
*****************************************************************************/

void MainWindow::showNoMasterMessage() {
    QMessageBox msgBox;
    qnode.log(QNode::None,"Not Connected!");
    //msgBox.setText("Couldn't find the ros master.");
    //msgBox.exec();
    //msgBox.close();
}

/*
 * These triggers whenever the button is clicked, regardless of whether it
 * is already checked or not.
 */

void MainWindow::on_button_connect_clicked(bool check ) {
    if ( ui.checkbox_use_environment->isChecked() ) {
        if ( !qnode.init() ) {
            showNoMasterMessage();
        } else {
            ui.button_connect->setEnabled(false);
        }
    } else {
        if ( ! qnode.init(ui.line_edit_master->text().toStdString(),
                          ui.line_edit_host->text().toStdString()) ) {
            showNoMasterMessage();
        } else {
            ui.button_connect->setEnabled(false);
            ui.line_edit_master->setReadOnly(true);
            ui.line_edit_host->setReadOnly(true);
            //ui.line_edit_topic->setReadOnly(true);
        }
    }
}


void MainWindow::on_checkbox_use_environment_stateChanged(int state) {
    bool enabled;
    if ( state == 0 ) {
        enabled = true;
    } else {
        enabled = false;
    }
    ui.line_edit_master->setEnabled(enabled);
    ui.line_edit_host->setEnabled(enabled);
    //ui.line_edit_topic->setEnabled(enabled);
}

/*****************************************************************************
** Implemenation [Slots][manually connected]
*****************************************************************************/

/**
 * This function is signalled by the underlying model. When the model changes,
 * this will drop the cursor down to the last line in the QListview to ensure
 * the user can always see the latest log message.
 */
void MainWindow::updateLoggingView() {
    ui.view_logging->scrollToBottom();
    //ui.progressBar->setValue(motor);
}

/*****************************************************************************
** Implementation [Menu]
*****************************************************************************/

void MainWindow::on_actionAbout_triggered() {
    QMessageBox::about(this, tr("About Dashbard"),tr("<h2>Robot dash</h2><p>Copyright Marios Georgiou 2012</p><p>Dash for tanky v0.6</p>"));
}

/*****************************************************************************
** Implementation [Configuration]
*****************************************************************************/

void MainWindow::ReadSettings() {
    QSettings settings("Qt-Ros Package", "tanky_gui");
    restoreGeometry(settings.value("geometry").toByteArray());
    restoreState(settings.value("windowState").toByteArray());
    QString master_url = settings.value("master_url",QString("http://192.168.1.2:11311/")).toString();
    QString host_url = settings.value("host_url", QString("192.168.1.3")).toString();
    //QString topic_name = settings.value("topic_name", QString("/chatter")).toString();
    ui.line_edit_master->setText(master_url);
    ui.line_edit_host->setText(host_url);
    //ui.line_edit_topic->setText(topic_name);
    bool remember = settings.value("remember_settings", false).toBool();
    ui.checkbox_remember_settings->setChecked(remember);
    bool checked = settings.value("use_environment_variables", false).toBool();
    ui.checkbox_use_environment->setChecked(checked);
    if ( checked ) {
    	ui.line_edit_master->setEnabled(false);
    	ui.line_edit_host->setEnabled(false);
    	//ui.line_edit_topic->setEnabled(false);
    }
}

void MainWindow::WriteSettings() {
    QSettings settings("Qt-Ros Package", "tanky_gui");
    settings.setValue("master_url",ui.line_edit_master->text());
    settings.setValue("host_url",ui.line_edit_host->text());
    //settings.setValue("topic_name",ui.line_edit_topic->text());
    settings.setValue("use_environment_variables",QVariant(ui.checkbox_use_environment->isChecked()));
    settings.setValue("geometry", saveGeometry());
    settings.setValue("windowState", saveState());
    settings.setValue("remember_settings",QVariant(ui.checkbox_remember_settings->isChecked()));

}

void MainWindow::closeEvent(QCloseEvent *event)
{
    WriteSettings();
    QMainWindow::closeEvent(event);
}

//===================================================================================================================

void MainWindow::button1_Handler()
{

    if (qnode.init() ){
        qnode.log(QNode::None,"Changing mode to ROS(1)");
        qnode.mode_set(1);
        qnode.robot_motors_speed_set(30,1);
    }
    else if ( !qnode.init() )
        showNoMasterMessage();

}
void MainWindow::robot_boot_Handler(){
    static int state=0;
    static QProcess *proc = new QProcess();
    std::stringstream ss;
    QString cmd;
    QString pid_str;
    bool runs;
    int pidd = NULL;
    if (qnode.init() ){
        if (state == 1){
            proc->kill();

            cmd = "gnome-terminal --geometry=50x10-0-10 -x kill ";
            pid_str.setNum(pidd);
            cmd = cmd + pid_str;
            //proc->start(cmd);
            //qnode.log(QNode::None,"Shutting down robot node..");
            state=0;
        }
        else{
            qnode.log(QNode::None,"Booting Robot Node...");

           proc->start("gnome-terminal --geometry=50x10-0-10 -x bash -c \"rosrun rosserial_python serial_node.py /dev/ttyUSB0\" ");

           //QStringList args;
           //args << "-x sh -c ~/ros_workspace/robot/tanky/driver_rviz_up.sh";
            //proc->start("gnome-terminal --geometry=50x10-0-10 -x bash -c \"echo b && read a\" ");
            //proc->start("gnome-terminal --geometry=50x10-0-10 -x bash -c \"roslaunch tanky init_rviz_laser_odom.launch\" ");


            //proc->start("gnome-terminal -x bash -c", QStringList() << ". ~/ros_workspace/robot/tanky/driver_rviz_up.sh");
           //'sleep 2 && dmesg && read e'
           // while(runs != TRUE);
           //for(long long int i=0;i<1000000000;i++){
           //     for(long long int z=0;z<100000000;z++);}
           // QThread::sleep(1000);
            pidd = proc->pid();
            ss << "PID: " << pidd;
            qnode.log(QNode::None,std::string(ss.str()));
            //  proc->start("gnome-terminal --geometry=50x10-0-10 ' rosrun rosserial_python serial_node.py /dev/ttyUSB0'");
            //proc->start("nautilus");
            state=1;}


    }
    else if ( !qnode.init() ){
        showNoMasterMessage();
    }


}
void MainWindow::vision_driver_open_Handler(){
    if (qnode.init() ){
        QProcess *proc1 = new QProcess();
        QProcess *proc2 = new QProcess();
        proc1->start("gnome-terminal --geometry=50x10-0-10 -x bash -c \"rm -R ~/.rviz\" ");
        proc2->start("gnome-terminal --geometry=50x10-0-10 -x bash -c \"roslaunch tanky init_rviz_laser_odom.launch\" ");
    }
    else if ( !qnode.init() ){
        showNoMasterMessage();
    }

}
void MainWindow::slam_Handler(){
    if (qnode.init() ){
        QProcess *proc = new QProcess();
        proc->start("gnome-terminal --geometry=50x10-0-10 -x bash -c \"roslaunch tanky slam.launch\" ");
    }
    else if ( !qnode.init() ){
        showNoMasterMessage();
    }

}

void MainWindow::dump_map_Handler(){
    if (qnode.init() ){
        QProcess *proc = new QProcess();
        proc->start("gnome-terminal --geometry=50x10-0-10 -x bash -c \"rosrun map_server map_saver -f ~/ros_workspace/robot/tanky/my_map123\" ");
    }
    else if ( !qnode.init() ){
        showNoMasterMessage();
    }
}

void MainWindow::roscore_Handler(){
        QProcess *proc = new QProcess();
        proc->start("gnome-terminal --geometry=50x10-0-10 -x bash -c \"roscore\" ");
}


void MainWindow::object_recognition_Handler(){
    if (qnode.init() ){
        QProcess *proc = new QProcess();
        proc->start("gnome-terminal --geometry=50x10-0-10 -x bash -c \"rosrun tanky object_recognition\" ");
    }
    else if ( !qnode.init() ){
        showNoMasterMessage();
    }
}

void MainWindow::rviz_Handler(){
    if (qnode.init() ){
        QProcess *proc = new QProcess();
        proc->start("gnome-terminal --geometry=50x10-0-10 -x bash -c \"roslaunch tanky rviz.launch\" ");
    }
    else if ( !qnode.init() ){
        showNoMasterMessage();
    }

}


void MainWindow::motor_left_speed_Handler(int speed){
    //ui.->setValue(speed);
    ui.motor_left_speed_bar->setValue(speed);
    ui.left_speed_lbl->setNum(speed);
}
void MainWindow::motor_right_speed_Handler(int speed){
    //ui.progressBar->setValue(speed);
    ui.motor_right_speed_bar->setValue(speed);
    ui.right_speed_lbl->setNum(speed);
}
void MainWindow::motor_left_current_Handler(int current){
    //ui.progressBar->setValue(speed);
    ui.left_current_lbl->setNum(current);
    ui.motor_left_current_bar->setValue(current);
}
void MainWindow::motor_right_current_Handler(int current){
    ui.right_current_lbl->setNum(current);
    ui.motor_right_current_bar->setValue(current);
}
void MainWindow::mode_Handler(int mode){

    if     (mode == 2)
        ui.mode_lbl->setText("MODE: R/C Manual");
    else if(mode == 1)
        ui.mode_lbl->setText("MODE: ROS ");
    else
        ui.mode_lbl->setText("MODE: SAFE");
    //else error
}
void MainWindow::emergency_Handler(){

    if (qnode.init() ){
        qnode.log(QNode::None,"Changing mode to SAFE(3)");
        qnode.mode_set(3);
        qnode.robot_motors_speed_set(0,0);
    }
    else if ( !qnode.init() )
        showNoMasterMessage();
}
void MainWindow::manual_Handler(){
    if (qnode.init() ){
        qnode.log(QNode::None,"Changing mode to MANUAL(2)");
        qnode.mode_set(2);
        qnode.robot_motors_speed_set(0,0);
    }
    else if ( !qnode.init() )
        showNoMasterMessage();
}
void MainWindow::cap_Handler(){
    if (qnode.init() ){
        qnode.log(QNode::None,"Changing Speed Cap Value");
        ui.cap_speed->setValue(ui.cap_speed_slider->value());
        qnode.motor_cap_set(ui.cap_speed_slider->value());
        ui.cap_lbl->setNum(ui.cap_speed_slider->value());
        //qnode.mode_set(2);
        //qnode.robot_motors_speed_set(0,0);
    }
    else if ( !qnode.init() )
        showNoMasterMessage();

}
void MainWindow::red_led_Handler(int val){
    ui.red_bar->setValue(val);
}
void MainWindow::yellow_led_Handler(int val){
    ui.yellow_bar->setValue(val);
}
void MainWindow::vert_slider_Handler(int val){
    ui.vert_slider->setValue(val);
}
void MainWindow::horiz_slider_Handler(int val){
    ui.horiz_slider->setValue(val);
}




}  // namespace tanky_gui

