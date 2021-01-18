#include <diagnostic_updater/diagnostic_updater.h>
#include <diagnostic_updater/publisher.h>
#include <sensor_msgs/JointState.h>

double posHeadYaw0;
double velHeadYaw0;
double effHeadYaw0;

double max_h_y = 2.085;
double min_h_y = -2.085;

double max_h_yv = 5.9;

double max_h_ye = 42.9;

double head_y_thr; // threshold head yaw warning
double head_e_thr; // threshold head yaw effort warning
double head_v_thr; // threshold head yaw velocity warning

void headCallBack(const sensor_msgs::JointState::ConstPtr& msg){
	posHeadYaw0 = msg->position[1];
    velHeadYaw0 = msg->velocity[1];
    effHeadYaw0 = msg->effort[1];
}

void posHead0_diagnostic(diagnostic_updater::DiagnosticStatusWrapper &stat){
    if(posHeadYaw0 <= (max_h_y*head_y_thr) && posHeadYaw0>(min_h_y*head_y_thr) && velHeadYaw0 <= (max_h_yv*head_v_thr) && effHeadYaw0 <= (max_h_ye*head_e_thr)) {
        stat.summary(diagnostic_msgs::DiagnosticStatus::OK,"OK");
    } else if((posHeadYaw0<max_h_y && posHeadYaw0>min_h_y) || velHeadYaw0<max_h_yv || effHeadYaw0<max_h_ye){
        stat.summary(diagnostic_msgs::DiagnosticStatus::WARN,"ATTENZIONE");
    } else {
        stat.summary(diagnostic_msgs::DiagnosticStatus::ERROR,"GUASTO");
    }
	stat.add("Diagnostic name", "Head-Yaw");
    stat.add("Head yaw position", posHeadYaw0);
    stat.add("Head yaw velocity", velHeadYaw0);
    stat.add("Head yaw effort", effHeadYaw0);
}

int main(int argc, char** argv){
	ros::init(argc, argv, "updaterHeadYawState");
	ros::NodeHandle n;
    n.getParam("/pepper/head_params/head_yaw_warnthr",head_y_thr);
    n.getParam("/pepper/head_params/head_yv_warnthr",head_v_thr);
    n.getParam("/pepper/head_params/head_ye_warnthr",head_e_thr);
	diagnostic_updater::Updater updater;
	ros::Subscriber sub=n.subscribe("/joint_states",1000,headCallBack);
	updater.setHardwareID("Robot-Head-Yaw");
	updater.add("Head Yaw", posHead0_diagnostic);
	ros::Rate loop_rate(100);
	while(n.ok())
		{
		ros::spinOnce();
		updater.update();
		loop_rate.sleep();
		}
	return 0;
}
