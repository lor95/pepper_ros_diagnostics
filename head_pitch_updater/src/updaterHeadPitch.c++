#include <diagnostic_updater/diagnostic_updater.h>
#include <diagnostic_updater/publisher.h>
#include <sensor_msgs/JointState.h>

double posHeadPitch0;
double posHeadYaw0;
double effHeadPitch0;
double velHeadPitch0;

double max_h_y = 2.085;
double min_h_y = -2.085;

double max_h_pv = 1;
double min_h_pv = -1;

double max_h_pe = 1;
double min_h_pe = -1;

double head_p_thr; // threshold head pitch warning
double head_e_thr; // threshold head pitch effort warning
double head_v_thr; // threshold head pitch velocity warning

double a(double val) {
    if(val < 0) {
        val = val * -1;
    }
    return val;
}

bool between(double var, double min, double max) {
    bool ret = false;
    if(var < max && var >= min) {
        ret = true;
    }
    return ret;
}

void headCallBack(const sensor_msgs::JointState::ConstPtr& msg){
	posHeadPitch0= msg -> position[0];
	posHeadYaw0 = msg->position[1];
    velHeadPitch0 = msg->velocity[0];
    effHeadPitch0 = msg->effort[0];
}

void posHead0_diagnostic(diagnostic_updater::DiagnosticStatusWrapper &stat){
    double min_h_p, max_h_p;
    if(between(a(posHeadYaw0),0,0.582)) {
        min_h_p = -0.707;
        max_h_p = 0.634;
    } else if(between(a(posHeadYaw0),0,0.582)) {
        min_h_p = -0.614;
        max_h_p = 0.365;
    } else { // 1.075 - 2.085
        min_h_p = -0.613;
        max_h_p = 0.236;
    }

    if(posHeadPitch0 <= (max_h_p*head_p_thr) && posHeadPitch0>(min_h_p*head_p_thr) && velHeadPitch0 <= (max_h_pv*head_v_thr) && velHeadPitch0>(min_h_pv*head_v_thr) && effHeadPitch0 <= (max_h_ye*head_e_thr) && effHeadPitch0>(min_h_y*head_e_thr)) {
        stat.summary(diagnostic_msgs::DiagnosticStatus::OK,"OK");
    } else if((posHeadPitch0<max_h_y && posHeadPitch0>min_h_y) || (velHeadPitch0<max_h_pv && velHeadPitch0>min_h_pv) || (effHeadPitch0<max_h_pe && effHeadPitch0>min_h_pe)){
        stat.summary(diagnostic_msgs::DiagnosticStatus::WARN,"ATTENZIONE");
    } else {
        stat.summary(diagnostic_msgs::DiagnosticStatus::ERROR,"GUASTO");
    }
    stat.add("Diagnostic name", "Head-Pitch");
    stat.add("Head pitch position", posHeadPitch0);
    stat.add("Head pitch velocity", velHeadPitch0);
    stat.add("Head pitch effort", effHeadPitch0);
}

int main(int argc, char** argv){
	ros::init(argc, argv, "updaterHeadPitchState");
	ros::NodeHandle n;
	n.getParam("/pepper/head_params/head_pitch_warnthr",head_p_thr);
    n.getParam("/pepper/head_params/head_pe_warnthr",head_e_thr);
    n.getParam("/pepper/head_params/head_pv_warnthr",head_v_thr);
	diagnostic_updater::Updater updater;
	ros::Subscriber sub=n.subscribe("/joint_states",1000,headCallBack);
	updater.setHardwareID("Robot-Head-Pitch");
	updater.add("Head Pitch", posHead0_diagnostic);
	ros::Rate loop_rate(100);
	while(n.ok())
		{
		ros::spinOnce();
		updater.update();
		loop_rate.sleep();
		}
	return 0;
}
