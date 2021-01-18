#include <diagnostic_updater/diagnostic_updater.h>
#include <diagnostic_updater/publisher.h>
#include <sensor_msgs/JointState.h>

float posKneePitch;
float velKneePitch;
float effortKneePitch;

float min_h_p = -0.5149; // min knee pitch pos
float max_h_p = 0.5149; // max knee pitch pos

float max_h_r = 5.86; // max knee pitch vel

float max_k_p = 130; // max knee pitch effort

float hip_p_thr; // threshold knee pitch pos warning
float hip_r_thr; // threshold knee pitch vel warning
float knee_p_thr; // threshold knee pitch effort warning

void kneePitchCallBack(const sensor_msgs::JointState::ConstPtr& msg){
	posKneePitch= msg -> position[4];
	velKneePitch= msg -> velocity[4];
	effortKneePitch= msg -> effort[4];
}

void posHip2_diagnostic(diagnostic_updater::DiagnosticStatusWrapper &stat){
	if(posKneePitch <= (max_h_p*hip_p_thr) && posKneePitch>(min_h_p*hip_p_thr)
	&& velKneePitch <= (max_h_r*hip_r_thr) && effortKneePitch <= (max_k_p*knee_p_thr)) {
		stat.summary(diagnostic_msgs::DiagnosticStatus::OK,"OK");
	} else if((posKneePitch<max_h_p && posKneePitch>min_h_p) 
	|| (velKneePitch<max_h_r)
	|| (effortKneePitch<max_k_p)){
		stat.summary(diagnostic_msgs::DiagnosticStatus::WARN,"ATTENZIONE");
	} else {
		stat.summary(diagnostic_msgs::DiagnosticStatus::ERROR,"GUASTO");
	}
	stat.add("Diagnostic name", "Knee Pitch");
	stat.add("Knee pitch position", posKneePitch);
	stat.add("Knee pitch velocity", velKneePitch); 
	stat.add("Knee Pitch effort", effortKneePitch); 
}

int main(int argc, char** argv){
	ros::init(argc, argv, "updaterKneePitchState");
	ros::NodeHandle n;
	n.getParam("/pepper/hip_params/hip_pitch_warnthr",hip_p_thr);
	n.getParam("/pepper/hip_params/hip_roll_warnthr",hip_r_thr);
	n.getParam("/pepper/hip_params/knee_pitch_warnthr",knee_p_thr);
	diagnostic_updater::Updater updater;
	ros::Subscriber sub=n.subscribe("/joint_states",1000,kneePitchCallBack);
	updater.setHardwareID("Robot-Knee Pitch");
	updater.add("Knee Pitch", posHip2_diagnostic);
	ros::Rate loop_rate(100);
	while(n.ok())
		{
		ros::spinOnce();
		updater.update();
		loop_rate.sleep();
		}
	return 0;
}
