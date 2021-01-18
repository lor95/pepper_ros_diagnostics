#include <diagnostic_updater/diagnostic_updater.h>
#include <diagnostic_updater/publisher.h>
#include <sensor_msgs/JointState.h>

float posHipPitch;
float velHipPitch;
float effortHipPitch;

float min_h_p = -1.0385; // min hip pitch pos
float max_h_p = 1.0385; // max hip pitch pos

float max_h_r = 5.86; // max hip pitch vel

float max_k_p = 130; // max hip pitch effort

float hip_p_thr; // threshold hip pitch pos warning
float hip_r_thr; // threshold hip pitch vel warning
float knee_p_thr; // threshold hip pitch effort warning

void hipPitchCallBack(const sensor_msgs::JointState::ConstPtr& msg){
	posHipPitch= msg -> position[2];
	velHipPitch= msg -> velocity[2];
	effortHipPitch= msg -> effort[2];
}

void posHip0_diagnostic(diagnostic_updater::DiagnosticStatusWrapper &stat){
	if(posHipPitch <= (max_h_p*hip_p_thr) && posHipPitch>(min_h_p*hip_p_thr)
	&& velHipPitch <= (max_h_r*hip_r_thr) && effortHipPitch <= (max_k_p*knee_p_thr)) {
		stat.summary(diagnostic_msgs::DiagnosticStatus::OK,"OK");
	} else if((posHipPitch<max_h_p && posHipPitch>min_h_p) 
	|| (velHipPitch<max_h_r)
	|| (effortHipPitch<max_k_p)){
		stat.summary(diagnostic_msgs::DiagnosticStatus::WARN,"ATTENZIONE");
	} else {
		stat.summary(diagnostic_msgs::DiagnosticStatus::ERROR,"SI E' ROTTO");
	}

	stat.add("Diagnostic name", "Hip Pitch");
	stat.add("Hip pitch position", posHipPitch);
	stat.add("Hip pitch velocity", velHipPitch); 
	stat.add("Hip Pitch effort", effortHipPitch); 
}

int main(int argc, char** argv){
	ros::init(argc, argv, "updaterHipPitchState");
	ros::NodeHandle n;
	n.getParam("/pepper/hip_params/hip_pitch_warnthr",hip_p_thr);
	n.getParam("/pepper/hip_params/hip_roll_warnthr",hip_r_thr);
	n.getParam("/pepper/hip_params/knee_pitch_warnthr",knee_p_thr);
	diagnostic_updater::Updater updater;
	ros::Subscriber sub=n.subscribe("/joint_states",1000,hipPitchCallBack);
	updater.setHardwareID("Robot-Hip Pitch");
	updater.add("Hip Pitch", posHip0_diagnostic);
	ros::Rate loop_rate(100);
	while(n.ok())
		{
		ros::spinOnce();
		updater.update();
		loop_rate.sleep();
		}
	return 0;
}
