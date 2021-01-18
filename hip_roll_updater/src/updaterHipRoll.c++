#include <diagnostic_updater/diagnostic_updater.h>
#include <diagnostic_updater/publisher.h>
#include <sensor_msgs/JointState.h>

float posHipRoll;
float velHipRoll;
float effortHipRoll;

float min_h_p = -0.5149; // min hip roll pos
float max_h_p = 0.5149; // max hip roll pos

float max_h_r = 4.54; // max hip roll vel

float max_k_p = 69.5; // max hip roll effort

float hip_p_thr; // threshold hip roll pos warning
float hip_r_thr; // threshold hip roll vel warning
float knee_p_thr; // threshold hip roll effort warning

void hipRollCallBack(const sensor_msgs::JointState::ConstPtr& msg){
	posHipRoll= msg -> position[3];
	velHipRoll= msg -> velocity[3];
	effortHipRoll= msg -> effort[3];
}

void posHip1_diagnostic(diagnostic_updater::DiagnosticStatusWrapper &stat){
	if(posHipRoll <= (max_h_p*hip_p_thr) && posHipRoll>(min_h_p*hip_p_thr)
	&& velHipRoll <= (max_h_r*hip_r_thr) && effortHipRoll <= (max_k_p*knee_p_thr)) {
		stat.summary(diagnostic_msgs::DiagnosticStatus::OK,"OK");
	} else if((posHipRoll<max_h_p && posHipRoll>min_h_p) 
	|| (velHipRoll<max_h_r)
	|| (effortHipRoll<max_k_p)){
		stat.summary(diagnostic_msgs::DiagnosticStatus::WARN,"ATTENZIONE");
	} else {
		stat.summary(diagnostic_msgs::DiagnosticStatus::ERROR,"SI E' ROTTO");
	}

	stat.add("Diagnostic name", "Hip Roll");
	stat.add("Hip roll position", posHipRoll);
	stat.add("Hip roll velocity", velHipRoll); 
	stat.add("Hip roll effort", effortHipRoll); 
}

int main(int argc, char** argv){
	ros::init(argc, argv, "updaterHipRollState");
	ros::NodeHandle n;
	n.getParam("/pepper/hip_params/hip_pitch_warnthr",hip_p_thr);
	n.getParam("/pepper/hip_params/hip_roll_warnthr",hip_r_thr);
	n.getParam("/pepper/hip_params/knee_pitch_warnthr",knee_p_thr);
	diagnostic_updater::Updater updater;
	ros::Subscriber sub=n.subscribe("/joint_states",1000,hipRollCallBack);
	updater.setHardwareID("Robot-Hip Roll");
	updater.add("Hip Roll", posHip1_diagnostic);
	ros::Rate loop_rate(100);
	while(n.ok())
		{
		ros::spinOnce();
		updater.update();
		loop_rate.sleep();
		}
	return 0;
}
