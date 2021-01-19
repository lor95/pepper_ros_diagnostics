
#include <diagnostic_updater/diagnostic_updater.h>
#include <diagnostic_updater/publisher.h>
#include <sensor_msgs/JointState.h>
#include <cmath>

float posShoulderRoll0;
float velShoulderRoll0;
float effShoulderRoll0;

float min_ls_r = 0.0087; // min left shoulder roll
float max_ls_r = 1.5620; // max left shoulder roll

float max_ls_rv = 7.5;

float max_ls_re = 23.8;

float ls_r_thr; // threshold left shoulder pitch position warning
float ls_rv_thr; // threshold left shoulder pitch velocity warning
float ls_re_thr; // threshold left shoulder pitch effort warning


void leftArmCallBack(const sensor_msgs::JointState::ConstPtr& msg){
	//posizioni del braccio
	posShoulderRoll0= msg -> position[8];
	velShoulderRoll0= msg -> velocity[8];
	effShoulderRoll0= msg -> effort[8];
}


void posShoulderroll_diagnostic(diagnostic_updater::DiagnosticStatusWrapper &stat){
	float soglia = fabs(max_ls_r-min_ls_r)-fabs((max_ls_r-min_ls_r)*ls_r_thr);
	float interval[3] = {min_ls_r+soglia ,max_ls_r-soglia ,soglia};
	if(posShoulderRoll0 <= (max_ls_r- interval[2]) && posShoulderRoll0>(min_ls_r+ interval[2])) {
		stat.summary(diagnostic_msgs::DiagnosticStatus::OK,"OK");
	} else if(((posShoulderRoll0 < interval[0] && posShoulderRoll0 > min_ls_r) || (posShoulderRoll0 > interval[1] && posShoulderRoll0 < max_ls_r)) || (velShoulderRoll0 >= max_ls_rv*ls_rv_thr ) || (effShoulderRoll0 >= max_ls_re*ls_re_thr)) {
		stat.summary(diagnostic_msgs::DiagnosticStatus::WARN,"ATTENZIONE");
	} else {
		stat.summary(diagnostic_msgs::DiagnosticStatus::ERROR,"GUASTO");
	}

	stat.add("Diagnostic name", "LeftShoulderRoll");
	stat.add("Shoulder pitch position", posShoulderRoll0);
	stat.add("Shoulder pitch velocity", velShoulderRoll0);
	stat.add("Shoulder pitch effort", effShoulderRoll0);
}


int main(int argc, char** argv){
	ros::init(argc, argv, "updaterLeftArmState");
	ros::NodeHandle n;
	n.getParam("/pepper/left_arm_params/lshoulder_roll_warnthr",ls_r_thr);
	n.getParam("/pepper/left_arm_params/lshoulder_rollV_warnthr",ls_rv_thr);
	n.getParam("/pepper/left_arm_params/lshoulder_rollE_warnthr",ls_re_thr);
	diagnostic_updater::Updater updater;
	ros::Subscriber sub=n.subscribe("/joint_states",1000,leftArmCallBack);
	updater.setHardwareID("Robot-LeftArm");
	updater.add("Left Shoulder roll", posShoulderroll_diagnostic);
	ros::Rate loop_rate(100);
	while(n.ok())
		{
		ros::spinOnce();
		updater.update();
		loop_rate.sleep();
		}
	return 0;
}
