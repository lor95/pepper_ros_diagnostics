
#include <diagnostic_updater/diagnostic_updater.h>
#include <diagnostic_updater/publisher.h>
#include <sensor_msgs/JointState.h>
#include <cmath>

float posShoulderRoll0;
float velShoulderRoll0;
float effShoulderRoll0;

float min_rs_r = -1.5620; // min left shoulder roll
float max_rs_r = -0.0087; // max left shoulder roll

float max_rs_rv = 7.5;

float max_rs_re = 23.8;

float rs_r_thr; // threshold left shoulder pitch position warning
float rs_rv_thr; // threshold left shoulder pitch velocity warning
float rs_re_thr; // threshold left shoulder pitch effort warning


void rightArmCallBack(const sensor_msgs::JointState::ConstPtr& msg){
	//posizioni del braccio
	posShoulderRoll0= msg -> position[13];
	velShoulderRoll0= msg -> velocity[13];
	effShoulderRoll0= msg -> effort[13];
}


void posShoulderroll_diagnostic(diagnostic_updater::DiagnosticStatusWrapper &stat){
	float soglia = fabs(max_rs_r-min_rs_r)-fabs((max_rs_r-min_rs_r)*rs_r_thr);
	float interval[3] = {min_rs_r+soglia ,max_rs_r-soglia ,soglia};
	if(posShoulderRoll0 <= (interval[1]) && posShoulderRoll0>(interval[0])) {
		stat.summary(diagnostic_msgs::DiagnosticStatus::OK,"OK");
	} else if((posShoulderRoll0 < interval[0] && posShoulderRoll0 > min_rs_r) || (posShoulderRoll0 > interval[1] && posShoulderRoll0 < max_rs_r)) {
		stat.summary(diagnostic_msgs::DiagnosticStatus::WARN,"ATTENZIONE");
	} else {
		stat.summary(diagnostic_msgs::DiagnosticStatus::ERROR,"GUASTO");
	}
	stat.add("Diagnostic name", "RightShoulderRoll");
	stat.add("Shoulder pitch position", posShoulderRoll0);
	stat.add("Shoulder pitch velocity", velShoulderRoll0);
	stat.add("Shoulder pitch effort", effShoulderRoll0);
}


int main(int argc, char** argv){
	ros::init(argc, argv, "updaterLeftArmState");
	ros::NodeHandle n;
	n.getParam("/pepper/right_arm_params/rshoulder_roll_warnthr",rs_r_thr);
	n.getParam("/pepper/right_arm_params/rshoulder_rollV_warnthr",rs_rv_thr);
	n.getParam("/pepper/right_arm_params/rshoulder_rollE_warnthr",rs_re_thr);
	diagnostic_updater::Updater updater;
	ros::Subscriber sub=n.subscribe("/joint_states",1000,rightArmCallBack);
	updater.setHardwareID("Robot-RightArm");
	updater.add("Right Shoulder Roll", posShoulderroll_diagnostic);
	ros::Rate loop_rate(100);
	while(n.ok())
		{
		ros::spinOnce();
		updater.update();
		loop_rate.sleep();
		}
	return 0;
}
