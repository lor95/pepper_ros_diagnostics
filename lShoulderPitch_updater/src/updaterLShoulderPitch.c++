
#include <diagnostic_updater/diagnostic_updater.h>
#include <diagnostic_updater/publisher.h>
#include <sensor_msgs/JointState.h>
#include <cmath>

float posShoulderPitch0;
float velShoulderPitch0;
float effShoulderPitch0;

float min_ls_p = -2.0857; // min left shoulder pitch
float max_ls_p = 2.0857; // max left shoulder pitch

float max_ls_pv = 1;

float max_ls_pe = 1;

float ls_p_thr; // threshold left shoulder pitch position warning
float ls_pv_thr; // threshold left shoulder pitch velocity warning
float ls_pe_thr; // threshold left shoulder pitch effort warning


void leftArmCallBack(const sensor_msgs::JointState::ConstPtr& msg){
	//posizioni del braccio
	posShoulderPitch0= msg -> position[7];
	velShoulderPitch0= msg -> velocity[7];
	effShoulderPitch0= msg -> effort[7];
}


void posShoulderpitch_diagnostic(diagnostic_updater::DiagnosticStatusWrapper &stat){
	if(posShoulderPitch0 <= (max_ls_p * ls_p_thr) && posShoulderPitch0>(min_ls_p * ls_p_thr)) {
		stat.summary(diagnostic_msgs::DiagnosticStatus::OK,"OK");
	} else if(((posShoulderPitch0 < (min_ls_p * ls_p_thr) && posShoulderPitch0 > min_ls_p) || (posShoulderPitch0 > (max_ls_p * ls_p_thr) && posShoulderPitch0 < max_ls_p)) || (velShoulderPitch0 >= max_ls_pv*ls_pv_thr) || (effShoulderPitch0 >= max_ls_pe * ls_pe_thr)) {
		stat.summary(diagnostic_msgs::DiagnosticStatus::WARN,"ATTENZIONE");
	} else {
		stat.summary(diagnostic_msgs::DiagnosticStatus::ERROR,"GUASTO");
	}

	stat.add("Diagnostic name", "LeftShoulderPitch");
	stat.add("Shoulder pitch position", posShoulderPitch0);
	stat.add("Shoulder pitch velocity", velShoulderPitch0);
	stat.add("Shoulder pitch velocity", effShoulderPitch0);
}





int main(int argc, char** argv){
	ros::init(argc, argv, "updaterLeftArmState");
	ros::NodeHandle n;
	n.getParam("/pepper/left_arm_params/lshoulder_pitch_warnthr",ls_p_thr);
	n.getParam("/pepper/left_arm_params/lshoulder_pitchV_warnthr",ls_pv_thr);
	n.getParam("/pepper/left_arm_params/lshoulder_pitchE_warnthr",ls_pe_thr);
	diagnostic_updater::Updater updater;
	ros::Subscriber sub=n.subscribe("/joint_states",1000,leftArmCallBack);
	updater.setHardwareID("Robot-LeftArm");
	updater.add("Position Left Shoulder pitch", posShoulderpitch_diagnostic);
	ros::Rate loop_rate(100);
	while(n.ok())
		{
		ros::spinOnce();
		updater.update();
		loop_rate.sleep();
		}
	return 0;
}
