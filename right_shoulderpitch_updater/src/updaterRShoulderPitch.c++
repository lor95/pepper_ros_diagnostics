
#include <diagnostic_updater/diagnostic_updater.h>
#include <diagnostic_updater/publisher.h>
#include <sensor_msgs/JointState.h>
#include <cmath>

float posShoulderPitch0;
float velShoulderPitch0;
float effShoulderPitch0;

float min_rs_p = -2.0857; // min left shoulder pitch
float max_rs_p = 2.0857; // max left shoulder pitch

float max_rs_pv = 5.9;

float max_rs_pe = 42.9;

float rs_p_thr; // threshold left shoulder pitch position warning
float rs_pv_thr; // threshold left shoulder pitch velocity warning
float rs_pe_thr; // threshold left shoulder pitch effort warning


void rightArmCallBack(const sensor_msgs::JointState::ConstPtr& msg){
	//posizioni del braccio
	posShoulderPitch0= msg -> position[12];
	velShoulderPitch0= msg -> velocity[12];
	effShoulderPitch0= msg -> effort[12];
}


void posShoulderpitch_diagnostic(diagnostic_updater::DiagnosticStatusWrapper &stat){
	if(posShoulderPitch0 <= (max_rs_p * rs_p_thr) && posShoulderPitch0>(min_rs_p * rs_p_thr)) {
		stat.summary(diagnostic_msgs::DiagnosticStatus::OK,"OK");
	} else if(((posShoulderPitch0 < (min_rs_p * rs_p_thr) && posShoulderPitch0 > min_rs_p) || (posShoulderPitch0 > (max_rs_p * rs_p_thr) && posShoulderPitch0 < max_rs_p)) || (velShoulderPitch0 >= max_rs_pv*rs_pv_thr) || (effShoulderPitch0 >= max_rs_pe * rs_pe_thr)) {
		stat.summary(diagnostic_msgs::DiagnosticStatus::WARN,"ATTENZIONE");
	} else {
		stat.summary(diagnostic_msgs::DiagnosticStatus::ERROR,"GUASTO");
	}

	stat.add("Diagnostic name", "RightShoulderPitch");
	stat.add("Shoulder pitch position", posShoulderPitch0);
	stat.add("Shoulder pitch velocity", velShoulderPitch0);
	stat.add("Shoulder pitch velocity", effShoulderPitch0);
}





int main(int argc, char** argv){
	ros::init(argc, argv, "updaterLeftArmState");
	ros::NodeHandle n;
	n.getParam("/pepper/right_arm_params/rshoulder_pitch_warnthr",rs_p_thr);
	n.getParam("/pepper/right_arm_params/rshoulder_pitchV_warnthr",rs_pv_thr);
	n.getParam("/pepper/right_arm_params/rshoulder_pitchE_warnthr",rs_pe_thr);
	diagnostic_updater::Updater updater;
	ros::Subscriber sub=n.subscribe("/joint_states",1000,rightArmCallBack);
	updater.setHardwareID("Robot-RightArm");
	updater.add("Right Shoulder pitch", posShoulderpitch_diagnostic);
	ros::Rate loop_rate(100);
	while(n.ok())
		{
		ros::spinOnce();
		updater.update();
		loop_rate.sleep();
		}
	return 0;
}
