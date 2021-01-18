
#include <diagnostic_updater/diagnostic_updater.h>
#include <diagnostic_updater/publisher.h>
#include <sensor_msgs/JointState.h>
#include <cmath>


float posWristYaw0;
float velWristYaw0;
float effWristYaw0;

float min_lw_y = -1.8239;
float max_lw_y = 1.8239;
float max_lw_yv = 1.8239;
float max_lw_ye = 1.8239;


float lw_y_thr;// threshold left wrist yaw warning
float lw_ye_thr;// threshold left wrist yaw warning
float lw_yv_thr;// threshold left wrist yaw warning

void leftArmCallBack(const sensor_msgs::JointState::ConstPtr& msg){
	posWristYaw0= msg ->position[9];
	velWristYaw0= msg ->velocity[9];
	effWristYaw0= msg ->effort[9];
}

void posWristyaw_diagnostic(diagnostic_updater::DiagnosticStatusWrapper &stat){
	if(posWristYaw0 <= (max_lw_y*lw_y_thr) && posWristYaw0 > (min_lw_y*lw_y_thr)) {
		stat.summary(diagnostic_msgs::DiagnosticStatus::OK,"OK");
	} else if(((posWristYaw0 < (min_lw_y*lw_y_thr) && posWristYaw0 > min_lw_y) || (posWristYaw0 > (max_lw_y*lw_y_thr) && posWristYaw0 < max_lw_y)) || (velWristYaw0 >= max_lw_yv*lw_yv_thr && velWristYaw0 < max_lw_yv) || (effWristYaw0>= max_lw_ye*lw_ye_thr && effWristYaw0 < max_lw_ye)) {
		stat.summary(diagnostic_msgs::DiagnosticStatus::WARN,"ATTENZIONE");
	} else {
		stat.summary(diagnostic_msgs::DiagnosticStatus::ERROR,"GUASTO");
	}

	stat.add("Diagnostic name", "LeftWrist");
	stat.add("Shoulder pitch position", posWristYaw0);
	stat.add("Shoulder pitch velocity", velWristYaw0);
	stat.add("Shoulder pitch effort", effWristYaw0);
}


int main(int argc, char** argv){
	ros::init(argc, argv, "updaterLeftArmState");
	ros::NodeHandle n;
	n.getParam("/pepper/left_arm_params/lwrist_yawV_warnthr",lw_yv_thr);
	n.getParam("/pepper/left_arm_params/lwrist_yawE_warnthr",lw_ye_thr);
	n.getParam("/pepper/left_arm_params/lwrist_yaw_warnthr",lw_y_thr);
	diagnostic_updater::Updater updater;
	ros::Subscriber sub=n.subscribe("/joint_states",1000,leftArmCallBack);
	updater.setHardwareID("Robot-LeftArm");
	updater.add("Position Left Wrist roll", posWristyaw_diagnostic);
	ros::Rate loop_rate(100);
	while(n.ok())
		{
		ros::spinOnce();
		updater.update();
		loop_rate.sleep();
		}
	return 0;
}
