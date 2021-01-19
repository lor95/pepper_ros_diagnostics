
#include <diagnostic_updater/diagnostic_updater.h>
#include <diagnostic_updater/publisher.h>
#include <sensor_msgs/JointState.h>
#include <cmath>


float posWristYaw0;
float velWristYaw0;
float effWristYaw0;

float min_rw_y = -1.8239;
float max_rw_y = 1.8239;
float max_rw_yv = 17.3;
float max_rw_ye = 4.9;


float rw_y_thr;// threshold left wrist yaw warning
float rw_ye_thr;// threshold left wrist yaw warning
float rw_yv_thr;// threshold left wrist yaw warning

void rightArmCallBack(const sensor_msgs::JointState::ConstPtr& msg){
	posWristYaw0= msg ->position[14];
	velWristYaw0= msg ->velocity[14];
	effWristYaw0= msg ->effort[14];
}

void posWristyaw_diagnostic(diagnostic_updater::DiagnosticStatusWrapper &stat){
	if(posWristYaw0 <= (max_rw_y*rw_y_thr) && posWristYaw0 > (min_rw_y*rw_y_thr)) {
		stat.summary(diagnostic_msgs::DiagnosticStatus::OK,"OK");
	} else if(((posWristYaw0 < (min_rw_y*rw_y_thr) && posWristYaw0 > min_rw_y) || (posWristYaw0 > (max_rw_y*rw_y_thr) && posWristYaw0 < max_rw_y)) || (velWristYaw0 >= max_rw_yv*rw_yv_thr && velWristYaw0 < max_rw_yv) || (effWristYaw0>= max_rw_ye*rw_ye_thr && effWristYaw0 < max_rw_ye)) {
		stat.summary(diagnostic_msgs::DiagnosticStatus::WARN,"ATTENZIONE");
	} else {
		stat.summary(diagnostic_msgs::DiagnosticStatus::ERROR,"GUASTO");
	}

	stat.add("Diagnostic name", "RightWrist");
	stat.add("Wrist Yaw position", posWristYaw0);
	stat.add("Wrist Yaw velocity", velWristYaw0);
	stat.add("Wrist Yaw effort", effWristYaw0);
}


int main(int argc, char** argv){
	ros::init(argc, argv, "updaterRightArmState");
	ros::NodeHandle n;
	n.getParam("/pepper/right_arm_params/rwrist_yawV_warnthr",rw_yv_thr);
	n.getParam("/pepper/right_arm_params/rwrist_yawE_warnthr",rw_ye_thr);
	n.getParam("/pepper/right_arm_params/rwrist_yaw_warnthr",rw_y_thr);
	diagnostic_updater::Updater updater;
	ros::Subscriber sub=n.subscribe("/joint_states",1000,rightArmCallBack);
	updater.setHardwareID("Robot-RightArm");
	updater.add("Right Wrist Yaw", posWristyaw_diagnostic);
	ros::Rate loop_rate(100);
	while(n.ok())
		{
		ros::spinOnce();
		updater.update();
		loop_rate.sleep();
		}
	return 0;
}
