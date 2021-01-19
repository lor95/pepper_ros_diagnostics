
#include <diagnostic_updater/diagnostic_updater.h>
#include <diagnostic_updater/publisher.h>
#include <sensor_msgs/JointState.h>
#include <cmath>


float posElbowYaw0;
float velElbowYaw0;
float effElbowYaw0;


float min_re_y= -2.0857;
float max_re_y = 2.0857;

float max_re_yv = 5.9;
float max_re_ye = 42.9;

float re_y_thr;// threshold left elbow yaw warning
float re_yv_thr;// threshold left elbow yaw warning
float re_ye_thr;// threshold left elbow yaw warning


void rightArmCallBack(const sensor_msgs::JointState::ConstPtr& msg){
	posElbowYaw0= msg -> position[11];
	velElbowYaw0= msg -> position[11];
	effElbowYaw0= msg -> position[11];	
}

int between (float var, float min, float max){
    bool ret = false;
    if(var < max && var >= min) {
        ret = true;
    }
    return ret;
}

void posElbowyaw_diagnostic(diagnostic_updater::DiagnosticStatusWrapper &stat){
	float soglia = fabs(max_re_y-min_re_y)-fabs((max_re_y-min_re_y)*re_y_thr);
	float interval[3] = {min_re_y+soglia ,max_re_y-soglia ,soglia};
	if(posElbowYaw0 <= (max_re_y-interval[2]) && posElbowYaw0>(min_re_y+interval[2])) {
		stat.summary(diagnostic_msgs::DiagnosticStatus::OK,"OK");
	} else if(((posElbowYaw0 < interval[0] && posElbowYaw0 > min_re_y) || (posElbowYaw0 > interval[1] && posElbowYaw0 < max_re_y)) || (velElbowYaw0 >= max_re_yv*re_yv_thr && velElbowYaw0 <= max_re_yv) || (effElbowYaw0>=re_ye_thr*max_re_ye && effElbowYaw0 <= max_re_ye)){
		stat.summary(diagnostic_msgs::DiagnosticStatus::WARN,"ATTENZIONE");
	} else {
		stat.summary(diagnostic_msgs::DiagnosticStatus::ERROR,"GUASTO");
	}

	stat.add("Diagnostic name", "RightElbow");
	stat.add("Elbow yaw position", posElbowYaw0);
	stat.add("Elbow yaw velocity", velElbowYaw0);
	stat.add("Elbow yaw effort", effElbowYaw0);
}


int main(int argc, char** argv){
	ros::init(argc, argv, "updaterRightArmState");
	ros::NodeHandle n;
	n.getParam("/pepper/left_arm_params/lelbow_yawE_warnthr",re_ye_thr);
	n.getParam("/pepper/left_arm_params/lelbow_yaw_warnthr",re_y_thr);
	n.getParam("/pepper/left_arm_params/lelbow_yawV_warnthr",re_yv_thr);
	diagnostic_updater::Updater updater;
	ros::Subscriber sub=n.subscribe("/joint_states",1000,rightArmCallBack);
	updater.setHardwareID("Robot-RightArm");
	updater.add("Right Elbow yaw", posElbowyaw_diagnostic);
	ros::Rate loop_rate(100);
	while(n.ok())
		{
		ros::spinOnce();
		updater.update();
		loop_rate.sleep();
		}
	return 0;
}
