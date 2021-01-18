
#include <diagnostic_updater/diagnostic_updater.h>
#include <diagnostic_updater/publisher.h>
#include <sensor_msgs/JointState.h>
#include <cmath>

float posElbowRoll0;
float velElbowRoll0;
float effElbowRoll0;
float posElbowYaw0;

double map[5][3] = {{-2.0857,-1.36135,-0.0087},
                    {-1.0471,-1.36135,-0.0087},
                    {0.0,-1.5620,-0.0087},
		    {1.7366,-1.5620,-0.0087},
                    {2.0857,-1.4486,-0.0087}};

float min_le_y= -2.0857;
float max_le_y = 2.0857;

float max_le_rv= 7.5;
float max_le_re = 23.8;


float le_r_thr;
float le_rv_thr;
float le_re_thr;

void leftArmCallBack(const sensor_msgs::JointState::ConstPtr& msg){
	posElbowRoll0= msg -> position[5];
	velElbowRoll0= msg -> velocity[5];
	effElbowRoll0= msg -> effort[5];
	posElbowYaw0= msg -> position[6];
}

int between (float var, float min, float max){
    bool ret = false;
    if(var < max && var >= min) {
        ret = true;
    }
    return ret;
}


void posElbowroll_diagnostic(diagnostic_updater::DiagnosticStatusWrapper &stat){
	float min_le_r;
	float max_le_r;
	for(int i = 0; i<5; i++) {
        	if(between(posElbowYaw0,map[i][0],map[i+1][0])) {
			min_le_r = map[i][1];
			max_le_r = map[i][2];
            		break;
        	}
    	}
	float soglia = fabs(max_le_r-min_le_r)-fabs((max_le_r-min_le_r)*le_r_thr);
	float interval[3] = {min_le_r+soglia ,max_le_r-soglia ,soglia};
	if(posElbowRoll0 <= (max_le_r-interval[2]) && posElbowRoll0>(min_le_r+interval[2])) {
	} else if(((posElbowRoll0 < interval[0] && posElbowRoll0 > min_le_r) || (posElbowRoll0 > interval[1] && posElbowRoll0 < max_le_r)) || (velElbowRoll0 >= max_le_rv*le_rv_thr && velElbowRoll0 <= max_le_rv) || (effElbowRoll0 >= max_le_re*le_re_thr && effElbowRoll0 <= max_le_re)){
		stat.summary(diagnostic_msgs::DiagnosticStatus::WARN,"ATTENZIONE");
	} else {
		stat.summary(diagnostic_msgs::DiagnosticStatus::ERROR,"GUASTO");
	}

	stat.add("Diagnostic name", "LeftElbow");
	stat.add("Elbow roll position", posElbowRoll0);
	stat.add("Elbow roll velocity", velElbowRoll0);
	stat.add("Elbow roll effort", effElbowRoll0);
}

int main(int argc, char** argv){
	ros::init(argc, argv, "updaterLeftArmState");
	ros::NodeHandle n;
	n.getParam("/pepper/left_arm_params/lelbow_roll_warnthr",le_r_thr);
	n.getParam("/pepper/left_arm_params/lelbow_rollV_warnthr",le_rv_thr);
	n.getParam("/pepper/left_arm_params/lelbow_rollE_warnthr",le_re_thr);
	diagnostic_updater::Updater updater;
	ros::Subscriber sub=n.subscribe("/joint_states",1000,leftArmCallBack);
	updater.setHardwareID("Robot-LeftArm");
	updater.add("Left Elbow roll", posElbowroll_diagnostic);
	ros::Rate loop_rate(100);
	while(n.ok())
		{
		ros::spinOnce();
		updater.update();
		loop_rate.sleep();
		}
	return 0;
}
