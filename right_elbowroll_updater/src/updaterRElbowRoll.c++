
#include <diagnostic_updater/diagnostic_updater.h>
#include <diagnostic_updater/publisher.h>
#include <sensor_msgs/JointState.h>
#include <cmath>

float posElbowRoll0;
float velElbowRoll0;
float effElbowRoll0;
float posElbowYaw0;

double map[5][3] = {{-2.0857,0.0087,1.4486},
                    {-1.0471,0.0087,1.5620},
                    {0.0,0.0087,1.5620},
		    {1.7366,0.0087,1.36135},
                    {2.0857,0.0087,1.36135}};

float min_re_y= -2.0857;
float max_re_y = 2.0857;

float max_re_rv= 7.5;
float max_re_re = 23.8;


float re_r_thr;
float re_rv_thr;
float re_re_thr;

void leftArmCallBack(const sensor_msgs::JointState::ConstPtr& msg){
	posElbowRoll0= msg -> position[10];
	velElbowRoll0= msg -> velocity[10];
	effElbowRoll0= msg -> effort[10];
	posElbowYaw0= msg -> position[11];
}

int between (float var, float min, float max){
    bool ret = false;
    if(var < max && var >= min) {
        ret = true;
    }
    return ret;
}



void posElbowroll_diagnostic(diagnostic_updater::DiagnosticStatusWrapper &stat){
	float min_re_r;
	float max_re_r;
	for(int i = 0; i<5; i++) {
        	if(between(posElbowYaw0,map[i][0],map[i+1][0])) {
			min_re_r = map[i][1];
			max_re_r = map[i][2];
            		break;
        	}
    	}
	float soglia = fabs(max_re_r-min_re_r)-fabs((max_re_r-min_re_r)*re_r_thr);
	float interval[3] = {min_re_r+soglia ,max_re_r-soglia ,soglia};
	if(posElbowRoll0 <= (max_re_r-interval[2]) && posElbowRoll0>(min_re_r+interval[2])) {
	} else if(((posElbowRoll0 < interval[0] && posElbowRoll0 > min_re_r) || (posElbowRoll0 > interval[1] && posElbowRoll0 < max_re_r)) || (velElbowRoll0 >= max_re_rv*re_rv_thr && velElbowRoll0 <= max_re_rv) || (effElbowRoll0 >= max_re_re*re_re_thr && effElbowRoll0 <= max_re_re)){
		stat.summary(diagnostic_msgs::DiagnosticStatus::WARN,"ATTENZIONE");
	} else {
		stat.summary(diagnostic_msgs::DiagnosticStatus::ERROR,"GUASTO");
	}

	stat.add("Diagnostic name", "RightElbow");
	stat.add("Elbow roll position", posElbowRoll0);
	stat.add("Elbow roll velocity", velElbowRoll0);
	stat.add("Elbow roll effort", effElbowRoll0);
}


int main(int argc, char** argv){
	ros::init(argc, argv, "updaterRightArmState");
	ros::NodeHandle n;
	n.getParam("/pepper/right_arm_params/relbow_roll_warnthr",re_r_thr);
	n.getParam("/pepper/right_arm_params/relbow_rollV_warnthr",re_rv_thr);
	n.getParam("/pepper/right_arm_params/relbow_rollE_warnthr",re_re_thr);
	diagnostic_updater::Updater updater;
	ros::Subscriber sub=n.subscribe("/joint_states",1000,leftArmCallBack);
	updater.setHardwareID("Robot-LeftArm");
	updater.add("Right Elbow roll", posElbowroll_diagnostic);
	ros::Rate loop_rate(100);
	while(n.ok())
		{
		ros::spinOnce();
		updater.update();
		loop_rate.sleep();
		}
	return 0;
}
