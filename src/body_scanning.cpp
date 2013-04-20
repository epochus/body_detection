/* Retrieves transform data from openni_tracker and puts it into a text file.
 *
 * This was created for the BWI Body Scanning Project for CS378.
 * Author: Zhen Guang Wu
 *
 * */

#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <fstream>
using namespace std;

string BASE_FRAME = "/openni_depth_frame";
string FRAMES[] = {"head_1", "neck_1", "torso_1", "left_shoulder_1", "right_shoulder_1", 
		"left_hand_1", "right_hand_1", "left_elbow_1", "right_elbow_1", "left_hip_1",
	 	"right_hip_1", "left_knee_1", "right_knee_1", "left_foot_1", "right_foot_1"};

 int main(int argc, char** argv) {
 	ros::init(argc, argv, "body_scanning");
 	ros::NodeHandle node;
 	tf::TransformListener listener;

 	while(node.ok()) {

 		tf::StampedTransform tf_head;
 		try {
 			listener.lookupTransform("/openni_depth_frame", "head_1", 
 									ros::Time(0), tf_head);

 			double x_head = tf_head.getOrigin().x();
 			double y_head = tf_head.getOrigin().y();
			double z_head = tf_head.getOrigin().z();

 			float a = 0.0;
 			a = z_head;
 			
 			ofstream outfile("info.txt", ios_base::binary);
 			outfile << "You are " << a << " away";
 			outfile.close();

 		}
 		catch (tf::TransformException ex) {
 			ROS_ERROR("%s", ex.what());
 		}
 	}
 	return 0;
 }
