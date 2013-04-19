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
string FRAMES[] = {"head", "neck", "torso", "left_shoulder", "right_shoulder", 
				   "left_hand", "right_hand", "left_elbow", "right_elbow", "left_hip",
				   "right_hip", "left_knee", "right_knee", "left_foot", "right_foot"};

 int main(int argc, char** argv) {
 	ros::init(argc, argv, "body_scanning");
 	ros::NodeHandle node;
 	tf::TransformListener listener;

 	while(node.ok()) {

 		tf::StampedTransform transform;
 		try {
 			listener.lookupTransform("/openni_depth_frame", "head", 
 									ros::Time(0), transform);

 			

 			//writing to text file
 			/* ofstream outfile("info.txt", ios_base::binary);
 			string str = "You are " + [pointx] " away";
 			outfile << str.c_str();
 			outfile.close();
 			*/

 		}
 		catch (tf::TransformException ex) {
 			ROS_ERROR("%s", ex.what());
 		}
 	}
 	return 0;
 }