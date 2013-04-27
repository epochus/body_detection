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

string MAIN_FRAME = "/openni_depth_frame";
string FRAMES[] = {"head_1", "neck_1", "torso_1", "left_shoulder_1", "right_shoulder_1", 
		"left_hand_1", "right_hand_1", "left_elbow_1", "right_elbow_1", "left_hip_1",
	 	"right_hip_1", "left_knee_1", "right_knee_1", "left_foot_1", "right_foot_1"};

 int main(int argc, char** argv) {
 	ros::init(argc, argv, "body_scanning");
 	ros::NodeHandle node;
 	tf::TransformListener listener;

 	while(node.ok()) {

 		tf::StampedTransform tf_head;
 		tf::StampedTransform tf_neck;
 		tf::StampedTransform tf_torso;
 		tf::StampedTransform tf_left_shoulder;
 		tf::StampedTransform tf_right_shoulder;
 		tf::StampedTransform tf_left_hand;
 		tf::StampedTransform tf_right_hand;
 		tf::StampedTransform tf_left_elbow;
 		tf::StampedTransform tf_right_elbow;
 		tf::StampedTransform tf_left_hip;
 		tf::StampedTransform tf_right_hip;
 		tf::StampedTransform tf_left_knee;
 		tf::StampedTransform tf_right_knee;
 		tf::StampedTransform tf_left_foot;
 		tf::StampedTransform tf_right_foot;
 		try {
 			// head
 			listener.lookupTransform(MAIN_FRAME, FRAMES[0], 
 									ros::Time(0), tf_head);

 			double x_head = tf_head.getOrigin().x();
 			double y_head = tf_head.getOrigin().y();
			double z_head = tf_head.getOrigin().z();

			// neck
 			listener.lookupTransform(MAIN_FRAME, FRAMES[1], 
 									ros::Time(0), tf_neck);

 			double x_neck = tf_head.getOrigin().x();
 			double y_neck = tf_head.getOrigin().y();
			double z_neck = tf_head.getOrigin().z();

			// torso
 			listener.lookupTransform(MAIN_FRAME, FRAMES[2], 
 									ros::Time(0), tf_torso);

 			double x_torso = tf_head.getOrigin().x();
 			double y_torso = tf_head.getOrigin().y();
			double z_torso = tf_head.getOrigin().z();

 			// left shoulder
 			listener.lookupTransform(MAIN_FRAME, FRAMES[3], 
 									ros::Time(0), tf_left_shoulder);

 			double x_left_shoulder = tf_head.getOrigin().x();
 			double y_left_shoulder = tf_head.getOrigin().y();
			double z_left_shoulder = tf_head.getOrigin().z();

 			// right shoulder
 			listener.lookupTransform(MAIN_FRAME, FRAMES[4], 
 									ros::Time(0), tf_right_shoulder);

 			double x_right_shoulder = tf_head.getOrigin().x();
 			double y_right_shoulder = tf_head.getOrigin().y();
			double z_right_shoulder = tf_head.getOrigin().z();

 			// left hand
 			listener.lookupTransform(MAIN_FRAME, FRAMES[5], 
 									ros::Time(0), tf_left_hand);

 			double x_left_hand = tf_head.getOrigin().x();
 			double y_left_hand = tf_head.getOrigin().y();
			double z_left_hand = tf_head.getOrigin().z();

 			// right hand
 			listener.lookupTransform(MAIN_FRAME, FRAMES[6], 
 									ros::Time(0), tf_right_hand);

 			double x_right_hand = tf_head.getOrigin().x();
 			double y_right_hand = tf_head.getOrigin().y();
			double z_right_hand = tf_head.getOrigin().z();

 			// left elbow
 			listener.lookupTransform(MAIN_FRAME, FRAMES[7], 
 									ros::Time(0), tf_left_elbow);

 			double x_left_elbow = tf_head.getOrigin().x();
 			double y_left_elbow = tf_head.getOrigin().y();
			double z_left_elbow = tf_head.getOrigin().z();

 			// right elbow
 			listener.lookupTransform(MAIN_FRAME, FRAMES[8], 
 									ros::Time(0), tf_right_elbow);

 			double x_right_elbow = tf_head.getOrigin().x();
 			double y_right_elbow = tf_head.getOrigin().y();
			double z_right_elbow = tf_head.getOrigin().z();

 			// left hip
 			listener.lookupTransform(MAIN_FRAME, FRAMES[9], 
 									ros::Time(0), tf_left_hip);

 			double x_left_hip = tf_head.getOrigin().x();
 			double y_left_hip = tf_head.getOrigin().y();
			double z_left_hip = tf_head.getOrigin().z();

 			// right hip
 			listener.lookupTransform(MAIN_FRAME, FRAMES[10], 
 									ros::Time(0), tf_right_hip);

 			double x_right_hip = tf_head.getOrigin().x();
 			double y_right_hip = tf_head.getOrigin().y();
			double z_right_hip = tf_head.getOrigin().z();

 			// left knee
 			listener.lookupTransform(MAIN_FRAME, FRAMES[11], 
 									ros::Time(0), tf_left_knee);

 			double x_left_knee = tf_head.getOrigin().x();
 			double y_left_knee = tf_head.getOrigin().y();
			double z_left_knee = tf_head.getOrigin().z();

 			// right knee
 			listener.lookupTransform(MAIN_FRAME, FRAMES[12], 
 									ros::Time(0), tf_right_knee);

 			double x_right_knee = tf_head.getOrigin().x();
 			double y_right_knee = tf_head.getOrigin().y();
			double z_right_knee = tf_head.getOrigin().z();

 			// left foot
 			listener.lookupTransform(MAIN_FRAME, FRAMES[13], 
 									ros::Time(0), tf_left_foot);

 			double x_left_foot = tf_head.getOrigin().x();
 			double y_left_foot = tf_head.getOrigin().y();
			double z_left_foot = tf_head.getOrigin().z();

 			// right foot
 			listener.lookupTransform(MAIN_FRAME, FRAMES[14], 
 									ros::Time(0), tf_right_foot);

 			double x_right_foot = tf_head.getOrigin().x();
 			double y_right_foot = tf_head.getOrigin().y();
			double z_right_foot = tf_head.getOrigin().z();

			// End of body transforms

 			ofstream outfile("info.txt", ios_base::binary);
 			outfile << "You are " << x_torso << " away";
 			outfile.close();

 		}
 		catch (tf::TransformException ex) {
 			ROS_ERROR("%s", ex.what());
 		}
 	}
 	return 0;
 }
