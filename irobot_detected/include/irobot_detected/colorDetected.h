// colorDetected.h
#ifndef COLORDETECTED_H_
#define COLORDETECTED_H_
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>
#include <eigen3/Eigen/Dense>
#include <opencv2/opencv.hpp>
#include <irobot_detected/Pose3D.h>
#include <geometry_msgs/Point32.h>
#include <geometry_msgs/Point.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <vector>
#include <math.h>
#include <boost/concept_check.hpp>
#define Imagecenterx  376
#define Imagecentery  240
using namespace std;
using namespace cv;
using namespace Eigen;

namespace irobotDetected{
class ColorDetected
{
public:
	ros::NodeHandle nh_;
	ros::NodeHandle nh_param;
	ColorDetected(ros::NodeHandle nh);
	~ColorDetected();
	void initialize();
	void drawArrow(cv::Mat& img, cv::Point pStart, cv::Point pEnd, int len, int alpha,cv::Scalar color, int thickness, int lineType);
	ros::Publisher goal_pose_pub;
	ros::Publisher  T_vec_pub;
	ros::Subscriber image_rect_sub;
	ros::Subscriber quadrotorPos_sub;
	ros::Subscriber quaternion_sub;	
	vector<vector<Point2f> > image_color_area_deteced(float boxsize[], cv::Mat & img, vector<vector<Point> > contour1, vector<int> removal);
	vector<vector<Point2f> > image_convex_point(cv::Mat &img, vector<vector<Point2f> > vertex, float boxsize[]);
	void solveIrobotToDronePose(vector<vector<Point2f> > vertex, vector<vector<Point2f> > anglept, double T_vec[]);
	void irobot_global_state();
	void image_rect_callback(const sensor_msgs::ImageConstPtr &msg);
	Eigen::VectorXd Body_to_Global(double Body_arry[], float theta_angle, float Quater[]);
	
private:
	int threshold_param1;
	int blur_param;
	int threshold_size;
	ros::Time startTime;
	ros::Time nowTime;
	ros::Duration timer;
};	
}


#endif