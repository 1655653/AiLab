#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "stdlib.h"
#include "std_msgs/String.h"
#include <string>
#include <iostream>
#include <math.h>
#include "tf/transform_listener.h"
#include "sensor_msgs/PointCloud.h"
#include "tf/message_filter.h"
#include "message_filters/subscriber.h"
#include "laser_geometry/laser_geometry.h"
#include "opencv/cv.h"
#include "opencv/highgui.h"
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/features2d/features2d.hpp>
void callback(const sensor_msgs::LaserScan::ConstPtr& input){
	
	using namespace cv;
	using namespace std;
	
	int img_size = 800;
	int scalar = 20;
	
	Mat img(img_size, img_size, CV_8UC3, Scalar(255, 255, 255));
	
	const Scalar black = Scalar(0);
	
	float theta = input->angle_min;
	float min_theta;
	
	float min = (input->ranges)[0];
	float distance;
	
	Point p1 = Point((img_size/2) + scalar*min*cos(theta), (img_size/2) + scalar*min*sin(theta));
	Point p2;
	
	int size = (input->ranges).size();
	
	for(int i = 1; i < size; ++i){
		
		distance = (input->ranges)[i];
		
		if(distance < min) {
			min = distance;
			min_theta = theta;
		}
	
		theta = theta + input->angle_increment;
		
		p2 = Point((img_size/2) + scalar*distance*cos(theta), (img_size/2) + scalar*distance*sin(theta));
		
		line(img, p1, p2, Scalar(0, 0, 0), 1, LINE_8 , 0);
		
		p1 = p2;
		
	}
	
	circle(img, Point((img_size/2) + scalar*min*cos(min_theta), (img_size/2) + scalar*min*sin(min_theta)), 5, Scalar(0, 0, 255), 1, LINE_8, 0);
	imshow("Shortest distance", img);
	waitKey(1);
  
}

int main(int argc, char **argv){

	ros::init(argc, argv, "find_near_obstacle");
	ros::NodeHandle n;
	ros::Subscriber sub = n.subscribe<sensor_msgs::LaserScan>("base_scan", 1000, callback);
	ros::spin();

  return 0;
}
