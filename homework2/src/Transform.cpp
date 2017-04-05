#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <iostream>
#include <fstream>
#include <string>
#include "std_msgs/String.h"
#include "sensor_msgs/LaserScan.h"
using namespace std;
///ATTENZIONE: IL FILE VERRÀ CREATO ALL'INTERNO DELLA DIRECTORY IN CUI SI LANCIA IL Tranform.cpp


///creo una typedef callback la quale  passerò nel main come "finta callback"
typedef const boost::function< void(const sensor_msgs::LaserScan::ConstPtr &)>  callback;

///creo una classe con campi la "vera" callback e il listener
class myClass{
  public:
	tf::TransformListener listener;
    void base_scanCallback(const sensor_msgs::LaserScan::ConstPtr& msg );
};

void myClass::base_scanCallback(const sensor_msgs::LaserScan::ConstPtr& msg)
{	
	
	
	tf::StampedTransform transform;
	ofstream myfile;
	

	
    try{
		///passo come argomenti base laser pose w.r.t. odom frame
		listener.lookupTransform("/odom", "/base_laser_link", ros::Time(0), transform);
      ///apertura file txt
		myfile.open ("REPORT_HOMEWORK1.txt",ios::out|ios::app|ios::ate);
      ///scrittura su file
		myfile << "LASER";
		myfile << " x: " << transform.getOrigin().x() ;
		myfile << " y: " << transform.getOrigin().y();
		myfile << " theta: "<< getYaw(transform.getRotation());
		myfile << " stamp: "<< transform.stamp_.toSec()<<"\n";
		myfile.close();

    }
    ///controllo d'errore
    catch (tf::TransformException ex){
      ROS_ERROR("%s",ex.what());
      ros::Duration(1.0).sleep();
    }
    
    
    /*	DEBUGGING
	printf("x: %lf\n", transform.getOrigin().x());
	printf("y: %lf\n", transform.getOrigin().y());
	printf("theta: %lf\n", getYaw(transform.getRotation()));
	printf("stamp: %lf\n", transform.stamp_.toSec());
	*/

	
	
    
  
  
}


int main(int argc, char** argv){
	
	
	
	
	
	///inizializzo nodo,ros e listener
	ros::init(argc, argv, "Transform");
	myClass list;
	
	ros::NodeHandle node;
	///se esiste, viene rimosso il file esistente
	remove("REPORT_HOMEWORK1.txt");
	///creo la callback che prende la "vera" callback e la trasforma
	callback bellaCallback = boost::bind(&myClass::base_scanCallback, &list, _1);
	///sottoscrivo a basescan per leggere i contenuti
	ros::Subscriber sub = node.subscribe("base_scan", 1000, bellaCallback);
	
   
  
	ros::spin();
  
  return 0;
};
