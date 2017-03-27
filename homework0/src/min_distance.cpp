#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "stdlib.h"
#include "std_msgs/String.h"
#include <string>
#include <iostream>

//creo una typedef callback la quale  passerò nel main come "finta callback"
typedef const boost::function< void(const sensor_msgs::LaserScan::ConstPtr &)>  callback;

//creo una classe con campi la "vera" callback e il pub
class myClass{
  public:
	ros::Publisher pub;
    void base_scanCallback(const sensor_msgs::LaserScan::ConstPtr& msg );
};


void myClass::base_scanCallback(const sensor_msgs::LaserScan::ConstPtr& msg)
{
	//trovo minimo
	int n = msg->ranges.size();
	float min = msg->ranges[0];
	for ( int i = 0; i < n; i++)
	{
		if(msg->ranges[i] < min)
			min = msg->ranges[i];
	}
	//metto il minimo dentro il campo "data" del msgs e lo pubblico 
	std_msgs::String ret;
	ret.data = std::to_string(min);
	this->pub.publish(ret); //accedo al campo pub e pubblico
    ROS_INFO("Min : [%lf]", min);
}

int main(int argc, char **argv)
{
   //inizializzo
   ros::init(argc, argv, "listener");
   
   //creo il nodo
   ros::NodeHandle n;
	
	//creo una classe, nel campo "pub" della classe creo implicitamente il topic
   myClass nome;
   nome.pub= n.advertise<std_msgs::String>("xtopic", 1000);
   
   //creo la callback che prende la "vera" callback e la trasforma
   callback bellaCallback = boost::bind(&myClass::base_scanCallback, &nome, _1);
  
   ros::Subscriber sub = n.subscribe("base_scan", 1000,bellaCallback);

   ros::spin();

  return 0;
}
