#include <ros/ros.h>

#include "sensor_msgs/LaserScan.h"

#include <actionlib/server/simple_action_server.h>
#include <projectAI/goalistAction.h>
#include <std_msgs/Int32.h>
#include <geometry_msgs/Twist.h>

#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>
#include <tf/message_filter.h>

#define pi 3.141592653589793238462643383279502884
#define SAFE_DIST 0.4

double const_angular_speed = 0.4;
double const_linear_speed = 0.3;
bool obstacle;


class GoalistAction {
	enum state_1 {activated, moving, arrived};			//Manage visit
	enum state_2 {rotating, finished};					//Manage rotation	
	
	protected:
		ros::NodeHandle nh_;
		tf::TransformListener listener_;
		
		actionlib::SimpleActionServer<projectAI::goalistAction> as_;
		std::string action_name_;
		
		projectAI::goalistResult result_;
		
		ros::Publisher cmd_vel_pub_;									
		
		
		state_1 state_1_;			//Manage visit									
		state_2 state_2_;			//Manage rotation									
		
		///----The objective of this parms is to iterate the goal list-----------///
		geometry_msgs::Point current_start_point_;						
		geometry_msgs::Point current_destination_point_;				
		
		double arrived_displacement_;		//Euclidean distance
		
		ros::Subscriber button_sub_;		//Button to manage Marrtino
		int button;
		
		ros::Subscriber scan_sub;
		
		double final_theta_;				//Final angulation

	public:
		GoalistAction(std::string name) :
			as_(nh_, name, boost::bind(&GoalistAction::execute, this, _1), false),
			action_name_(name) {
			state_1_ = arrived;
			state_2_ = finished;
			
			cmd_vel_pub_ = nh_.advertise<geometry_msgs::Twist>("cmd_vel", 10);
			button_sub_ = nh_.subscribe("joy_control/button", 1000, &GoalistAction::set_button, this);
			
			scan_sub = nh_.subscribe("base_scan", 1000, &GoalistAction::near_obstacle, this);
			
			obstacle = false;
			as_.start();
		}

		~GoalistAction(void) {
		}
		
		static double eulerianDist(geometry_msgs::Point p1, geometry_msgs::Point p2) {
			return sqrt(pow(p1.x - p2.x, 2) + pow(p1.y - p2.y, 2));
		}
		
		static void to360(double& angle) {
			while(angle < 0)
				angle += 2.0 * pi;
			while(angle > 2.0 * pi)
				angle -= 2.0 * pi;
		}
		
		static void mapto360(double& angle) {
			angle += pi * (5.0 / 2.0);
		}
		
		void set_button(const std_msgs::Int32& msg) {
			button = msg.data;
			if (button == 1) std::cout << "Joypad said me to run\n";
			if (button == 2) std::cout << "Joypad said me to stop\n";
			
			
		}
		
		
		/////////////////////////////////////////////////////////////////////////////////////////////
		//---------
		void near_obstacle(const sensor_msgs::LaserScan::ConstPtr& msg){
			if (obstacle) return ;

			//trovo minimo
			int n = msg->ranges.size();
			float min = msg->ranges[0];
			for ( int i = 0; i < n; i++){
				if(msg->ranges[i] < min)
					min = msg->ranges[i];
			}
			

			if (min < SAFE_DIST ) {
				std::cout << "\nThere is an obstacles nearby, at: " << min << "\n";
				obstacle = true;
				return;
			}
			obstacle = false;
			
			
		}
		
		/////////////////////////////////////////////////////////////////////////////////////////////
		
		//----- actuator_1() calcola velocità e angolo con cui giungere a destinazione e fa un checking dell'enum state_1 -------//
		
		void actuator_1() {		
			if (obstacle) return ;

			
			if(state_1_ == arrived || button == 2)
				return;
			
				
			tf::StampedTransform robot_position;
			try {
				listener_.waitForTransform("/map", "/base_link", ros::Time(0), ros::Duration(3.0));
				listener_.lookupTransform("/map", "/base_link", ros::Time(0), robot_position);
			}
			catch(tf::TransformException ex) {
				ROS_ERROR("%s", ex.what());
				ros::Duration(1.0).sleep();
			}
				
				
			/* ------- ANGOLO DESIDERATO ------- */
			double s3n = current_destination_point_.x - robot_position.getOrigin().x();
			double c0s = - current_destination_point_.y + robot_position.getOrigin().y();
			if(c0s == 0) c0s += 0.01;
			
			double desired_angle = atan(s3n / c0s);
			to360(desired_angle);
			
			if(c0s < 0)													// il codominio dell'arcotangente è [-90, 90] gradi
				if(s3n < 0)
					desired_angle += pi;
				else
					desired_angle -= pi;
			
			/* ------- ANGOLO ATTUALE ------- */
			double current_angle = tf::getYaw(robot_position.getRotation());
			mapto360(current_angle);
			to360(current_angle);
			
			/* ------- SFASAMENTO ANGOLARE ------- */
			double angular_displacement_1 = std::abs(desired_angle - current_angle);
			double angular_displacement_2 = std::abs(angular_displacement_1 - (pi * 2.0));
			double angular_displacement = !(angular_displacement_1 < angular_displacement_2)?angular_displacement_2 : angular_displacement_1;
			
			/* ------- VELOCITA' ANGOLARE ------- */
			double angular_speed = const_angular_speed;
			
			double opp = current_angle + pi;							// calcolo angolo opposto a quello attuale
			to360(opp);
			
			if(current_angle <= pi)										// faccio prima a girarmi a sinistra o a destra?
				if(desired_angle > current_angle && desired_angle < opp)
					angular_speed = angular_speed; // a sx
				else
					angular_speed = -angular_speed; // a dx
			else
				if(desired_angle < current_angle && desired_angle > opp)
					angular_speed = -angular_speed; // a dx
				else 
					angular_speed = angular_speed; // a sx
			
			/* ------- VELOCITA' LINEARE ------- */
			double linear_speed = 0.0;
			if(angular_displacement < (pi / 6.0))
				linear_speed = const_linear_speed;
			
			if(angular_displacement < (pi / 16)) 
				angular_speed = 0.0;
			
			geometry_msgs::Twist motion_msg;
			motion_msg.linear.x = linear_speed;
			motion_msg.angular.z = angular_speed;
			
			///-------Checking state_1-----------///
			switch(state_1_) {
				case activated: {
					current_start_point_.x = robot_position.getOrigin().x();
					current_start_point_.y = robot_position.getOrigin().y();
					
					arrived_displacement_ = eulerianDist(current_start_point_, current_destination_point_);
					state_1_ = moving;
					
					break;
				}
				
				case moving: {
					geometry_msgs::Point pose;
					pose.x = robot_position.getOrigin().x();
					pose.y = robot_position.getOrigin().y();
					

					
					double linear_displacement = eulerianDist(pose, current_start_point_);
					
					if(linear_displacement > arrived_displacement_) {
						std::cout << "Real x: " << robot_position.getOrigin().x() << " y: " << robot_position.getOrigin().y() << "\n";
						state_1_ = arrived;
						state_2_ = rotating;
						
						std::cout << "i'm rotating to the original position \n";
						while(state_2_ != finished)
							actuator_2();					
						
						std::cout << "sleeping \n";
						ros::Duration(5).sleep();
						std::cout << "i'm awake \n";
					}
					else						
						cmd_vel_pub_.publish(motion_msg);
						
					
					break;
				}
				case arrived: {
					break;
				}
				default: {}
			}
			
		}
		
		
		//----actuator_2() calcola velocità e angolo con cui ruotare una volta giunto alla fine e fa il checking dell'enum state_2
		void actuator_2() {
			if (obstacle) return ;

			
			if(state_2_ == finished || button == 2)
				return;
				
			tf::StampedTransform robot_position;
			try {
				listener_.waitForTransform("/map", "/base_link", ros::Time(0), ros::Duration(3.0));
				listener_.lookupTransform("/map", "/base_link", ros::Time(0), robot_position);
			}
			catch(tf::TransformException ex) {
				ROS_ERROR("%s", ex.what());
				ros::Duration(1.0).sleep();
			}			
			
			/* ------- ANGOLO DESIDERATO ------- */
			double desired_angle = final_theta_;
			mapto360(desired_angle);
			to360(desired_angle);
			
			/* ------- ANGOLO ATTUALE ------- */
			double current_angle = tf::getYaw(robot_position.getRotation());
			mapto360(current_angle);
			to360(current_angle);
			
			/* ------- SFASAMENTO ANGOLARE ------- */
			double angular_displacement_1 = std::abs(desired_angle - current_angle);
			double angular_displacement_2 = std::abs(angular_displacement_1 - (pi * 2.0));
			double angular_displacement = !(angular_displacement_1 < angular_displacement_2)?angular_displacement_2 : angular_displacement_1;
			
			/* ------- VELOCITA' ANGOLARE ------- */
			double angular_speed = const_angular_speed;
			
			double opp = current_angle + pi;							// calcolo angolo opposto a quello attuale
			to360(opp);
			
			if(current_angle <= pi)										// faccio prima a girarmi a sinistra o a destra?
				if(desired_angle > current_angle && desired_angle < opp)
					angular_speed = angular_speed; // a sx
				else
					angular_speed = -angular_speed; // a dx
			else
				if(desired_angle < current_angle && desired_angle > opp)
					angular_speed = -angular_speed; // a dx
				else 
					angular_speed = angular_speed; // a sx
			
			geometry_msgs::Twist motion_msg;
			motion_msg.angular.z = angular_speed;
			
			///----Checking state_2()---------///
			switch(state_2_) {
				case rotating: {						
					if(angular_displacement < (pi * 0.05)) {
						std::cout << "Real theta: " << getYaw(robot_position.getRotation()) << "\n";
						state_2_ = finished;
					}
					else
						cmd_vel_pub_.publish(motion_msg);
					
					break;
				}
				case finished: {
					break;
				}
				default: {}
			}
			
		}
		void execute(const projectAI::goalistGoalConstPtr& goal) {
			bool success = true;
			std::cout << "VISITING " << goal->goals_number << " VERTEXES\n";
			
			
			for(int i = 0; i < goal->goals_number; i++) {
				current_destination_point_ = goal->goals_list[i];
				
				std::cout << "Destination x: " << current_destination_point_.x << " y: " << current_destination_point_.y << "\n";
				
				state_1_ = activated;
				
				///-----While i'm not arrived, i will execute actuator1()----///
				while(state_1_ != arrived){
					
					if(obstacle){
						std::cout << action_name_.c_str() << " found an obstacle, closing demo\n";
						as_.setPreempted();
						success = false;
						return;
					}
					actuator_1();
					
				}
					
				
				if(as_.isPreemptRequested() || !ros::ok()) {
					std::cout << action_name_.c_str() << ": Preempted\n";
					as_.setPreempted();
					success = false;
					break;
				}
			}
			
			std::cout << "\nFINAL ROTATION\n";
			final_theta_ = goal->final_theta;
			std::cout << "Final theta: " << final_theta_ << "\n";
			
			
			///-----While i haven't the right rotation, i will execute actuator2()----///
			state_2_ = rotating;
			while(state_2_ != finished)
				actuator_2();
			
			if(as_.isPreemptRequested() || !ros::ok()) {
				std::cout << action_name_.c_str() << ": Preempted\n";
				as_.setPreempted();
				success = false;
			}
			
			if(success) {
				std::cout << action_name_.c_str() << ": Succeeded\n";
				as_.setSucceeded(result_);
			}
			
			std::cout << "__________________________________\n\n";
		}
};







int main(int argc, char** argv) {
	
	ros::init(argc, argv, "server");
	

	GoalistAction nav("server");
	
	ros::spin();

	return 0;
}
