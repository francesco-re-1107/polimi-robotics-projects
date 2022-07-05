#include "ros/ros.h"
#include "sensor_msgs/JointState.h"
#include "geometry_msgs/TwistStamped.h"
#include <robotics_project_1/calibrationConfig.h>
#include "robotics_project_1/WheelsRpm.h"
#include <dynamic_reconfigure/server.h>
#include <math.h>
#include "robotics_project_1/Reset.h"

#define T 5 //gear ratio

/**
 * This node:
 * 		1. Subscribes to /wheel_states topic
 * 		2. Reads rpm and ticks 
 * 		3. Apply kinematics to calculate linear and angular velocity
 * 		4. Publish the calculated velocities to /cmd_vel topic
 */
class Kinematics {
	private:
		ros::NodeHandle nh;
		ros::Subscriber wheelStatesSub;
		ros::Publisher cmdVelPub;
		ros::Publisher rpmPub;
		ros::ServiceServer resetService;
		
		std::vector<double> wheelsSpeed{0, 0, 0, 0}; //fl, fr, rl, rr  rad/s
		std::vector<double> prevTicks{0, 0, 0, 0}; //fl, fr, rl, rr

		std::vector<double> calculatedLinearVelocity{0, 0}; //x, y  m/s
		double calculatedAngularVelocity; //ω rad/s
		ros::Time lastUpdate;

		double cpr;
		double wheels_radius;
		double width;
		double length;

		bool firstMessage = true;

		//dynamic reconfigure for calibration
		dynamic_reconfigure::Server<robotics_project_1::calibrationConfig> dynServer;
		dynamic_reconfigure::Server<robotics_project_1::calibrationConfig>::CallbackType dynCallback;


	public:

		Kinematics() {
			//initialize wheel states subscriber
			this->wheelStatesSub = this->nh.subscribe(
				"wheel_states",
				1000, 
				&Kinematics::wheelStatesCallback,
				this
			);

			this->lastUpdate = ros::Time::now();

			this->firstMessage = true;

			//initialize calculated velocity publisher
			this->cmdVelPub = this->nh.advertise<geometry_msgs::TwistStamped>("cmd_vel", 1000);

			//initialize wheels rpm (calculated with inverse kinematics) publisher
			this->rpmPub = this->nh.advertise<robotics_project_1::WheelsRpm>("wheels_rpm", 1000);

			//reset service
			this->resetService = this->nh.advertiseService("reset_kinematics", &Kinematics::resetServiceCallback, this);

			//dynamic reconfigure for calibration
			dynCallback = boost::bind(&Kinematics::calibrationParamsChangeCallback, this, _1, _2);
			dynServer.setCallback(dynCallback);
		}

		void run() {
			ros::spin();
		}

		/**
		 * This method is called when the /reset_kinematics service is called
		 */
		bool resetServiceCallback(robotics_project_1::Reset::Request &req, robotics_project_1::Reset::Response &res) {
			ROS_INFO("Reset variables in kinematics");

			this->prevTicks = {0,0,0,0};
			this->lastUpdate = ros::Time::now();
			this->firstMessage = true;
			this->calculatedLinearVelocity = {0, 0};
			this->calculatedAngularVelocity = 0;

			return true;
		}

		/**
		 * This function is called when any of the calibration param is changed
		 */
		void calibrationParamsChangeCallback(robotics_project_1::calibrationConfig &config, uint32_t level) {
			this->cpr = config.cpr;
			this->wheels_radius = config.wheels_radius;
			this->width = config.width;
			this->length = config.length;

			ROS_INFO("Change calibration params [CPR: %f; WR: %f; W: %f; L: %f]", cpr, wheels_radius, width, length);
		}

		/**
		 * This callback is called when a message is published on the /wheel_states topic
		 */
		void wheelStatesCallback(const sensor_msgs::JointState::ConstPtr& msg) {
			auto msgTime = msg->header.stamp;

			if(firstMessage){
				this->lastUpdate = msgTime;
				this->firstMessage = false;
			}else{
				/*
				//calculate wheels speed using RPM
				this->wheelsSpeed[0] = msg->velocity[0] / 60 * 1/T;
				this->wheelsSpeed[1] = msg->velocity[1] / 60 * 1/T;
				this->wheelsSpeed[2] = msg->velocity[2] / 60 * 1/T;
				this->wheelsSpeed[3] = msg->velocity[3] / 60 * 1/T;
				*/
				
				//calculate wheels speed using TICKS
				double deltaTime = msgTime.toSec() - this->lastUpdate.toSec();

				for(int i = 0; i < 4; i++){
					//number of ticks made in delta time
					double deltaTicks = msg->position[i] - this->prevTicks[i];

					//from ticks to wheelsSpeed (rad/s)
					this->wheelsSpeed[i] = deltaTicks * (1.0/cpr) * 2 * M_PI * (1.0/deltaTime) * (1.0/T);
				}
				

				this->lastUpdate = msgTime;
				this->prevTicks = msg->position;

				this->computeKinematics();
				this->computeInverseKinematics();
			}
		}

		/**
		 * Calculate linear and angular velocities based on the wheels speed
		 */
		void computeKinematics() {
			double v1 = this->wheelsSpeed[0];
			double v2 = this->wheelsSpeed[1];
			double v3 = this->wheelsSpeed[2];
			double v4 = this->wheelsSpeed[3];

			//Vx -> longitudinal velocity
			this->calculatedLinearVelocity[0] = 
				wheels_radius/4 * (v1 + v2 + v3 + v4);
			
			//Vy -> transversal velocity
			this->calculatedLinearVelocity[1] = 
				wheels_radius/4 * ( - v1 + v2 + v3 - v4);

			//ω
			this->calculatedAngularVelocity = 
				wheels_radius/(4*(width + length)) * ( - v1 + v2 - v3 + v4);

			this->publishCalculatedVelocities();
		}

		/**
		 * Calculate angular velocities of the wheels based on the linear and angular velocity of the robot
		 */
		void computeInverseKinematics() {
			std::vector<double> wheelsSpeed{0, 0, 0, 0};
			
			double vx = calculatedLinearVelocity[0];
			double vy = calculatedLinearVelocity[1];
			double z = width + length;


			wheelsSpeed[0] = 1/wheels_radius * (vx - vy - z * calculatedAngularVelocity);
			wheelsSpeed[1] = 1/wheels_radius * (vx + vy + z * calculatedAngularVelocity);
			wheelsSpeed[2] = 1/wheels_radius * (vx + vy - z * calculatedAngularVelocity);
			wheelsSpeed[3] = 1/wheels_radius * (vx - vy + z * calculatedAngularVelocity);

			//publish message on topic /wheels_rpm
			robotics_project_1::WheelsRpm msg;

			msg.header.stamp = ros::Time::now();

			//multiply by 60 to convert rad/s to rad/min
			//and multiply for gear ratio cause the encoders are connected directly to motors
			msg.rpm_fl = wheelsSpeed[0] * 60 * T;
			msg.rpm_fr = wheelsSpeed[1] * 60 * T;
			msg.rpm_rl = wheelsSpeed[2] * 60 * T;
			msg.rpm_rr = wheelsSpeed[3] * 60 * T;

			this->rpmPub.publish(msg);
		}

		/**
		 * Publish velocities to /cmd_vel topic
		 */
		void publishCalculatedVelocities() {
			geometry_msgs::TwistStamped msg;

			msg.header.stamp = this->lastUpdate; //ros::Time::now();
			msg.twist.linear.x = this->calculatedLinearVelocity[0];
			msg.twist.linear.y = this->calculatedLinearVelocity[1];
			msg.twist.angular.z = this->calculatedAngularVelocity;

			this->cmdVelPub.publish(msg);
		}
};

int main(int argc, char **argv) {
	ros::init(argc, argv, "kinematics");

	Kinematics kinematics;
	kinematics.run();

	return 0;
}