#include "ros/ros.h"
#include <robotics_project_1/parametersConfig.h>
#include <dynamic_reconfigure/server.h>
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/TwistStamped.h"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include "robotics_project_1/Reset.h"

enum IntegrationMethod {
	EULER,
	RUNGE_KUTTA
};

class Odometry {
	private:
		ros::NodeHandle nh;
		ros::Subscriber cmdVelSub;
		ros::Publisher odomPub;
		ros::ServiceServer resetService;
		IntegrationMethod integrationMethod = EULER;
		
		//tf broadcaster
		tf2_ros::TransformBroadcaster br;
  		geometry_msgs::TransformStamped transformStamped;
		
		//odometry
		std::vector<double> position{0, 0}; //x, y
		double theta = 0; //angle

		//velocities
		std::vector<double> linearVelocity{0, 0}; //x, y  m/s
		double angularVelocity; //ω rad/s

		//dynamic reconfigure
		dynamic_reconfigure::Server<robotics_project_1::parametersConfig> dynServer;
		dynamic_reconfigure::Server<robotics_project_1::parametersConfig>::CallbackType dynCallback;

		//last received velocity
		geometry_msgs::Twist lastTwist;
		geometry_msgs::Twist newTwist;

		bool firstMessage = true;

		ros::Time lastUpdate;
		double deltaTime = 0;

	public:

		Odometry() {
			//initialize cmd_vel subscriber
			this->cmdVelSub = this->nh.subscribe(
				"cmd_vel",
				1000, 
				&Odometry::cmdVelCallback,
				this
			);

			//get initial pose
			ros::param::get("initial_pose/x", position[0]);
			ros::param::get("initial_pose/y", position[1]);
			ros::param::get("initial_pose/theta", this->theta);

			//dynamic reconfigure for integration method
			dynCallback = boost::bind(&Odometry::integrationMethodChangeCallback, this, _1, _2);
			dynServer.setCallback(dynCallback);
			
			//reset service
			this->resetService = this->nh.advertiseService("reset", &Odometry::resetServiceCallback, this);

			//initialize calculated odometry publisher
			this->odomPub = this->nh.advertise<nav_msgs::Odometry>("odom", 1000);

			this->lastUpdate = ros::Time::now();
		}

		void run() {
			ros::spin();
		}

		/**
		 * This method is called when the /reset service is called
		 */
		bool resetServiceCallback(robotics_project_1::Reset::Request &req, robotics_project_1::Reset::Response &res) {
			
			this->resetPose(req.x, req.y, req.theta);

			return true;
		}

		/**
		 * This method reset the pose to the given values and reset every variables of the odometry
		 */
		void resetPose(double x, double y, double theta){
			ROS_INFO("Reset odometry position to X: %f Y: %f Z: %f", x, y, theta);
			this->position[0] = x;
			this->position[1] = y;
			this->theta = theta;
			this->lastTwist = geometry_msgs::Twist();
			this->newTwist = geometry_msgs::Twist();
			this->linearVelocity = {0, 0};
			this->angularVelocity = 0;
			this->lastUpdate = ros::Time::now();
			this->firstMessage = true;

			this->publishCalculatedOdometry();
			this->publishTF();
		}

		/**
		 * This callback is called when a message is published on the /cmd_vel topic
		 */
		void cmdVelCallback(const geometry_msgs::TwistStamped::ConstPtr& msg) {
			auto msgTime = msg->header.stamp;

			if(firstMessage){
				this->lastUpdate = msgTime;
				this->firstMessage = false;
				this->lastTwist = msg->twist;
			}else{
				this->deltaTime = msgTime.toSec() - this->lastUpdate.toSec();
				
				this->lastUpdate = msgTime;

				this->lastTwist = this->newTwist;
				
				this->newTwist = msg->twist;

				this->computeOdometry();
			}
		}

		/**
		 * Compute odometry given velocities (lastTwist) and delta time from previous velocity update
		 */
		void computeOdometry() {

			if(integrationMethod == EULER){
				this->position[0] = 
						this->position[0] + lastTwist.linear.x * deltaTime * cos(this->theta) + lastTwist.linear.y * deltaTime * sin(this->theta);
				this->position[1] = 
						this->position[1] + lastTwist.linear.x * deltaTime * sin(this->theta) + lastTwist.linear.y * deltaTime * cos(this->theta);
			} else {
				double approximatedTheta = this->theta + (lastTwist.angular.z * deltaTime) / 2;

				this->position[0] = 
						this->position[0] + lastTwist.linear.x * deltaTime * cos(approximatedTheta) + lastTwist.linear.y * deltaTime * sin(approximatedTheta);
				this->position[1] = 
						this->position[1] + lastTwist.linear.x * deltaTime * sin(approximatedTheta) + lastTwist.linear.y * deltaTime * cos(approximatedTheta);

			}
			
			this->theta = this->theta + lastTwist.angular.z * deltaTime;

			this->publishCalculatedOdometry();
		}

		/**
		 * Publish odometry to /odom topic
		 */
		void publishCalculatedOdometry() {
			nav_msgs::Odometry msg;

			msg.header.stamp = this->lastUpdate;//ros::Time::now();
  			msg.header.frame_id = "odom";
			msg.child_frame_id = "base_link";

			//pose
			tf2::Quaternion o;
			o.setRPY(0, 0, this->theta);

			msg.pose.pose.orientation.x = o.x();
			msg.pose.pose.orientation.y = o.y();
			msg.pose.pose.orientation.z = o.z();
			msg.pose.pose.orientation.w = o.w();

			msg.pose.pose.position.x = this->position[0];
			msg.pose.pose.position.y = this->position[1];
			msg.pose.pose.position.z = 0.0;

			//twist
			msg.twist.twist = this->newTwist;

			this->odomPub.publish(msg);

			this->publishTF();
		}

		void publishTF(){
			//header
			this->transformStamped.header.stamp = this->lastUpdate;//ros::Time::now();
			this->transformStamped.header.frame_id = "odom";
			this->transformStamped.child_frame_id = "base_link";

			//x,y
			this->transformStamped.transform.translation.x = this->position[0];
			this->transformStamped.transform.translation.y = this->position[1];
			this->transformStamped.transform.translation.z = 0.0;

			//theta
			tf2::Quaternion o;
			o.setRPY(0, 0, this->theta);

			this->transformStamped.transform.rotation.x = o.x();
			this->transformStamped.transform.rotation.y = o.y();
			this->transformStamped.transform.rotation.z = o.z();
			this->transformStamped.transform.rotation.w = o.w();

			br.sendTransform(this->transformStamped);
		}

		/**
		 * This callback is called when the integration method changes
		 */
		void integrationMethodChangeCallback(robotics_project_1::parametersConfig &config, uint32_t level) {
			this->integrationMethod = static_cast<IntegrationMethod>(config.integration_method);
			
			ROS_INFO("Change integration method to %s", this->integrationMethod == EULER ? "E" : "RK");
		}
};

int main(int argc, char **argv) {
	ros::init(argc, argv, "odometry");

	Odometry odometry;
	odometry.run();

	return 0;
}