#include "ros/ros.h"
#include "nav_msgs/Odometry.h"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>


class OdometryTF {
	private:
		ros::NodeHandle nh;
        ros::Subscriber odomSub;

        //tf broadcaster
		tf2_ros::TransformBroadcaster br;
  		geometry_msgs::TransformStamped transformStamped;


	public:
		OdometryTF() {
            //initialize odom subscriber
			this->odomSub = this->nh.subscribe(
				"odom",
				1000, 
				&OdometryTF::odomCallback,
				this
			);

        }

        void run() {
			ros::spin();
		}

        /**
		 * This callback is called when a message is published on the /odom topic
		 */
		void odomCallback(const nav_msgs::Odometry::ConstPtr& msg) {
			auto msgTime = msg->header.stamp;

			//header
			this->transformStamped.header.stamp = ros::Time::now();
			this->transformStamped.header.frame_id = "odom";
			this->transformStamped.child_frame_id = "base_link";

            
			//position
			this->transformStamped.transform.translation.x = msg->pose.pose.position.x;
			this->transformStamped.transform.translation.y = msg->pose.pose.position.y;
			this->transformStamped.transform.translation.z = 0.0;

			//orientation
			this->transformStamped.transform.rotation = msg->pose.pose.orientation;

            //send odom transformation
			br.sendTransform(this->transformStamped);
		}

};

int main(int argc, char **argv) {
	ros::init(argc, argv, "odometryTF");

	OdometryTF node;
	node.run();

	return 0;
}