#ifndef _RANGE_FUSION_H
#define _RANGE_FUSION_H

#include <ros/ros.h>
#include <sensor_msgs/Range.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <tf/transform_datatypes.h>

#define TOPIC_DIST_MIDDLE "/distance_middle"
#define TOPIC_DIST_FORWARD "/distance_forward"
#define TOPIC_DIST_BACKWARD "/distance_backward"
#define TOPIC_VEL "/mavros/local_position/velocity"
#define TOPIC_POSE "/mavros/local_position/pose"
#define TOPIC_DIST_FUSED "/mavros/distance_sensor/laser_1_sub"
#define TOPIC_QUEUE_SIZE 10

#define SENSOR_ANG_FRONT 1.05
#define SENSOR_POSX_FRONT 0.2
#define SENSOR_ANG_BACK -1.05
#define SENSOR_POSX_BACK -0.2
#define SENSOR_RNG_MIN 0.1
#define SENSOR_RNG_MAX 40.0F

class RangeFusion {

public:
	RangeFusion(ros::NodeHandle& node, int rate_hz);
	void run();

private:
    #pragma region members

    // current NodeHandle
	ros::NodeHandle _n;

	// timing
    int _rate;
	// low pass filter time costant
	const static float alpha_lp = 0.2;

    // topics pub/sub
    ros::Subscriber _sub_distance_middle;
	ros::Subscriber _sub_distance_forward;
	ros::Subscriber _sub_distance_backward;
	ros::Subscriber _sub_velocity;
	ros::Subscriber _sub_pose;
	ros::Publisher _pub_fused_distance;
    // range_message
	sensor_msgs::Range _fused_distance;

    // distances
	float _distance_middle;
	float _distance_forward;
	float _distance_backward;
    // velocities in NED frame
	float _vx;
	float _vz;
	// euler angles
	double _pitch;
	double _roll;
	double _yaw;
	// terrain points: p0 -> back .. p2 -> front
	float _p0_x, _p0_y, _p1_x, _p1_y, _p2_x, _p2_y;

    #pragma endregion

    #pragma region methods

	// topic callbacks
    void _read_distance_middle(const sensor_msgs::Range::ConstPtr& msg);
	void _read_distance_forward(const sensor_msgs::Range::ConstPtr& msg);
	void _read_distance_backward(const sensor_msgs::Range::ConstPtr& msg);
	void _read_velocity(const geometry_msgs::TwistStamped::ConstPtr& msg);
	void _read_pose(const geometry_msgs::PoseStamped::ConstPtr& msg);

	void _main();

	float _dist_point_rect(float px, float py, float m, float q);

	#pragma endregion
};

#endif