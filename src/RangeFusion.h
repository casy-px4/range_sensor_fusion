#include <ros/ros.h>
#include <sensor_msgs/Range.h>
#include <geometry_msgs/TwistStamped.h>

#define DIST_MIDDLE_TOPIC "/distance_middle"
#define DIST_FORWARD_TOPIC "/distance_forward"
#define DIST_BACKWARD_TOPIC "/distance_backward"
#define VEL_TOPIC "/mavros/local_position/velocity"
#define DIST_FUSED_TOPIC "/mavros/distance_sensor/laser_1_sub"

#define TOPIC_QUEUE_SIZE 10

class RangeFusion {

public:
	RangeFusion(ros::NodeHandle& node, int rate_hz, float min_range_m, float max_range_m);
	void run();

private:
    #pragma region members

    // current NodeHandle
	ros::NodeHandle _n;

    // topics pub/sub
    ros::Subscriber _sub_distance_middle;
	ros::Subscriber _sub_distance_forward;
	ros::Subscriber _sub_distance_backward;
	ros::Subscriber _sub_velocity;
	ros::Publisher _pub_fused_distance;

    // range_message
	sensor_msgs::Range _fused_distance;

    // timing
	ros::Duration _dt;
    int _rate;

    // distances
	float _distance_middle;
	float _distance_forward;
	float _distance_backward;

    // velocity x in NED ref
	float _vx;

    // low pass filter time costant
	const float alpha_lp = 0.2;

    #pragma endregion

    #pragma region methods

    void _read_distance_middle(const sensor_msgs::Range::ConstPtr& msg);

	void _read_distance_forward(const sensor_msgs::Range::ConstPtr& msg);

	void _read_distance_backward(const sensor_msgs::Range::ConstPtr& msg);

	void _read_velocity(const geometry_msgs::TwistStamped::ConstPtr& msg);

	void _main();

	void _calcualte_plane(float d1, float d2, bool forward, float *m, float *b);

    #pragma endregion
};