#include "RangeFusion.h"

RangeFusion::RangeFusion(ros::NodeHandle& node, int rate_hz) {
    _n = node;
    // initialization
    _rate = rate_hz;
    _distance_middle = 0;
    _distance_forward = 0;
    _distance_backward = 0;
    _p0_x = 0; _p0_y = 0;
    _p1_x = 0; _p1_y = 0;
    _p2_x = 0; _p2_y = 0;
    // fused_distance message
    _fused_distance.range = 0;
    _fused_distance.min_range = SENSOR_RNG_MIN;
    _fused_distance.max_range = SENSOR_RNG_MAX;
    // register pub/sub
    _sub_distance_middle = _n.subscribe(TOPIC_DIST_MIDDLE, TOPIC_QUEUE_SIZE, &RangeFusion::_read_distance_middle, this);
    _sub_distance_forward = _n.subscribe(TOPIC_DIST_FORWARD, TOPIC_QUEUE_SIZE, &RangeFusion::_read_distance_forward, this);
    _sub_distance_backward = _n.subscribe(TOPIC_DIST_BACKWARD, TOPIC_QUEUE_SIZE, &RangeFusion::_read_distance_backward, this);
    _sub_velocity = _n.subscribe(TOPIC_VEL, TOPIC_QUEUE_SIZE, &RangeFusion::_read_velocity, this);
    _sub_pose = _n.subscribe(TOPIC_POSE, TOPIC_QUEUE_SIZE, &RangeFusion::_read_pose, this);
    _pub_fused_distance = _n.advertise<sensor_msgs::Range>(TOPIC_DIST_FUSED, TOPIC_QUEUE_SIZE);
}

void RangeFusion::run() {
    ros::Rate loop_rate(_rate);
    while (ros::ok())
    {
        ROS_INFO_ONCE("POS_MIXER: RUNNING");
        this->_main();
        ros::spinOnce();
        loop_rate.sleep();
    }
}

void RangeFusion::_read_distance_middle(const sensor_msgs::Range::ConstPtr& msg) {
   _distance_middle = msg->range;
}

void RangeFusion::_read_distance_forward(const sensor_msgs::Range::ConstPtr& msg) {
    _distance_forward = msg->range;
}

void RangeFusion::_read_distance_backward(const sensor_msgs::Range::ConstPtr& msg) {
    _distance_backward = msg->range;
}

void RangeFusion::_read_velocity(const geometry_msgs::TwistStamped::ConstPtr& msg) {
    _vx = msg->twist.linear.x;
    _vz = msg->twist.linear.z;
}

void RangeFusion::_read_pose(const geometry_msgs::PoseStamped::ConstPtr& msg) {
    tf::Quaternion q(msg->pose.orientation.x, msg->pose.orientation.y, msg->pose.orientation.z, msg->pose.orientation.w);
    tf::Matrix3x3 m(q);
    m.getRPY(_roll, _pitch, _yaw);
}

void RangeFusion::_main() {
    float d_back = cos(_roll) * _distance_backward;
    float d_midd = cos(_roll) * _distance_middle;
    float d_fron = cos(_roll) * _distance_forward;

    float x0 = SENSOR_POSX_BACK + sin(-_pitch + SENSOR_ANG_BACK) * d_back;
    float y0 = -cos(-_pitch + SENSOR_ANG_BACK) * d_back;
    float x1 = sin(-_pitch) * d_midd;
    float y1 = -cos(-_pitch) * d_midd;
    float x2 = SENSOR_POSX_FRONT + sin(-_pitch + SENSOR_ANG_FRONT) * d_fron;
    float y2 = -cos(-_pitch + SENSOR_ANG_FRONT) * d_fron;

    float m0 = x1 == x0 ? 10e6 : (y1 - y0) / (x1 - x0);
    float q0 = y0 - m0 * x0;
    float m1 = x2 == x1 ? 10e6 : (y2 - y1) / (x2 - x1);
    float q1 = y1 - m1 * x1;

    float current_alt = _vx > 0 ? _dist_point_rect(0, 0, m1, q1) : _dist_point_rect(0, 0, m0, q0);
    
    float dt = 1.0f / (float) _rate;
    float dx = _vx * dt;
    float dz = _vz * dt;

    float m_b = x0 == _p0_x ? 10e6 : (y0 - _p0_y) / (x0 - _p0_x);
    float q_b = _p0_y - m_b * _p0_x;
    float m_m = x1 == _p1_x ? 10e6 : (y1 - _p1_y) / (x1 - _p1_x);
    float q_m = _p1_y - m_m * _p1_x;
    float m_f = x2 == _p2_x ? 10e6 : (y2 - _p2_y) / (x2 - _p2_x);
    float q_f = _p2_y - m_f * _p2_x;

    _p0_x = x0; _p0_y = y0;
    _p1_x = x1; _p1_y = y1;
    _p2_x = x2; _p2_y = y2;
    
    float future_alt = (_vx > 0 && fabs(_vx) > 0.1f) ? std::min(_dist_point_rect(dx, dz, m_f, q_f), _dist_point_rect(dx, dz, m_m, q_m)) :
                                 std::min(_dist_point_rect(dx, dz, m_b, q_b), _dist_point_rect(dx, dz, m_m, q_m));

    // ROS_INFO("x0:%f y0:%f x1:%f y1:%f x2:%f y2:%f", x0, y0, x1, y1, x2, y2);
    // ROS_INFO("m0:%f q0:%f m1:%f q1:%f", m0, q0, m1, q1);
    ROS_INFO("\nm_b:%f, q_b:%f \nm_m:%f, q_m:%f \nm_f:%f, q_f:%f", m_b, q_b, m_m, q_m, m_f, q_f);
    _fused_distance.range = future_alt;
    _pub_fused_distance.publish(_fused_distance);
}

float RangeFusion::_dist_point_rect(float px, float py, float m, float q) {
    return fabs(py - (m * px + q)) / sqrt(pow(m, 2.0) + 1.0);
}