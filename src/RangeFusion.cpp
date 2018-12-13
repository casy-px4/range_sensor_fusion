#include "RangeFusion.h"

RangeFusion::RangeFusion(ros::NodeHandle& node, int rate_hz) {
    _n = node;
    // initialization
    _rate = rate_hz;
    _distance_middle = 0;
    _distance_forward = 0;
    _distance_backward = 0;
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
   _last_midd = ros::Time::now();
}

void RangeFusion::_read_distance_forward(const sensor_msgs::Range::ConstPtr& msg) {
    _distance_forward = msg->range;
    _last_forw = ros::Time::now();
}

void RangeFusion::_read_distance_backward(const sensor_msgs::Range::ConstPtr& msg) {
    _distance_backward = msg->range;
    _last_back = ros::Time::now();
}

void RangeFusion::_read_velocity(const geometry_msgs::TwistStamped::ConstPtr& msg) {
    _vx = msg->twist.linear.x;
    _vz = msg->twist.linear.z;
}

void RangeFusion::_read_pose(const geometry_msgs::PoseStamped::ConstPtr& msg) {
    static float last_x = 0, last_z = 0;
    tf::Quaternion q(msg->pose.orientation.x, msg->pose.orientation.y, msg->pose.orientation.z, msg->pose.orientation.w);
    tf::Matrix3x3 m(q);
    m.getRPY(_roll, _pitch, _yaw);
    _dx = msg->pose.position.x - last_x;
    _dz = msg->pose.position.z - last_z;
    last_x = msg->pose.position.x;
    last_z = msg->pose.position.z;
}

void RangeFusion::_main() {
    static float smooth_d_back = 0, smooth_d_midd = 0, smooth_d_fron = 0;

    if (!_check_sensor_timeout()) 
        return;

    float d_back = cos(_roll) * _distance_backward;
    float d_midd = cos(_roll) * _distance_middle;
    float d_fron = cos(_roll) * _distance_forward;

    float dt = 1.0f / (float) _rate;
    float time_const = _alpha_lp * dt;
    _low_pass_filter(d_back, &smooth_d_back, time_const);
    _low_pass_filter(d_midd, &smooth_d_midd, time_const);
    _low_pass_filter(d_fron, &smooth_d_fron, time_const);

    float x0 = SENSOR_POSX_BACK + sin(-_pitch + SENSOR_ANG_BACK) * smooth_d_back;
    float y0 = -cos(-_pitch + SENSOR_ANG_BACK) * smooth_d_back;
    float x1 = sin(-_pitch) * smooth_d_midd;
    float y1 = -cos(-_pitch) * smooth_d_midd;
    float x2 = SENSOR_POSX_FRONT + sin(-_pitch + SENSOR_ANG_FRONT) * smooth_d_fron;
    float y2 = -cos(-_pitch + SENSOR_ANG_FRONT) * smooth_d_fron;

    float m0 = x1 == x0 ? 10e6 : (y1 - y0) / (x1 - x0);
    float q0 = y0 - m0 * x0;
    float m1 = x2 == x1 ? 10e6 : (y2 - y1) / (x2 - x1);
    float q1 = y1 - m1 * x1;
    float ang0 = tfDegrees(atan(m0));
    float ang1 = tfDegrees(atan(m1));

    float abs_vx = fabs(_vx);
    float current_m = _vx > 0 ? m1 : m0;
    float d = current_m < 0 ? 0 : pow(_vx * current_m, 2.0f);

    float est_alt = std::min(_dist_point_rect(0, 0, m1, q1), _dist_point_rect(0, 0, m0, q0));
    float est_atl_w = 1 / (1 + d);

    float pred_alt = (_vx > 0 ? smooth_d_fron : smooth_d_back) / (1 + d);
    float pred_alt_w = d;

    float avg_alt = (est_alt * est_atl_w + pred_alt * pred_alt_w) / (est_atl_w + pred_alt_w);

    _fused_distance.range = avg_alt;
    _pub_fused_distance.publish(_fused_distance);

    ROS_INFO("\n \
    ==============================================\n \
    ang1:%.3f; q1:%.3f\n \
    dist_forw:%.3f; dist_mid:%.3f; dist_back:%.3f; vx:%.3f\n \
    raw_f:%.3f; raw_m:%.3f; raw_b:%.3f\n \
    est_atl:%.3f; wgth:%.3f\n \
    prd_alt:%.3f; wgth:%.3f\n \
    avg:%.3f\n \
    ==============================================",
    ang1, q1, 
    smooth_d_fron, smooth_d_midd, smooth_d_back, _vx, 
    d_fron, d_midd, d_back,
    est_alt, est_atl_w,
    pred_alt, pred_alt_w, 
    avg_alt);
}

bool RangeFusion::_check_sensor_timeout() {
    ros::Time current = ros::Time::now();
    ros::Duration delta_back = current - _last_back;
    ros::Duration delta_midd = current - _last_midd;
    ros::Duration delta_forw = current - _last_forw;
    ROS_INFO("s:%d, n:%d", delta_back.sec, delta_back.nsec);
    if (delta_back.sec * 1e9 + delta_back.nsec > SENSOR_TIMEOUT_MS * 1e6) {
        ROS_ERROR("distance_sensor_back: Timeout!");
        return false;
    }
    if (delta_midd.sec * 1e9 + delta_midd.nsec > SENSOR_TIMEOUT_MS * 1e6) {
        ROS_ERROR("distance_sensor_midd: Timeout!");
        return false;
    }
    if (delta_forw.sec * 1e9 + delta_forw.nsec > SENSOR_TIMEOUT_MS * 1e6) {
        ROS_ERROR("distance_sensor_forw: Timeout!");
        return false;
    }
    return true;
}

float RangeFusion::_dist_point_rect(float px, float py, float m, float q) {
    return fabs(py - (m * px + q)) / sqrt(pow(m, 2.0) + 1.0);
}

void RangeFusion::_low_pass_filter(float in, float* out, float time_const) {
    *out = time_const * in + (1 - time_const) * (*out);
}