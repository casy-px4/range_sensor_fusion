#include "RangeFusion.h"

RangeFusion::RangeFusion(ros::NodeHandle& node, int rate_hz, float min_range_m, float max_range_m) {
    _n = node;

    // initialization
    _rate = rate_hz;
    _distance_middle = 0;
    _distance_forward = 0;
    _distance_backward = 0;

    _fused_distance.range = 0;
    _fused_distance.min_range = min_range_m;
    _fused_distance.max_range = max_range_m;

    // register pub/sub
    _sub_distance_middle = _n.subscribe(DIST_MIDDLE_TOPIC, TOPIC_QUEUE_SIZE, &RangeFusion::_read_distance_middle, this);
    _sub_distance_forward = _n.subscribe(DIST_FORWARD_TOPIC, TOPIC_QUEUE_SIZE, &RangeFusion::_read_distance_forward, this);
    _sub_distance_backward = _n.subscribe(DIST_BACKWARD_TOPIC, TOPIC_QUEUE_SIZE, &RangeFusion::_read_distance_backward, this);
    _sub_velocity = _n.subscribe(VEL_TOPIC, TOPIC_QUEUE_SIZE, &RangeFusion::_read_velocity, this);
    _pub_fused_distance = _n.advertise<sensor_msgs::Range>(DIST_FUSED_TOPIC, TOPIC_QUEUE_SIZE);
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
}

void RangeFusion::_main() {
    float min = std::min(_distance_forward, _distance_backward);
    float m,b;
    if (min == _distance_forward){
        _calcualte_plane(_distance_middle, _distance_forward, true, &m, &b);
        ROS_INFO("Forward. Distances: %.3f, %.3f.",_distance_middle,_distance_forward);
    } else {
        _calcualte_plane(_distance_middle, _distance_backward, false, &m, &b);
        ROS_INFO("Forward. Distances: %.3f, %.3f.",_distance_middle,_distance_backward);
    }
    _fused_distance.header.stamp = ros::Time::now();
    float d = sqrt(pow(b, 2.0)/(pow(m, 2.0)+1.0));		//minimum distance from center sensor to plane
    _fused_distance.range = d;					//distance for distance_sensor message		//distance
    ROS_INFO("Plane m: %.3f b: %.3f. D: %.3f",m,b,d);			//distance
    
    _pub_fused_distance.publish(_fused_distance);
}

void RangeFusion::_calcualte_plane(float d1, float d2, bool forward, float *m, float *b) {
    //forward true --> forward and middle; type false --> backward and middle
    //d1 always the middle measurement
    //d1 forward or backward depending on the closest
    float x1,x2,y1,y2;
    x1 = 0;
    y1 = -d1;
    if (forward){
        x2 = 0.2 + d2*sin((double)1.05);
        y2 = -d2*cos((double)1.05);
    } else {
        x2 = -0.2 - d2*sin((double)1.05);
        y2 = -d2*cos((double)1.05);
    }
    //it should never happen that x1==x2
    float m_temp, b_temp;
    m_temp = (y2-y1)/(x2-x1);
    b_temp = y1-m_temp*x1;
    *m = m_temp;
    *b = b_temp;
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