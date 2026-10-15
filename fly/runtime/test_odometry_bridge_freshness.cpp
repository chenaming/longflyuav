#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>
#include <uav_control_fsm/odometry_bridge.hpp>
#include <set>
#include <mutex>
#include <iostream>

int main(int argc,char** argv) {
    ros::init(argc,argv,"test_odin_forwarding_isolated");
    ros::NodeHandle nh("/fly_odin_forwarding_isolated");
    nh.setParam("odometry_bridge/vision_pose_pub_rate_hz",50.0);
    OdometryBridge bridge(nh,77);bridge.setLocationSource("Fast_Lio");
    std::mutex mutex;std::set<uint64_t> source,received;int count=0;bool altered=false;
    auto sub=nh.subscribe<geometry_msgs::PoseStamped>("/uav77/mavros/vision_pose/pose",100,
        [&](const geometry_msgs::PoseStamped::ConstPtr& m){
            std::lock_guard<std::mutex> lock(mutex);++count;
            if(!source.count(m->header.stamp.toNSec()) || m->pose.position.x!=1.25)altered=true;
            received.insert(m->header.stamp.toNSec());
        });
    auto pub=nh.advertise<nav_msgs::Odometry>("/uav77/fast_lio_odometry",100);
    ros::AsyncSpinner spinner(1);spinner.start();
    const auto wait=ros::WallTime::now()+ros::WallDuration(2);
    while(pub.getNumSubscribers()==0 && ros::WallTime::now()<wait)ros::WallDuration(.01).sleep();
    const auto end=ros::WallTime::now()+ros::WallDuration(1);
    while(ros::WallTime::now()<end) {
        nav_msgs::Odometry m;m.header.stamp=ros::Time::now();m.header.frame_id="odom";
        m.pose.pose.position.x=1.25;m.pose.pose.orientation.w=1.;
        {std::lock_guard<std::mutex> lock(mutex);source.insert(m.header.stamp.toNSec());}
        pub.publish(m);ros::WallDuration(.0025).sleep();
    }
    ros::WallDuration(.25).sleep();int before;
    {std::lock_guard<std::mutex> lock(mutex);before=count;}
    ros::WallDuration(.3).sleep();
    std::lock_guard<std::mutex> lock(mutex);
    const bool ok=!altered && received.size()==size_t(count) && count==before && count>=35 && count<=65;
    std::cout<<"isolated forwarding count="<<count<<" unique="<<received.size()
             <<" continued_after_source_stop="<<(count-before)<<" source_stamp_altered="<<altered
             <<" passed="<<ok<<std::endl;
    return ok?0:1;
}
