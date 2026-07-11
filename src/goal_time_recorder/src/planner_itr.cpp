#include <ros/ros.h>
#include <actionlib_msgs/GoalStatusArray.h>
#include <std_msgs/Float64.h>
#include <ros/time.h>
#include <actionlib_msgs/GoalStatus.h>

class GoalTimeRecorder {
private:
    ros::NodeHandle nh_;
    ros::Subscriber status_sub_;
    ros::Publisher time_pub_;
    std::map<std::string, ros::Time> goal_start_times_;

public:
    GoalTimeRecorder() {
        // 直接硬编码订阅 /robot1/move_base/status 话题
        status_sub_ = nh_.subscribe("/robot1/move_base/status", 10, 
                                  &GoalTimeRecorder::statusCallback, this);
        
        // 直接硬编码发布 /robot1/goal_reach_time 话题
        time_pub_ = nh_.advertise<std_msgs::Float64>("/robot1/goal_reach_time", 10);
        
        ROS_INFO("Goal time recorder initialized, subscribing to /robot1/move_base/status");
        ROS_INFO("Publisher status: %s", time_pub_.getTopic().c_str());
    }
    
    void statusCallback(const actionlib_msgs::GoalStatusArray::ConstPtr& status_msg) {
        ROS_INFO("statusCallback triggered, %d goals in status list", (int)status_msg->status_list.size());
        
        for (const auto& status : status_msg->status_list) {
            // 打印完整状态信息
            ROS_INFO("Goal ID: %s, Status: %d, Text: %s", 
                    status.goal_id.id.c_str(), status.status, status.text.c_str());
            
            // 处理目标完成状态（修改为同时处理 SUCCEEDED 和 PREEMPTED）
            if (status.status == actionlib_msgs::GoalStatus::SUCCEEDED || 
                status.status == actionlib_msgs::GoalStatus::PREEMPTED) {
                
                ROS_INFO("Goal %s completed with status %d", 
                        status.goal_id.id.c_str(), status.status);
                
                if (goal_start_times_.find(status.goal_id.id) != goal_start_times_.end()) {
                    ros::Duration duration = ros::Time::now() - goal_start_times_[status.goal_id.id];
                    std_msgs::Float64 time_msg;
                    time_msg.data = duration.toSec();
                    
                    // 检查发布器是否有效
                    if (time_pub_.getNumSubscribers() > 0) {
                        time_pub_.publish(time_msg);
                        ROS_INFO("Published time: %.2f seconds, Subscribers: %d", 
                                duration.toSec(), time_pub_.getNumSubscribers());
                    } else {
                        ROS_WARN("No subscribers to /robot1/goal_reach_time, not publishing");
                    }
                    
                    // 根据状态类型输出不同日志
                    if (status.status == actionlib_msgs::GoalStatus::SUCCEEDED) {
                        ROS_INFO("Goal %s reached in %.2f seconds", 
                                status.goal_id.id.c_str(), duration.toSec());
                    } else {
                        ROS_INFO("Goal %s preempted after %.2f seconds (treated as completed)", 
                                status.goal_id.id.c_str(), duration.toSec());
                    }
                    
                    goal_start_times_.erase(status.goal_id.id);
                } else {
                    ROS_WARN("No start time recorded for goal %s", status.goal_id.id.c_str());
                }
            } else if (status.status == actionlib_msgs::GoalStatus::PENDING) {
                goal_start_times_[status.goal_id.id] = ros::Time::now();
                ROS_INFO("Goal %s is PENDING, start timing", status.goal_id.id.c_str());
            } else {
                ROS_DEBUG("Goal %s has status %d", status.goal_id.id.c_str(), status.status);
            }
        }
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "goal_time_recorder");
    GoalTimeRecorder recorder;
    ROS_INFO("Node initialized, waiting for move_base status messages...");
    ros::spin();
    return 0;
}

