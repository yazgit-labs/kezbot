#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/Pose2D.h>
#include <nav_msgs/Odometry.h>

std::string robot_name;

void poseCallback(const geometry_msgs::Pose2D& msg){
    static tf::TransformBroadcaster br;
    tf::Transform transform;
    transform.setOrigin(tf::Vector3(msg.x, msg.y, 0.0) );
    tf::Quaternion q;
    q.setRPY(0, 0, msg.theta);
    transform.setRotation(q);

    br.sendTransform(
        tf::StampedTransform(
            transform,
            ros::Time::now(),
            robot_name+"/odom",
            robot_name+"/base_link"
        )
    );

}

void odomCallback(const nav_msgs::Odometry& msg){
    static tf::TransformBroadcaster br;
    tf::Transform transform;

    transform.setOrigin(tf::Vector3(msg.pose.pose.position.x, msg.pose.pose.position.y, 0.0) );
    
    tf::Quaternion q(
    msg.pose.pose.orientation.x,
    msg.pose.pose.orientation.y,
    msg.pose.pose.orientation.z,
    msg.pose.pose.orientation.w
    );
    transform.setRotation(q);
    br.sendTransform(
        tf::StampedTransform(
            transform,
            ros::Time::now(),
            "base_pose_ground_truth",
            robot_name + "/base_link"    

        )        
    );

}


int main(int argc, char **argv){
    ros::init(argc, argv, "kezbot_tf_broadcaster");
    if (argc != 2){
        ROS_ERROR("need robot name as argument");
        return -1;
    };
    robot_name = argv[1];

    ros::NodeHandle node;
    ros::Subscriber sub = node.subscribe(robot_name+"/odom", 10, &odomCallback);

    ros::spin();
    return 0;


}
