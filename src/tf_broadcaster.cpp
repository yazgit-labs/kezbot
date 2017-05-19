#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/Pose.h>


std::string robot_name;

void poseCallback(const geometry_msgs::Pose& msg){
    static tf::TransformBroadcaster br;
    tf::Transform transform;
    transform.setOrigin(tf::Vector3(msg.position.x, msg.position.y, 0.0) );
    tf::Quaternion q(
        tfScalar(msg.orientation.x),
        tfScalar(msg.orientation.y),
        tfScalar(msg.orientation.z),
        tfScalar(msg.orientation.w)
    );
    transform.setRotation(q);

    br.sendTransform(
        tf::StampedTransform(
            transform,
            ros::Time::now(),
            "odom",
            robot_name+"/base_link"
        )
    );

}

int main(int argc, char **argv){
    ros::init(argc, argv, "my_tf_broadcaster");
    if (argc != 2){
        ROS_ERROR("need robot name as argument");
        return -1;
    };
    robot_name = argv[1];

    ros::NodeHandle node;
    ros::Subscriber sub = node.subscribe(robot_name+"/pose", 10, &poseCallback);

    ros::spin();
    return 0;
}
