#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>

#define PI 3.1415926

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

int main(int argc, char **argv)
{
    // 初始化节点
    ROS_INFO("init node");
    ros::init(argc, argv, "simple_navigation_goals");

    if (argc < 3) // 参数不足
    {
        ROS_ERROR("the parameter is incorrect");
        ROS_INFO("please input: rosrun simple_navigation_goals simple_navigation_goals x y (optional)theta_in_degree");
        ROS_INFO("for example : rosrun simple_navigation_goals simple_navigation_goals 1.0 1.0");
        ROS_INFO("              rosrun simple_navigation_goals simple_navigation_goals 1.0 1.0 90");
        return 1;
    }

    // 建立通信
    ROS_INFO("set communication");
    MoveBaseClient ac("move_base", true);

    // 等待响应
    ROS_INFO("wait for response");
    while (!ac.waitForServer(ros::Duration(5.0)))
        ROS_INFO("waiting for the move_base action server to come up");
    ROS_INFO("response received");

    // 创建 goal 类对象
    move_base_msgs::MoveBaseGoal goal;

    // 设置目标点的坐标
    goal.target_pose.header.frame_id = "map";         // 绝对坐标
    goal.target_pose.header.stamp = ros::Time::now(); // 时间戳

    // 设置目标位置
    goal.target_pose.pose.position.x = atof(argv[1]);
    goal.target_pose.pose.position.y = atof(argv[2]);
    goal.target_pose.pose.position.z = 0;

    // 设置目标指向
    double half_theta = 0;
    if (argc == 4)
        half_theta = atof(argv[3]) / 180.0 * PI / 2.0; // 角度转换为弧度，并减半
    goal.target_pose.pose.orientation.w = cos(half_theta);
    goal.target_pose.pose.orientation.x = 0;
    goal.target_pose.pose.orientation.y = 0;
    goal.target_pose.pose.orientation.z = sin(half_theta);

    // 发送 goal
    ROS_INFO("set goal position (x,y,z) = (%lf, %lf, 0)", goal.target_pose.pose.position.x, goal.target_pose.pose.position.y);
    ROS_INFO("set goal orientation (w,x,y,z) = (%lf, 0, 0, %lf)", goal.target_pose.pose.orientation.w, goal.target_pose.pose.orientation.z);
    ac.sendGoal(goal);

    // 等待反馈结果
    // ac.waitForResult();

    return 0;
}