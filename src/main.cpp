#include "main.hpp"

bool debug = false;
bool interrupted = false;
bool mutated = true;
geometry_msgs::PoseStamped msg_out;
ros::Subscriber joy_sub_;

// The circle constant tau = 2*pi. One tau is one rotation in radians.
double tau = 2 * M_PI * 0.9999;

// SIGINT Handler
void handleInterrupt() {
    Log("Interrupt handled");
    interrupted = true;
}

double v1 = 0;
double v2 = 0;
double v3 = 0;
double v4 = 0;
double v5 = 0;
double v6 = 0;

void joyCallback(const sensor_msgs::Joy::ConstPtr& joy)
{
    if(joy->buttons[0] == 1) v1 += 0.05;
    if(joy->buttons[1] == 1) v1 -= 0.05;
    if(joy->buttons[2] == 1) v2 += 0.05;
    if(joy->buttons[3] == 1) v2 -= 0.05;
    if(joy->buttons[4] == 1) v3 += 0.05;
    if(joy->buttons[5] == 1) v3 -= 0.05;
    if(joy->buttons[6] == 1) v4 += 0.05;
    if(joy->buttons[7] == 1) v4 -= 0.05;
    if(joy->buttons[8] == 1) v5 += 0.05;
    if(joy->buttons[9] == 1) v5 -= 0.05;
    if(joy->buttons[10] == 1) v6 += 0.05;
    if(joy->buttons[11] == 1) v6 -= 0.05;
    mutated = true;
}


int main(int argc, char *argv[]) {

    ros::init(argc, argv, "move_group_interface_tutorial");
    ros::NodeHandle node_handle;
    ros::NodeHandle nh;

    signal(SIGINT, (__sighandler_t) handleInterrupt);
    // ROS spinning must be running for the MoveGroupInterface to get information
    // about the robot's state. One way to do this is to start an AsyncSpinner
    // beforehand.
    ros::AsyncSpinner spinner(1);
    spinner.start();


    joy_sub_ = nh.subscribe<sensor_msgs::Joy>("joy", 10, &joyCallback);

    // BEGIN_TUTORIAL
    //
    // Setup
    // ^^^^^
    //
    // MoveIt operates on sets of joints called "planning groups" and stores them in an object called
    // the `JointModelGroup`. Throughout MoveIt the terms "planning group" and "joint model group"
    // are used interchangeably
    static const std::string PLANNING_GROUP = "manipulator";

    // The :planning_interface:`MoveGroupInterface` class can be easily
    // setup using just the name of the planning group you would like to control and plan for.
    moveit::planning_interface::MoveGroupInterface move_group_interface(PLANNING_GROUP);

    // We will use the :planning_interface:`PlanningSceneInterface`
    // class to add and remove collision objects in our "virtual world" scene
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

    // Raw pointers are frequently used to refer to the planning group for improved performance.
    const moveit::core::JointModelGroup* joint_model_group =
            move_group_interface.getCurrentState()->getJointModelGroup(PLANNING_GROUP);

    moveit::core::RobotStatePtr current_state = move_group_interface.getCurrentState();

    moveit::planning_interface::MoveGroupInterface::Plan my_plan;

    std::vector<double> joint_group_positions;
    current_state->copyJointGroupPositions(joint_model_group, joint_group_positions);

    move_group_interface.setMaxVelocityScalingFactor(0.4);
    move_group_interface.setMaxAccelerationScalingFactor(0.4);

    std::cout.precision(4);


    v1 = joint_group_positions[0];
    v2 = joint_group_positions[1];
    v3 = joint_group_positions[2];
    v4 = joint_group_positions[3];
    v5 = joint_group_positions[4];
    v6 = joint_group_positions[6];


    while(!interrupted) {
        if(!mutated) continue;

    joint_group_positions[0] = v1;
        joint_group_positions[1] = v2;
        joint_group_positions[2] = v3;
        joint_group_positions[3] = v4;
        joint_group_positions[4] = v5;
        joint_group_positions[5] = v6;

        move_group_interface.setJointValueTarget(joint_group_positions);

        bool success = (move_group_interface.plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS);

        move_group_interface.execute(my_plan);
        mutated = false;
        current_state->copyJointGroupPositions(joint_model_group, joint_group_positions);


        /*v1 = joint_group_positions[0];
        v2 = joint_group_positions[1];
        v3 = joint_group_positions[2];
        v4 = joint_group_positions[3];
        v5 = joint_group_positions[4];
        v6 = joint_group_positions[6];*/
    }

    ros::shutdown();
    return 0;
}

void listener(ros::Publisher &pub) {
    msg_out.header.stamp = ros::Time::now();
    msg_out.header.frame_id = "gps";

    //msg_out.pose.position.x = GetLatFloat();
    //msg_out.pose.position.y = GetLonFloat();
    msg_out.pose.position.z = 0.0f;

    pub.publish(msg_out);
}


// Log a message.
void Log(const char *msg) {
    printf("[ARM_BASIC_CTRL] %s\n", msg);
}

// Concat and log 2 char arrays.
void ConcatLog(const char *c1, char *c2) {
    printf("[ARM_BASIC_CTRL] %s%s\n", c1, c2);
}

void PrintValue(double d) {
    std::cout << std::setw(10) << d << "\n";
}