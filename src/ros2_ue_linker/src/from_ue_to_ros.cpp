#include <memory>
#include <string>
#include <vector>
#include <sys/ipc.h>
#include <sys/shm.h>
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "control_msgs/action/follow_joint_trajectory.hpp"
#include "trajectory_msgs/msg/joint_trajectory.hpp"
#include "trajectory_msgs/msg/joint_trajectory_point.hpp"

const int NUM_JOINTS = 24;

struct ShmStruct
{
    key_t key;
    int shmid;
    uint8_t* shm_data;
};

struct JointCommands
{
    unsigned long msg_id;
    int joint_id;
    float time_delta;
    float joint_commands[NUM_JOINTS];
} JointCommandsMsg;

std::shared_ptr<rclcpp::Node> node;
bool common_goal_accepted = false;
rclcpp_action::ResultCode common_resultcode = rclcpp_action::ResultCode::UNKNOWN;
int common_action_result = control_msgs::action::FollowJointTrajectory_Result::SUCCESSFUL;

void common_goal_response(rclcpp_action::ClientGoalHandler<control_msgs::action::FollowJointTrajectory>::SharedPtr goal_handle)
{
    RCLCPP_DEBUG(node->get_logger(), "common_goal_response time: %f", rclcpp::Clock().now().seconds());
    if(!goal_handle)
    {
        common_goal_accepted = false;
        printf("Goal rejected\n");
    } else
    {
        common_goal_accepted = true;
        printf("Goal accepted\n");
    }
}

void common_result_response(const rclcpp_action::ClientGoalHandler<control_msgs::action::FollowJointTrajectory>::WrappedResult & result)
{
    printf("common_result_response time: %f\n", rclcpp::Clock().now().seconds());
    common_resultcode = result.code;
    common_action_result = result.result->error_code;
    switch(result.code)
    {
        case rclcpp_action::ResultCode::SUCCEEDED:
            printf("SUCCEEDED result code\n");
            break;
        case rclcpp_action::ResultCode::ABORTED:
            printf("Goal was aborted\n");
            return;
        case rclcpp_action::ResultCode::CANCELED:
            printf("Goal was canceled\n");
            return;
        default:
            printf("Unknown result code\n");
            return;
    }
}

void common_feedback(rclcpp_action::ClientGoalHandle<control_msgs::action::FollowJointTrajectory>::SharedPtr,
                    const std::shared_ptr<const control_msgs::action::FollowJointTrajectory::Feedback> feedback)
{
    std::cout << "Feedback desired position:";
    for(auto & x : feedback->desired.positions)
    {
        std::cout << x <<"\t";
    }
    std::cout << std::endl;
    
}

void SetupSharedMemory(ShmStruct &shmstruct)
{
    shmstruct.key = ftok("/home/lucas/Documents/Unreal Projects/ROS2_Simulation_ABMI/Source/ROS2_Simulation_ABMI/data.conf", 1);
    shmstruct.shmid = shmget(shmstruct.key, sizeof(JointCommandsMsg), 0666 | IPC_CREAT);
    if (shmstruct.shmid == -1)
    {
        RCLCPP_ERROR(node->get_logger(), "ERROR CREATING SHARED MEMORY");
    }
    else
    {
        shmstruct.shm_data = (uint8_t *)shmat(shmstruct.shmid, NULL, 0);
        if (shmstruct.shm_data == (void *)-1)
        {
            RCLCPP_ERROR(node->get_logger(), "ERROR ATTACHING SHARED MEMORY");
        }
    }
}

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    node = std::make_shared<rclcpp::Node>("from_ue_to_ros");
    rclcpp_action::Client<control_msgs::action::FollowJointTrajectory>::SharedPtr action_client;
    action_client = rclcpp_action::create_client<control_msgs::action::FollowJointTrajectory>(node->get_node_base_interface(),
                                                                                              node->get_node_graph_interface(),
                                                                                              node->get_node_logging_interface(),
                                                                                              node->get_node_waitables_interface(),
                                                                                              "arm_controller/follow_joint_trajectory");

    bool response = action_client->wait_for_action_server(std::chrono::seconds(3));
    if(!response)
    {
        throw std::runtime_error("could not get action server");
    }

    std::cout << "created action server" << std::endl;

    std::vector<std::string> joint_names = {"wrist_pitch_lower", "wrist_yaw", "wrist_pitch_upper", "index_yaw", "middle_yaw", 
        "ring_yaw", "pinky_yaw", "index_pitch", "index_knuckle", "index_tip",
        "middle_pitch", "middle_knuckle", "middle_tip", "ring_pitch", "ring_knuckle",
        "ring_tip", "pinky_pitch", "pinky_knuckle", "pinky_tip", "thumb_yaw",
        "thumb_roll", "thumb_pitch", "thumb_knuckle", "thumb_tip"};

    ShmStruct Shm;
    SetupSharedMemory(Shm);
    RCLCPP_INFO(node->get_logger(), "Start listening joints commands...");
    unsigned long last_id = 0;
    unsigned long msg_id = 0;

    std::vector<trajectory_msgs::msg::JointTrajectoryPoint> points;
    trajectory_msgs::msg::JointTrajectoryPoint point;
    point.positions.resize(NUM_JOINTS);
    points.push_back(point);

    while(rclcpp::ok())
    {
        unsigned long *q = (unsigned long*)Shm.shm_data;
        msg_id = *q;
        if(msg_id > last_id)
        {
            JointCommandsMsg = *(JointCommands*)Shm.shm_data;
            // Assuming you want to construct a JointTrajectory message
            // and send it to the action server
            control_msgs::action::FollowJointTrajectory_Goal goal_msg;
            goal_msg.goal_id.stamp = rclcpp::Clock().now();
            goal_msg.goal_id.id = "goal_id";
            goal_msg.trajectory.joint_names = joint_names;
            trajectory_msgs::msg::JointTrajectoryPoint trajectory_point;
            trajectory_point.time_from_start = rclcpp::Duration(JointCommandsMsg.time_delta);
            trajectory_point.positions.assign(JointCommandsMsg.joint_commands, JointCommandsMsg.joint_commands + NUM_JOINTS);
            goal_msg.trajectory.points.push_back(trajectory_point);
            auto goal_handle_future = action_client->async_send_goal(goal_msg, common_goal_response);
        }
        last_id = msg_id;
    }

    rclcpp::spin(node);

    rclcpp::shutdown();
    return 0;
}
