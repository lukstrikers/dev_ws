#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "sys/ipc.h"
#include "sys/shm.h"

const int NUM_JOINTS = 24;

struct shm_msg
{
    unsigned long msg_id;
    double position[NUM_JOINTS];
} shm_msg;

class TFSubscriber : public rclcpp::Node
{
public:
    TFSubscriber()
        : Node("minimal_subscriber")
    {
        subscription_ = this->create_subscription<sensor_msgs::msg::JointState>(
            "/joint_states", 10, std::bind(&TFSubscriber::topic_callback, this, std::placeholders::_1));

        RCLCPP_INFO(this->get_logger(), "TF Subscriber is subscribed to /joint_states topic");

        key = ftok("/home/lucas/Documents/Unreal Projects/ROS2_Simulation_ABMI/Source/ROS2_Simulation_ABMI/data.conf", 1);
        shmid = shmget(key, sizeof(shm_msg), 0666 | IPC_CREAT);
        if (shmid == -1)
        {
            RCLCPP_ERROR(this->get_logger(), "ERROR CREATING SHARED MEMORY");
        }
        else
        {
            shm_data = (uint8_t *)shmat(shmid, NULL, 0);
            if (shm_data == (void *)-1)
            {
                RCLCPP_ERROR(this->get_logger(), "ERROR ATTACHING SHARED MEMORY");
            }
        }
    }
    void detach_shm()
    {
        shmdt(shm_data);
        shmctl(shmid, IPC_RMID, NULL);
    }

private:
    void topic_callback(const sensor_msgs::msg::JointState::SharedPtr msg)
    {
        static unsigned long msg_id = 0;
        double *position = msg->position.data();
        msg_id++;
        shm_msg.msg_id = msg_id;
        RCLCPP_INFO(this->get_logger(), "START");
        // Assuming NUM_JOINTS is the size of the joint_positions array
        if (msg->position.size() == NUM_JOINTS)
        {
            for (size_t i = 0; i < NUM_JOINTS; ++i)
            {
                // Access the joint position and update shm_msg
                shm_msg.position[i] = position[i];
            }

            memcpy(shm_data, &shm_msg, sizeof(shm_msg));
            RCLCPP_INFO(this->get_logger(), "Wrote data to shared memory");
        }
        else
        {
            RCLCPP_ERROR(this->get_logger(), "Received joint positions with unexpected size");
        }
        RCLCPP_INFO(this->get_logger(), "STOP");
    }

    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr subscription_;
    key_t key;
    int shmid;
    uint8_t *shm_data;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    std::shared_ptr<TFSubscriber> Subscriber = std::make_shared<TFSubscriber>();
    rclcpp::spin(Subscriber);
    Subscriber->detach_shm();
    rclcpp::shutdown();
    return 0;
}
