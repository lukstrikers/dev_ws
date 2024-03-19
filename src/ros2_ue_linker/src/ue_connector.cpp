#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "sys/ipc.h"
#include "sys/shm.h"

const int NUM_JOINTS = 24;

const int MAX_JOINT_NAME_LENGTH = 18; // Taille maximale du nom du joint

struct shm_msg
{
    unsigned long msg_id;
    double position[NUM_JOINTS];
    char joint_names[NUM_JOINTS][MAX_JOINT_NAME_LENGTH]; // Tableau de noms de joint
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

        key = ftok("/home/lucas/Documents/Unreal Projects/ROS2_Simulation_ABMI/Source/ROS2_Simulation_ABMI/data.conf", 2);
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
        //shmctl(shmid, IPC_RMID, NULL);
    }

private:
    private:
    void topic_callback(const sensor_msgs::msg::JointState::SharedPtr msg)
    {
        static unsigned long msg_id = 0;
        double *position = &(msg->position)[0];
        std::vector<std::string> joint_names = msg->name; // Extract joint names from message
        
        msg_id++;
        serialize(joint_names, position, msg_id);
    }

    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr subscription_;
    key_t key;
    int shmid;
    uint8_t *shm_data;

    void serialize(std::vector<std::string>& joint_names, double* position, int msg_id)
    {
        unsigned long *q = (unsigned long*)shm_data;
        *q = msg_id;
        q++;

        double* t = (double*)q;
        int i = 0;
        while (i < joint_names.size())
        {
            std::cout << "Joint name: " << joint_names[i] << ", Value: " << position[i] << std::endl; // Print joint name and value
            *t = position[i];
            t++;
            i++;
        }
        q++;
        char* name = (char*)q;
        i = 0;
        while (i < joint_names.size())
        {
            std::cout << "Joint name: " << joint_names[i][0] << std::endl;
            *name = joint_names[i][0];
            name++;
            i++;
        }
    }
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
