#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "sys/ipc.h"
#include "sys/shm.h"

const int NUM_JOINTS = 4;

struct shm_msg
{
    unsigned int msg_id;
    double position[NUM_JOINTS];
    //double velocity[NUM_JOINTS];
}shm_msg;

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
            shmid = shmget(key, sizeof(shm_msg), 0666|IPC_CREAT);
            if(shmid == -1)
            {
                RCLCPP_ERROR(this->get_logger(), "ERROR CREATING SHARED MEMORY");
            }
            else
            {
                shm_data = (uint8_t *)shmat(shmid, NULL, 0);
                if(shm_data == (void *)-1)
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
        void topic_callback(const sensor_msgs::msg::JointState::SharedPtr msg) const
        {
            static unsigned long msg_id = 0;
            double* position = &(msg->position)[0];
            //double* velocity = &(msg->velocity)[0];
            msg_id++;
            serialize(position, msg_id);
        }
        rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr subscription_;
        key_t key;
        int shmid;
        uint8_t *shm_data;

        void serialize(double* position, unsigned int msg_id) const
        {
            unsigned long *q = (unsigned long*)shm_data;
            *q = msg_id;
            q++;
            double* t = (double*)q;
            int i = 0;
            while (i < NUM_JOINTS)
            {
                *t = position[i];
                t++;
                i++;
                //printf("%i\n", i);
            }
        }
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    std::shared_ptr<TFSubscriber> Subscriber = std::make_shared<TFSubscriber>();
    rclcpp::spin(Subscriber);
    Subscriber->detach_shm();
    rclcpp::shutdown();
    return 0;
}