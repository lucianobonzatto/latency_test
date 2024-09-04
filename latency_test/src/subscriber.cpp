#include <cstdio>
#include <chrono>
#include <memory>
#include <fstream>

#include <rclcpp/rclcpp.hpp>
#include "latency_test_msgs/msg/data.hpp"
#include "latency_test_msgs/srv/file_request.hpp"

using namespace std::chrono;
#define IN_SIZE (50)
#define SUBS_NUMBER (10000)
#define TIMEOUT_SECONDS (10)

class SubscriberTestNode : public rclcpp::Node
{
public:
    SubscriberTestNode() : Node("subscriber_test_node")
    {
        initialize_latencies();

        // Criar assinantes
        for (int i = 0; i < 50; i++)
        {
            subscription_[i] = this->create_subscription<latency_test_msgs::msg::Data>(
                "chatter", 10,
                [this, i](latency_test_msgs::msg::Data::SharedPtr msg)
                { this->subscription_callback(msg, i); });
        }

        timer_ = this->create_wall_timer(
            std::chrono::seconds(TIMEOUT_SECONDS),
            [this]() {
                save_latencies_to_csv();
                rclcpp::shutdown();
            });
    }

private:
    void subscription_callback(latency_test_msgs::msg::Data::SharedPtr msg, int id)
    {
        auto latency = this->now() - msg->header.stamp;
        if (msg->sequence_number < IN_SIZE)
        {
            latencies[msg->sequence_number][id] = latency.nanoseconds();
            publish_time[msg->sequence_number][id] = msg->header.stamp.sec * 1000000000LL + msg->header.stamp.nanosec;
            // RCLCPP_INFO(this->get_logger(), "%d: %ld ns", msg->sequence_number, latency.nanoseconds());
            if(msg->sequence_number == IN_SIZE-1)
            {
                RCLCPP_INFO(this->get_logger(), "%d: finalized", id);
            }
        }
    }

    void save_latencies_to_csv()
    {
        std::string csv_file_path = "/home/ubuntu22/ros2_ws/src/latency_test/data/subscriber/name.csv";
        RCLCPP_INFO(this->get_logger(), "save CSV file: %s", csv_file_path.c_str());
        std::ofstream csv_file(csv_file_path, std::ios::out | std::ios::trunc);

        if (!csv_file.is_open())
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to open CSV file.");
            return;
        }

        // Escrever o cabeçalho no arquivo CSV
        csv_file << "Sequence;Data_size;publish_interval;cpu;vm;vm_bytes;Latency(ns);publish_time(ns)" << std::endl;
    //     for (size_t i = 0; i < IN_SIZE; ++i)
    //     {
    //         csv_file << i << ";"
    //                  << test_request.size << ";"
    //                  << test_request.publish_interval << ";"
    //                  << test_request.cpu  << ";"
    //                  << test_request.vm  << ";"
    //                  << test_request.vm_bytes  << ";"
    //                  << latencies[i]  << ";"
    //                  << publish_time[i]
    //                  << std::endl;
    //         latencies[i] = 0;
    //         publish_time[i] = 0;
    //     }
        csv_file.close();
    }

    void initialize_latencies()
    {
        for (size_t j = 0; j < SUBS_NUMBER; j++)
        {
            for (size_t i = 0; i < IN_SIZE; i++)
            {
                latencies[i][j] = 0;
                publish_time[i][j] = 0;
            }
        }
    }
    
    rclcpp::Subscription<latency_test_msgs::msg::Data>::SharedPtr subscription_[SUBS_NUMBER];
    long latencies[IN_SIZE][SUBS_NUMBER];
    long publish_time[IN_SIZE][SUBS_NUMBER];
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<SubscriberTestNode>();
    rclcpp::spin(node);

    rclcpp::shutdown();
    return 0;
}
