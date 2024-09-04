#include <cstdio>
#include <chrono>
#include <memory>
#include <fstream>

#include <rclcpp/rclcpp.hpp>
#include "latency_test_msgs/msg/data.hpp"
#include "latency_test_msgs/srv/file_request.hpp"

using namespace std::chrono;
#define OUT_SIZE (50)

class LatencyTestNode : public rclcpp::Node
{
public:
    LatencyTestNode() : Node("latency_test_node")
    {
        initialize_latencies();
        message.sequence_number = 0;
        message.header.frame_id = "base_frame";

        // Criar um publicador
        publisher_ = this->create_publisher<latency_test_msgs::msg::Data>("chatter", 10);

        // Criar um assinante
        subscription_ = this->create_subscription<latency_test_msgs::msg::Data>(
            "chatter", 10,
            [this](latency_test_msgs::msg::Data::SharedPtr msg)
            { this->subscription_callback(msg); });

        // Criar um serviço
        service_ = this->create_service<latency_test_msgs::srv::FileRequest>("start_test", 
            [this](const std::shared_ptr<latency_test_msgs::srv::FileRequest::Request> request,
            std::shared_ptr<latency_test_msgs::srv::FileRequest::Response>      response)
            { this->start_test(request, response); });
    }

private:
    void subscription_callback(latency_test_msgs::msg::Data::SharedPtr msg)
    {
        auto latency = this->now() - msg->header.stamp;
        if (msg->sequence_number < OUT_SIZE)
        {
            latencies[msg->sequence_number] = latency.nanoseconds();
            publish_time[msg->sequence_number] = msg->header.stamp.sec * 1000000000LL + msg->header.stamp.nanosec;
            RCLCPP_INFO(this->get_logger(), "%d: %ld ns", msg->sequence_number, latency.nanoseconds());
            if(msg->sequence_number == OUT_SIZE-1)
            {
                timer_->cancel();
                save_latencies_to_csv();
                msg->sequence_number = 0;
            }
        }
        else
        {
            timer_->cancel();
        }
    }

    void save_latencies_to_csv()
    {
        std::string csv_file_path = "/home/ubuntu22/ros2_ws/src/latency_test/data/stress/" + test_request.name + ".csv";
        RCLCPP_INFO(this->get_logger(), "save CSV file: %s", csv_file_path.c_str());
        std::ofstream csv_file(csv_file_path, std::ios::out | std::ios::trunc);

        if (!csv_file.is_open())
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to open CSV file.");
            return;
        }

        // Escrever o cabeçalho no arquivo CSV
        csv_file << "Sequence;Data_size;publish_interval;cpu;vm;vm_bytes;Latency(ns);publish_time(ns)" << std::endl;
        for (size_t i = 0; i < OUT_SIZE; ++i)
        {
            csv_file << i << ";"
                     << test_request.size << ";"
                     << test_request.publish_interval << ";"
                     << test_request.cpu  << ";"
                     << test_request.vm  << ";"
                     << test_request.vm_bytes  << ";"
                     << latencies[i]  << ";"
                     << publish_time[i]
                     << std::endl;
            latencies[i] = 0;
            publish_time[i] = 0;
        }
        csv_file.close();
    }

    void initialize_latencies()
    {
        for (size_t i = 0; i < OUT_SIZE; i++)
        {
            latencies[i] = 0;
            publish_time[i] = 0;
        }
    }
    
    void start_test(const std::shared_ptr<latency_test_msgs::srv::FileRequest::Request> request,
            std::shared_ptr<latency_test_msgs::srv::FileRequest::Response>      response)
    {
        test_request = *request;
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"),
                    "request: %s \n\tdata_size: %ld publish_interval: %ld cpu: %ld vm: %ld vm_bytes: %ld",
                    test_request.name.c_str(),
                    test_request.size,
                    test_request.publish_interval,
                    test_request.cpu,
                    test_request.vm,
                    test_request.vm_bytes
                    );

        initialize_latencies();
        message.sequence_number = 0;
        message.data.resize(test_request.size);
        std::iota(message.data.begin(), message.data.end(), 1);
        message.header.frame_id = "base_frame";

        // Definir um temporizador para publicar mensagens
        timer_ = this->create_wall_timer(milliseconds(test_request.publish_interval), 
            [this](){
            message.header.stamp = this->now();
            publisher_->publish(message); 
            message.sequence_number++;});

        response->response = 1;
    }

    rclcpp::Publisher<latency_test_msgs::msg::Data>::SharedPtr publisher_;
    rclcpp::Subscription<latency_test_msgs::msg::Data>::SharedPtr subscription_;
    rclcpp::Service<latency_test_msgs::srv::FileRequest>::SharedPtr service_;
    rclcpp::TimerBase::SharedPtr timer_;
    latency_test_msgs::msg::Data message;
    latency_test_msgs::srv::FileRequest::Request test_request;
    long latencies[OUT_SIZE];
    long publish_time[OUT_SIZE];
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<LatencyTestNode>();
    rclcpp::spin(node);

    rclcpp::shutdown();
    return 0;
}
