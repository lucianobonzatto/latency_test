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
        message.sequence_number = 0;
        message.header.frame_id = "base_frame";

        // Criar um publicador
        publisher_ = this->create_publisher<latency_test_msgs::msg::Data>(
            "chatter", 10);

        // Criar um serviço
        service_ = this->create_service<latency_test_msgs::srv::FileRequest>("start_test", 
            [this](const std::shared_ptr<latency_test_msgs::srv::FileRequest::Request> request,
            std::shared_ptr<latency_test_msgs::srv::FileRequest::Response>      response)
            { this->start_test(request, response); });
    }

private:
    
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
    rclcpp::Service<latency_test_msgs::srv::FileRequest>::SharedPtr service_;
    rclcpp::TimerBase::SharedPtr timer_;
    latency_test_msgs::msg::Data message;
    latency_test_msgs::srv::FileRequest::Request test_request;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<LatencyTestNode>();
    rclcpp::spin(node);

    rclcpp::shutdown();
    return 0;
}
