#include <cstdio>
#include <chrono>
#include <memory>
#include <fstream>

#include <rclcpp/rclcpp.hpp>
#include "latency_test_msgs/msg/data.hpp"
#include "latency_test_msgs/srv/publish_request.hpp"

using namespace std::chrono;
#define OUT_SIZE (50)

class PublishTestNode : public rclcpp::Node
{
public:
    PublishTestNode() : Node("publisher_test_node")
    {
        message.sequence_number = 0;
        message.header.frame_id = "base_frame";

        // Criar um publicador
        publisher_ = this->create_publisher<latency_test_msgs::msg::Data>(
            "chatter", 10);

        // Criar um serviço
        service_ = this->create_service<latency_test_msgs::srv::PublishRequest>("start_publisher", 
            [this](const std::shared_ptr<latency_test_msgs::srv::PublishRequest::Request> request,
            std::shared_ptr<latency_test_msgs::srv::PublishRequest::Response>      response)
            { this->start_test(request, response); });
    }

private:
    
    void start_test(const std::shared_ptr<latency_test_msgs::srv::PublishRequest::Request> request,
            std::shared_ptr<latency_test_msgs::srv::PublishRequest::Response>      response)
    {
        test_request = *request;
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"),
                    "request: \n\tdata_size: %ld publish_interval: %ld",
                    test_request.size,
                    test_request.publish_interval
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
            message.sequence_number++;
            if(message.sequence_number == OUT_SIZE)
            {
                timer_->cancel();
                message.sequence_number = 0;
            }
            });

        response->response = 1;
    }

    rclcpp::Publisher<latency_test_msgs::msg::Data>::SharedPtr publisher_;
    rclcpp::Service<latency_test_msgs::srv::PublishRequest>::SharedPtr service_;
    rclcpp::TimerBase::SharedPtr timer_;
    latency_test_msgs::msg::Data message;
    latency_test_msgs::srv::PublishRequest::Request test_request;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<PublishTestNode>();
    rclcpp::spin(node);

    rclcpp::shutdown();
    return 0;
}
