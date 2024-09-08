#include <cstdio>
#include <chrono>
#include <memory>
#include <fstream>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include "latency_test_msgs/msg/datamult.hpp"
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

        service_ = this->create_service<latency_test_msgs::srv::PublishRequest>("start_publisher", 
            [this](const std::shared_ptr<latency_test_msgs::srv::PublishRequest::Request> request,
            std::shared_ptr<latency_test_msgs::srv::PublishRequest::Response> response)
            { this->start_test(request, response); });
    }

private:
    
    void start_test(const std::shared_ptr<latency_test_msgs::srv::PublishRequest::Request> request,
            std::shared_ptr<latency_test_msgs::srv::PublishRequest::Response> response)
    {
        test_request = *request;
        int num_publishers_ = test_request.publisher_number;

        RCLCPP_INFO(rclcpp::get_logger("rclcpp"),
                    "request: \n\tdata_size: %ld publish_interval: %ld",
                    test_request.size,
                    test_request.publish_interval);

        // Criar o número de publishers definido pela chamada do serviço
        publishers_.clear();
        for (int i = 0; i < num_publishers_; ++i)
        {
            auto publisher = this->create_publisher<latency_test_msgs::msg::Datamult>(
                "chatter", 10);
            publishers_.push_back(publisher);
        }

        // Preparar a mensagem
        message.sequence_number = 0;
        message.data.resize(test_request.size);
        std::iota(message.data.begin(), message.data.end(), 1);
        message.header.frame_id = "base_frame";

        // Definir um temporizador para publicar mensagens
        timer_ = this->create_wall_timer(milliseconds(test_request.publish_interval), 
            [this](){
            for (size_t i = 0; i < publishers_.size(); ++i)
            {
                message.publisher_number = i;
                message.header.stamp = this->now();
                publishers_[i]->publish(message);
            }
            message.sequence_number++;
            if (message.sequence_number == OUT_SIZE)
            {
                timer_->cancel();
                publishers_.clear(); // remove os publishers
                message.sequence_number = 0;
            }
        });

        response->response = 1;
    }

    std::vector<rclcpp::Publisher<latency_test_msgs::msg::Datamult>::SharedPtr> publishers_;
    rclcpp::Service<latency_test_msgs::srv::PublishRequest>::SharedPtr service_;
    rclcpp::TimerBase::SharedPtr timer_;
    latency_test_msgs::msg::Datamult message;
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
