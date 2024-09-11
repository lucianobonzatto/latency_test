#include <cstdio>
#include <chrono>
#include <memory>
#include <fstream>

#include <rclcpp/rclcpp.hpp>
#include "latency_test_msgs/msg/datamult.hpp"
#include "latency_test_msgs/srv/subscribe_request.hpp"  // Inclua o serviço SubscribeRequest

using namespace std::chrono;
#define IN_SIZE (100)
#define MAX_SUBS_NUMBER (1000)
#define MAX_PUBL_NUMBER (1000)
#define TIMEOUT_SECONDS (10)

class SubscriberTestNode : public rclcpp::Node
{
public:
    SubscriberTestNode() : Node("subscriber_test_node")
    {
        // Alocando memória dinamicamente para os arrays
        latencies = new long*[MAX_PUBL_NUMBER * IN_SIZE];
        publish_time = new long*[MAX_PUBL_NUMBER * IN_SIZE];
        publisher_number = new long*[MAX_PUBL_NUMBER * IN_SIZE];

        for (int i = 0; i < MAX_PUBL_NUMBER * IN_SIZE; ++i)
        {
            latencies[i] = new long[MAX_SUBS_NUMBER]();
            publish_time[i] = new long[MAX_SUBS_NUMBER]();
            publisher_number[i] = new long[MAX_SUBS_NUMBER]();
        }

        initialize_latencies();

        service_ = this->create_service<latency_test_msgs::srv::SubscribeRequest>(
            "start_subscribers",
            [this](const std::shared_ptr<latency_test_msgs::srv::SubscribeRequest::Request> request,
                   std::shared_ptr<latency_test_msgs::srv::SubscribeRequest::Response> response)
            {
                this->handle_service_request(request, response);
            });
    }

    ~SubscriberTestNode()
    {
        // Liberar a memória alocada dinamicamente
        for (int i = 0; i < MAX_PUBL_NUMBER * IN_SIZE; ++i)
        {
            delete[] latencies[i];
            delete[] publish_time[i];
            delete[] publisher_number[i];
        }
        delete[] latencies;
        delete[] publish_time;
        delete[] publisher_number;
    }

private:
    void handle_service_request(const std::shared_ptr<latency_test_msgs::srv::SubscribeRequest::Request> request,
                                std::shared_ptr<latency_test_msgs::srv::SubscribeRequest::Response> response)
    {
        test_request = *request;
        RCLCPP_INFO(this->get_logger(), "request: %ld %ld %ld %ld", 
            request->size,
            request->publish_interval,
            request->publisher_number,
            request->subscriber_number);

        int num_subscribers = request->subscriber_number;

        // Criar os assinantes conforme solicitado no serviço
        subscribers_.clear();
        for (int i = 0; i < num_subscribers; i++)
        {
            auto subscription = this->create_subscription<latency_test_msgs::msg::Datamult>(
                "chatter", 10,
                [this, i](latency_test_msgs::msg::Datamult::SharedPtr msg)
                { this->subscription_callback(msg, i); });
            subscribers_.push_back(subscription);
        }

        timer_ = this->create_wall_timer(
            std::chrono::seconds(TIMEOUT_SECONDS),
            [this]() {
                save_latencies_to_csv();
                initialize_latencies();
                subscribers_.clear();
                timer_->cancel();
            });
        
        // Exemplo de como definir a resposta do serviço
        response->response = true;
    }

    void subscription_callback(latency_test_msgs::msg::Datamult::SharedPtr msg, int id)
    {
        auto latency = this->now() - msg->header.stamp;
        if (msg->sequence_number < IN_SIZE)
        {
            int position = msg->sequence_number + msg->publisher_number * IN_SIZE;
            publisher_number[position][id] = msg->publisher_number;
            latencies[position][id] = latency.nanoseconds();
            publish_time[position][id] = msg->header.stamp.sec * 1000000000LL + msg->header.stamp.nanosec;
        }
    }

    void save_latencies_to_csv()
    {
        std::string name =  "size_" + std::to_string(test_request.size) +
                            "_interval_" + std::to_string(test_request.publish_interval) +
                            "_publisher_" + std::to_string(test_request.publisher_number) +
                            "_subscriber_" + std::to_string(test_request.subscriber_number);

        std::string csv_file_path = "/home/ubuntu22/ros2_ws/src/latency_test/data/subscriber/" + name + ".csv";
        RCLCPP_INFO(this->get_logger(), "save CSV file: %s", csv_file_path.c_str());
        std::ofstream csv_file(csv_file_path, std::ios::out | std::ios::trunc);

        if (!csv_file.is_open())
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to open CSV file.");
            return;
        }

        // Escrever o cabeçalho no arquivo CSV
        csv_file << "Sequence;subscriber_id;publisher_id;Data_size;publish_interval;publisher_number;subscriber_number;Latency(ns);publish_time(ns)" << std::endl;
        for (size_t j = 0; j < subscribers_.size(); ++j)
        {
            for (int i = 0; i < (test_request.publisher_number * IN_SIZE); ++i)
            {
                csv_file << i % IN_SIZE << ";"
                         << j << ";"
                         << publisher_number[i][j] << ";"
                         << test_request.size << ";"
                         << test_request.publish_interval << ";"
                         << test_request.publisher_number << ";"
                         << test_request.subscriber_number << ";"
                         << latencies[i][j]  << ";"
                         << publish_time[i][j]
                         << std::endl;
                latencies[i][j] = 0;
                publish_time[i][j] = 0;
            }
        }
        csv_file.close();
    }

    void initialize_latencies()
    {
        for (int i = 0; i < MAX_PUBL_NUMBER * IN_SIZE; ++i)
        {
            for (int j = 0; j < MAX_SUBS_NUMBER; ++j)
            {
                latencies[i][j] = 0;
                publish_time[i][j] = 0;
                publisher_number[i][j] = 0;
            }
        }
    }

    std::vector<rclcpp::Subscription<latency_test_msgs::msg::Datamult>::SharedPtr> subscribers_;
    long** latencies;
    long** publish_time;
    long** publisher_number;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Service<latency_test_msgs::srv::SubscribeRequest>::SharedPtr service_;
    latency_test_msgs::srv::SubscribeRequest::Request test_request;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<SubscriberTestNode>();
    rclcpp::spin(node);

    rclcpp::shutdown();
    return 0;
}
