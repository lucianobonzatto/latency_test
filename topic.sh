#!/bin/bash

# Definir vetores de valores para cada parâmetro
sizes=(10 100 200 400 600 800 1000 5000 10000 50000 100000 500000 1000000)
# sizes=(10)
publish_intervals=(10)
# publisher_numbers=(1 2 3 4 5 6 7 8 9 10 20 30 40 60 80 100)
publisher_numbers=(1 2 3 4 5 6 7 8 9 10 20 30)
# publisher_numbers=(1)
subscriber_numbers=(1 2 3 4 5 6 7 8 9 10 20 30)
# subscriber_numbers=(1)

# Loop sobre os valores para chamar os serviços
for size in "${sizes[@]}"; do
  for publish_interval in "${publish_intervals[@]}"; do
    for publisher_number in "${publisher_numbers[@]}"; do
      for subscriber_number in "${subscriber_numbers[@]}"; do

        echo "Chamando subscribers com: size=$size, publish_interval=$publish_interval, publisher_number=$publisher_number, subscriber_number=$subscriber_number"
        ros2 service call /start_subscribers latency_test_msgs/srv/SubscribeRequest "{size: $size, publish_interval: $publish_interval, publisher_number: $publisher_number, subscriber_number: $subscriber_number}"

        sleep 1

        echo "Chamando publisher com: size=$size, publish_interval=$publish_interval, publisher_number=$publisher_number"
        ros2 service call /start_publisher latency_test_msgs/srv/PublishRequest "{size: $size, publish_interval: $publish_interval, publisher_number: $publisher_number}"

        sleep 10

      done
    done
  done
done
