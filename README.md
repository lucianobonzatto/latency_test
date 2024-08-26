# latency_test

```
ros2 run latency_test publisher
```

```
ros2 service call /start_test latency_test_msgs/srv/FileRequest "{name: 'latencies', size: 10, publish_interval: 10, cpu: 0, vm: 0, vm_bytes: 0}"
```