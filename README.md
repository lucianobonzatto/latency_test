# latency_test

```
ros2 run latency_test publisher
```

```
ros2 service call /start_test latency_test_msgs/srv/FileRequest "{name: 'latencies', size: 10, publish_interval: 10, cpu: 0, vm: 0, vm_bytes: 0}"
```

# TODO

corrigir o jitter

retirar outliers

analisar latencia no timer

```
-------------------------------------------------------------
| publis  |    topico   |   subsc   |    stress   |   rede  |
-------------------------------------------------------------
| 1       |    1        |   1       |    sim      |   nao   |
| mult    |    1        |   1       |    nao      |   nao   |
| 1       |    1        |   mult    |    nao      |   nao   |
-------------------------------------------------------------
```