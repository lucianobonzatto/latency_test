# latency_test

```
ros2 run latency_test publisher
```

```
ros2 service call /start_test latency_test_msgs/srv/FileRequest "{name: 'latencies', size: 10, publish_interval: 10, cpu: 0, vm: 0, vm_bytes: 0}"
```

# TODO

retirar outliers -> remover baseado no desvio padrão
calcular a previsibilidade pegando pares de casos e depois fazer a média dos resultados
calcular a moda
```
-------------------------------------------------------------
| publis  |    topico   |   subsc   |    stress   |   rede  |
-------------------------------------------------------------
| 1       |    1        |   1       |  sim/nao    |   nao   |
| mult    |    1        |   1       |    nao      |   nao   |
| 1       |    1        |   mult    |    nao      |   nao   |
| mult    |    1        |   mult    |    nao      |   nao   |
-------------------------------------------------------------
```