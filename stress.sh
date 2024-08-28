#!/bin/bash
# Script para rodar stress e chamar serviço ROS2

# Configurações iniciais
DATA_SIZE=10
PUBLISH_INTERVAL=10
STRESS_DURATION=10 # Duração do estresse em segundos

# Função para chamar o serviço ROS2
call_ros2_service() {
  ros2 service call /start_test latency_test_msgs/srv/FileRequest "{name: '$TEST_NAME', size: $DATA_SIZE, publish_interval: $PUBLISH_INTERVAL, cpu: $1, vm: $2, vm_bytes: $3}"
}

# Função para criar o nome do arquivo
name() {
  TEST_NAME="cpu_$1_vm_$2_vmbytes_$3_size_$4_interval_$5"
}

# Níveis de estresse
CPU_LOAD=(1)
VM_LOAD=(1 2 4)
VM_BYTES=(50000)

# Loop através de diferentes configurações de estresse
for cpu in "${CPU_LOAD[@]}"; do
  for vm in "${VM_LOAD[@]}"; do
    for vm_bytes in "${VM_BYTES[@]}"; do
      name $cpu $vm $vm_bytes $DATA_SIZE $PUBLISH_INTERVAL
      
      # Aplicar estresse
      stress-ng --vm $vm --vm-bytes $vm_bytes --timeout ${STRESS_DURATION}
      
      # Chamar o serviço ROS2
    #   call_ros2_service $cpu $vm $vm_bytes
      
      # Esperar o teste ROS2 terminar
    #   sleep $STRESS_DURATION # Pausa para assegurar que o sistema tenha tempo de estabilizar
      
      echo "Teste com CPU=$cpu, VM=$vm, VM_BYTES=$vm_bytes concluído."
    done
  done
done

echo "Todos os testes concluídos."
