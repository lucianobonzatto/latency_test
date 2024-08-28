#!/bin/bash
DATA_SIZE=10
PUBLISH_INTERVAL=10
STRESS_DURATION=10
CPU_LOAD=(1)
VM_LOAD=(1 2 3 4 5 6)
VM_BYTES=(1000000000 2000000000 3000000000 4000000000 5000000000)

call_ros2_service() {
  ros2 service call /start_test latency_test_msgs/srv/FileRequest "{name: '$TEST_NAME', size: $DATA_SIZE, publish_interval: $PUBLISH_INTERVAL, cpu: $1, vm: $2, vm_bytes: $3}"
}
name() {
  TEST_NAME="cpu_$1_vm_$2_vmbytes_$3_size_$4_interval_$5"
}

for vm in "${VM_LOAD[@]}"; do
  for vm_bytes in "${VM_BYTES[@]}"; do
    echo "Aplicando estresse com CPU=0, VM=$vm, VM_BYTES=$vm_bytes"
    name 0 $vm $vm_bytes $DATA_SIZE $PUBLISH_INTERVAL

    stress-ng --vm $vm --vm-bytes $vm_bytes --timeout ${STRESS_DURATION}s &
    STRESS_PID=$!

    sleep 5
    call_ros2_service 0 $vm $vm_bytes
    
    wait $STRESS_PID
    echo "Teste com CPU=0, VM=$vm, VM_BYTES=$vm_bytes concluído."
    sleep 5 
  done
done


# for cpu in "${CPU_LOAD[@]}"; do
#   for vm in "${VM_LOAD[@]}"; do
#     for vm_bytes in "${VM_BYTES[@]}"; do
#       echo "Aplicando estresse com CPU=$cpu, VM=$vm, VM_BYTES=$vm_bytes"
#       name $cpu $vm $vm_bytes $DATA_SIZE $PUBLISH_INTERVAL

#       stress-ng --cpu $cpu --vm $vm --vm-bytes $vm_bytes --timeout ${STRESS_DURATION}s &
#       STRESS_PID=$!

#       sleep 3
#       call_ros2_service $cpu $vm $vm_bytes
      
#       wait $STRESS_PID
#       echo "Teste com CPU=$cpu, VM=$vm, VM_BYTES=$vm_bytes concluído."
#       sleep 5 
#     done
#   done
# done

echo "Todos os testes concluídos."
