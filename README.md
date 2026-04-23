# Gazebo
Para abrir o Gazebo do CR3, rode:

```
ros2 launch cr3_custom_gazebo cr3_gazebo.launch.py
```
# Gazebo + Moveit
ros2 launch cr3_moveit_config gazebo_moveit.launch.py

# Teleoperação

Abra o Gazebo (sem o Moveit para não haver conflito) no Terminal1:
```
ros2 launch cr3_custom_gazebo cr3_gazebo.launch.py
```
Rode o nó do servo no terminal 2:
```
ros2 launch cr3_moveit_config servo.launch.py
```
No terminal 3, habilite o serviço do servo:
```
ros2 service call /servo_node/start_servo std_srvs/srv/Trigger "{}"
```
E rode este comando para verificar se o tópico está recebendo os sinais de controle (os sinais aparecem depois que o arquivo python estiver rodando): 
```
ros2 topic echo /servo_node/delta_twist_cmds
```
No terminal 4 (opcional), rode este comando para verificar se está havendo algum erro de singularidade (deve retornar sempre 0, 2 indica erro de singularidade):
```
ros2 topic echo /servo_node/status
```
No terminal 5, ajuste a pose inicial do robô para uma posição longe da singularidade:
```
ros2 action send_goal /cr3_arm_controller/follow_joint_trajectory control_msgs/action/FollowJointTrajectory "{trajectory: {joint_names: ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6'], points: [{positions: [0.0, 0.6072, -1.7223, -0.2949, 1.6134, 0.0], time_from_start: {sec: 2, nanosec: 0}}]}}"
```
rode o programa de controle pelo teclado:
```
cd cr3_arm/cr3_teleop/scripts
python3 keyboard_teleop.py
```

# Teleoperação Robô real
Terminal 1 (O Bringup Original da Dobot):

```
export IP_address=SEU.IP.DO.ROBO (já configurado no bashrc)
ros2 launch dobot_bringup_v3 dobot_bringup_ros2.launch.py
```

Terminal 2 (MoveIt + Servo Real):
```
ros2 launch cr3_moveit_config real_moveit_servo.launch.py
```

Terminal 3 (A Nossa Ponte Mágica):
```
export IP_address=SEU.IP.DO.ROBO (já configurado no bashrc)
cd ~/cr3_arm/cr3_teleop/scripts
python3 real_robot_bridge.py
```

Terminal 4 (Ativar o Servo):
```
ros2 service call /servo_node/start_servo std_srvs/srv/Trigger "{}"
```
Terminal 5 (Seu Teclado):
```
cd ~/cr3_arm/cr3_teleop/scripts
python3 keyboard_teleop.py
```