import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory
from sensor_msgs.msg import JointState
import os
import math

# Importa a API da Dobot (certifique-se de que dobot_api.py está na mesma pasta)
from dobot_api import DobotApiMove

class RealRobotBridge(Node):
    def __init__(self):
        super().__init__('real_robot_bridge')

        # 1. Configuração do IP do Robô
        self.ip = os.getenv("IP_address", "192.168.1.6") # Mude para o IP padrão do seu robô se necessário
        try:
            self.move_api = DobotApiMove(self.ip, 30003)
            self.get_logger().info(f"Conectado à porta de movimento (30003) no IP {self.ip}")
        except Exception as e:
            self.get_logger().error(f"Falha ao conectar no robô: {e}")

        # 2. RELAY DO FEEDBACK: Lê do tópico da Dobot e repassa para o MoveIt
        self.js_sub = self.create_subscription(JointState, '/joint_states_robot', self.js_relay_callback, 10)
        self.js_pub = self.create_publisher(JointState, '/joint_states', 10)

        # 3. INTERCEPTAÇÃO DE COMANDOS: Lê o Servo e envia para a API
        self.traj_sub = self.create_subscription(JointTrajectory, '/cr3_arm_controller/joint_trajectory', self.traj_callback, 10)

    def js_relay_callback(self, msg):
        # Repassa a mensagem do feedback.py para o tópico padrão do MoveIt
        msg.header.stamp = self.get_clock().now().to_msg()
        self.js_pub.publish(msg)

    def traj_callback(self, msg):
        if len(msg.points) > 0:
            # O MoveIt envia em Radianos. A API do Dobot espera Graus.
            target_rad = msg.points[0].positions
            target_deg = [math.degrees(pos) for pos in target_rad]

            # Tempo de lookahead. No seu servo_parameters.yaml o publish_period é 0.02s
            t = 0.03

            # Envia o comando ServoJ para a API do Dobot (dynParams é uma lista vazia [])
            self.move_api.ServoJ(
                target_deg[0], target_deg[1], target_deg[2],
                target_deg[3], target_deg[4], target_deg[5],
                t, []
            )

def main(args=None):
    rclpy.init(args=args)
    node = RealRobotBridge()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()