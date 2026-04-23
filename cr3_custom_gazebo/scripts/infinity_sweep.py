#!/usr/bin/env python3

"""
Envia uma trajetória contínua para o controlador do CR3.

Observação:
- Este script NÃO passa por "todas as posições possíveis" do robô.
- Ele faz uma varredura contínua em espaço articular usando senoides
  com frequências e fases diferentes para explorar muitas poses.
- O movimento resultante é suave e útil para teste de simulação.

Controlador esperado:
  /cr3_arm_controller/joint_trajectory

Tipo de mensagem:
  trajectory_msgs/msg/JointTrajectory
"""

import math
from typing import List

import rclpy
from rclpy.node import Node

from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


class Cr3InfinitySweep(Node):
    def __init__(self) -> None:
        super().__init__("cr3_infinity_sweep")

        self.publisher_ = self.create_publisher(
            JointTrajectory,
            "/cr3_arm_controller/joint_trajectory",
            10,
        )

        self.joint_names: List[str] = [
            "joint1",
            "joint2",
            "joint3",
            "joint4",
            "joint5",
            "joint6",
        ]

        # Amplitudes seguras/moderadas para a simulação.
        # Ajuste se quiser movimentos maiores ou menores.
        self.amplitudes = [0.7, 0.6, 0.8, 0.6, 0.5, 0.8]

        # Offsets para manter o braço longe de poses ruins.
        self.offsets = [0.0, -0.5, 1.0, 0.0, 0.3, 0.0]

        # Frequências angulares relativas.
        # Combinações diferentes ajudam a explorar mais poses.
        self.freqs = [1.0, 2.0, 1.5, 2.5, 1.2, 1.8]

        # Fases iniciais diferentes.
        self.phases = [0.0, math.pi / 2, math.pi / 4, math.pi, math.pi / 3, math.pi / 6]

        # Período de publicação.
        self.dt = 0.2

        # Horizonte curto de trajetória.
        # O controlador receberá sempre um ponto "à frente".
        self.lookahead_sec = 1.0

        self.t = 0.0
        self.timer = self.create_timer(self.dt, self.publish_trajectory)

        self.get_logger().info("Nó de varredura articular iniciado.")

    def generate_positions(self, t: float) -> List[float]:
        """Gera uma pose articular suave em função do tempo."""
        q = []
        for amp, off, freq, phase in zip(
            self.amplitudes, self.offsets, self.freqs, self.phases
        ):
            value = off + amp * math.sin(freq * t + phase)
            q.append(value)
        return q

    def publish_trajectory(self) -> None:
        """
        Publica uma trajetória com 2 pontos:
        - ponto atual
        - ponto futuro curto

        Isso tende a produzir um movimento mais suave no controlador.
        """
        msg = JointTrajectory()
        msg.joint_names = self.joint_names

        # Ponto 1
        p1 = JointTrajectoryPoint()
        p1.positions = self.generate_positions(self.t)
        p1.time_from_start.sec = 0
        p1.time_from_start.nanosec = int(0.5 * 1e9)

        # Ponto 2
        future_t = self.t + self.lookahead_sec
        p2 = JointTrajectoryPoint()
        p2.positions = self.generate_positions(future_t)
        p2.time_from_start.sec = 1
        p2.time_from_start.nanosec = int(0.0 * 1e9)

        msg.points = [p1, p2]

        self.publisher_.publish(msg)
        self.t += self.dt


def main(args=None) -> None:
    rclpy.init(args=args)
    node = Cr3InfinitySweep()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Encerrando nó.")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()