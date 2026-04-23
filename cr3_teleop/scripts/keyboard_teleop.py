import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped
import sys
import select
import termios
import tty
import threading

msg = """
Controle o Braço! (MoveIt Servo)
---------------------------
Translação (Movimento Linear):
  W / S : Eixo X (Frente/Trás)
  A / D : Eixo Y (Esquerda/Direita)
  Q / E : Eixo Z (Cima/Baixo)

Orientação (Movimento Angular):
  U / O : Rotação X (Roll)
  I / K : Rotação Y (Pitch)
  J / L : Rotação Z (Yaw)

O robô para automaticamente ao soltar a tecla.
CTRL-C para sair.
"""

class KeyboardTeleop(Node):
    def __init__(self):
        super().__init__('keyboard_teleop')
        self.publisher_ = self.create_publisher(TwistStamped, '/servo_node/delta_twist_cmds', 10)
        
        # Separamos os vetores linear e angular
        self.current_linear = [0.0, 0.0, 0.0]
        self.current_angular = [0.0, 0.0, 0.0]
        
        # Publica a 10Hz de forma independente
        self.timer = self.create_timer(0.1, self.timer_callback)

    def timer_callback(self):
        twist_msg = TwistStamped()
        twist_msg.header.stamp = self.get_clock().now().to_msg()
        twist_msg.header.frame_id = 'base_link'
        
        # Atribui velocidades lineares
        twist_msg.twist.linear.x = self.current_linear[0]
        twist_msg.twist.linear.y = self.current_linear[1]
        twist_msg.twist.linear.z = self.current_linear[2]
        
        # Atribui velocidades angulares (Orientação)
        twist_msg.twist.angular.x = self.current_angular[0]
        twist_msg.twist.angular.y = self.current_angular[1]
        twist_msg.twist.angular.z = self.current_angular[2]
        
        self.publisher_.publish(twist_msg)

    def update_twist(self, key):
        # Mapeamento Linear
        mapping_linear = {
            'w': [1.0, 0.0, 0.0], 's': [-1.0, 0.0, 0.0],
            'a': [0.0, 1.0, 0.0], 'd': [0.0, -1.0, 0.0],
            'q': [0.0, 0.0, 1.0], 'e': [0.0, 0.0, -1.0],
        }
        
        # Mapeamento Angular
        mapping_angular = {
            'u': [1.0, 0.0, 0.0], 'o': [-1.0, 0.0, 0.0],
            'i': [0.0, 1.0, 0.0], 'k': [0.0, -1.0, 0.0],
            'j': [0.0, 0.0, 1.0], 'l': [0.0, 0.0, -1.0],
        }
        
        if key in mapping_linear:
            self.current_linear = mapping_linear[key]
            self.current_angular = [0.0, 0.0, 0.0] # Zera rotação ao mover linearmente
        elif key in mapping_angular:
            self.current_angular = mapping_angular[key]
            self.current_linear = [0.0, 0.0, 0.0]  # Zera translação ao rotacionar
        else:
            self.current_linear = [0.0, 0.0, 0.0]
            self.current_angular = [0.0, 0.0, 0.0]

def main(args=None):
    rclpy.init(args=args)
    node = KeyboardTeleop()
    print(msg)

    # Salva as configurações originais do terminal
    settings = termios.tcgetattr(sys.stdin)

    # Função que vai rodar em paralelo apenas lendo o teclado
    def keyboard_loop():
        tty.setraw(sys.stdin.fileno()) # Trava o terminal para não imprimir as teclas
        try:
            while rclpy.ok():
                # Espera 0.1s por uma tecla
                rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
                if rlist:
                    key = sys.stdin.read(1)
                    if key == '\x03':  # Código do CTRL-C
                        break
                    node.update_twist(key)
                else:
                    node.update_twist('') # Zera o movimento se nada for pressionado
        except Exception as e:
            pass
        finally:
            # Restaura o terminal antes de fechar
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
            node.destroy_node()
            rclpy.shutdown()

    # Inicia a Thread do teclado
    thread = threading.Thread(target=keyboard_loop)
    thread.start()

    # Inicia o loop do ROS 2 na Thread principal
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        thread.join()

if __name__ == '__main__':
    main()