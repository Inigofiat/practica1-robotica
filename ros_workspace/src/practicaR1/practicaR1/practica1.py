#Crear un Suscriptor en ROS 2 que lea en tiempo real el estado de las articulaciones del robot.

from typing import List, Dict

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

TOPIC_ARTICULACIONES: str = "/joint_states"
TAMANO_COLA: int = 10 
NANOSEGUNDOS_POR_SEGUNDO: float = 1e9

ARTICULACIONES: List[str] = [
    "shoulder_pan_joint", #base
    "shoulder_lift_joint", #hombro
    "elbow_joint", #codo
    "wrist_1_joint", #muñeca 1
    "wrist_2_joint", #muñeca 2
    "wrist_3_joint"] #muñeca 3

class SuscriptorArticulaciones(Node):
    def __init__(self) -> None:
        super().__init__("suscriptor_articulaciones")
        self.create_subscription(
            JointState, TOPIC_ARTICULACIONES, self._leer_articulaciones, TAMANO_COLA
        )
        
    def _leer_articulaciones (self, mensaje: JointState) -> None:
        tiempo: float = mensaje.header.stamp.sec + mensaje.header.stamp.nanosec / NANOSEGUNDOS_POR_SEGUNDO
        indice: Dict[str, int] = {}
        for posicion_en_lista, nombre in enumerate(mensaje.name):
            indice[nombre] = posicion_en_lista
        hay_velocidad: bool = len(mensaje.velocity) == len(mensaje.name)
        
        for articulacion in ARTICULACIONES:
            if articulacion not in indice:
                continue
            i = indice[articulacion]
            posicion = mensaje.position[i]
            if hay_velocidad:
                velocidad = mensaje.velocity[i] 
            else:
                velocidad = 0.0
            self.get_logger().info(
                f"t={tiempo:.3f}s  {articulacion}: pos={posicion:+.4f} rad  vel={velocidad:+.4f} rad/s"
            )

def main() -> None:
    rclpy.init()
    nodo = SuscriptorArticulaciones()
    try:
        rclpy.spin(nodo)
    except KeyboardInterrupt:
        pass
    nodo.destroy_node()
    rclpy.try_shutdown()


if __name__ == "__main__":
    main()