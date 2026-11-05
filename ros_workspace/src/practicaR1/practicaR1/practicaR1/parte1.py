#T1: Crear un Suscriptor en ROS 2 que lea en tiempo real el estado de las articulaciones del robot.

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

TOPIC_TRAYECTORIA: str = "/trayectoria_actual"
SIN_TRAYECTORIA: str = ""

MEDIO_PI: float = pi/2 # --> 90º 

POSICION_INICIAL: List[float] = [0.0, -MEDIO_PI, -MEDIO_PI, -MEDIO_PI, MEDIO_PI, 0.0]  

#TRAYECTORIA JOINT
POSICIONES_JOINT: List[List[float]] = [
    [pi / 4,  -MEDIO_PI, -MEDIO_PI, -MEDIO_PI, MEDIO_PI, 0.0], # gira la base 45°
    [-pi / 4, -MEDIO_PI, -pi / 3, -2 * pi / 3, MEDIO_PI, pi / 4] # base -45°, codo y muñecas
]

#TRAYECTORIA LINEAL
LADO_CUADRADO_M: float = 0.10

#TRAYECTORIA CIRCULAR
RADIO_CIRCULO_M: float = 0.05
PUNTOS_CIRCULO: int = 36 # 36 puntos = uno cada 10 grados

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

#T2: Ejecución de múltiples trayectorias

class MultiplesTrayectorias(Control_robot):
    def __init__(self) -> None: 
        super().__init__()
        self._publicador = self.create_publisher(String, TOPIC_TRAYECTORIA, TAMANO_COLA)

    def ejecutar_trayectorias(self) -> None:
        trayectorias: Dict[str, Callable[[], bool]] = {
            "joint": self._trayectoria_joint,
            "lineal": self._trayectoria_lineal,
            "circular": self._trayectoria_circular,
        }
        for nombre, ejecutar in trayectorias.items():
            self._ir_a_posicion_inicial()
            self.get_logger().info(f"Iniciando trayectoria {nombre.upper()}")
            self._anunciar_trayectoria(nombre)
            exito: bool = ejecutar()
            self._anunciar_trayectoria(SIN_TRAYECTORIA)
            if exito:
                self.get_logger().info(f"Trayectoria {nombre.upper()} completada")
            else:
                self.get_logger().error(f"La trayectoria {nombre.upper()} no se pudo completar")
        self._ir_a_posicion_inicial()

    def _anunciar_trayectoria(self, nombre: str) -> None:
        mensaje = String()
        mensaje.data = nombre
        self._publicador.publish(mensaje)

    def _ir_a_posicion_inicial(self) -> None:
        if not self.mover_articulaciones(POSICION_INICIAL):
            self.get_logger().warn("No se pudo alcanzar la posición inicial")

    def _trayectoria_joint(self) -> bool:
        for posiciones in POSICIONES_JOINT:
            if not self.mover_articulaciones(posicion):
                return False
        return True

    def _trayectoria_lineal(self) -> bool:
        esquina: Pose = self.pose_actual()
        cuadrado: List[Pose] = []

        esquina.position.x += LADO_CUADRADO_M
        cuadrado.append(copy.deepcopy(esquina))
        esquina.position.y += LADO_CUADRADO_M
        cuadrado.append(copy.deepcopy(esquina))
        esquina.position.x -= LADO_CUADRADO_M
        cuadrado.append(copy.deepcopy(esquina))
        esquina.position.y -= LADO_CUADRADO_M
        cuadrado.append(copy.deepcopy(esquina))

        return self.mover_trayectoria(cuadrado)

    def _trayectoria_circular(self) -> bool:
        inicio: Pose = self.pose_actual()
        centro_x: float = inicio.position.x + RADIO_CIRCULO_M
        centro_y: float = inicio.position.y

        circulo: List[Pose] = []
        for numero_punto in range(PUNTOS_CIRCULO + 1):
            angulo: float = pi + 2 * pi * numero_punto / PUNTOS_CIRCULO
            punto: Pose = copy.deepcopy(inicio)
            punto.position.x = centro_x + RADIO_CIRCULO_M * cos(angulo)
            punto.position.y = centro_y + RADIO_CIRCULO_M * sin(angulo)
            circulo.append(punto)

        return self.mover_trayectoria(circulo)            


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