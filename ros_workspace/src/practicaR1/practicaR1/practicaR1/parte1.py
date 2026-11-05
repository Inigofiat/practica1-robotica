#T1: Crear un Suscriptor en ROS 2 que lea en tiempo real el estado de las articulaciones del robot.

import copy
import csv
import os
from datetime import datetime
from math import cos, pi, sin
from typing import Callable, Dict, List, Optional, TextIO

import rclpy
from geometry_msgs.msg import Pose
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import String

from practicaR1.control_robot import ControlRobot

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

#T3: constantes del grabador de datos
CARPETA_DATOS: str = os.path.expanduser("~/ros_workspace/datos_trayectorias")
FORMATO_FECHA_ARCHIVO: str = "%Y%m%d_%H%M%S"
INTERVALO_MINIMO_S: float = 1e-4  # mensajes más juntos que esto se descartan
PERIODO_LOG_S: float = 1.0  # mostrar el progreso como mucho una vez por segundo

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

class MultiplesTrayectorias(ControlRobot):
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
        for posicion in POSICIONES_JOINT:
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
        for numero_punto in range(1, PUNTOS_CIRCULO + 1):
            angulo: float = pi + 2 * pi * numero_punto / PUNTOS_CIRCULO
            punto: Pose = copy.deepcopy(inicio)
            punto.position.x = centro_x + RADIO_CIRCULO_M * cos(angulo)
            punto.position.y = centro_y + RADIO_CIRCULO_M * sin(angulo)
            circulo.append(punto)

        return self.mover_trayectoria(circulo)


#T3: Procesar los datos

class GrabadorCinematica(Node):
    """Guarda posición, velocidad y aceleración de las articulaciones en un CSV por trayectoria."""

    def __init__(self) -> None:
        super().__init__("grabador_cinematica")
        os.makedirs(CARPETA_DATOS, exist_ok=True)

        self._trayectoria_actual: str = SIN_TRAYECTORIA
        self._archivo: Optional[TextIO] = None
        self._escritor_csv = None
        self._tiempo_inicio: float = 0.0
        self._tiempo_anterior: Optional[float] = None
        self._velocidades_anteriores: List[float] = []
        self._filas_guardadas: int = 0

        self.create_subscription(
            String, TOPIC_TRAYECTORIA, self._cambiar_trayectoria, TAMANO_COLA
        )
        self.create_subscription(
            JointState, TOPIC_ARTICULACIONES, self._guardar_muestra, TAMANO_COLA
        )
        self.get_logger().info(f"Guardando los datos en {CARPETA_DATOS}")

    def cerrar(self) -> None:
        """Cierra el CSV abierto, si lo hay. Se llama al terminar el programa."""
        self._cerrar_csv()

    def _cambiar_trayectoria(self, mensaje: String) -> None:
        """Cierra el CSV anterior y abre uno nuevo cuando cambia la trayectoria."""
        if mensaje.data == self._trayectoria_actual:
            return
        self._cerrar_csv()
        self._trayectoria_actual = mensaje.data
        if self._trayectoria_actual != SIN_TRAYECTORIA:
            self._abrir_csv(self._trayectoria_actual)

    def _abrir_csv(self, nombre_trayectoria: str) -> None:
        """Crea un CSV nuevo, p. ej. joint_20261026_190512.csv, y escribe la cabecera."""
        fecha: str = datetime.now().strftime(FORMATO_FECHA_ARCHIVO)
        ruta: str = os.path.join(CARPETA_DATOS, f"{nombre_trayectoria}_{fecha}.csv")
        self._archivo = open(ruta, "w", newline="")
        self._escritor_csv = csv.writer(self._archivo)

        cabecera: List[str] = ["tiempo_s", "tiempo_relativo_s"]
        for magnitud in ("pos", "vel", "acc"):
            for articulacion in ARTICULACIONES:
                cabecera.append(f"{magnitud}_{articulacion}")
        self._escritor_csv.writerow(cabecera)

        self._tiempo_anterior = None
        self._velocidades_anteriores = []
        self._filas_guardadas = 0
        self.get_logger().info(f"Grabando '{nombre_trayectoria}' en {ruta}")

    def _cerrar_csv(self) -> None:
        """Cierra el CSV en curso, asegurando que todo queda escrito en disco."""
        if self._archivo is None:
            return
        self._archivo.close()
        self.get_logger().info(
            f"Trayectoria '{self._trayectoria_actual}' guardada: {self._filas_guardadas} filas"
        )
        self._archivo = None
        self._escritor_csv = None

    def _guardar_muestra(self, mensaje: JointState) -> None:
        """Calcula la aceleración y escribe una fila por cada mensaje de /joint_states."""
        if self._escritor_csv is None:
            return

        tiempo: float = (
            mensaje.header.stamp.sec
            + mensaje.header.stamp.nanosec / NANOSEGUNDOS_POR_SEGUNDO
        )
        indice: Dict[str, int] = {}
        for posicion_en_lista, nombre in enumerate(mensaje.name):
            indice[nombre] = posicion_en_lista
        if not all(articulacion in indice for articulacion in ARTICULACIONES):
            return
        if len(mensaje.velocity) != len(mensaje.name):
            return

        posiciones: List[float] = [mensaje.position[indice[a]] for a in ARTICULACIONES]
        velocidades: List[float] = [mensaje.velocity[indice[a]] for a in ARTICULACIONES]

        if self._tiempo_anterior is None:
            self._tiempo_inicio = tiempo
            aceleraciones: List[float] = [0.0] * len(ARTICULACIONES)
        else:
            intervalo: float = tiempo - self._tiempo_anterior
            if intervalo < INTERVALO_MINIMO_S:
                return
            aceleraciones = self._derivar(velocidades, self._velocidades_anteriores, intervalo)

        self._escritor_csv.writerow(
            [tiempo, tiempo - self._tiempo_inicio] + posiciones + velocidades + aceleraciones
        )
        self._filas_guardadas += 1
        self._tiempo_anterior = tiempo
        self._velocidades_anteriores = velocidades
        self.get_logger().info(
            f"'{self._trayectoria_actual}': {self._filas_guardadas} filas guardadas",
            throttle_duration_sec=PERIODO_LOG_S,
        )

    @staticmethod
    def _derivar(actuales: List[float], anteriores: List[float], intervalo: float) -> List[float]:
        """Derivada numérica de cada articulación: (valor_ahora - valor_antes) / intervalo."""
        derivadas: List[float] = []
        for ahora, antes in zip(actuales, anteriores):
            derivadas.append((ahora - antes) / intervalo)
        return derivadas


def main() -> None:
    rclpy.init()
    nodo = SuscriptorArticulaciones()
    try:
        rclpy.spin(nodo)
    except KeyboardInterrupt:
        pass
    nodo.destroy_node()
    rclpy.try_shutdown()


def main_trayectorias() -> None:
    """Tarea 2: solo las trayectorias."""
    rclpy.init()
    nodo = MultiplesTrayectorias()
    try:
        nodo.ejecutar_trayectorias()
    except KeyboardInterrupt:
        pass
    nodo.ejecutor.shutdown()
    nodo.destroy_node()
    rclpy.try_shutdown()


def main_grabador() -> None:
    """Tarea 3: solo el grabador (junto a trayectorias, en otra terminal)."""
    rclpy.init()
    grabador = GrabadorCinematica()
    try:
        rclpy.spin(grabador)
    except KeyboardInterrupt:
        pass
    grabador.cerrar()
    grabador.destroy_node()
    rclpy.try_shutdown()


def main_practica() -> None:
    """Tareas 2 y 3 a la vez: mueve el robot y guarda un CSV por trayectoria."""
    rclpy.init()
    grabador = GrabadorCinematica()
    trayectorias = MultiplesTrayectorias()
    trayectorias.ejecutor.add_node(grabador)
    try:
        trayectorias.ejecutar_trayectorias()
    except KeyboardInterrupt:
        pass
    trayectorias.ejecutor.shutdown()
    grabador.cerrar()
    grabador.destroy_node()
    trayectorias.destroy_node()
    rclpy.try_shutdown()


if __name__ == "__main__":
    main()