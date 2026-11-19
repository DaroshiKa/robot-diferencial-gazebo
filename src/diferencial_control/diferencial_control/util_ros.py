"""Utilidades comunes de los nodos."""
import rclpy
from rclpy.executors import ExternalShutdownException


def girar(nodo, al_salir=None):
    """rclpy.spin con cierre limpio (Ctrl+C / fin del launch) sin trazas de error."""
    try:
        rclpy.spin(nodo)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except RuntimeError:
        # carrera conocida de rclpy al recibir SIGINT mientras toma un mensaje
        if rclpy.ok():
            raise
    finally:
        if al_salir is not None:
            try:
                al_salir()
            except Exception:
                pass
        try:
            nodo.destroy_node()
        except Exception:
            pass
        rclpy.try_shutdown()
