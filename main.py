from cerebro import CerebroRobot
from plano import MundoSimulacion
 
def iniciar_Todo():
    cerebro = CerebroRobot()
    mi_mundo = MundoSimulacion()
 
    print("Simulación iniciada — Robot operando con regla de mano derecha")
 
    try:
        while True:
            distancias_lidar = mi_mundo.simular_lidar()
            cerebro.percibir_entorno(distancias_lidar)
            ordenes_motor = cerebro.decidir_accion()
            mi_mundo.mover_robot(ordenes_motor)
            mi_mundo.actualizar_fisicas()
    except KeyboardInterrupt:
        import pybullet as p
        p.disconnect()
        print("Simulación finalizada.")
 
if __name__ == "__main__":
    iniciar_Todo()