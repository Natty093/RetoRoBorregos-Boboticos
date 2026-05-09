from cerebro import CerebroRobot
from plano import MundoSimulacion

def iniciar_Todo():
    # Instanciamos el trabajo de ambos
    cerebro = CerebroRobot()
    mi_mundo = MundoSimulacion()

    print("Simulación iniciada, Robot operando")

    # Ciclo infinito de simulación (El "Game Loop")
    while True:
        distancias_lidar = mi_mundo.simular_lidar()
        cerebro.percibir_entorno(distancias_lidar)
        ordenes_motor = cerebro.decidir_accion()
        mi_mundo.mover_robot(ordenes_motor)
        mi_mundo.actualizar_fisicas()

if __name__ == "__main__":
    iniciar_Todo()