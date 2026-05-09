from cerebro import CerebroRobot
from plano import MundoSimulacion

def iniciar_hackathon():
    # Instanciamos el trabajo de ambos
    mi_cerebro = CerebroRobot()
    mi_mundo = MundoSimulacion()

    print("¡Simulación iniciada! Robot operando de forma autónoma.")

    # Ciclo infinito de simulación (El "Game Loop")
    while True:
        # 1. PERCEPCIÓN: El mundo lee los sensores y se los pasa al cerebro
        distancia_actual = mi_mundo.simular_sensor_distancia()
        mi_cerebro.percibir_entorno(distancia_actual)

        # 2. PLANEACIÓN: El cerebro procesa y toma una decisión
        ordenes_motor = mi_cerebro.decidir_accion()

        # 3. CONTROL: El cerebro envía las órdenes al mundo físico
        mi_mundo.mover_robot(ordenes_motor)

        # 4. FÍSICAS: El simulador avanza un paso
        mi_mundo.actualizar_fisicas()

if __name__ == "__main__":
    iniciar_hackathon()