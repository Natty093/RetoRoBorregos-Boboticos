import pybullet as p
import pybullet_data
import time
import math

class MundoSimulacion:
    def __init__(self):
        # 1. Conectar al simulador con interfaz gráfica
        self.cliente = p.connect(p.GUI)
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        
        # 2. Activar la gravedad (Crucial para físicas realistas)
        p.setGravity(0, 0, -9.81)
        
        # 3. Cargar el entorno
        self.suelo = p.loadURDF("plane.urdf")
        
        # 4. Cargar el robot en la posición X=0, Y=0, Z=0.5
        self.robot = p.loadURDF("r2d2.urdf", [0, 0, 0.5])
        
        # 5. Colocar un obstáculo (un cubo) a 3 metros frente al robot
        self.obstaculo = p.loadURDF("cube.urdf", [3, 0, 0.5], globalScaling=0.5)

    def simular_sensor_distancia(self):
        """
        Simula el módulo de 'Percepción'.
        Calcula a qué distancia está el obstáculo del robot.
        """
        pos_robot, _ = p.getBasePositionAndOrientation(self.robot)
        pos_obs, _ = p.getBasePositionAndOrientation(self.obstaculo)
        
        # Distancia euclidiana básica (plano X, Y)
        distancia = math.sqrt((pos_obs[0] - pos_robot[0])**2 + (pos_obs[1] - pos_robot[1])**2)
        return distancia

    def mover_robot(self, ordenes):
        """
        Simula el módulo de 'Control'.
        Recibe el diccionario del cerebro e inyecta la velocidad en los motores.
        """
        velocidad = ordenes["velocidad_motor"]
        giro = ordenes["fuerza_giro"]
        
        # Índices de las llantas en el modelo R2D2 de PyBullet
        ruedas = [2, 3, 6, 7] 
        
        # Lógica diferencial simple para poder girar
        vel_izq = velocidad - giro
        vel_der = velocidad + giro
        
        p.setJointMotorControlArray(
            self.robot, 
            ruedas, 
            p.VELOCITY_CONTROL, 
            targetVelocities=[vel_izq, vel_der, vel_izq, vel_der]
        )

    def actualizar_fisicas(self):
        """Hace avanzar el reloj de la simulación."""
        p.stepSimulation()
        time.sleep(1./240.) # Mantiene la simulación estable a 240Hz