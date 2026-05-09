import pybullet as p
import pybullet_data
import time

class MundoSimulacion:
    def __init__(self):
        self.cliente = p.connect(p.GUI)
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        p.setGravity(0, 0, -9.81)
        
        # 1. Suelo base
        self.suelo = p.loadURDF("plane.urdf")
        
        # 2. Cargar tu diseño de SolidWorks
        # useFixedBase=True asegura que no se mueva al chocar
        self.laberinto = p.loadURDF("mapa.urdf", [0, 0, 0], useFixedBase=True)
        
        # 3. Cargar el robot (Ajusta la posición inicial para que no aparezca dentro de un muro)
        self.robot = p.loadURDF("r2d2.urdf", [1, 1, 0.5]) 

    def simular_sensor_distancia(self):
        """
        Simula un Lidar (Láser). Lanza un rayo desde el robot hacia adelante.
        """
        pos_robot, ori_robot = p.getBasePositionAndOrientation(self.robot)
        matriz_rotacion = p.getMatrixFromQuaternion(ori_robot)
        
        # El rayo sale del robot y llega 3 metros hacia adelante
        # La dirección frontal en R2D2 suele ser el eje X o Y del modelo
        direccion_frontal = [matriz_rotacion[0], matriz_rotacion[3], matriz_rotacion[6]]
        final_rayo = [
            pos_robot[0] + direccion_frontal[0] * 3,
            pos_robot[1] + direccion_frontal[1] * 3,
            pos_robot[2]
        ]
        
        resultado = p.rayTest(pos_robot, final_rayo)
        hit_fraction = resultado[0][2] # 0.0 es contacto inmediato, 1.0 es nada detectado
        
        distancia = hit_fraction * 3 # Convertir fracción a metros
        return distancia

    def mover_robot(self, ordenes):
        # ... (Tu misma lógica de motores de antes)
        velocidad = ordenes["velocidad_motor"]
        giro = ordenes["fuerza_giro"]
        ruedas = [2, 3, 6, 7] 
        vel_izq = velocidad - giro
        vel_der = velocidad + giro
        p.setJointMotorControlArray(
            self.robot, ruedas, p.VELOCITY_CONTROL, 
            targetVelocities=[vel_izq, vel_der, vel_izq, vel_der]
        )

    def actualizar_fisicas(self):
        p.stepSimulation()
        time.sleep(1./240.) # Mantiene la simulación estable a 240Hz