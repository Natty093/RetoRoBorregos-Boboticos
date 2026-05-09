import pybullet as p
import pybullet_data
import time

class MundoSimulacion:
    def __init__(self):
        # Conexión y configuración inicial
        self.cliente = p.connect(p.GUI)
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        p.setGravity(0, 0, -9.81)
        
        # 1. Cargar Suelo y Mapa de SolidWorks
        self.suelo = p.loadURDF("plane.urdf")
        # Asegúrate de que MAPA.urdf y MAPA.stl estén en la misma carpeta
        self.laberinto = p.loadURDF("MAPA.urdf", [0, 0, 0], useFixedBase=True)
        
        # 2. Cargar el robot en una zona libre del mapa
        # (X=1, Y=1 suele ser un buen inicio, ajústalo según tu diseño)
        self.robot = p.loadURDF("r2d2.urdf", [1, 1, 0.5]) 
        
        # ID para la línea roja del Lidar (para poder borrarla y redibujarla)
        self.lidar_debug_id = -1

    def simular_sensor_distancia(self):
        """Simula un rayo láser hacia adelante y dibuja una línea roja en la GUI."""
        pos_robot, ori_robot = p.getBasePositionAndOrientation(self.robot)
        matriz_rotacion = p.getMatrixFromQuaternion(ori_robot)
        
        # Dirección frontal (Eje X local del robot)
        direccion_frontal = [matriz_rotacion[0], matriz_rotacion[3], matriz_rotacion[6]]
        
        # El rayo mide hasta 3 metros
        rango_max = 3
        inicio_rayo = [pos_robot[0], pos_robot[1], pos_robot[2] + 0.2] # Un poco elevado
        final_rayo = [
            inicio_rayo[0] + direccion_frontal[0] * rango_max,
            inicio_rayo[1] + direccion_frontal[1] * rango_max,
            inicio_rayo[2]
        ]
        
        # Ejecutar el test de rayo
        resultado = p.rayTest(inicio_rayo, final_rayo)
        hit_fraction = resultado[0][2] 
        distancia = hit_fraction * rango_max
        
        # --- VISUALIZACIÓN DEL RAYO ---
        # Borramos la línea anterior y dibujamos una nueva para ver el sensor
        if self.lidar_debug_id != -1:
            p.removeUserDebugItem(self.lidar_debug_id)
        
        color_linea = [1, 0, 0] if distancia < 1.0 else [0, 1, 0] # Rojo si está cerca
        self.lidar_debug_id = p.addUserDebugLine(inicio_rayo, final_rayo, color_linea)
        
        return distancia

    def mover_robot(self, ordenes):
        """Aplica las velocidades a los motores del R2D2."""
        velocidad = ordenes["velocidad_motor"]
        giro = ordenes["fuerza_giro"]
        
        # Índices de ruedas del modelo R2D2 oficial
        ruedas_izquierdas = [2, 6]
        ruedas_derechas = [3, 7]
        
        vel_izq = velocidad - giro
        vel_der = velocidad + giro
        
        # Aplicar a las ruedas
        p.setJointMotorControlArray(
            self.robot, ruedas_izquierdas + ruedas_derechas, 
            p.VELOCITY_CONTROL, 
            targetVelocities=[vel_izq, vel_izq, vel_der, vel_der]
        )

    def actualizar_fisicas(self):
        p.stepSimulation()
        time.sleep(1./240.)

# --- LÓGICA DE CONTROL (EL CEREBRO) ---
class CerebroRobot:
    def __init__(self):
        self.umbral_obstaculo = 1.2 # Distancia en metros para empezar a girar

    def decidir_movimiento(self, distancia_frente):
        ordenes = {"velocidad_motor": 0, "fuerza_giro": 0}
        
        if distancia_frente > self.umbral_obstaculo:
            # Si el camino está despejado, avanza rápido
            ordenes["velocidad_motor"] = 15
            ordenes["fuerza_giro"] = 0
        else:
            # Si hay un muro cerca, frena y gira fuerte
            ordenes["velocidad_motor"] = 2
            ordenes["fuerza_giro"] = 8 # Gira sobre su propio eje
            
        return ordenes

# --- BUCLE PRINCIPAL ---
if __name__ == "__main__":
    sim = MundoSimulacion()
    cerebro = CerebroRobot()
    
    try:
        while True:
            # 1. Medir distancia con el Lidar
            dist = sim.simular_sensor_distancia()
            
            # 2. El cerebro decide qué hacer
            instrucciones = cerebro.decidir_movimiento(dist)
            
            # 3. Ejecutar movimiento en la física
            sim.mover_robot(instrucciones)
            
            # 4. Actualizar el mundo
            sim.actualizar_fisicas()
            
    except KeyboardInterrupt:
        p.disconnect()
        print("Simulación finalizada.")