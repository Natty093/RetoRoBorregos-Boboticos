import pybullet as p
import pybullet_data
import time
import math # ¡Súper importante para los ángulos del LiDAR!

class MundoSimulacion:
    def __init__(self):
        # Conexión y configuración inicial
        self.cliente = p.connect(p.GUI)
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        p.setGravity(0, 0, -9.81)
        
        # 1. Cargar Suelo y Mapa
        self.suelo = p.loadURDF("plane.urdf")
        #self.laberinto = p.loadURDF("MAPA.urdf", [0, 0, 0], useFixedBase=True)
        
        # 2. Cargar el robot
        self.robot = p.loadURDF("r2d2.urdf", [1, 1, 0.5]) 
        
        # Lista para guardar los IDs de las líneas rojas y no saturar la memoria
        self.lidar_ids = []

    def simular_lidar(self):
        """Lanza el barrido de 9 rayos láser y dibuja las líneas."""
        num_rayos = 9 
        rango_max = 3.0 
        
        pos_robot, ori_robot = p.getBasePositionAndOrientation(self.robot)
        matriz_rot = p.getMatrixFromQuaternion(ori_robot)
        
        # Eje Y local porque es el frente del modelo R2D2
        frente_x, frente_y = matriz_rot[1], matriz_rot[4] 
        altura_laser = pos_robot[2] + 0.2
        
        origen_rayos = [pos_robot[0], pos_robot[1], altura_laser]
        rayos_inicio = [origen_rayos] * num_rayos
        rayos_fin = []
        
        # Limpiamos los rayos del frame anterior
        for line_id in self.lidar_ids:
            p.removeUserDebugItem(line_id)
        self.lidar_ids.clear()

        # Calculamos la trayectoria de los 9 rayos
        for i in range(num_rayos):
            angulo = (math.pi / (num_rayos - 1)) * i - (math.pi / 2) 
            
            rx = frente_x * math.cos(angulo) - frente_y * math.sin(angulo)
            ry = frente_x * math.sin(angulo) + frente_y * math.cos(angulo)
            
            x_fin = pos_robot[0] + (rx * rango_max)
            y_fin = pos_robot[1] + (ry * rango_max)
            
            rayos_fin.append([x_fin, y_fin, altura_laser]) 
            
        # Lanzamos los rayos al motor de físicas
        resultados = p.rayTestBatch(rayos_inicio, rayos_fin)
        distancias_lidar = [res[2] * rango_max for res in resultados]
        
        # Dibujamos las nuevas líneas (Verde si está libre, Rojo si detecta muro)
        for i in range(num_rayos):
            color_linea = [1, 0, 0] if distancias_lidar[i] < 1.2 else [0, 1, 0]
            line_id = p.addUserDebugLine(origen_rayos, rayos_fin[i], color_linea)
            self.lidar_ids.append(line_id)
            
        return distancias_lidar

    def mover_robot(self, ordenes):
        """Aplica las velocidades a los motores del R2D2."""
        velocidad = ordenes["velocidad_motor"]
        giro = ordenes["fuerza_giro"]
        
        ruedas_izquierdas = [2, 6]
        ruedas_derechas = [3, 7]
        
        vel_izq = velocidad - giro
        vel_der = velocidad + giro
        
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
        self.distancia_segura = 0.8 
        self.umbral_perdida_pared = 1.2 

    def decidir_movimiento(self, distancias_lidar):
        """
        Recibe los 9 rayos y toma una decisión basada en la regla de la mano derecha.
        """
        ordenes = {"velocidad_motor": 0, "fuerza_giro": 0}
        
        # Separar la visión en 3 zonas (Izquierda, Frente, Derecha)
        izquierda = min(distancias_lidar[0:3])
        frente = min(distancias_lidar[3:6])
        derecha = min(distancias_lidar[6:9])

        if derecha > self.umbral_perdida_pared:
            # Perdimos la pared a la derecha, hay que girar a buscarla
            ordenes["velocidad_motor"] = 4
            ordenes["fuerza_giro"] = 6
            
        elif frente < self.distancia_segura:
            # Callejón sin salida de frente, girar a la izquierda
            ordenes["velocidad_motor"] = 2
            ordenes["fuerza_giro"] = -8
            
        else:
            # Pared a la derecha y frente libre, avanzar rápido
            ordenes["velocidad_motor"] = 15
            ordenes["fuerza_giro"] = 0
            
        return ordenes

# --- BUCLE PRINCIPAL ---
if __name__ == "__main__":
    sim = MundoSimulacion()
    cerebro = CerebroRobot()
    
    print("Iniciando exploración autónoma con LiDAR...")
    try:
        while True:
            # 1. Medir con los 9 rayos
            distancias = sim.simular_lidar()
            
            # 2. El cerebro analiza el arreglo de distancias
            instrucciones = cerebro.decidir_movimiento(distancias)
            
            # 3. Mover y actualizar
            sim.mover_robot(instrucciones)
            sim.actualizar_fisicas()
            
    except KeyboardInterrupt:
        p.disconnect()
        print("Simulación finalizada.")