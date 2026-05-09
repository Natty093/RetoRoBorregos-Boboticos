import pybullet as p
import pybullet_data
import time
import math
 
# Cajas de colision extraidas directamente del STL real (MAPA.STL)

 
class MundoSimulacion:
    def __init__(self):
        self.cliente = p.connect(p.GUI)
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        p.setGravity(0, 0, -9.81)
 
        p.resetDebugVisualizerCamera(
            cameraDistance=12,
            cameraYaw=0,
            cameraPitch=-60,
            cameraTargetPosition=[5.15, -5.15, 0]
        )
 
        # Suelo
        self.suelo = p.loadURDF("plane.urdf", [0, 0, 0])
 
        # Mapa visual (STL) - sin colision
        orientacion_mapa = p.getQuaternionFromEuler([math.pi / 2, 0, 0])
        self.laberinto = p.loadURDF(
            "mapa.urdf",
            basePosition=[0, 0, 0],
            baseOrientation=orientacion_mapa,
            useFixedBase=True,
            globalScaling=0.001,
            flags=p.URDF_ENABLE_CACHED_GRAPHICS_SHAPES
        )
        p.setCollisionFilterGroupMask(self.laberinto, -1, 0, 0)
 
        # Paredes como cajas
     
        # Robot
        # spawn_z=0.06: fondo rueda = 0.06 + (-0.005) - 0.055 = 0.0 (toca suelo exacto)
        self.robot = p.loadURDF("robot.urdf", [3.2, -9.2, 0.06])
 
        # Dinamica: fuerza reducida (30N), damping alto, friccion alta
        for j in range(p.getNumJoints(self.robot)):
            p.setJointMotorControl2(self.robot, j, p.VELOCITY_CONTROL, force=0)
            p.changeDynamics(self.robot, j,
                             lateralFriction=2.5,
                             spinningFriction=0.002,
                             rollingFriction=0.002)
 
        # Chasis: damping muy alto para evitar volcado
        p.changeDynamics(self.robot, -1,
                         lateralFriction=0.1,
                         linearDamping=0.8,
                         angularDamping=0.9999)
 
        self.ruedas_izq = [0, 2]
        self.ruedas_der = [1, 3]
        self.lidar_ids = []
 
        print("Asentando robot...")
        for _ in range(300):
            p.stepSimulation()
        print("Iniciando navegacion autonoma!")
 
    # ------------------------------------------------------------------
    def simular_lidar(self):
        num_rayos = 9
        rango_max = 1.5
 
        pos_robot, ori_robot = p.getBasePositionAndOrientation(self.robot)
 
        # --- FIX 1: detectar inclinacion excesiva ---
        # Extraer angulo de roll y pitch del quaternion
        euler = p.getEulerFromQuaternion(ori_robot)
        roll  = abs(euler[0])
        pitch = abs(euler[1])
        inclinado = (roll > math.radians(25) or pitch > math.radians(25))
 
        m = p.getMatrixFromQuaternion(ori_robot)
        frente_x = m[0]
        frente_y = m[1]
 
        # --- FIX 2: normalizar vector frente ---
        frente_len = math.sqrt(frente_x**2 + frente_y**2)
        if frente_len > 1e-6:
            frente_x /= frente_len
            frente_y /= frente_len
 
        # --- FIX 3: altura laser FIJA en 0.10m sobre el suelo ---
        # (no depende de pos_robot[2] para que no falle al inclinarse)
        altura_laser = 0.10
 
        # Borrar rayos anteriores
        p.removeAllUserDebugItems()
        self.lidar_ids.clear()
 
        OFFSET = 0.22
        rayos_inicio, rayos_fin = [], []
        for i in range(num_rayos):
            angulo = (math.pi / (num_rayos - 1)) * i - (math.pi / 2)
            rx = frente_x * math.cos(angulo) - frente_y * math.sin(angulo)
            ry = frente_x * math.sin(angulo) + frente_y * math.cos(angulo)
            inicio = [pos_robot[0] + rx * OFFSET, pos_robot[1] + ry * OFFSET, altura_laser]
            fin    = [pos_robot[0] + rx * rango_max, pos_robot[1] + ry * rango_max, altura_laser]
            rayos_inicio.append(inicio)
            rayos_fin.append(fin)
 
        resultados = p.rayTestBatch(rayos_inicio, rayos_fin)
 
        # hitFraction es fraccion del segmento inicio->fin
        # distancia desde centro robot = OFFSET + frac * (rango_max - OFFSET)
        distancias = [OFFSET + res[2] * (rango_max - OFFSET) for res in resultados]
 
        for i in range(num_rayos):
            color = [1, 0, 0] if distancias[i] < rango_max * 0.99 else [0, 1, 0]
            lid = p.addUserDebugLine(rayos_inicio[i], rayos_fin[i], color, lineWidth=1.5)
            self.lidar_ids.append(lid)
 
        # Si el robot esta muy inclinado, devolver distancias cortas en todas
        # direcciones para que el cerebro frene en lugar de acelerar
        if inclinado:
            distancias = [0.20] * num_rayos
 
        return distancias
 
    # ------------------------------------------------------------------
    def mover_robot(self, ordenes):
        velocidad = ordenes["velocidad_motor"]
        giro = ordenes["fuerza_giro"]
        vel_izq = velocidad - giro
        vel_der = velocidad + giro
 
        # --- FIX 4: fuerza reducida de 150->30 N para evitar volcado ---
        p.setJointMotorControlArray(
            self.robot,
            self.ruedas_izq + self.ruedas_der,
            p.VELOCITY_CONTROL,
            targetVelocities=[vel_izq, vel_izq, vel_der, vel_der],
            forces=[30, 30, 30, 30]
        )
 
    # ------------------------------------------------------------------
    def actualizar_fisicas(self):
        p.stepSimulation()
        time.sleep(1. / 240.)