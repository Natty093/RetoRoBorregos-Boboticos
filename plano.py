import pybullet as p
import pybullet_data
import time
import math
 
# Cajas de pared extraídas directamente del STL (pos_x, pos_y, pos_z, size_x, size_y, size_z)
# Coordenadas ya en sistema PyBullet (con rotación +pi/2 aplicada)
WALL_BOXES = [
    (6.2064,-5.3193,0.2500,1.7503,0.1500,0.5000),
    (8.0567,-4.6193,0.2500,1.7503,0.1500,0.5000),
    (8.9318,-4.9693,0.2500,0.1500,0.7000,0.5000),
    (8.0567,-5.3193,0.2500,1.7503,0.1500,0.5000),
    (0.8271,-3.1189,0.2500,1.3541,0.1500,0.5000),
    (1.5042,-3.4689,0.2500,0.1500,0.7000,0.5000),
    (0.8271,-3.8189,0.2500,1.3541,0.1500,0.5000),
    (2.5300,-3.1189,0.2500,1.8517,0.1500,0.5000),
    (3.4058,-3.4689,0.2500,0.1500,0.7000,0.5000),
    (2.5300,-3.8189,0.2500,1.8517,0.1500,0.5000),
    (4.3561,-3.1189,0.2500,1.7503,0.1500,0.5000),
    (5.2312,-3.4689,0.2500,0.1500,0.7000,0.5000),
]
 
 
class MundoSimulacion:
    def __init__(self):
        self.cliente = p.connect(p.GUI)
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        p.setGravity(0, 0, -9.81)
 
        p.resetDebugVisualizerCamera(
            cameraDistance=2,
            cameraYaw=45,
            cameraPitch=-35,
            cameraTargetPosition=[5.15, -5.15, 0]
        )
 
        # Suelo
        self.suelo = p.loadURDF("plane.urdf", [0, 0, 0])
 
        # Mapa visual (STL) - solo para que se vea bonito, sin colisión
        orientacion_mapa = p.getQuaternionFromEuler([math.pi / 2, 0, 0])
        self.laberinto = p.loadURDF(
            "mapa.urdf",
            basePosition=[0, 0, 0],
            baseOrientation=orientacion_mapa,
            useFixedBase=True,
            globalScaling=0.001,
            flags=p.URDF_ENABLE_CACHED_GRAPHICS_SHAPES
        )
        # Deshabilitar colisión del STL completamente (usamos cajas)
        # linkIndex=0 es el base_link del URDF de un solo link
        p.setCollisionFilterGroupMask(self.laberinto, -1, 0, 0)
 
        # Paredes como cajas individuales (colisión exacta)
        self.paredes = []
        color_pared = [0.6, 0.6, 0.6, 1]
        for bx, by, bz, sx, sy, sz in WALL_BOXES:
            col = p.createCollisionShape(p.GEOM_BOX,
                                         halfExtents=[sx/2, sy/2, sz/2])
            vis = p.createVisualShape(p.GEOM_BOX,
                                      halfExtents=[sx/2, sy/2, sz/2],
                                      rgbaColor=color_pared)
            wall_id = p.createMultiBody(baseMass=0,
                                        baseCollisionShapeIndex=col,
                                        baseVisualShapeIndex=vis,
                                        basePosition=[bx, by, bz])
            self.paredes.append(wall_id)
 
        # Robot en el centro del laberinto
        self.robot = p.loadURDF("robot.urdf", [5.2, -5.2, 0.12])
 
        # Liberar joints y dar friccion a ruedas
        for j in range(p.getNumJoints(self.robot)):
            p.setJointMotorControl2(self.robot, j, p.VELOCITY_CONTROL, force=0)
            p.changeDynamics(self.robot, j,
                             lateralFriction=2.0,
                             spinningFriction=0.001,
                             rollingFriction=0.001)
        p.changeDynamics(self.robot, -1,
                         lateralFriction=0.1,
                         linearDamping=0.1,
                         angularDamping=0.5)
 
        self.ruedas_izq = [0, 2]
        self.ruedas_der = [1, 3]
        self.lidar_ids = []
 
        print("Asentando robot...")
        for _ in range(200):
            p.stepSimulation()
        print("Iniciando navegacion autonoma!")
 
    # ------------------------------------------------------------------
    def simular_lidar(self):
        num_rayos = 9
        rango_max = 3.0
 
        pos_robot, ori_robot = p.getBasePositionAndOrientation(self.robot)
 
        # CORRECCIÓN: getMatrixFromQuaternion devuelve matriz 3x3 en row-major:
        # [m0 m1 m2]   <- vector X local (frente del robot en PyBullet)
        # [m3 m4 m5]   <- vector Y local
        # [m6 m7 m8]   <- vector Z local
        # El vector "adelante" en XY es la primera fila: (m[0], m[1])
        m = p.getMatrixFromQuaternion(ori_robot)
        frente_x = m[0]  # componente X del eje X local del robot
        frente_y = m[1]  # componente Y del eje X local del robot  ← antes era m[3] (error)
 
        # La altura del láser debe estar dentro del rango vertical de las cajas.
        # Las cajas tienen bz=0.25 y sz=0.5 → van de z=0 a z=0.5
        # Usamos z=0.15 para que el rayo siempre cruce las paredes
        altura_laser = 0.15
        origen = [pos_robot[0], pos_robot[1], altura_laser]
 
        for lid in self.lidar_ids:
            p.removeUserDebugItem(lid)
        self.lidar_ids.clear()
 
        rayos_inicio, rayos_fin = [], []
        for i in range(num_rayos):
            # Barre 180° centrado en el frente del robot (-90° a +90°)
            angulo = (math.pi / (num_rayos - 1)) * i - (math.pi / 2)
            # Rotar el vector frente por el ángulo dado
            rx = frente_x * math.cos(angulo) - frente_y * math.sin(angulo)
            ry = frente_x * math.sin(angulo) + frente_y * math.cos(angulo)
            rayos_inicio.append(origen)
            rayos_fin.append([
                pos_robot[0] + rx * rango_max,
                pos_robot[1] + ry * rango_max,
                altura_laser
            ])
 
        resultados = p.rayTestBatch(rayos_inicio, rayos_fin)
        # res[2] es la fracción del recorrido (0.0-1.0) donde golpeó
        distancias = [res[2] * rango_max for res in resultados]
 
        for i in range(num_rayos):
            color = [1, 0, 0] if distancias[i] < rango_max * 0.99 else [0, 1, 0]
            lid = p.addUserDebugLine(origen, rayos_fin[i], color, lineWidth=1.5)
            self.lidar_ids.append(lid)
 
        return distancias
 
    # ------------------------------------------------------------------
    def mover_robot(self, ordenes):
        velocidad = ordenes["velocidad_motor"]
        giro = ordenes["fuerza_giro"]
        vel_izq = velocidad - giro
        vel_der = velocidad + giro
        p.setJointMotorControlArray(
            self.robot,
            self.ruedas_izq + self.ruedas_der,
            p.VELOCITY_CONTROL,
            targetVelocities=[vel_izq, vel_izq, vel_der, vel_der],
            forces=[50, 50, 50, 50]
        )
 
    # ------------------------------------------------------------------
    def actualizar_fisicas(self):
        p.stepSimulation()
        time.sleep(1. / 240.)