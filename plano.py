import pybullet as p
import pybullet_data
import time
import math

# Cajas de colision generadas DIRECTAMENTE del STL (MAPA.STL)
# Usando flood-fill sobre geometria vertical del mesh
WALL_BOXES = [
    (0.0,-5.15,0.25,0.15,10.3,0.5),
    (0.15,-8.885,0.25,0.15,2.5299,0.5),
    (0.15,-4.0562,0.25,0.15,6.8277,0.5),
    (0.15,-0.2901,0.25,0.15,0.2803,0.5),
    (0.2901,-0.15,0.25,0.2803,0.15,0.5),
    (0.3962,-0.3962,0.25,0.4924,0.4924,0.5),
    (0.8271,-7.6201,0.25,1.3541,0.15,0.5),
    (0.8271,-7.4701,0.25,1.3541,0.15,0.5),
    (0.8744,-4.8008,0.25,0.464,0.15,0.5),
    (1.1064,-4.8008,0.25,0.15,0.15,0.5),
    (1.5041,-7.5451,0.25,0.15,0.15,0.5),
    (1.5041,-3.378,0.25,0.15,0.15,0.5),
    (1.5041,-1.2477,0.25,0.15,0.15,0.5),
    (1.65,-8.6148,0.25,0.15,0.15,0.5),
    (1.65,-4.8008,0.25,0.15,0.15,0.5),
    (2.0079,-8.6898,0.25,0.7159,0.15,0.5),
    (2.0025,-8.5398,0.25,0.705,0.15,0.5),
    (2.0079,-4.8008,0.25,0.7159,0.15,0.5),
    (2.1423,-3.303,0.25,1.2763,0.15,0.5),
    (2.1423,-1.3227,0.25,1.2763,0.15,0.5),
    (2.2159,-5.3443,0.25,0.15,0.9371,0.5),
    (2.2173,-3.453,0.25,1.4263,0.15,0.5),
    (2.2173,-1.1727,0.25,1.4263,0.15,0.5),
    (2.2909,-5.8129,0.25,0.15,0.15,0.5),
    (2.3659,-5.2693,0.25,0.15,1.0871,0.5),
    (2.6754,-0.15,0.25,4.0659,0.15,0.5),
    (2.8489,-10.15,0.25,5.3977,0.15,0.5),
    (2.7805,-2.3129,0.25,0.15,1.9803,0.5),
    (2.9305,-2.9579,0.25,0.15,0.9902,0.5),
    (2.9305,-1.7428,0.25,0.15,1.1402,0.5),
    (3.0151,-8.2813,0.25,1.3201,0.817,0.5),
    (2.9744,-7.3938,0.25,0.15,0.9584,0.5),
    (3.1224,-6.9146,0.25,0.2959,0.15,0.5),
    (3.2703,-7.3978,0.25,0.15,0.9665,0.5),
    (3.4311,-5.8129,0.25,0.15,0.15,0.5),
    (3.3561,-5.2693,0.25,0.15,1.0871,0.5),
    (3.4311,-4.7258,0.25,0.15,0.15,0.5),
    (3.5061,-5.2693,0.25,0.15,1.0871,0.5),
    (3.5099,-2.4629,0.25,1.1589,0.15,0.5),
    (3.5099,-2.3129,0.25,1.1589,0.15,0.5),
    (4.0894,-8.6455,0.25,0.15,0.15,0.5),
    (4.0894,-7.5451,0.25,0.15,0.15,0.5),
    (4.0894,-2.3879,0.25,0.15,0.15,0.5),
    (4.3545,-7.4701,0.25,0.5303,0.15,0.5),
    (4.5667,-9.3307,0.25,0.15,0.15,0.5),
    (4.6197,-8.7205,0.25,1.0606,0.15,0.5),
    (4.6197,-8.5705,0.25,1.0606,0.15,0.5),
    (4.6947,-7.6201,0.25,1.2106,0.15,0.5),
    (4.7083,-0.7364,0.25,0.15,1.1727,0.5),
    (4.7833,-4.6375,0.25,0.4333,0.15,0.5),
    (4.8848,-7.2637,0.25,0.5303,0.4128,0.5),
    (4.8583,-0.6614,0.25,0.15,1.0227,0.5),
    (5.0,-5.2561,0.25,0.15,1.0871,0.5),
    (5.0,-3.3428,0.25,0.15,2.4394,0.5),
    (5.0572,-9.4057,0.25,0.9811,0.15,0.5),
    (5.1322,-9.2557,0.25,1.1311,0.15,0.5),
    (5.075,-5.7996,0.25,0.15,0.15,0.5),
    (5.15,-10.3,0.25,10.3,0.15,0.5),
    (5.15,-8.6455,0.25,0.15,0.15,0.5),
    (5.225,-6.8509,0.25,0.15,0.4128,0.5),
    (5.15,-6.8509,0.25,0.15,0.4128,0.5),
    (5.15,-5.3311,0.25,0.15,0.9371,0.5),
    (5.15,-3.4928,0.25,0.15,2.4394,0.5),
    (5.15,-0.075,0.25,10.3,0.15,0.5),
    (5.3,-7.1323,0.25,0.15,0.9756,0.5),
    (5.4396,-1.3227,0.25,1.4625,0.15,0.5),
    (5.5477,-9.7779,0.25,0.15,0.7443,0.5),
    (5.5477,-7.5451,0.25,0.15,0.15,0.5),
    (5.5146,-1.1727,0.25,1.3125,0.15,0.5),
    (5.5854,-4.8625,0.25,0.8708,0.15,0.5),
    (5.5854,-2.1231,0.25,1.1708,0.15,0.5),
    (5.6977,-9.7029,0.25,0.15,0.8943,0.5),
    (5.6604,-4.7125,0.25,1.0208,0.15,0.5),
    (5.6604,-2.2731,0.25,1.0208,0.15,0.5),
    (5.7843,-7.6201,0.25,0.4731,0.15,0.5),
    (5.8593,-7.4701,0.25,0.6231,0.15,0.5),
    (6.0208,-8.0953,0.25,0.15,0.9504,0.5),
    (6.0208,-5.3311,0.25,0.15,0.9371,0.5),
    (6.0958,-8.5705,0.25,0.15,0.15,0.5),
    (6.0958,-5.7996,0.25,0.15,0.15,0.5),
    (6.1708,-8.0203,0.25,0.15,1.1004,0.5),
    (6.1708,-5.2561,0.25,0.15,1.0871,0.5),
    (6.1708,-3.378,0.25,0.15,0.15,0.5),
    (6.1708,-2.1981,0.25,0.15,0.15,0.5),
    (6.1708,-1.2477,0.25,0.15,0.15,0.5),
    (6.3676,-0.15,0.25,3.0186,0.15,0.5),
    (6.7023,-10.15,0.25,2.0091,0.15,0.5),
    (7.5496,-3.453,0.25,2.7576,0.15,0.5),
    (7.5496,-3.303,0.25,2.7576,0.15,0.5),
    (7.7069,-9.8243,0.25,0.15,0.6515,0.5),
    (7.7069,-5.3886,0.25,0.15,1.3523,0.5),
    (7.7819,-9.4985,0.25,0.15,0.15,0.5),
    (7.7819,-6.0648,0.25,0.15,0.15,0.5),
    (7.8569,-9.8243,0.25,0.15,0.6515,0.5),
    (7.8569,-5.4636,0.25,0.15,1.2023,0.5),
    (7.8769,-0.7231,0.25,0.15,1.1462,0.5),
    (8.0269,-0.6481,0.25,0.15,0.9962,0.5),
    (8.3176,-4.8625,0.25,0.9216,0.15,0.5),
    (8.343,-1.2962,0.25,0.9322,0.15,0.5),
    (8.3926,-10.15,0.25,1.0716,0.15,0.5),
    (8.3584,-7.6868,0.25,0.15,0.15,0.5),
    (8.418,-1.1462,0.25,0.7822,0.15,0.5),
    (8.6434,-7.7618,0.25,0.5701,0.15,0.5),
    (8.5684,-7.6118,0.25,0.4201,0.15,0.5),
    (8.7784,-6.2371,0.25,0.15,2.7493,0.5),
    (8.8091,-1.2212,0.25,0.15,0.15,0.5),
    (8.9284,-9.8243,0.25,0.15,0.6515,0.5),
    (8.9284,-6.3121,0.25,0.15,2.8993,0.5),
    (8.9284,-4.7125,0.25,2.4431,0.15,0.5),
    (8.9284,-3.378,0.25,0.15,0.15,0.5),
    (8.9284,-2.1981,0.25,0.15,0.15,0.5),
    (9.0034,-9.4985,0.25,0.15,0.15,0.5),
    (9.0784,-9.8243,0.25,0.15,0.6515,0.5),
    (9.0885,-0.15,0.25,2.1231,0.15,0.5),
    (9.5392,-4.8625,0.25,1.2216,0.15,0.5),
    (9.5392,-2.2731,0.25,1.2216,0.15,0.5),
    (9.5392,-2.1231,0.25,1.2216,0.15,0.5),
    (9.6142,-10.15,0.25,1.0716,0.15,0.5),
    (10.15,-7.5062,0.25,0.15,5.2875,0.5),
    (10.15,-3.4928,0.25,0.15,2.4394,0.5),
    (10.15,-1.1365,0.25,0.15,1.9731,0.5),
    (10.3,-5.15,0.25,0.15,10.3,0.5),
]


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

        # Paredes como cajas de colision
        self.paredes = []
        color_pared = [0.55, 0.55, 0.55, 0.0]
        for bx, by, bz, sx, sy, sz in WALL_BOXES:
            col = p.createCollisionShape(p.GEOM_BOX, halfExtents=[sx/2, sy/2, sz/2])
            vis = p.createVisualShape(p.GEOM_BOX, halfExtents=[sx/2, sy/2, sz/2],
                                      rgbaColor=color_pared)
            wall_id = p.createMultiBody(baseMass=0,
                                        baseCollisionShapeIndex=col,
                                        baseVisualShapeIndex=vis,
                                        basePosition=[bx, by, bz])
            self.paredes.append(wall_id)

        # Robot
        self.robot = p.loadURDF("robot.urdf", [3.2, -9.2, 0.06])

        for j in range(p.getNumJoints(self.robot)):
            p.setJointMotorControl2(self.robot, j, p.VELOCITY_CONTROL, force=0)
            p.changeDynamics(self.robot, j,
                             lateralFriction=2.5,
                             spinningFriction=0.002,
                             rollingFriction=0.002)

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

    def simular_lidar(self):
        num_rayos = 9
        rango_max = 1.5

        pos_robot, ori_robot = p.getBasePositionAndOrientation(self.robot)

        euler = p.getEulerFromQuaternion(ori_robot)
        roll  = abs(euler[0])
        pitch = abs(euler[1])
        inclinado = (roll > math.radians(25) or pitch > math.radians(25))

        m = p.getMatrixFromQuaternion(ori_robot)
        frente_x = m[0]
        frente_y = m[1]

        frente_len = math.sqrt(frente_x**2 + frente_y**2)
        if frente_len > 1e-6:
            frente_x /= frente_len
            frente_y /= frente_len

        altura_laser = 0.10

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
        distancias = [OFFSET + res[2] * (rango_max - OFFSET) for res in resultados]

        for i in range(num_rayos):
            color = [1, 0, 0] if distancias[i] < rango_max * 0.99 else [0, 1, 0]
            lid = p.addUserDebugLine(rayos_inicio[i], rayos_fin[i], color, lineWidth=1.5)
            self.lidar_ids.append(lid)

        if inclinado:
            distancias = [0.20] * num_rayos

        return distancias

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
            forces=[30, 30, 30, 30]
        )

    def actualizar_fisicas(self):
        p.stepSimulation()
        time.sleep(1. / 240.)