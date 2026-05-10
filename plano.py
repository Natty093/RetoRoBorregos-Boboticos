# ==========================================
# REEMPLAZA COMPLETO simular_lidar()
# EN PLANO.PY
# ==========================================

def simular_lidar(self):

    num_rayos = 40

    rango_max = 0.75

    velocidad_rotacion = 0.12

    if not hasattr(self, "angulo_lidar"):

        self.angulo_lidar = 0

    self.angulo_lidar += velocidad_rotacion

    pos_robot, ori_robot = (
        p.getBasePositionAndOrientation(
            self.robot
        )
    )

    m = p.getMatrixFromQuaternion(
        ori_robot
    )

    frente_x = m[0]
    frente_y = m[1]

    derecha_x = m[3]
    derecha_y = m[4]

    altura_laser = 0.08

    origen = [
        pos_robot[0],
        pos_robot[1],
        altura_laser
    ]

    p.removeAllUserDebugItems()

    rayos_inicio = []
    rayos_fin = []

    for i in range(num_rayos):

        angulo = (
            (2 * math.pi / num_rayos) * i
            +
            self.angulo_lidar
        )

        rx = (
            frente_x * math.cos(angulo)
            +
            derecha_x * math.sin(angulo)
        )

        ry = (
            frente_y * math.cos(angulo)
            +
            derecha_y * math.sin(angulo)
        )

        inicio = origen

        fin = [
            pos_robot[0] + rx * rango_max,
            pos_robot[1] + ry * rango_max,
            altura_laser
        ]

        rayos_inicio.append(inicio)

        rayos_fin.append(fin)

    resultados = p.rayTestBatch(
        rayos_inicio,
        rayos_fin
    )

    distancias = []

    for i, res in enumerate(resultados):

        hit = res[0] != -1

        distancia = (
            res[2] * rango_max
        )

        distancias.append(
            distancia
        )

        color = (
            [1,0,0]
            if hit
            else [0,1,0]
        )

        p.addUserDebugLine(
            rayos_inicio[i],
            rayos_fin[i],
            color,
            lineWidth=1.8
        )

    return distancias