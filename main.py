from cerebro import CerebroRobot
from plano import MundoSimulacion

import pybullet as p
import time

modo_pov = False


def actualizar_camara(mi_mundo):

    pos, ori = p.getBasePositionAndOrientation(
        mi_mundo.robot
    )

    matriz = p.getMatrixFromQuaternion(ori)

    frente_x = matriz[0]
    frente_y = matriz[1]

    cam_x = pos[0]
    cam_y = pos[1]
    cam_z = pos[2] + 0.22

    target_x = cam_x + frente_x * 2
    target_y = cam_y + frente_y * 2
    target_z = cam_z

    view = p.computeViewMatrix(
        [cam_x, cam_y, cam_z],
        [target_x, target_y, target_z],
        [0, 0, 1]
    )

    proj = p.computeProjectionMatrixFOV(
        90,
        1.6,
        0.02,
        30
    )

    p.getCameraImage(
        640,
        480,
        view,
        proj
    )


def iniciar_Todo():

    global modo_pov

    cerebro = CerebroRobot()

    mi_mundo = MundoSimulacion()

    while True:

        teclas = p.getKeyboardEvents()

        if (
            ord('v') in teclas
            and teclas[ord('v')] & p.KEY_WAS_TRIGGERED
        ):

            modo_pov = not modo_pov

            if not modo_pov:

                p.resetDebugVisualizerCamera(
                    cameraDistance=12,
                    cameraYaw=0,
                    cameraPitch=-60,
                    cameraTargetPosition=[5, -5, 0]
                )

        distancias = mi_mundo.simular_lidar()

        pos, _ = p.getBasePositionAndOrientation(
            mi_mundo.robot
        )

        cerebro.percibir_entorno(
            distancias,
            pos_robot=pos
        )

        ordenes = cerebro.decidir_accion()

        mi_mundo.mover_robot(ordenes)

        mi_mundo.actualizar_fisicas()

        if modo_pov:

            actualizar_camara(mi_mundo)


if __name__ == "__main__":
    iniciar_Todo()