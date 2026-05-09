from cerebro import CerebroRobot
from plano import MundoSimulacion
import pybullet as p
import os
import time

INICIO = (3.2, -9.2)
META   = (9.5, -0.7)
RADIO_META = 0.35

NOMBRES_RAYO = [
    "IZQ-EXT", "IZQ-MED", "IZQ-CER",
    "FRT-IZQ", "FRT-CEN", "FRT-DER",
    "DER-CER", "DER-MED", "DER-EXT",
]


def imprimir_estado(distancias, estado, ordenes, pos_robot, dist_meta, tick, t_inicio):
    if tick % 30 != 0:
        return
    os.system("cls" if os.name == "nt" else "clear")
    elapsed = time.time() - t_inicio
    mm, ss = divmod(int(elapsed), 60)
    print("=" * 56)
    print("        ROBOT - NAVEGACION AUTONOMA EN LABERINTO")
    print("=" * 56)
    print(f"  Tiempo: {mm:02d}:{ss:02d}   Pos: ({pos_robot[0]:.2f}, {pos_robot[1]:.2f})")
    print(f"  Distancia a la meta: {dist_meta:.2f} m")
    print()
    print("  LIDAR (distancias en metros):")
    print()
    for nombre, dist in zip(NOMBRES_RAYO, distancias):
        barra   = int((dist / 1.5) * 22)
        bloques = "X" * barra + "." * (22 - barra)
        alerta  = " CERCA!" if dist < 0.28 else (" precaucion" if dist < 0.52 else "")
        print(f"  {nombre:8s}  {bloques}  {dist:.2f} m{alerta}")
    print()
    estado_display = "** ESCAPE **" if estado == "_ESCAPE" else estado
    print(f"  ESTADO:    {estado_display}")
    print(f"  Velocidad: {ordenes['velocidad_motor']:>5.1f}   Giro: {ordenes['fuerza_giro']:>+6.1f}")
    print("=" * 56)


def crear_marcadores():
    vis_inicio = p.createVisualShape(p.GEOM_SPHERE, radius=0.15,
                                     rgbaColor=[0.0, 1.0, 0.2, 0.85])
    p.createMultiBody(baseMass=0, baseVisualShapeIndex=vis_inicio,
                      basePosition=[INICIO[0], INICIO[1], 0.15])
    p.addUserDebugText("INICIO", [INICIO[0], INICIO[1], 0.45],
                       textColorRGB=[0, 1, 0], textSize=1.4)

    vis_meta = p.createVisualShape(p.GEOM_SPHERE, radius=0.15,
                                   rgbaColor=[1.0, 0.15, 0.1, 0.85])
    p.createMultiBody(baseMass=0, baseVisualShapeIndex=vis_meta,
                      basePosition=[META[0], META[1], 0.15])
    p.addUserDebugText("META", [META[0], META[1], 0.45],
                       textColorRGB=[1, 0.3, 0], textSize=1.4)

    p.addUserDebugLine([INICIO[0], INICIO[1], 0.05],
                       [META[0],   META[1],   0.05],
                       lineColorRGB=[1, 1, 0], lineWidth=1.0)


def iniciar_Todo():
    cerebro  = CerebroRobot()
    mi_mundo = MundoSimulacion()
    crear_marcadores()

    print("Simulacion iniciada - Robot navegando de INICIO a META")
    print(f"  Inicio : {INICIO}  |  Meta: {META}\n")

    tick     = 0
    t_inicio = time.time()
    llego    = False

    try:
        while True:
            if not p.isConnected():
                print("\nVentana cerrada. Saliendo.")
                break

            pos_robot, _ = p.getBasePositionAndOrientation(mi_mundo.robot)
            dist_meta = ((pos_robot[0] - META[0])**2 +
                         (pos_robot[1] - META[1])**2) ** 0.5

            # Llegada a la meta
            if dist_meta < RADIO_META and not llego:
                llego = True
                elapsed = time.time() - t_inicio
                mm, ss = divmod(int(elapsed), 60)
                p.setJointMotorControlArray(
                    mi_mundo.robot,
                    mi_mundo.ruedas_izq + mi_mundo.ruedas_der,
                    p.VELOCITY_CONTROL,
                    targetVelocities=[0, 0, 0, 0],
                    forces=[200, 200, 200, 200]
                )
                p.addUserDebugText(f"LLEGO! {mm:02d}:{ss:02d}",
                                   [META[0], META[1], 0.7],
                                   textColorRGB=[1, 1, 0], textSize=2.0)
                os.system("cls" if os.name == "nt" else "clear")
                print("=" * 56)
                print("  ** ROBOT LLEGO A LA META! **")
                print(f"     Tiempo total: {mm:02d}:{ss:02d}")
                print(f"     Posicion final: ({pos_robot[0]:.2f}, {pos_robot[1]:.2f})")
                print("=" * 56)
                print("\nPresiona Ctrl+C para cerrar.")

            if llego:
                mi_mundo.actualizar_fisicas()
                continue

            # Ciclo normal — se pasa pos_robot al cerebro para detectar atasco
            distancias_lidar = mi_mundo.simular_lidar()
            cerebro.percibir_entorno(distancias_lidar, pos_robot=pos_robot)
            ordenes_motor = cerebro.decidir_accion()
            mi_mundo.mover_robot(ordenes_motor)
            mi_mundo.actualizar_fisicas()

            imprimir_estado(distancias_lidar, cerebro.estado, ordenes_motor,
                            pos_robot, dist_meta, tick, t_inicio)
            tick += 1

    except KeyboardInterrupt:
        print("\nSimulacion finalizada.")
    except p.error as e:
        print(f"\nPyBullet desconectado: {e}")
    except Exception:
        import traceback
        traceback.print_exc()
        input("Presiona Enter para cerrar...")
    finally:
        try:
            p.disconnect()
        except Exception:
            pass


if __name__ == "__main__":
    iniciar_Todo()