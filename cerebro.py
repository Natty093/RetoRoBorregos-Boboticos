# ==========================================
# CEREBRO.PY COMPLETO
# ==========================================

import math
from collections import deque


class CerebroRobot:

    # ==========================================
    # DISTANCIAS
    # ==========================================

    D_STOP  = 0.12
    D_CURVA = 0.18
    D_LIBRE = 0.30

    # ==========================================
    # VELOCIDADES EXTREMAS
    # ==========================================

    VEL_MAX     = 350
    VEL_CURVA   = 260
    VEL_RETRO   = -180

    GIRO_PIVOTE = 150
    GIRO_CURVA  = 90
    GIRO_ESCAPE = 180

    # ==========================================
    # ANTI ATASCO
    # ==========================================

    ATASCO_VENTANA = 40
    ATASCO_UMBRAL  = 0.06

    ESCAPE_TICKS = 8

    def __init__(self):

        self.estado = "AVANZAR"

        self._pos_hist = deque(
            maxlen=self.ATASCO_VENTANA
        )

        self._escape_ticks = 0

        self._escape_accion = None

        self.confirmacion_libre = 0

    # ==========================================
    # ENTORNO
    # ==========================================

    def percibir_entorno(
        self,
        distancias,
        pos_robot=None
    ):

        if pos_robot:

            self._pos_hist.append(
                (pos_robot[0], pos_robot[1])
            )

        # ==========================================
        # ESCAPE
        # ==========================================

        if self._escape_ticks > 0:

            self._escape_ticks -= 1

            self.estado = "_ESCAPE"

            return

        # ==========================================
        # ATASCADO
        # ==========================================

        if self._atascado():

            izquierda = sum(
                distancias[:20]
            )

            derecha = sum(
                distancias[20:]
            )

            self._escapar(
                izquierda,
                derecha
            )

            return

        # ==========================================
        # MEJOR DIRECCION
        # ==========================================

        mejor_indice = max(
            range(len(distancias)),
            key=lambda i: distancias[i]
        )

        mejor_distancia = distancias[
            mejor_indice
        ]

        # ==========================================
        # SI NO HAY ESPACIO
        # ==========================================

        if mejor_distancia < self.D_LIBRE:

            self.confirmacion_libre = 0

            if mejor_indice < len(distancias)//2:

                self.estado = "PIVOTAR_IZQ"

            else:

                self.estado = "PIVOTAR_DER"

            return

        # ==========================================
        # DOBLE VERIFICACION
        # ==========================================

        self.confirmacion_libre += 1

        if self.confirmacion_libre < 2:

            return

        # ==========================================
        # ALINEACION
        # ==========================================

        centro = len(distancias)//2

        error = mejor_indice - centro

        if abs(error) > 3:

            if error < 0:

                self.estado = "PIVOTAR_IZQ"

            else:

                self.estado = "PIVOTAR_DER"

            return

        # ==========================================
        # AVANZAR
        # ==========================================

        self.estado = "AVANZAR"

    # ==========================================
    # DECIDIR ACCION
    # ==========================================

    def decidir_accion(self):

        e = self.estado

        if e == "_ESCAPE":

            return self._escape_accion

        if e == "AVANZAR":

            return {
                "velocidad_motor": self.VEL_MAX,
                "fuerza_giro": 0
            }

        if e == "PIVOTAR_IZQ":

            return {
                "velocidad_motor": 90,
                "fuerza_giro": -self.GIRO_PIVOTE
            }

        if e == "PIVOTAR_DER":

            return {
                "velocidad_motor": 90,
                "fuerza_giro": self.GIRO_PIVOTE
            }

        return {
            "velocidad_motor": 0,
            "fuerza_giro": 0
        }

    # ==========================================
    # ATASCADO
    # ==========================================

    def _atascado(self):

        if len(self._pos_hist) < self.ATASCO_VENTANA:
            return False

        dx = (
            self._pos_hist[-1][0]
            -
            self._pos_hist[0][0]
        )

        dy = (
            self._pos_hist[-1][1]
            -
            self._pos_hist[0][1]
        )

        dist = math.sqrt(dx * dx + dy * dy)

        return dist < self.ATASCO_UMBRAL

    # ==========================================
    # ESCAPE
    # ==========================================

    def _escapar(
        self,
        izquierda,
        derecha
    ):

        giro = (
            -self.GIRO_ESCAPE
            if izquierda > derecha
            else self.GIRO_ESCAPE
        )

        self._escape_accion = {

            "velocidad_motor": self.VEL_RETRO,

            "fuerza_giro": giro
        }

        self._escape_ticks = self.ESCAPE_TICKS

        self.estado = "_ESCAPE"