import math
from collections import deque


class CerebroRobot:
    """
    Navegacion por laberinto con 9 rayos LIDAR.
    Indices: 0=IZQ-EXT, 1=IZQ-MED, 2=IZQ-CER, 3=FRT-IZQ, 4=FRT-CEN, 5=FRT-DER,
             6=DER-CER, 7=DER-MED, 8=DER-EXT
    """

    # ------------------------------------------------------------------ config
    D_STOP      = 0.22   # distancia critica: frenar ya
    D_PELIGRO   = 0.32   # muy cerca: girar fuerte
    D_PRECAUCION= 0.55   # cerca: corregir suave
    D_LIBRE     = 0.80   # frente libre: avanzar full

    VEL_NORMAL  = 22
    VEL_SUAVE   = 12
    VEL_RETRO   = -16
    GIRO_FUERTE = 18
    GIRO_SUAVE  = 6
    GIRO_ESCAPE = 22

    ATASCO_VENTANA   = 150   # ticks (~2.5s a 60Hz)
    ATASCO_UMBRAL    = 0.10  # metros minimos en la ventana
    ESCAPE_DURACION  = 100   # ticks de maniobra de escape

    # Si llevamos N ticks girando sin avanzar, forzar cambio de direccion
    GIRO_TIMEOUT = 160

    def __init__(self):
        self.estado = 'AVANZAR'

        # Historial de posicion para detectar atasco
        self._pos_hist   = deque(maxlen=self.ATASCO_VENTANA)
        self._tick       = 0

        # Maniobra de escape
        self._escape_ticks  = 0
        self._escape_accion = None

        # Control de giro continuo
        self._giro_dir    = None   # 'IZQ' o 'DER' — direccion elegida al inicio del giro
        self._giro_ticks  = 0     # cuantos ticks llevamos girando seguido

        # Memoria de giros recientes para evitar sesgo
        # Guarda 'IZQ'/'DER' por cada giro completado
        self._historial_giros = deque(maxlen=6)

    # ------------------------------------------------------------------ main
    def percibir_entorno(self, distancias, pos_robot=None):
        self._tick += 1

        i0, i1, i2 = distancias[0], distancias[1], distancias[2]  # izquierda
        f0, f1, f2 = distancias[3], distancias[4], distancias[5]  # frente
        d0, d1, d2 = distancias[6], distancias[7], distancias[8]  # derecha

        frente_min   = min(f0, f1, f2)
        frente_max   = max(f0, f1, f2)
        espacio_izq  = (i0 + i1 + i2) / 3
        espacio_der  = (d0 + d1 + d2) / 3
        espacio_tot  = espacio_izq + espacio_der + frente_min

        # ---------- registro de posicion
        if pos_robot is not None:
            self._pos_hist.append((pos_robot[0], pos_robot[1]))

        atascado = self._detectar_atasco()

        # ---------- escape en curso: no interrumpir
        if self._escape_ticks > 0:
            self._escape_ticks -= 1
            self.estado = '_ESCAPE'
            return

        # ---------- activar escape
        if atascado:
            self._iniciar_escape(espacio_izq, espacio_der)
            return

        # ---------- logica normal
        self._navegar(
            frente_min, frente_max, f0, f1, f2,
            espacio_izq, espacio_der, espacio_tot,
            i0, i1, i2, d0, d1, d2
        )

    # ------------------------------------------------------------------ privados
    def _detectar_atasco(self):
        if len(self._pos_hist) < self.ATASCO_VENTANA:
            return False
        dx = self._pos_hist[-1][0] - self._pos_hist[0][0]
        dy = self._pos_hist[-1][1] - self._pos_hist[0][1]
        return math.sqrt(dx*dx + dy*dy) < self.ATASCO_UMBRAL

    def _iniciar_escape(self, espacio_izq, espacio_der):
        self._pos_hist.clear()
        # Girar hacia el lado con MAS espacio al retroceder
        dir_escape = 'IZQ' if espacio_izq >= espacio_der else 'DER'
        # Si llevamos muchos giros en la misma dir, forzar el contrario
        if self._sesgo_giro(dir_escape):
            dir_escape = 'DER' if dir_escape == 'IZQ' else 'IZQ'
        giro = -self.GIRO_ESCAPE if dir_escape == 'IZQ' else self.GIRO_ESCAPE
        self._escape_accion = {'velocidad_motor': self.VEL_RETRO, 'fuerza_giro': giro}
        self._escape_ticks  = self.ESCAPE_DURACION
        self._giro_dir      = None
        self._giro_ticks    = 0
        self.estado = '_ESCAPE'

    def _sesgo_giro(self, dir_propuesta):
        """True si los ultimos N giros son todos en dir_propuesta (hay sesgo)."""
        if len(self._historial_giros) < 4:
            return False
        return all(g == dir_propuesta for g in list(self._historial_giros)[-4:])

    def _elegir_direccion_giro(self, espacio_izq, espacio_der):
        """
        Elige IZQ o DER para girar, considerando:
        1. Donde hay mas espacio (principal)
        2. Historial de giros recientes (anti-sesgo)
        3. La direccion ya comprometida (anti-oscilacion)
        """
        # Si ya estamos girando en una dir, mantenerla hasta timeout
        if self._giro_dir is not None and self._giro_ticks < self.GIRO_TIMEOUT:
            return self._giro_dir

        # Elegir por espacio
        if abs(espacio_izq - espacio_der) > 0.08:
            mejor = 'IZQ' if espacio_izq > espacio_der else 'DER'
        else:
            # Espacio similar: romper empate con historial (ir al lado menos girado)
            n_izq = sum(1 for g in self._historial_giros if g == 'IZQ')
            n_der = sum(1 for g in self._historial_giros if g == 'DER')
            mejor = 'IZQ' if n_izq <= n_der else 'DER'

        # Anti-sesgo: si los ultimos 4 giros son iguales, forzar cambio
        if self._sesgo_giro(mejor):
            mejor = 'DER' if mejor == 'IZQ' else 'IZQ'

        # Nueva direccion: registrar y reiniciar contador
        if mejor != self._giro_dir:
            if self._giro_dir is not None:
                self._historial_giros.append(self._giro_dir)
            self._giro_dir   = mejor
            self._giro_ticks = 0

        return mejor

    def _navegar(self, frente_min, frente_max, f0, f1, f2,
                 espacio_izq, espacio_der, espacio_tot,
                 i0, i1, i2, d0, d1, d2):

        # ---- CASO 1: completamente encerrado
        if espacio_tot < self.D_STOP * 3:
            dir_ = self._elegir_direccion_giro(espacio_izq, espacio_der)
            self._giro_ticks += 1
            self.estado = 'GIRAR_IZQUIERDA' if dir_ == 'IZQ' else 'GIRAR_DERECHA'
            return

        # ---- CASO 2: pared muy cerca al frente -> girar en sitio
        if frente_min < self.D_PELIGRO:
            dir_ = self._elegir_direccion_giro(espacio_izq, espacio_der)
            self._giro_ticks += 1
            # timeout: llevar demasiado tiempo -> forzar cambio
            if self._giro_ticks > self.GIRO_TIMEOUT:
                self._historial_giros.append(self._giro_dir)
                self._giro_dir   = 'DER' if self._giro_dir == 'IZQ' else 'IZQ'
                self._giro_ticks = 0
                dir_ = self._giro_dir
            self.estado = 'GIRAR_IZQUIERDA' if dir_ == 'IZQ' else 'GIRAR_DERECHA'
            return

        # ---- Llegar hasta aqui significa que el frente tiene algo de espacio
        # Resetear control de giro
        if self._giro_dir is not None and frente_min > self.D_PRECAUCION:
            self._historial_giros.append(self._giro_dir)
            self._giro_dir   = None
            self._giro_ticks = 0

        # ---- CASO 3: pared cerca al frente -> curvar suave
        if frente_min < self.D_PRECAUCION:
            # Curvar hacia el lado con mas espacio
            if espacio_izq >= espacio_der:
                self.estado = 'GIRAR_IZQUIERDA_SUAVE'
            else:
                self.estado = 'GIRAR_DERECHA_SUAVE'
            return

        # ---- CASO 4: frente libre -> avanzar recto
        # Correccion diagonal minima: solo si un lado del frente esta muy cerrado
        if abs(f0 - f2) > 0.20:
            if f0 < f2:
                self.estado = 'CORREGIR_DERECHA'
            else:
                self.estado = 'CORREGIR_IZQUIERDA'
        else:
            self.estado = 'AVANZAR'

    # ------------------------------------------------------------------ output
    def decidir_accion(self):
        e = self.estado

        if e == '_ESCAPE':
            return self._escape_accion or {'velocidad_motor': self.VEL_RETRO, 'fuerza_giro': self.GIRO_ESCAPE}

        if e == 'AVANZAR':
            return {'velocidad_motor': self.VEL_NORMAL, 'fuerza_giro': 0}

        if e == 'CORREGIR_DERECHA':
            return {'velocidad_motor': self.VEL_NORMAL - 2, 'fuerza_giro': self.GIRO_SUAVE}

        if e == 'CORREGIR_IZQUIERDA':
            return {'velocidad_motor': self.VEL_NORMAL - 2, 'fuerza_giro': -self.GIRO_SUAVE}

        if e == 'GIRAR_IZQUIERDA_SUAVE':
            return {'velocidad_motor': self.VEL_SUAVE, 'fuerza_giro': -self.GIRO_FUERTE}

        if e == 'GIRAR_DERECHA_SUAVE':
            return {'velocidad_motor': self.VEL_SUAVE, 'fuerza_giro': self.GIRO_FUERTE}

        if e == 'GIRAR_IZQUIERDA':
            return {'velocidad_motor': 0, 'fuerza_giro': -self.GIRO_FUERTE}

        if e == 'GIRAR_DERECHA':
            return {'velocidad_motor': 0, 'fuerza_giro': self.GIRO_FUERTE}

        return {'velocidad_motor': 0, 'fuerza_giro': 0}