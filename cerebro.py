import math
from collections import deque


class CerebroRobot:
    """
    9 rayos LIDAR: 0=IZQ-EXT 1=IZQ-MED 2=IZQ-CER
                   3=FRT-IZQ 4=FRT-CEN 5=FRT-DER
                   6=DER-CER 7=DER-MED 8=DER-EXT
    """

    # -- distancias
    D_STOP      = 0.35
    D_CURVA     = 0.60
    D_LIBRE     = 0.70

    # -- velocidades
    VEL_MAX     = 22
    VEL_CURVA   = 12
    VEL_RETRO   = -15
    GIRO_PIVOTE = 14
    GIRO_CURVA  = 10
    GIRO_SUAVE  = 4
    GIRO_ESCAPE = 16

    # -- anti-atasco
    ATASCO_VENTANA = 150
    # FIX 1: umbral más generoso para no detectar pivote como atasco
    ATASCO_UMBRAL  = 0.25
    GIRO_MAX_TICKS = 130
    ESCAPE_TICKS   = 60   # FIX 2: menos tiempo en reversa, evita chocar la pared de atrás

    # FIX 3: más confirmaciones antes de arrancar (evita falsos positivos)
    CONFIRM_NEEDED = 5

    def __init__(self):
        self.estado = 'AVANZAR'
        self._pos_hist = deque(maxlen=self.ATASCO_VENTANA)
        self._escape_ticks  = 0
        self._escape_accion = None
        self._giro_dir   = None
        self._giro_ticks = 0
        self._hist_giros = deque(maxlen=8)
        self._confirm    = 0
        # FIX 4: contador de ticks en pivote para suprimir detección de atasco
        self._pivotando_ticks = 0

    # ------------------------------------------------------------ public
    def percibir_entorno(self, distancias, pos_robot=None):
        if pos_robot:
            self._pos_hist.append((pos_robot[0], pos_robot[1]))

        i0,i1,i2 = distancias[0], distancias[1], distancias[2]
        f0,f1,f2 = distancias[3], distancias[4], distancias[5]
        d0,d1,d2 = distancias[6], distancias[7], distancias[8]

        frente_min  = min(f0, f1, f2)
        espacio_izq = (i0+i1+i2) / 3
        espacio_der = (d0+d1+d2) / 3

        # escape activo
        if self._escape_ticks > 0:
            self._escape_ticks -= 1
            if frente_min > self.D_CURVA and self._escape_ticks > 20:
                self._escape_ticks = 0
            self.estado = '_ESCAPE'
            return

        # FIX 5: solo checar atasco si NO estamos pivotando intencionalmente
        en_pivote = self._giro_dir is not None
        if not en_pivote and self._atascado():
            self._escapar(espacio_izq, espacio_der)
            return

        # pivote demasiado largo → escapar
        if self._giro_dir and self._giro_ticks >= self.GIRO_MAX_TICKS:
            self._escapar(espacio_izq, espacio_der)
            return

        self._decidir(f0, f1, f2, frente_min, espacio_izq, espacio_der)

    def decidir_accion(self):
        e = self.estado
        if e == '_ESCAPE':
            return self._escape_accion or {'velocidad_motor': self.VEL_RETRO, 'fuerza_giro': self.GIRO_ESCAPE}
        if e == 'AVANZAR':
            return {'velocidad_motor': self.VEL_MAX,   'fuerza_giro': 0}
        if e == 'CORREGIR_DER':
            return {'velocidad_motor': self.VEL_MAX,   'fuerza_giro':  self.GIRO_SUAVE}
        if e == 'CORREGIR_IZQ':
            return {'velocidad_motor': self.VEL_MAX,   'fuerza_giro': -self.GIRO_SUAVE}
        if e == 'CURVAR_DER':
            return {'velocidad_motor': self.VEL_CURVA, 'fuerza_giro':  self.GIRO_CURVA}
        if e == 'CURVAR_IZQ':
            return {'velocidad_motor': self.VEL_CURVA, 'fuerza_giro': -self.GIRO_CURVA}
        if e == 'PIVOTAR_DER':
            return {'velocidad_motor': 0, 'fuerza_giro':  self.GIRO_PIVOTE}
        if e == 'PIVOTAR_IZQ':
            return {'velocidad_motor': 0, 'fuerza_giro': -self.GIRO_PIVOTE}
        return {'velocidad_motor': 0, 'fuerza_giro': 0}

    # ------------------------------------------------------------ private
    def _atascado(self):
        if len(self._pos_hist) < self.ATASCO_VENTANA:
            return False
        dx = self._pos_hist[-1][0] - self._pos_hist[0][0]
        dy = self._pos_hist[-1][1] - self._pos_hist[0][1]
        return math.sqrt(dx*dx + dy*dy) < self.ATASCO_UMBRAL

    def _escapar(self, espacio_izq, espacio_der):
        self._pos_hist.clear()
        self._giro_dir   = None
        self._giro_ticks = 0
        self._confirm    = 0
        self._pivotando_ticks = 0
        dir_ = 'IZQ' if espacio_izq >= espacio_der else 'DER'
        if len(self._hist_giros) >= 4 and all(g == dir_ for g in list(self._hist_giros)[-4:]):
            dir_ = 'DER' if dir_ == 'IZQ' else 'IZQ'
        giro = -self.GIRO_ESCAPE if dir_ == 'IZQ' else self.GIRO_ESCAPE
        self._escape_accion = {'velocidad_motor': self.VEL_RETRO, 'fuerza_giro': giro}
        self._escape_ticks  = self.ESCAPE_TICKS
        self.estado = '_ESCAPE'

    def _dir_pivote(self, espacio_izq, espacio_der):
        if self._giro_dir:
            return self._giro_dir
        if abs(espacio_izq - espacio_der) > 0.08:
            dir_ = 'IZQ' if espacio_izq > espacio_der else 'DER'
        else:
            n_izq = sum(1 for g in self._hist_giros if g == 'IZQ')
            n_der = sum(1 for g in self._hist_giros if g == 'DER')
            dir_ = 'IZQ' if n_izq <= n_der else 'DER'
        self._giro_dir   = dir_
        self._giro_ticks = 0
        return dir_

    def _frente_libre(self, f0, f1, f2):
        """True si al menos 2 rayos frontales contiguos superan D_LIBRE."""
        l = [f0 >= self.D_LIBRE, f1 >= self.D_LIBRE, f2 >= self.D_LIBRE]
        return (l[0] and l[1]) or (l[1] and l[2])

    def _decidir(self, f0, f1, f2, frente_min, espacio_izq, espacio_der):

        libre = self._frente_libre(f0, f1, f2)

        # ---- frente bloqueado: pivota ----
        if not libre:
            self._confirm = 0
            dir_ = self._dir_pivote(espacio_izq, espacio_der)
            self._giro_ticks += 1
            # FIX 6: limpiar historial de posición mientras pivotamos
            # para que no cuente como atasco cuando arranquemos
            self._pos_hist.clear()
            self.estado = 'PIVOTAR_IZQ' if dir_ == 'IZQ' else 'PIVOTAR_DER'
            return

        # ---- frente libre: acumular confirmaciones ----
        self._confirm += 1
        if self._confirm < self.CONFIRM_NEEDED:
            dir_ = self._dir_pivote(espacio_izq, espacio_der)
            self._giro_ticks += 1
            self.estado = 'PIVOTAR_IZQ' if dir_ == 'IZQ' else 'PIVOTAR_DER'
            return

        # ---- confirmado: liberar pivote y avanzar ----
        self._confirm = 0
        if self._giro_dir:
            self._hist_giros.append(self._giro_dir)
            self._giro_dir   = None
            self._giro_ticks = 0

        # cerca pero con espacio: curvar hacia el lado más abierto
        if frente_min < self.D_CURVA:
            if espacio_izq >= espacio_der:
                self.estado = 'CURVAR_IZQ'
            else:
                self.estado = 'CURVAR_DER'
            return

        # libre: avanzar con corrección mínima si hay sesgo lateral
        if abs(f0 - f2) > 0.25:
            self.estado = 'CORREGIR_DER' if f0 < f2 else 'CORREGIR_IZQ'
        else:
            self.estado = 'AVANZAR'