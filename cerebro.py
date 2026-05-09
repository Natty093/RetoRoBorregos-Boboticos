import math

class CerebroRobot:
    def __init__(self):
        self.estado = 'AVANZAR'

        # Umbrales de distancia
        self.D_PELIGRO    = 0.28
        self.D_PRECAUCION = 0.52

        # Anti-atasco: historial de posicion
        self._pos_historia  = []   # ultimas posiciones (x, y)
        self._tick          = 0
        self._atascado      = False
        self._escape_ticks  = 0    # cuantos ticks quedan de maniobra de escape
        self._escape_accion = None

        # Direccion preferida de giro (para no oscilar)
        self._giro_preferido = None  # 'IZQ' o 'DER'
        self._giro_ticks     = 0     # ticks que llevamos girando en la misma dir

    # ------------------------------------------------------------------
    def percibir_entorno(self, distancias_lidar, pos_robot=None):
        """
        9 rayos: indices 0-2 Izquierda, 3-5 Frente, 6-8 Derecha.
        pos_robot: tupla (x, y, z) opcional para deteccion de atasco.
        """
        self._tick += 1

        izq_ext = distancias_lidar[0]
        izq_med = distancias_lidar[1]
        izq_cer = distancias_lidar[2]
        frt_izq = distancias_lidar[3]
        frt_cen = distancias_lidar[4]
        frt_der = distancias_lidar[5]
        der_cer = distancias_lidar[6]
        der_med = distancias_lidar[7]
        der_ext = distancias_lidar[8]

        frente_min  = min(frt_izq, frt_cen, frt_der)
        espacio_izq = (izq_ext + izq_med + izq_cer) / 3
        espacio_der = (der_cer + der_med + der_ext) / 3
        espacio_total = espacio_izq + espacio_der + frente_min

        # --- Deteccion de atasco por posicion ---
        if pos_robot is not None:
            self._pos_historia.append((pos_robot[0], pos_robot[1]))
            if len(self._pos_historia) > 120:   # ventana de ~2s a 60Hz
                self._pos_historia.pop(0)

            if len(self._pos_historia) >= 120:
                dx = self._pos_historia[-1][0] - self._pos_historia[0][0]
                dy = self._pos_historia[-1][1] - self._pos_historia[0][1]
                desplazamiento = math.sqrt(dx*dx + dy*dy)
                self._atascado = desplazamiento < 0.08  # menos de 8cm en 2s
            else:
                self._atascado = False

        # --- Si hay maniobra de escape en curso, no interrumpir ---
        if self._escape_ticks > 0:
            self._escape_ticks -= 1
            self.estado = '_ESCAPE'
            return

        # --- Activar escape si atascado ---
        if self._atascado:
            self._pos_historia.clear()
            self._atascado = False
            # Retroceder y girar hacia el lado mas libre
            if espacio_izq >= espacio_der:
                self._escape_accion = {"velocidad_motor": -18, "fuerza_giro": -20}
            else:
                self._escape_accion = {"velocidad_motor": -18, "fuerza_giro": +20}
            self._escape_ticks = 90   # ~1.5s de escape
            self._giro_preferido = None
            self.estado = '_ESCAPE'
            return

        # --- Logica normal de navegacion ---

        # Caso: completamente bloqueado por todos lados -> giro en sitio
        if espacio_total < self.D_PELIGRO * 3:
            self._activar_giro_sitio(espacio_izq, espacio_der)
            return

        # Caso: pared justo al frente
        if frente_min < self.D_PELIGRO:
            # Mantener direccion preferida para no oscilar
            if self._giro_preferido is None:
                self._giro_preferido = 'IZQ' if espacio_izq >= espacio_der else 'DER'
            self._giro_ticks += 1

            # Si llevamos mas de 3s girando en la misma dir sin avanzar -> cambiar
            if self._giro_ticks > 180:
                self._giro_preferido = 'DER' if self._giro_preferido == 'IZQ' else 'IZQ'
                self._giro_ticks = 0

            if self._giro_preferido == 'IZQ':
                self.estado = 'GIRAR_IZQUIERDA'
            else:
                self.estado = 'GIRAR_DERECHA'
            return

        # Caso: pared cerca al frente
        if frente_min < self.D_PRECAUCION:
            self._giro_preferido = None
            self._giro_ticks = 0
            if espacio_izq >= espacio_der:
                self.estado = 'GIRAR_IZQUIERDA_SUAVE'
            else:
                self.estado = 'GIRAR_DERECHA_SUAVE'
            return

        # Caso: frente libre — avanzar con pequena correccion lateral
        self._giro_preferido = None
        self._giro_ticks = 0

        if frt_cen < self.D_PRECAUCION * 1.5:
            if frt_izq > frt_der:
                self.estado = 'CORREGIR_IZQUIERDA'
            elif frt_der > frt_izq:
                self.estado = 'CORREGIR_DERECHA'
            else:
                self.estado = 'AVANZAR'
        else:
            self.estado = 'AVANZAR'

    # ------------------------------------------------------------------
    def _activar_giro_sitio(self, espacio_izq, espacio_der):
        """Giro en sitio (velocidad=0 en un lado) cuando todo esta bloqueado."""
        if espacio_izq >= espacio_der:
            self.estado = 'GIRAR_IZQUIERDA'
        else:
            self.estado = 'GIRAR_DERECHA'

    # ------------------------------------------------------------------
    def decidir_accion(self):
        if self.estado == '_ESCAPE':
            return self._escape_accion or {"velocidad_motor": -15, "fuerza_giro": 15}

        if self.estado == 'AVANZAR':
            return {"velocidad_motor": 20, "fuerza_giro": 0}

        if self.estado == 'CORREGIR_DERECHA':
            return {"velocidad_motor": 18, "fuerza_giro": 3}

        if self.estado == 'CORREGIR_IZQUIERDA':
            return {"velocidad_motor": 18, "fuerza_giro": -3}

        if self.estado == 'GIRAR_IZQUIERDA_SUAVE':
            return {"velocidad_motor": 12, "fuerza_giro": -8}

        if self.estado == 'GIRAR_DERECHA_SUAVE':
            return {"velocidad_motor": 12, "fuerza_giro": 8}

        if self.estado == 'GIRAR_IZQUIERDA':
            return {"velocidad_motor": 0, "fuerza_giro": -15}

        if self.estado == 'GIRAR_DERECHA':
            return {"velocidad_motor": 0, "fuerza_giro": 15}

        return {"velocidad_motor": 0, "fuerza_giro": 0}