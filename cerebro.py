class CerebroRobot:
    def __init__(self):
        self.estado = 'BUSCANDO_PARED'
        self.distancia_segura = 0.5    # metros — si hay pared a menos de esto, gira
        self.umbral_perdida_pared = 1.0  # metros — si derecha > esto, la perdimos
 
    def percibir_entorno(self, distancias_lidar):
        """
        Recibe 9 distancias del LiDAR en metros.
        Índices: 0-2 (Izquierda), 3-5 (Frente), 6-8 (Derecha).
        Regla de la mano derecha: siempre seguir la pared de la derecha.
        """
        izquierda = min(distancias_lidar[0:3])
        frente    = min(distancias_lidar[3:6])
        derecha   = min(distancias_lidar[6:9])
 
        if frente < self.distancia_segura:
            # Hay pared enfrente: girar izquierda (prioridad máxima)
            self.estado = 'GIRAR_IZQUIERDA'
 
        elif derecha > self.umbral_perdida_pared:
            # Perdimos la pared de la derecha: girar derecha para encontrarla
            self.estado = 'GIRAR_DERECHA'
 
        else:
            # Pared a la derecha, camino libre al frente: avanzar
            self.estado = 'AVANZAR'
 
    def decidir_accion(self):
        """Traduce el estado a órdenes de motor."""
        if self.estado == 'AVANZAR':
            return {"velocidad_motor": 10, "fuerza_giro": 0}
 
        elif self.estado == 'GIRAR_DERECHA':
            return {"velocidad_motor": 4, "fuerza_giro": 5}
 
        elif self.estado == 'GIRAR_IZQUIERDA':
            return {"velocidad_motor": 3, "fuerza_giro": -6}
 
        # Por si acaso
        return {"velocidad_motor": 0, "fuerza_giro": 0}