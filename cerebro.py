class CerebroRobot:
    def __init__(self):
        self.estado = 'BUSCANDO_PARED'
        self.distancia_segura = 0.8 
        self.umbral_perdida_pared = 1.2 

    def percibir_entorno(self, distancias_lidar):
        """
        Recibe la lista de 9 distancias del LiDAR de PyBullet.
        Indices: 0-2 (Izquierda), 3-5 (Frente), 6-8 (Derecha).
        """
        izquierda = min(distancias_lidar[0:3])
        frente = min(distancias_lidar[3:6])
        derecha = min(distancias_lidar[6:9])

        # Regla de mano derechaaaaaa
        if derecha > self.umbral_perdida_pared:
            self.estado = 'GIRAR_DERECHA'
            
        elif frente < self.distancia_segura:
            self.estado = 'GIRAR_IZQUIERDA'
            
        else:
            self.estado = 'AVANZAR'

    def decidir_accion(self):
        if self.estado == 'AVANZAR':
            # Ambos motores hacia adelante rápido
            return {"velocidad_motor": 15, "fuerza_giro": 0}
            
        elif self.estado == 'GIRAR_DERECHA':
            # Disminuye la velocidad y aplica un giro positivo
            return {"velocidad_motor": 5, "fuerza_giro": 6} 
            
        elif self.estado == 'GIRAR_IZQUIERDA':
            # Disminuye la velocidad y aplica un giro negativo
            return {"velocidad_motor": 5, "fuerza_giro": -6}