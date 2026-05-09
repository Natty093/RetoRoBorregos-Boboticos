class CerebroRobot:
    def __init__(self):
        # El robot siempre inicia con la intención de ir hacia adelante
        self.estado = 'AVANZAR'

    def percibir_entorno(self, distancia_obstaculo):
        """
        Esta función simula la lectura de un sensor. 
        Tu amigo desde mundo.py te enviará el dato de la distancia.
        """
        # Si detectamos un obstáculo a menos de 1 metro (unidad de PyBullet)
        if distancia_obstaculo < 1.0: 
            self.estado = 'EVADIR'
        else:
            self.estado = 'AVANZAR'

    def decidir_accion(self):
        """
        Dependiendo del estado, el cerebro decide qué comandos de motor enviar.
        """
        if self.estado == 'AVANZAR':
            # Vamos rápido y derecho
            return {"velocidad_motor": 10, "fuerza_giro": 0} 
            
        elif self.estado == 'EVADIR':
            # Bajamos la velocidad y giramos para esquivar
            return {"velocidad_motor": 2, "fuerza_giro": 5}