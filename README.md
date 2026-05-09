# 🤖 Laberinto Autónomo — Reto Roborregos
 
Simulación de un robot con navegación autónoma que resuelve un laberinto usando LIDAR virtual en PyBullet. El robot escanea su entorno, detecta caminos libres y avanza sin intervención humana.
 
---
 
## ¿Qué hace?
 
- El robot arranca en una posición inicial dentro del laberinto
- Usa **9 rayos LIDAR** para medir distancias en todas las direcciones
- Cuando detecta una pared al frente, **gira en sitio escaneando** hasta encontrar un camino libre
- Una vez confirmado el hueco, **avanza directo hacia él**
- Cuenta con detección de atasco y maniobra de escape automática
- El recorrido termina cuando el robot llega a la meta
---
 
## Estructura del proyecto
 
```
├── cerebro.py   # Lógica de navegación autónoma (estados, LIDAR, escaneo)
├── plano.py     # Entorno PyBullet (mundo, paredes, robot, física)
├── main.py      # Loop principal (conecta todo, detecta meta, muestra HUD)
├── robot.urdf   # Modelo del robot
└── mapa.urdf    # Modelo visual del laberinto
```
 
---
 
## Requisitos
 
- Python 3.8+
- PyBullet
Instala la dependencia con:
 
```bash
pip install pybullet
```
 
---
 
## Cómo correrlo
 
```bash
python main.py
```
 
Se abrirá la ventana de simulación de PyBullet y el robot comenzará a navegar automáticamente del punto de inicio a la meta.
 
Para cerrar la simulación presiona `Ctrl+C` en la terminal.
O `esq`
---
 
## Parámetros clave (cerebro.py)
 
| Parámetro | Valor | Descripción |
|---|---|---|
| `D_LIBRE` | 0.50 m | Distancia mínima para considerar el frente despejado |
| `D_PELIGRO` | 0.32 m | Distancia a la que frena y empieza a escanear |
| `VEL_MAX` | 20 | Velocidad máxima de avance |
| `GIRO_SCAN` | 13 | Fuerza de giro durante el escaneo |
| `CONFIRM_LIBRE` | 4 ticks | Confirmaciones consecutivas antes de avanzar |
 
---
 
## Estados del robot
 
| Estado | Descripción |
|---|---|
| `AVANZAR` | Camino libre, avanza recto |
| `SCAN_IZQ / SCAN_DER` | Girando en sitio buscando hueco |
| `CURVAR_IZQ / CURVAR_DER` | Curvando suave hacia el lado más abierto |
| `CORREGIR_IZQ / CORREGIR_DER` | Corrección lateral mínima mientras avanza |
| `_ESCAPE` | Retrocede con giro para salir de un atasco |
 
---
 
## Equipo
- Natalie
- Leo
Desarrollado para el **Reto Roborregos** — Tecnológico de Monterrey
