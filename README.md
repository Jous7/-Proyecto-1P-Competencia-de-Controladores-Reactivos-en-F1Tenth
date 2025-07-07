# 🏎️ Proyecto F1TENTH: Controlador Reactivo en ROS 2

Este proyecto implementa un **controlador reactivo autónomo** para un vehículo simulado F1TENTH utilizando ROS 2. El vehículo navega por una pista usando el algoritmo **Follow the Gap**, evitando obstáculos en tiempo real con LIDAR.

El sistema también incluye:
- 🧮 Conteo automático de vueltas
- ⏱️ Cronómetro por vuelta
- 📹 Evidencia en video del funcionamiento

---

## ⚙️ Enfoque utilizado: Follow the Gap

El controlador utiliza el enfoque **Follow the Gap**, que trabaja así:

1. Se procesan los datos del LIDAR y se limita el campo de visión al frente.
2. Se elimina el obstáculo más cercano aplicando un “radio de burbuja”.
3. Se identifican los segmentos libres ("gaps") y se elige el más amplio.
4. Se calcula la dirección óptima y se ajusta la velocidad en función del ángulo.
5. Se publica el comando de movimiento al topic `/drive`.

Este enfoque permite al vehículo navegar evitando obstáculos sin necesidad de mapas o planificación previa.

---

## 🧠 Estructura del Código

ros2_controller_project/
├── controller_node.py # Nodo principal con lógica de navegación
├── README.md # Documentación del proyecto
├── LICENSE # Licencia MIT
└── videos/
└── demo_video.mp4 # (opcional) Video demostrativo


Funciones principales:

- `lidar_callback(msg)`: Aplica Follow the Gap y ajusta dirección y velocidad.
- `odom_callback(msg)`: Detecta paso por meta, cuenta vueltas y mide el tiempo.

---

## 🚀 Instrucciones de ejecución

1. **Clona el repositorio** en tu workspace de ROS 2:

```bash
cd ~/ros2_ws/src
git clone https://github.com/Jous7/Proyecto-1P-Competencia-de-Controladores-Reactivos-en-F1Tenth.git
cd ..
colcon build
source install/setup.bash

2. **Lanza el simulador F1TENTH**

3. **Ejecuta el nodo controlador**

ros2 run ros2_controller_project controller_node


Conteo de vueltas y cronómetro

    Cada vez que el vehículo pasa por la posición inicial (definida manualmente), se registra una vuelta.

    Se calcula el tiempo transcurrido desde la vuelta anterior.

    Los resultados se muestran en consola, por ejemplo:

🏁 Vuelta 1 completada en 61.23 segundos
🏁 Vuelta 2 completada en 60.89 segundos

Autor
Patricio Caicedo
Proyecto Primer Parcial ESPOL-Vehiculos no tripulados
Licencia
Este proyecto está distribuido bajo la Licencia MIT.
