# 🐢 Taller ROS 2: Comunicación entre Nodos con Turtlesim

## 🎯 Objetivo del taller

Desarrollar dos nodos ROS 2 que se comuniquen entre sí mediante tópicos para controlar el movimiento de *Turtlesim* en un patrón **triangular**, utilizando la información de odometría.

---

## 🧱 Parte 1: Crear el Workspace

```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws
colcon build
source install/setup.bash
```

---

## 📦 Parte 2: Crear el paquete

Creamos un paquete llamado `turtle_controller` con soporte para Python y las dependencias necesarias:

```bash
cd ~/ros2_ws/src
ros2 pkg create turtle_controller --build-type ament_python --dependencies rclpy geometry_msgs turtlesim
```

Estructura esperada del paquete:

```
turtle_controller/
├── turtle_controller/
│   └── __init__.py
├── package.xml
├── setup.cfg
├── setup.py
```

---

## 🐢 Parte 3: Ejecutar Turtlesim

En una nueva terminal:

```bash
source /opt/ros/humble/setup.bash
ros2 run turtlesim turtlesim_node
```

---

## 📥 Parte 4: Nodo Subscriber — `odom_listener.py`

Creamos el archivo:

```bash
cd ~/ros2_ws/src/turtle_controller/turtle_controller
touch odom_listener.py
chmod +x odom_listener.py
```

### 📘 ¿Qué hace este nodo?

- Se suscribe al tópico `/turtle1/pose`, donde Turtlesim publica su odometría.
- Cada vez que recibe una actualización, imprime en consola las coordenadas X, Y y el ángulo `theta`.

### 📄 Código:

```python
import rclpy
from rclpy.node import Node
from turtlesim.msg import Pose

class OdomListener(Node):
    def __init__(self):
        super().__init__('odom_listener')
        self.subscription = self.create_subscription(
            Pose,
            '/turtle1/pose',
            self.listener_callback,
            10)

    def listener_callback(self, msg):
        self.get_logger().info(
            f'Odom: x={msg.x:.2f}, y={msg.y:.2f}, theta={msg.theta:.2f}')

def main(args=None):
    rclpy.init(args=args)
    node = OdomListener()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
```

---

## 📤 Parte 5: Nodo Publisher — `triangle_mover.py`

Creamos el archivo:

```bash
touch triangle_mover.py
chmod +x triangle_mover.py
```

### 📘 ¿Qué hace este nodo?

- Publica comandos de velocidad en el tópico `/turtle1/cmd_vel`.
- Mueve la tortuga en línea recta durante unos segundos, luego gira aproximadamente 120°.
- Repite este comportamiento para formar un movimiento triangular.

### 📄 Código:

```python
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class TriangleMover(Node):
    def __init__(self):
        super().__init__('triangle_mover')
        self.publisher = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        self.timer = self.create_timer(0.5, self.move_in_triangle)
        self.step = 0
        self.turning = False

    def move_in_triangle(self):
        msg = Twist()

        if not self.turning:
            msg.linear.x = 2.0
            msg.angular.z = 0.0
            self.publisher.publish(msg)
            self.step += 1
            if self.step % 10 == 0:
                self.turning = True
                self.step = 0
        else:
            msg.linear.x = 0.0
            msg.angular.z = 2.0
            self.publisher.publish(msg)
            self.step += 1
            if self.step >= 6:
                self.turning = False
                self.step = 0

def main(args=None):
    rclpy.init(args=args)
    node = TriangleMover()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
```

---

## 🚀 Parte 6: Compilar y ejecutar

```bash
cd ~/ros2_ws
colcon build
source install/setup.bash
```

Lanzar los nodos en terminales separadas:

```bash
ros2 run turtle_controller odom_listener
ros2 run turtle_controller triangle_mover
```

---

## 📤 Entregables

Cada estudiante debe entregar:

- ✅ El archivo `odom_listener.py`.
- ✅ El archivo `triangle_mover.py`.
- ✅ Un video donde se vea la tortuga moviéndose en un patrón triangular, mientras ambos nodos están activos.
