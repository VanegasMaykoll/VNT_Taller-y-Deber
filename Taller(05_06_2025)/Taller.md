# 🐢 Taller ROS 2: Comunicación entre Nodos con Turtlesim

## 🎯 Objetivo del taller

Desarrollar dos nodos ROS 2 que se comuniquen entre sí mediante tópicos para controlar el movimiento de *Turtlesim* en un patrón **triangular**, utilizando la información de odometría.

---

## 🧱 Parte 1: Crear el Workspace

Antes de comenzar a desarrollar con ROS 2, es necesario crear un **workspace**, que es un entorno de trabajo donde se almacenarán y compilarán todos los paquetes del proyecto.

Los siguientes comandos te permiten crear y preparar ese entorno:

```bash
mkdir -p ~/ros2_ws/src
```

Este comando crea una carpeta llamada `ros2_ws` dentro de tu directorio personal (`~`) y, dentro de ella, una subcarpeta llamada `src`. Esta carpeta `src` es esencial, ya que es donde se colocarán los paquetes ROS que vas a crear o clonar. `colcon build` buscará automáticamente los paquetes dentro de esta carpeta para compilarlos.

```bash
cd ~/ros2_ws
colcon build
```

Después de crear la estructura del workspace, cambiamos al directorio raíz (`ros2_ws`) y ejecutamos `colcon build`, que es la herramienta recomendada en ROS 2 para compilar workspaces. Esta orden compila todos los paquetes dentro de `src` y genera las carpetas `build/`, `install/` y `log/`. La carpeta `install/` es la que contendrá los archivos necesarios para ejecutar los nodos y scripts.

```bash
source install/setup.bash
```

Este comando configura el entorno actual de la terminal para que reconozca los paquetes y nodos que has compilado en tu workspace. Es **muy importante** ejecutar este comando **en cada nueva terminal** antes de correr cualquier nodo.


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

### 📄 Código explicado:

```python
import rclpy
from rclpy.node import Node
from turtlesim.msg import Pose
```

* Se importan los módulos necesarios:

  * `rclpy`: la biblioteca cliente de ROS 2 para Python.
  * `Node`: clase base para crear nodos.
  * `Pose`: tipo de mensaje que contiene los datos de odometría de Turtlesim.

```python
class OdomListener(Node):
    def __init__(self):
        super().__init__('odom_listener')
```

* Se define una clase llamada `OdomListener`, que hereda de `Node`, lo que convierte esta clase en un nodo ROS 2.
* En el constructor (`__init__`), se inicializa el nodo con el nombre `"odom_listener"`.

```python
        self.subscription = self.create_subscription(
            Pose,
            '/turtle1/pose',
            self.listener_callback,
            10)
```

* Aquí se crea una **suscripción**:

  * El tipo de mensaje que espera es `Pose`.
  * El tópico al que se suscribe es `/turtle1/pose`.
  * La función que se ejecutará cada vez que se reciba un mensaje es `self.listener_callback`.
  * El número `10` indica el tamaño del búfer o *queue* de mensajes.

```python
    def listener_callback(self, msg):
        self.get_logger().info(
            f'Odom: x={msg.x:.2f}, y={msg.y:.2f}, theta={msg.theta:.2f}')
```

* Esta función se llama automáticamente cuando llega un mensaje.
* `msg` es el mensaje de tipo `Pose` recibido.
* La función extrae los valores `x`, `y` y `theta` y los imprime en consola usando el *logger* del nodo con formato de 2 decimales.

```python
def main(args=None):
    rclpy.init(args=args)
    node = OdomListener()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
```

* Se define la función principal del script.
* `rclpy.init()` inicializa el sistema ROS 2 en Python.
* Se crea una instancia del nodo `OdomListener`.
* `rclpy.spin(node)` mantiene el nodo en ejecución escuchando mensajes.
* Al finalizar (por ejemplo, al presionar Ctrl+C), se destruye el nodo y se apaga ROS 2.
---

¡Perfecto! Aquí tienes la sección **mejorada y explicada** del nodo `triangle_mover.py`, con el propósito de que los estudiantes comprendan **cada parte del código**:

---

## 📤 Parte 5: Nodo Publisher — `triangle_mover.py`

Primero, creamos el archivo del nodo:

```bash
touch triangle_mover.py
chmod +x triangle_mover.py
```

### 📘 ¿Qué hace este nodo?

Este nodo se encarga de **enviar comandos de velocidad** al tópico `/turtle1/cmd_vel`, el cual controla el movimiento de la tortuga en Turtlesim. La lógica está diseñada para que la tortuga avance en línea recta por algunos segundos, luego gire aproximadamente 120°, y repita este patrón tres veces para formar un **triángulo equilátero**.


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

### 📄 Código explicado:

```python
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
```
* Se importan las bibliotecas necesarias:
  * `rclpy`: biblioteca cliente de ROS 2 para Python.
  * `Node`: clase base para definir nodos.
  * `Twist`: tipo de mensaje usado para controlar velocidad lineal y angular (muy común en robótica móvil).


```python
class TriangleMover(Node):
    def __init__(self):
        super().__init__('triangle_mover')
```
* Se define la clase `TriangleMover` que hereda de `Node`, lo cual convierte esta clase en un nodo.
* Se inicializa el nodo con el nombre `"triangle_mover"`.


```python
        self.publisher = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
```
* Se crea un **publicador** de mensajes tipo `Twist` en el tópico `/turtle1/cmd_vel`.
* Este tópico controla el movimiento de la tortuga.
* El parámetro `10` es el tamaño del *queue* de mensajes.


```python
        self.timer = self.create_timer(0.5, self.move_in_triangle)
```
* Se configura un **temporizador** que llama a la función `self.move_in_triangle()` cada 0.5 segundos.
* Esto genera un ciclo repetitivo de movimiento.


```python
        self.step = 0
        self.turning = False
```
* `step` lleva un conteo de los ciclos del temporizador.
* `turning` indica si la tortuga está girando o avanzando en línea recta.


```python
    def move_in_triangle(self):
        msg = Twist()
```
* Se crea una nueva instancia del mensaje `Twist`, que contiene los comandos de velocidad a publicar.


```python
        if not self.turning:
            msg.linear.x = 2.0
            msg.angular.z = 0.0
            self.publisher.publish(msg)
            self.step += 1
            if self.step % 10 == 0:
                self.turning = True
                self.step = 0
```
* Si **no está girando**, se establece una velocidad **lineal** (2.0) y se mantiene sin rotación.
* Esto hace que la tortuga avance recto.
* Cada 10 pasos de 0.5 segundos (5 segundos), cambia al estado de giro (`turning = True`).


```python
        else:
            msg.linear.x = 0.0
            msg.angular.z = 2.0
            self.publisher.publish(msg)
            self.step += 1
            if self.step >= 6:
                self.turning = False
                self.step = 0
```
* Si **está girando**, se establece una velocidad angular de 2.0 rad/s y sin avance lineal.
* Después de 6 pasos de 0.5 segundos (3 segundos), se detiene el giro y vuelve a avanzar recto.
* Este giro dura lo suficiente para aproximar un ángulo de 120°, completando así los vértices de un triángulo.


```python
def main(args=None):
    rclpy.init(args=args)
    node = TriangleMover()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
```
* Se define la función principal del script.
* Se inicializa el entorno de ROS 2.
* Se crea una instancia del nodo `TriangleMover` y se mantiene activo con `rclpy.spin()`.
* Cuando se detiene, el nodo se destruye y ROS 2 se apaga correctamente.
  
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
