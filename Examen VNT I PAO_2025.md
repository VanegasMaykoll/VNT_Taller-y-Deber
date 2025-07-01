# 🧪 Examen del Primer Parcial — Vehículos No Tripulados  
**Parcial:** PAO I - 2025  
**Duración:** 1 hora 30 minutos  
**Modalidad:** Individual  
**Evaluación:** Práctica en vivo

---

## 🎯 Objetivo

Desarrollar un nodo funcional en **ROS 2 Humble** que permita la **teleoperación del vehículo F1Tenth** mediante teclado, y que además implemente un **freno de emergencia** si el vehículo se acerca a menos de **50 cm de un obstáculo**.

---

## 🔧 Requisitos Técnicos

### 1. Nodo de Teleoperación

- Permitir el control del vehículo usando el teclado.
- Publicar comandos de velocidad en el tópico correspondiente.
- Suscribirse a la odometría del simulador y mostrar información de movimiento del vehículo.

### 2. Sistema de Freno de Emergencia

- Detectar obstáculos a menos de **0.5 m** usando datos del LiDAR.
- En caso de riesgo, detener inmediatamente el vehículo (comando de velocidad 0).
- La lógica puede integrarse en el mismo nodo de teleoperación o en uno adicional.

### 3. Simulador y Mapa

- Se utilizará un mapa específico del simulador F1Tenth.
- **Mapa proporcionado**: `___________` ← *(completar con el nombre del mapa)*
- INSERTA EL MAPA AQUI WINTER DE MIELDA MANITO

---

## 📤 Entregables

- Archivo comprimido `.zip` o `.tar.gz` con:
  - Código fuente del nodo o nodos implementados.
  - (Opcional) Scripts auxiliares si se dividió la lógica en varios nodos.

---

## 🎥 Evaluación en Vivo

Durante la evaluación, cada estudiante deberá:

- Compartir pantalla y ejecutar su solución.
- Mostrar que el robot responde a comandos de teclado.
- Demostrar que el freno de emergencia se activa al acercarse a un obstáculo.

---

## 🌟 Punto Extra Opcional (Follow the Gap)

- Los estudiantes pueden implementar un nodo alternativo basado en el algoritmo **Follow the Gap** en vez del de teleoperación.
- Este nodo debe identificar el espacio libre y conducir automáticamente hacia él.
- Debe incorporar el mismo criterio de frenado ante obstáculos a menos de 50 cm.
- **Puntaje adicional:** Hasta **+5 puntos** sobre 30.

---

## 📊 Rúbrica de Evaluación (30 puntos)

| Criterio                                           | Puntaje |
|----------------------------------------------------|---------|
| Nodo de teleoperación funcional                    | 8 pts   |
| Suscripción a odometría y despliegue de datos      | 4 pts   |
| Implementación del freno de emergencia funcional   | 10 pts  |
| Robustez del sistema y demostración en ejecución   | 5 pts   |
| Entregables completos y correctamente organizados  | 3 pts   |
| **Total**                                          | **30 pts** |

> **Extra:** +5 pts por implementar y demostrar el algoritmo Follow the Gap.

---

## 🛑 Reglas

- No se permite compartir código ni colaboración entre estudiantes.
- Cada solución deberá ser desarrollada de forma individual.
- El estudiante debe poder explicar su solución ante preguntas del docente.

¡Éxitos en su evaluación!
