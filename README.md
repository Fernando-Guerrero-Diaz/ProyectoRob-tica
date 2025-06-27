# Proyecto Final - Robot Autónomo en Webots

Este proyecto corresponde al trabajo final del curso **Robótica y Sistemas Autónomos (ICI 4150)**. Se implementó un robot móvil autónomo en el simulador Webots, capaz de navegar en entornos con y sin obstáculos utilizando sensores como LIDAR, GPS y sensores de proximidad, además de planificación con el algoritmo A*.

## Autores
- Martin Ignacio Vasquez Orellana  
- Fernando Agustín Guerrero Díaz  
- Nestor Raul Retamal Medina  

## Requisitos

- [Webots R2024a](https://cyberbotics.com/#download) o superior
- Sistema operativo: Windows, macOS o Linux
- Lenguaje de programación: C
- Git (opcional, para clonar el repositorio)

## Clonación del repositorio

```bash
git clone https://github.com/Fernando-Guerrero-Diaz/ProyectoRob-tica.git
cd ProyectoRob-tica
```
Estructura del proyecto

    /controllers/controlador/ → Código fuente del controlador en C

    /worlds/ → Archivos del entorno simulado (mapas con y sin obstáculos)

    README.md → Instrucciones de uso

    Otros archivos auxiliares y documentación
    
Cómo ejecutar el simulador

    Abrir Webots
    Lanza Webots y abre el proyecto:

        Ir a File > Open World

        Seleccionar el archivo mundo.wbt en la carpeta worlds/ 

    Compilar el controlador
    Asegúrate de que el archivo del controlador esté seleccionado correctamente en las propiedades del robot (controller = "controlador"). Luego:

        Webots compilará automáticamente si está bien configurado.

        Si no, ir a Build dentro de la pestaña de Controllers.

    Ejecutar la simulación
    Haz clic en el botón de play (▶️) para comenzar la simulación.
    El robot leerá sensores, planificará su ruta y se moverá hacia el objetivo evitando obstáculos.
    


