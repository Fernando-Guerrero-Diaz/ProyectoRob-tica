#include <webots/robot.h>
#include <webots/motor.h>
#include <webots/distance_sensor.h>
#include <webots/lidar.h>
#include <webots/gps.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>

#define TIME_STEP 64
#define GRID_DIM 80
#define TILE_SIZE 0.5
#define MAX_ROUTE 100
#define MAX_QUEUE 1000
#define BASE_SPEED 10.0

typedef struct {
  int x, y;
} GridCoord;
 
typedef struct {
  int x, y;
  int g, h, f;
  int prev;
} PathNode;

GridCoord pos_to_grid(double wx, double wy) {
  GridCoord pos;
  pos.x = (int)((wx + (GRID_DIM * TILE_SIZE) / 2.0) / TILE_SIZE);
  pos.y = (int)((wy + (GRID_DIM * TILE_SIZE) / 2.0) / TILE_SIZE);
  return pos;
}

int manhattan_dist(int x1, int y1, int x2, int y2) {
  return abs(x1 - x2) + abs(y1 - y2);
}

int compute_path(int map[GRID_DIM][GRID_DIM], GridCoord start, GridCoord goal, GridCoord output[], int max_len) {
  PathNode open[MAX_QUEUE];
  int open_size = 0;

  PathNode closed[GRID_DIM * GRID_DIM];
  int closed_size = 0;

  PathNode root = {start.x, start.y, 0, manhattan_dist(start.x, start.y, goal.x, goal.y), 0, -1};
  root.f = root.g + root.h;
  open[open_size++] = root;

  while (open_size > 0) {
    int idx_min = 0;
    for (int i = 1; i < open_size; i++)
      if (open[i].f < open[idx_min].f) idx_min = i;

    PathNode current = open[idx_min];
    for (int i = idx_min; i < open_size - 1; i++)
      open[i] = open[i + 1];
    open_size--;

    closed[closed_size++] = current;

    if (current.x == goal.x && current.y == goal.y) {
      int len = 0;
      PathNode p = current;
      while (p.prev != -1 && len < max_len) {
        output[len++] = (GridCoord){p.x, p.y};
        p = closed[p.prev];
      }
      output[len++] = start;
      for (int i = 0; i < len / 2; i++) {
        GridCoord temp = output[i];
        output[i] = output[len - i - 1];
        output[len - i - 1] = temp;
      }
      return len;
    }

    int dx[] = {1, 0, -1, 0};
    int dy[] = {0, -1, 0, 1};

    for (int d = 0; d < 4; d++) {
      int nx = current.x + dx[d];
      int ny = current.y + dy[d];
      if (nx < 0 || ny < 0 || nx >= GRID_DIM || ny >= GRID_DIM || map[nx][ny] == 1)
        continue;

      int seen = 0;
      for (int i = 0; i < closed_size; i++)
        if (closed[i].x == nx && closed[i].y == ny) {
          seen = 1;
          break;
        }
      if (seen) continue;

      int g_cost = current.g + 1;
      int h_cost = manhattan_dist(nx, ny, goal.x, goal.y);
      int f_cost = g_cost + h_cost;

      int in_queue = -1;
      for (int i = 0; i < open_size; i++)
        if (open[i].x == nx && open[i].y == ny) {
          in_queue = i;
          break;
        }

      if (in_queue != -1) {
        if (f_cost < open[in_queue].f) {
          open[in_queue].g = g_cost;
          open[in_queue].h = h_cost;
          open[in_queue].f = f_cost;
          open[in_queue].prev = closed_size - 1;
        }
      } else if (open_size < MAX_QUEUE) {
        PathNode neighbor = {nx, ny, g_cost, h_cost, f_cost, closed_size - 1};
        open[open_size++] = neighbor;
      }
    }
  }

  return 0;
}

int main() {
  wb_robot_init();

  // Motores
  WbDeviceTag motores[4];
  char motor_names[4][8] = {"wheel1", "wheel2", "wheel3", "wheel4"};
  for (int i = 0; i < 4; i++) {
    motores[i] = wb_robot_get_device(motor_names[i]);
    wb_motor_set_position(motores[i], INFINITY);
  }

  // Sensores de proximidad
  WbDeviceTag proximidad[2];
  char prox_names[2][10] = {"ds_left", "ds_right"};
  for (int i = 0; i < 2; i++) {
    proximidad[i] = wb_robot_get_device(prox_names[i]);
    wb_distance_sensor_enable(proximidad[i], TIME_STEP);
  }

  // LIDAR
  WbDeviceTag scanner = wb_robot_get_device("lidar");
  wb_lidar_enable(scanner, TIME_STEP);
  wb_lidar_enable_point_cloud(scanner);

  // GPS
  WbDeviceTag nav = wb_robot_get_device("gps");
  wb_gps_enable(nav, TIME_STEP);

  // Grilla y trayectoria
  int mapa[GRID_DIM][GRID_DIM] = {0};
  GridCoord ruta[MAX_ROUTE];
  int longitud = 0;
  int objetivo_actual = 1;
  const double UMBRAL_WP = 0.3;

  while (wb_robot_step(TIME_STEP) != -1) {
    bool obs_cerca = false;
    bool lidar_peligro = false;

    double velocidad_izq = 5.0, velocidad_der = 5.0;

    // Sensores
    for (int i = 0; i < 2; i++) {
      double lectura = wb_distance_sensor_get_value(proximidad[i]);
      if (lectura < 100.0) obs_cerca = true;
    }

    // GPS
    const double *coord = wb_gps_get_values(nav);
    double x = coord[0];
    double y = coord[1];
    printf("Posición actual del robot: %.2f, %.2f\n", x, y);

    // LIDAR y obstáculos
    const float *barrido = wb_lidar_get_range_image(scanner);
    int res = wb_lidar_get_horizontal_resolution(scanner);
    double apertura = wb_lidar_get_fov(scanner);

    for (int i = 0; i < res; i++) {
      double ang = -apertura / 2 + i * (apertura / res);
      double distancia = barrido[i];
      if (isinf(distancia)) continue;

      if (distancia < 1.0) {
        double ox = x + distancia * cos(ang);
        double oy = y + distancia * sin(ang);

        int cx = (int)((ox + GRID_DIM * TILE_SIZE / 2) / TILE_SIZE);
        int cy = (int)((oy + GRID_DIM * TILE_SIZE / 2) / TILE_SIZE);

        if (cx >= 0 && cx < GRID_DIM && cy >= 0 && cy < GRID_DIM)
          mapa[cx][cy] = 1;
      } 

      if (distancia < 0.15) lidar_peligro = true;
    }

    // Planificación de trayectoria
    GridCoord origen = pos_to_grid(x, y);
    GridCoord destino = pos_to_grid(3.5, 2);
    printf("Desde (%d, %d) hasta (%d, %d)\n", origen.x, origen.y, destino.x, destino.y);

    if (origen.x == destino.x && origen.y == destino.y) {
      printf("Meta alcanzada\n");
      while (1) {
        wb_motor_set_velocity(motores[0], 0);
        wb_motor_set_velocity(motores[1], 0);
        wb_motor_set_velocity(motores[2], 0);
        wb_motor_set_velocity(motores[3], 0);
      }
    }

    longitud = compute_path(mapa, origen, destino, ruta, MAX_ROUTE);
    printf("Ruta calculada con %d pasos\n", longitud);

    // Movimiento
    if (lidar_peligro || obs_cerca) {
      printf("Obstáculo detectado, maniobra evasiva\n");
      velocidad_izq = 5.0;
      velocidad_der = -5.0;
    } else if (longitud > 1) {
      if (objetivo_actual >= longitud) {
        velocidad_izq = 0;
        velocidad_der = 0;
        printf("Objetivo final alcanzado\n");
      } else {
        GridCoord wp = ruta[objetivo_actual];
        double gx = (wp.x * TILE_SIZE) - (GRID_DIM * TILE_SIZE / 2.0);
        double gy = (wp.y * TILE_SIZE) - (GRID_DIM * TILE_SIZE / 2.0);

        double dx = gx - x;
        double dy = gy - y;
        double dist = sqrt(dx * dx + dy * dy);
        double ang_dest = atan2(dy, dx);

        if (dist < UMBRAL_WP) {
          objetivo_actual++;
          printf("Waypoint %d alcanzado\n", objetivo_actual);
        } else if (ang_dest > 0.1) {
          printf("Ajustando rumbo a la derecha\n");
          velocidad_izq = BASE_SPEED;
          velocidad_der = BASE_SPEED * 0.3;
        } else if (ang_dest < -0.1) {
          printf("Ajustando rumbo a la izquierda\n");
          velocidad_izq = BASE_SPEED * 0.3;
          velocidad_der = BASE_SPEED;
        } else {
          printf("Avanzando recto\n");
          velocidad_izq = BASE_SPEED;
          velocidad_der = BASE_SPEED;
        }
      }
    }

    // Aplicar velocidades
    for (int i = 0; i < 4; i++)
      wb_motor_set_velocity(motores[i], (i % 2 == 0) ? velocidad_izq : velocidad_der);

    fflush(stdout);
  }

  wb_robot_cleanup();
  return 0;
}
