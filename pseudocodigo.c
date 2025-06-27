FUNC pos_to_grid(wx, wy):
    // Convierte coordenadas del mundo a índices de grilla
    x ← floor((wx + (GRID_DIM * TILE_SIZE) / 2) / TILE_SIZE)
    y ← floor((wy + (GRID_DIM * TILE_SIZE) / 2) / TILE_SIZE)
    DEVOLVER (x, y)

FUNC manhattan_dist(x1, y1, x2, y2):
    DEVOLVER abs(x1 – x2) + abs(y1 – y2)

FUNC compute_path(mapa, inicio, meta) → lista_de_puntos:
    open ← lista vacía de nodos A*
    closed ← lista vacía de nodos A*
    raiz ← { x=inicio.x, y=inicio.y, g=0,
              h=manhattan_dist(inicio.x, inicio.y, meta.x, meta.y),
              f=g+h, prev=–1 }
    INSERTAR raiz en open

    MIENTRAS open NO esté vacía:
        current ← nodo de open con f mínimo
        REMOVER current de open
        AGREGAR current a closed

        SI current en meta:
            ruta ← reconstruir_camino(closed, current)
            DEVOLVER ruta

        PARA cada dirección d en {arriba, derecha, abajo, izquierda}:
            nx, ny ← current.x + dx[d], current.y + dy[d]
            SI fuera de límites O mapa[nx][ny] es obstáculo: CONTINUAR
            SI (nx, ny) ya está en closed: CONTINUAR

            g_nuevo ← current.g + 1
            h_nuevo ← manhattan_dist(nx, ny, meta.x, meta.y)
            f_nuevo ← g_nuevo + h_nuevo

            SI (nx, ny) ya está en open COMO nodo_old:
                SI f_nuevo < nodo_old.f:
                    actualizar nodo_old con g=g_nuevo, h=h_nuevo, f=f_nuevo, prev=índice de current en closed
            SINO:
                neighbor ← { x=nx, y=ny, g=g_nuevo, h=h_nuevo, f=f_nuevo, prev=índice de current en closed }
                INSERTAR neighbor en open

    FIN MIENTRAS
    DEVOLVER lista vacía  // No hay ruta

FUNC reconstruir_camino(closed, nodo_final) → ruta:
    lista ← vacía
    n ← nodo_final
    MIENTRAS n.prev ≠ –1:
        AGREGAR (n.x, n.y) al inicio de lista
        n ← closed[n.prev]
    AGREGAR (n.x, n.y) al inicio de lista  // nodo inicio
    DEVOLVER lista

PROCESO PRINCIPAL:
    wb_robot_init()

    // Configurar motores
    PARA i EN 0..3:
        motores[i] ← wb_robot_get_device("wheel" + (i+1))
        wb_motor_set_position(motores[i], INFINITY)

    // Configurar sensores de proximidad
    PARA cada nombre EN {"ds_left", "ds_right"}:
        prox ← wb_robot_get_device(nombre)
        wb_distance_sensor_enable(prox, TIME_STEP)

    // Configurar LIDAR
    scanner ← wb_robot_get_device("lidar")
    wb_lidar_enable(scanner, TIME_STEP)
    wb_lidar_enable_point_cloud(scanner)

    // Configurar GPS
    gps ← wb_robot_get_device("gps")
    wb_gps_enable(gps, TIME_STEP)

    // Inicializar mapa y variables
    mapa ← matriz GRID_DIM×GRID_DIM con ceros
    ruta_actual ← vacía
    wp_index ← 1
    UMBRAL_WP ← 0.3

    MIENTRAS wb_robot_step(TIME_STEP) ≠ –1:
        obs_cerca ← FALSO
        peligro_lidar ← FALSO

        // Leer sensores de proximidad
        PARA cada sensor prox:
            lectura ← wb_distance_sensor_get_value(prox)
            SI lectura < UMBRAL_PROX:
                obs_cerca ← VERDADERO

        // Leer posición GPS
        (x, y) ← wb_gps_get_values(gps)
        imprimir("Posición:", x, y)

        // Actualizar mapa con LIDAR
        barrido ← wb_lidar_get_range_image(scanner)
        res ← wb_lidar_get_horizontal_resolution(scanner)
        fov ← wb_lidar_get_fov(scanner)
        PARA i EN 0..res–1:
            ang ← –fov/2 + i*(fov/res)
            dist ← barrido[i]
            SI es infinito(dist): CONTINUAR
            SI dist < DIST_OBSTÁCULO:
                ox ← x + dist * cos(ang)
                oy ← y + dist * sin(ang)
                (cx, cy) ← pos_to_grid(ox, oy)
                SI dentro de grilla(cx, cy):
                    mapa[cx][cy] ← 1
            SI dist < DIST_PELIGRO:
                peligro_lidar ← VERDADERO

        // Planificar ruta
        inicio ← pos_to_grid(x, y)
        meta ← pos_to_grid(X_META, Y_META)
        SI inicio == meta:
            detener_motores()
            ROMPER  // salir del bucle principal
        ruta_actual ← compute_path(mapa, inicio, meta)

        // Control de movimiento
        velocidad_izq ← BASE_SPEED
        velocidad_der ← BASE_SPEED
        SI peligro_lidar O obs_cerca:
            velocidad_izq ← BASE_SPEED
            velocidad_der ← –BASE_SPEED  // giro evasivo
        SINO SI tamaño(ruta_actual) > wp_index:
            wp ← ruta_actual[wp_index]
            (gx, gy) ← convertir_a_mundo(wp)
            dx ← gx – x;  dy ← gy – y
            dist_wp ← sqrt(dx² + dy²)
            ang_wp ← atan2(dy, dx)
            SI dist_wp < UMBRAL_WP:
                wp_index ← wp_index + 1
            SINO SI ang_wp > TOLERANCIA_ANG:
                velocidad_izq ← BASE_SPEED
                velocidad_der ← BASE_SPEED * 0.3
            SINO SI ang_wp < –TOLERANCIA_ANG:
                velocidad_izq ← BASE_SPEED * 0.3
                velocidad_der ← BASE_SPEED
            // sino: ya van recto

        // Enviar velocidades a los motores
        PARA i EN 0..3:
            si i par:
                wb_motor_set_velocity(motores[i], velocidad_izq)
            sino:
                wb_motor_set_velocity(motores[i], velocidad_der)

    FIN MIENTRAS

    wb_robot_cleanup()
