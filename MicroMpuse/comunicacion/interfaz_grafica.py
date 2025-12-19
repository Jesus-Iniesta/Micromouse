import tkinter as tk
from tkinter import ttk, messagebox, scrolledtext
import socket
import threading
import queue
import time
from collections import deque

# ====== CONFIGURACIÓN ======
DEFAULT_IP = "10.42.0.208"  # Pon aquí la IP que te dé el monitor serie de Arduino
ROBOT_PORT = 12345 

MAZE_COLS = 12
MAZE_ROWS = 7
CELL_SIZE = 60  # Aumentado de 50 a 60 para mejor visualización
# TARGET_X, TARGET_Y se usarán como referencia inicial, 
# pero la meta real se buscará dinámicamente (4 casillas abiertas).
TARGET_X, TARGET_Y = 5, 3  

# Direcciones
NORTH, EAST, SOUTH, WEST = 0, 1, 2, 3
DIRS = ['N', 'E', 'S', 'W']

class MazeCell:
    def __init__(self):
        self.walls = {'N': True, 'E': True, 'S': True, 'W': True}
        self.visited = False
        self.distance = 999

class FloodFillSolver:
    def __init__(self):
        self.maze = [[MazeCell() for _ in range(MAZE_ROWS)] for _ in range(MAZE_COLS)]
        # Inicializar paredes externas solamente
        for x in range(MAZE_COLS):
            for y in range(MAZE_ROWS):
                if x == 0: self.maze[x][y].walls['W'] = True
                if x == MAZE_COLS-1: self.maze[x][y].walls['E'] = True
                if y == 0: self.maze[x][y].walls['S'] = True
                if y == MAZE_ROWS-1: self.maze[x][y].walls['N'] = True
        self.update_distances()

    def update_distances(self):
        # Reiniciar distancias
        for col in self.maze:
            for cell in col:
                cell.distance = 999
        
        # BFS desde la meta (TARGET_X, TARGET_Y) hacia atrás.
        # NOTA: Cuando se encuentre la meta dinámica, esas celdas tendrán distancia 0
        # y el BFS se expandirá desde allí automáticamente.
        
        queue_bfs = deque()
        
        # Buscar si ya tenemos celdas marcadas como meta (distancia 0)
        found_dynamic_goal = False
        for x in range(MAZE_COLS):
            for y in range(MAZE_ROWS):
                if self.maze[x][y].distance == 0:
                    queue_bfs.append((x, y))
                    found_dynamic_goal = True
        
        # Si no hemos encontrado la meta dinámica aún, usamos la coordenada por defecto
        if not found_dynamic_goal:
            self.maze[TARGET_X][TARGET_Y].distance = 0
            queue_bfs.append((TARGET_X, TARGET_Y))
        
        while queue_bfs:
            x, y = queue_bfs.popleft()
            current_dist = self.maze[x][y].distance
            
            # Vecinos: (dx, dy, WallName, OppositeWall)
            neighbors = [
                (0, 1, 'N', 'S'), (1, 0, 'E', 'W'), 
                (0, -1, 'S', 'N'), (-1, 0, 'W', 'E')
            ]
            
            for dx, dy, wall, opp in neighbors:
                nx, ny = x + dx, y + dy
                if 0 <= nx < MAZE_COLS and 0 <= ny < MAZE_ROWS:
                    # Si NO hay pared entre ellos
                    if not self.maze[x][y].walls[wall]:
                        if self.maze[nx][ny].distance > current_dist + 1:
                            self.maze[nx][ny].distance = current_dist + 1
                            queue_bfs.append((nx, ny))

    def update_walls(self, rx, ry, facing, f_mm, l_mm, r_mm):
        # Mapeo relativo a absoluto
        # facing: 0=N, 1=E, 2=S, 3=W
        abs_dirs = ['N', 'E', 'S', 'W']
        
        # Umbral de pared (ajustado a tu hardware)
        WALL_THRESH = 150 
        
        has_front = f_mm < WALL_THRESH
        has_left = l_mm < WALL_THRESH
        has_right = r_mm < WALL_THRESH
        
        # Paredes absolutas
        w_front = abs_dirs[facing]
        w_left  = abs_dirs[(facing - 1) % 4]
        w_right = abs_dirs[(facing + 1) % 4]
        
        # Actualizar celda actual
        self.maze[rx][ry].walls[w_front] = has_front
        self.maze[rx][ry].walls[w_left] = has_left
        self.maze[rx][ry].walls[w_right] = has_right
        self.maze[rx][ry].visited = True
        
        # Actualizar vecinos (Paredes espejo)
        self.set_mirror_wall(rx, ry, w_front, has_front)
        self.set_mirror_wall(rx, ry, w_left, has_left)
        self.set_mirror_wall(rx, ry, w_right, has_right)
        
        self.update_distances()

    def set_mirror_wall(self, x, y, direction, has_wall):
        nx, ny = x, y
        opp = ''
        if direction == 'N': ny += 1; opp = 'S'
        elif direction == 'E': nx += 1; opp = 'W'
        elif direction == 'S': ny -= 1; opp = 'N'
        elif direction == 'W': nx -= 1; opp = 'E'
        
        if 0 <= nx < MAZE_COLS and 0 <= ny < MAZE_ROWS:
            self.maze[nx][ny].walls[opp] = has_wall
            
    def check_center_found(self):
        """
        Revisa si hemos encontrado la meta dinámica.
        Busca un bloque de 2x2 celdas sin paredes internas.
        """
        for x in range(MAZE_COLS - 1): # Corregido: self.cols -> MAZE_COLS
            for y in range(MAZE_ROWS - 1): # Corregido: self.rows -> MAZE_ROWS
                
                c00 = self.maze[x][y]       # Abajo-Izq
                c01 = self.maze[x][y+1]     # Arriba-Izq
                c10 = self.maze[x+1][y]     # Abajo-Der
                c11 = self.maze[x+1][y+1]   # Arriba-Der
                
                # Debemos haber visitado al menos una para confiar en los datos
                if not (c00.visited or c01.visited or c10.visited or c11.visited):
                    continue

                # Verificamos que estén abiertas ENTRE ELLAS (centro hueco)
                center_open = (
                    not c00.walls['N'] and 
                    not c00.walls['E'] and 
                    not c01.walls['E'] and 
                    not c10.walls['N']
                )
                
                if center_open:
                    print(f"¡META DETECTADA EN EL CUADRANTE ({x},{y})!")
                    # Establecer distancia 0 a estas 4 celdas para que el robot se quede ahí
                    c00.distance = 0
                    c01.distance = 0
                    c10.distance = 0
                    c11.distance = 0
                    return True # Meta encontrada
        return False

    def get_next_move(self, rx, ry, facing, exploration_mode=True):
        """
        Algoritmo de decisión mejorado que evalúa múltiples factores:
        1. Distancia flood-fill (prioridad base)
        2. Si la celda ha sido visitada (exploración)
        3. Costo de giro (preferir avanzar al frente)
        4. Número de celdas adyacentes sin explorar (potencial)
        5. Evitar callejones sin salida conocidos
        """
        
        # Movimientos posibles: (nuevo_facing, comando_robot, costo_giro)
        moves = [
            (facing, 'FORWARD', 0),                  # Sin giro
            ((facing - 1) % 4, 'TURNL', 1),          # Giro izquierda
            ((facing + 1) % 4, 'TURNR', 1),          # Giro derecha
            ((facing + 2) % 4, 'TURNU', 2)           # Media vuelta
        ]
        
        candidates = []  # Lista de opciones válidas con sus puntuaciones
        
        for new_face, cmd, turn_cost in moves:
            # Coordenadas a las que llegaríamos
            dx, dy = [(0,1), (1,0), (0,-1), (-1,0)][new_face]
            nx, ny = rx+dx, ry+dy
            
            # Verificar límites
            if 0 <= nx < MAZE_COLS and 0 <= ny < MAZE_ROWS:
                # Verificar pared
                wall_dir = ['N', 'E', 'S', 'W'][new_face]
                if not self.maze[rx][ry].walls[wall_dir]:
                    cell = self.maze[nx][ny]
                    
                    # ===== SISTEMA DE PUNTUACIÓN =====
                    score = 0.0
                    
                    # 1. Distancia flood-fill (factor más importante)
                    #    Menor distancia = mejor puntuación
                    score -= cell.distance * 100  # Peso: 100
                    
                    # 2. Preferencia por celdas no visitadas (solo en exploración)
                    if exploration_mode:
                        if not cell.visited:
                            score += 150  # Bonus grande por explorar nuevo territorio
                        
                        # Bonus adicional: contar vecinos sin explorar de la celda destino
                        unexplored_neighbors = self.count_unexplored_neighbors(nx, ny)
                        score += unexplored_neighbors * 30  # Bonus por potencial de exploración
                    
                    # 3. Preferir avanzar al frente (minimizar giros)
                    #    Menos giros = más eficiencia
                    score -= turn_cost * 20  # Penalización por girar
                    
                    # 4. Evitar callejones sin salida conocidos
                    #    Si una celda solo tiene 1 salida y ya fue visitada, penalizar
                    if cell.visited and exploration_mode:
                        exits = self.count_exits(nx, ny)
                        if exits <= 1:
                            score -= 50  # Penalización por callejón sin salida
                    
                    # 5. En modo retorno, priorizar ruta directa
                    if not exploration_mode:
                        # Penalizar menos los giros en modo retorno para ser más directo
                        score += turn_cost * 10  # Reducir penalización de giros
                    
                    # Guardar candidato con su puntuación
                    candidates.append({
                        'command': cmd,
                        'score': score,
                        'distance': cell.distance,
                        'visited': cell.visited,
                        'turn_cost': turn_cost,
                        'position': (nx, ny)
                    })
        
        # Si no hay candidatos, no hay movimiento posible
        if not candidates:
            return None
        
        # Ordenar candidatos por puntuación (mayor es mejor)
        candidates.sort(key=lambda x: x['score'], reverse=True)
        
        # Retornar el mejor movimiento
        best = candidates[0]
        
        # Debug: mostrar las opciones evaluadas (comentar en producción)
        # print(f"\n=== Decisión en ({rx},{ry}) orientación {DIRS[facing]} ===")
        # for i, c in enumerate(candidates[:3]):  # Mostrar top 3
        #     print(f"{i+1}. {c['command']}: score={c['score']:.1f}, dist={c['distance']}, "
        #           f"visited={c['visited']}, turns={c['turn_cost']}")
        
        return best['command']
    
    def count_unexplored_neighbors(self, x, y):
        """Cuenta cuántos vecinos adyacentes no han sido explorados"""
        count = 0
        neighbors = [
            (0, 1, 'N'), (1, 0, 'E'), (0, -1, 'S'), (-1, 0, 'W')
        ]
        for dx, dy, wall_dir in neighbors:
            nx, ny = x + dx, y + dy
            if 0 <= nx < MAZE_COLS and 0 <= ny < MAZE_ROWS:
                # Solo contar si es accesible (sin pared) y no visitado
                if not self.maze[x][y].walls[wall_dir] and not self.maze[nx][ny].visited:
                    count += 1
        return count
    
    def count_exits(self, x, y):
        """Cuenta cuántas salidas tiene una celda"""
        count = 0
        for wall_dir in ['N', 'E', 'S', 'W']:
            if not self.maze[x][y].walls[wall_dir]:
                count += 1
        return count
    
    def calculate_optimal_path(self, start_x, start_y, start_facing):
        """
        Calcula la ruta óptima desde (start_x, start_y) hasta la meta.
        Retorna una lista de comandos que el robot debe ejecutar.
        """
        path = []
        x, y = start_x, start_y
        facing = start_facing
        
        # Seguir el gradiente de distancia hasta llegar a la meta (distancia 0)
        visited_positions = set()
        max_steps = MAZE_COLS * MAZE_ROWS * 2  # Límite de seguridad
        steps = 0
        
        while self.maze[x][y].distance > 0 and steps < max_steps:
            if (x, y) in visited_positions:
                # Evitar loops infinitos
                break
            visited_positions.add((x, y))
            steps += 1
            
            # Buscar el mejor vecino (menor distancia)
            best_neighbor = None
            best_dist = 999
            best_direction = None
            
            neighbors = [
                (0, 1, 'N', 0),   # Norte
                (1, 0, 'E', 1),   # Este
                (0, -1, 'S', 2),  # Sur
                (-1, 0, 'W', 3)   # Oeste
            ]
            
            for dx, dy, wall_name, direction in neighbors:
                nx, ny = x + dx, y + dy
                if 0 <= nx < MAZE_COLS and 0 <= ny < MAZE_ROWS:
                    # Verificar que no haya pared
                    if not self.maze[x][y].walls[wall_name]:
                        if self.maze[nx][ny].distance < best_dist:
                            best_dist = self.maze[nx][ny].distance
                            best_neighbor = (nx, ny)
                            best_direction = direction
            
            if best_neighbor is None:
                # No hay camino disponible
                break
            
            # Calcular los comandos necesarios para moverse a esa celda
            target_direction = best_direction
            
            # Primero girar a la dirección correcta
            turn_diff = (target_direction - facing) % 4
            if turn_diff == 1:
                path.append('TURNR')
                facing = target_direction
            elif turn_diff == 2:
                path.append('TURNU')
                facing = target_direction
            elif turn_diff == 3:
                path.append('TURNL')
                facing = target_direction
            # Si turn_diff == 0, ya estamos orientados correctamente
            
            # Luego avanzar
            path.append('FORWARD')
            x, y = best_neighbor
        
        return path
    
    def update_distances_from_start(self):
        """
        Actualiza las distancias desde la posición inicial (0,0) hacia la meta.
        Útil para calcular la ruta de regreso desde la meta.
        """
        # Reiniciar distancias
        for col in self.maze:
            for cell in col:
                cell.distance = 999
        
        # BFS desde (0,0)
        queue_bfs = deque()
        self.maze[0][0].distance = 0
        queue_bfs.append((0, 0))
        
        while queue_bfs:
            x, y = queue_bfs.popleft()
            current_dist = self.maze[x][y].distance
            
            neighbors = [
                (0, 1, 'N', 'S'), (1, 0, 'E', 'W'), 
                (0, -1, 'S', 'N'), (-1, 0, 'W', 'E')
            ]
            
            for dx, dy, wall, opp in neighbors:
                nx, ny = x + dx, y + dy
                if 0 <= nx < MAZE_COLS and 0 <= ny < MAZE_ROWS:
                    if not self.maze[x][y].walls[wall]:
                        if self.maze[nx][ny].distance > current_dist + 1:
                            self.maze[nx][ny].distance = current_dist + 1
                            queue_bfs.append((nx, ny))

class MicromouseApp:
    def __init__(self, root):
        self.root = root
        self.root.title("Control Micromouse - Flood Fill")
        self.root.configure(bg='#f0f0f1')
        
        # Configurar tamaño de ventana para mejor visualización (aumentado para caber todo)
        min_width = 1500
        min_height = 900  # Aumentado de 700 a 900
        self.root.minsize(min_width, min_height)
        
        # Centrar la ventana en la pantalla
        screen_width = self.root.winfo_screenwidth()
        screen_height = self.root.winfo_screenheight()
        x = (screen_width - min_width) // 2
        y = (screen_height - min_height) // 2
        self.root.geometry(f"{min_width}x{min_height}+{x}+{y}")
        
        # Estado del Robot
        self.rx = 0
        self.ry = 0
        self.facing = NORTH # 0=N
        self.solver = FloodFillSolver()
        
        # Estado de ejecución
        self.exploration_phase = True  # True = exploración, False = ejecución óptima
        self.goal_found = False
        self.optimal_path = []  # Almacena la ruta óptima calculada
        
        # Sensores (valores actuales)
        self.sensor_front = 0
        self.sensor_left = 0
        self.sensor_right = 0
        self.last_command = "Ninguno"
        
        # Networking
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.settimeout(0.5)
        self.robot_ip = DEFAULT_IP
        self.connected = False
        
        # GUI Layout
        self.create_widgets()
        self.running = False
        self.calibrating = False
        
    def create_widgets(self):
        # ===== Panel Principal Derecho con Scroll =====
        # Contenedor principal
        frame_ctrl_container = tk.Frame(self.root, bg='#2c3e50')
        frame_ctrl_container.pack(side=tk.RIGHT, fill=tk.BOTH, expand=False)
        
        # Canvas para permitir scroll
        canvas_ctrl = tk.Canvas(frame_ctrl_container, bg='#2c3e50', width=500, 
                               highlightthickness=0)
        scrollbar = tk.Scrollbar(frame_ctrl_container, orient="vertical", command=canvas_ctrl.yview)
        
        # Frame scrollable dentro del canvas
        frame_ctrl = tk.Frame(canvas_ctrl, bg='#2c3e50', padx=20, pady=15)
        
        # Configurar el scroll
        frame_ctrl.bind(
            "<Configure>",
            lambda e: canvas_ctrl.configure(scrollregion=canvas_ctrl.bbox("all"))
        )
        
        canvas_ctrl.create_window((0, 0), window=frame_ctrl, anchor="nw")
        canvas_ctrl.configure(yscrollcommand=scrollbar.set)
        
        # Pack el canvas y scrollbar
        canvas_ctrl.pack(side="left", fill="both", expand=True)
        scrollbar.pack(side="right", fill="y")
        
        # Habilitar scroll con rueda del mouse
        def _on_mousewheel(event):
            canvas_ctrl.yview_scroll(int(-1*(event.delta/120)), "units")
        canvas_ctrl.bind_all("<MouseWheel>", _on_mousewheel)
        
        # Título
        title_label = tk.Label(frame_ctrl, text="🤖 CONTROL MICROMOUSE", 
                              font=('Arial', 16, 'bold'), bg='#2c3e50', fg='#ecf0f1')
        title_label.pack(pady=(0,10))
        
        # ===== Conexión =====
        conn_frame = tk.LabelFrame(frame_ctrl, text="Conexión", bg='#34495e', 
                                   fg='#ecf0f1', font=('Arial', 10, 'bold'), padx=10, pady=10)
        conn_frame.pack(fill=tk.X, pady=(0,10))
        
        tk.Label(conn_frame, text="Dirección IP:", bg='#34495e', fg='#ecf0f1').pack()
        self.entry_ip = tk.Entry(conn_frame, font=('Arial', 10), justify='center')
        self.entry_ip.insert(0, DEFAULT_IP)
        self.entry_ip.pack(pady=5)
        
        self.btn_connect = tk.Button(conn_frame, text="🔌 Conectar", command=self.do_ping,
                                     bg='#3498db', fg='white', font=('Arial', 10, 'bold'),
                                     activebackground='#2980b9', cursor='hand2')
        self.btn_connect.pack(pady=5)
        
        self.status_label = tk.Label(conn_frame, text="● Desconectado", 
                                     bg='#34495e', fg='#e74c3c', font=('Arial', 9))
        self.status_label.pack()
        
        # ===== Diagnóstico Sensores =====
        sensor_frame = tk.LabelFrame(frame_ctrl, text="📊 Sensores (mm)", bg='#34495e',
                                     fg='#ecf0f1', font=('Arial', 10, 'bold'), padx=8, pady=6)
        sensor_frame.pack(fill=tk.X, pady=(0,6))
        
        # Sensor Frontal
        front_frame = tk.Frame(sensor_frame, bg='#34495e')
        front_frame.pack(fill=tk.X, pady=1)
        tk.Label(front_frame, text="⬆️ Frontal:", bg='#34495e', fg='#ecf0f1', 
                width=11, anchor='w', font=('Arial', 9)).pack(side=tk.LEFT)
        self.sensor_front_label = tk.Label(front_frame, text="---", bg='#2c3e50', 
                                          fg='#2ecc71', font=('Courier', 12, 'bold'), width=10)
        self.sensor_front_label.pack(side=tk.LEFT, padx=3)
        
        # Sensor Izquierdo
        left_frame = tk.Frame(sensor_frame, bg='#34495e')
        left_frame.pack(fill=tk.X, pady=1)
        tk.Label(left_frame, text="⬅️ Izquierdo:", bg='#34495e', fg='#ecf0f1',
                width=11, anchor='w', font=('Arial', 9)).pack(side=tk.LEFT)
        self.sensor_left_label = tk.Label(left_frame, text="---", bg='#2c3e50',
                                         fg='#2ecc71', font=('Courier', 12, 'bold'), width=10)
        self.sensor_left_label.pack(side=tk.LEFT, padx=3)
        
        # Sensor Derecho
        right_frame = tk.Frame(sensor_frame, bg='#34495e')
        right_frame.pack(fill=tk.X, pady=1)
        tk.Label(right_frame, text="➡️ Derecho:", bg='#34495e', fg='#ecf0f1',
                width=11, anchor='w', font=('Arial', 9)).pack(side=tk.LEFT)
        self.sensor_right_label = tk.Label(right_frame, text="---", bg='#2c3e50',
                                          fg='#2ecc71', font=('Courier', 13, 'bold'), width=10)
        self.sensor_right_label.pack(side=tk.LEFT, padx=5)
        
        # Estado Actual
        status_frame = tk.Frame(sensor_frame, bg='#34495e')
        status_frame.pack(fill=tk.X, pady=5)
        tk.Label(status_frame, text="🎯 Comando:", bg='#34495e', fg='#ecf0f1',
                width=12, anchor='w', font=('Arial', 10)).pack(side=tk.LEFT)
        self.command_label = tk.Label(status_frame, text="Ninguno", bg='#2c3e50',
                                     fg='#f39c12', font=('Arial', 9, 'bold'), width=14)
        self.command_label.pack(side=tk.LEFT, padx=3)
        
        # Botón de calibración
        self.btn_calibrate = tk.Button(sensor_frame, text="🔧 Calibrar Sensores",
                                       command=self.toggle_calibration, bg='#9b59b6', fg='white',
                                       font=('Arial', 9, 'bold'), activebackground='#8e44ad',
                                       cursor='hand2', pady=5)
        self.btn_calibrate.pack(fill=tk.X, pady=(5,0))
        
        # ===== Control =====
        control_frame = tk.LabelFrame(frame_ctrl, text="Control", bg='#34495e',
                                      fg='#ecf0f1', font=('Arial', 10, 'bold'), padx=8, pady=6)
        control_frame.pack(fill=tk.X, pady=(0,6))
        
        self.btn_start = tk.Button(control_frame, text="▶️ INICIAR AUTOMÁTICO", 
                                   command=self.start_auto, bg='#27ae60', fg='white',
                                   font=('Arial', 11, 'bold'), activebackground='#229954',
                                   cursor='hand2', pady=8)
        self.btn_start.pack(fill=tk.X, pady=3, ipady=2)
        
        self.btn_stop = tk.Button(control_frame, text="⛔ PARADA DE EMERGENCIA",
                                 command=self.emergency_stop, bg='#e74c3c', fg='white',
                                 font=('Arial', 11, 'bold'), activebackground='#c0392b',
                                 cursor='hand2', pady=8)
        self.btn_stop.pack(fill=tk.X, pady=3, ipady=2)
        
        self.btn_reset = tk.Button(control_frame, text="🔄 REINICIAR RECORRIDO",
                                   command=self.reset_exploration, bg='#f39c12', fg='white',
                                   font=('Arial', 10, 'bold'), activebackground='#e67e22',
                                   cursor='hand2', pady=8)
        self.btn_reset.pack(fill=tk.X, pady=6, ipady=2)
        
        # ===== Control Manual / Calibración Motores =====
        motor_frame = tk.LabelFrame(frame_ctrl, text="🎮 Control Manual", bg='#34495e',
                                    fg='#ecf0f1', font=('Arial', 10, 'bold'), padx=8, pady=6)
        motor_frame.pack(fill=tk.X, pady=(0,6))
        
        # Fila 1: Avance completo de celda
        tk.Button(motor_frame, text="⬆️ Avanzar", command=lambda: self.manual_move("CELL_FORWARD"),
                 bg='#16a085', fg='white', font=('Arial', 9, 'bold'),
                 activebackground='#138d75', cursor='hand2', pady=4).pack(fill=tk.X, pady=1)
        
        # Fila 2: Giros de 90 grados
        turn_frame = tk.Frame(motor_frame, bg='#34495e')
        turn_frame.pack(fill=tk.X, pady=1)
        tk.Button(turn_frame, text="↪️ 90° Izq", command=lambda: self.manual_move("TURN_LEFT"),
                 bg='#2980b9', fg='white', font=('Arial', 9, 'bold'),
                 activebackground='#21618c', cursor='hand2', pady=4).pack(side=tk.LEFT, fill=tk.X, expand=True, padx=(0,1))
        tk.Button(turn_frame, text="↩️ 90° Der", command=lambda: self.manual_move("TURN_RIGHT"),
                 bg='#2980b9', fg='white', font=('Arial', 9, 'bold'),
                 activebackground='#21618c', cursor='hand2', pady=4).pack(side=tk.LEFT, fill=tk.X, expand=True, padx=(1,0))
        
        # Fila 3: Giro 180 grados y retroceso
        turn_back_frame = tk.Frame(motor_frame, bg='#34495e')
        turn_back_frame.pack(fill=tk.X, pady=1)
        tk.Button(turn_back_frame, text="🔄 180°", command=lambda: self.manual_move("TURN_180"),
                 bg='#8e44ad', fg='white', font=('Arial', 9, 'bold'),
                 activebackground='#7d3c98', cursor='hand2', pady=4).pack(side=tk.LEFT, fill=tk.X, expand=True, padx=(0,1))
        tk.Button(turn_back_frame, text="⬇️ Retro", command=lambda: self.manual_move("CELL_BACKWARD"),
                 bg='#d35400', fg='white', font=('Arial', 9, 'bold'),
                 activebackground='#ba4a00', cursor='hand2', pady=4).pack(side=tk.LEFT, fill=tk.X, expand=True, padx=(1,0))
        
        # Indicador de estado de control manual
        self.manual_status_label = tk.Label(motor_frame, text="⚪ Listo",
                                           bg='#34495e', fg='#95a5a6', font=('Arial', 8, 'italic'))
        self.manual_status_label.pack(pady=(3,0))
        
        # ===== Diagnóstico de Encoders/Motores =====
        diag_frame = tk.LabelFrame(frame_ctrl, text="🔍 Diagnóstico", bg='#34495e',
                                   fg='#ecf0f1', font=('Arial', 10, 'bold'), padx=8, pady=6)
        diag_frame.pack(fill=tk.X, pady=(0,6))
        
        # Botón de test de motores
        tk.Button(diag_frame, text="🧪 Test Motores (5 seg)", command=self.test_motors_encoders,
                 bg='#c0392b', fg='white', font=('Arial', 9, 'bold'),
                 activebackground='#a93226', cursor='hand2', pady=4).pack(fill=tk.X, pady=2)
        
        # Display de encoders en tiempo real
        enc_display = tk.Frame(diag_frame, bg='#2c3e50', relief=tk.SUNKEN, bd=1)
        enc_display.pack(fill=tk.X, pady=3, padx=2)
        
        enc_left_frame = tk.Frame(enc_display, bg='#2c3e50')
        enc_left_frame.pack(side=tk.LEFT, fill=tk.X, expand=True, padx=3, pady=3)
        tk.Label(enc_left_frame, text="Enc Izq:", bg='#2c3e50', fg='#ecf0f1', 
                font=('Arial', 8)).pack()
        self.enc_left_label = tk.Label(enc_left_frame, text="0", bg='#2c3e50',
                                       fg='#3498db', font=('Courier', 11, 'bold'))
        self.enc_left_label.pack()
        
        enc_right_frame = tk.Frame(enc_display, bg='#2c3e50')
        enc_right_frame.pack(side=tk.LEFT, fill=tk.X, expand=True, padx=3, pady=3)
        tk.Label(enc_right_frame, text="Enc Der:", bg='#2c3e50', fg='#ecf0f1',
                font=('Arial', 8)).pack()
        self.enc_right_label = tk.Label(enc_right_frame, text="0", bg='#2c3e50',
                                        fg='#3498db', font=('Courier', 11, 'bold'))
        self.enc_right_label.pack()
        
        # Notas de diagnóstico
        tk.Label(diag_frame, text="💡 Si una rueda va lenta/rápida:", 
                bg='#34495e', fg='#f39c12', font=('Arial', 7, 'bold')).pack(pady=(3,0))
        tk.Label(diag_frame, text="Revisa polaridad de motores en ESP32", 
                bg='#34495e', fg='#95a5a6', font=('Arial', 7)).pack(pady=(0,2))
        
        # ===== Log de Diagnóstico =====
        log_frame = tk.LabelFrame(frame_ctrl, text="📋 Logs", bg='#34495e',
                                 fg='#ecf0f1', font=('Arial', 9, 'bold'), padx=4, pady=4)
        log_frame.pack(fill=tk.BOTH, expand=True)
        
        self.log_area = scrolledtext.ScrolledText(log_frame, width=48, height=12,
                                                 bg='#1c1c1c', fg='#00ff00',
                                                 font=('Courier', 8), insertbackground='white',
                                                 wrap=tk.WORD)
        self.log_area.pack(fill=tk.BOTH, expand=True)
        
        # ===== Canvas Laberinto =====
        canvas_frame = tk.Frame(self.root, bg='#ecf0f1', padx=10, pady=10)
        canvas_frame.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)
        
        canvas_title = tk.Label(canvas_frame, text="🗺️ MAPA DEL LABERINTO",
                               font=('Arial', 14, 'bold'), bg='#ecf0f1', fg='#2c3e50')
        canvas_title.pack(pady=(0,8))
        
        self.canvas = tk.Canvas(canvas_frame, width=MAZE_COLS*CELL_SIZE, 
                               height=MAZE_ROWS*CELL_SIZE, bg="white", 
                               highlightthickness=3, highlightbackground='#34495e')
        self.canvas.pack(pady=5)
        
        # Leyenda
        legend_frame = tk.Frame(canvas_frame, bg='#ecf0f1')
        legend_frame.pack(pady=10)
        
        legends = [
            ("#ffcccc", "Robot"),
            ("#e6f3ff", "Visitado"),
            ("#ccffcc", "Meta"),
            ("white", "Sin explorar")
        ]
        
        for color, label in legends:
            leg = tk.Frame(legend_frame, bg='#ecf0f1')
            leg.pack(side=tk.LEFT, padx=8)
            tk.Canvas(leg, width=24, height=24, bg=color, highlightthickness=1,
                     highlightbackground='#2c3e50').pack(side=tk.LEFT, padx=3)
            tk.Label(leg, text=label, bg='#ecf0f1', font=('Arial', 10)).pack(side=tk.LEFT)
        
        self.draw_maze()
        
        # Log inicial con información
        self.log("===================", "INFO")
        self.log("🤖 INTERFAZ MICROMOUSE INICIADA", "SUCCESS")
        self.log("===================", "INFO")
        self.log("💡 Comandos soportados:", "INFO")
        self.log("  • FORWARD - Avanzar 1 celda", "INFO")
        self.log("  • BACKWARD - Retroceder 1 celda", "INFO")
        self.log("  • TURNL - Girar 90° izquierda", "INFO")
        self.log("  • TURNR - Girar 90° derecha", "INFO")
        self.log("  • TURNU - Girar 180° (media vuelta)", "INFO")
        self.log("  • SENSORS - Leer sensores", "INFO")
        self.log("  • STATUS - Consultar estado", "INFO")
        self.log("  • STOP - Detener robot", "INFO")
        self.log("===================", "INFO")

    def log(self, msg, level="INFO"):
        timestamp = time.strftime("%H:%M:%S")
        levels = {
            "INFO": "ℹ️",
            "SUCCESS": "✅",
            "WARNING": "⚠️",
            "ERROR": "❌",
            "SENSOR": "📡",
            "COMMAND": "🎮"
        }
        icon = levels.get(level, "•")
        formatted = f"[{timestamp}] {icon} {msg}\n"
        self.log_area.insert(tk.END, formatted)
        self.log_area.see(tk.END)
        
    def update_sensor_display(self, front, left, right):
        """Actualiza los valores de los sensores en la interfaz"""
        self.sensor_front = front
        self.sensor_left = left
        self.sensor_right = right
        
        # Actualizar etiquetas con colores según distancia
        def get_color(value):
            if value < 100: return '#e74c3c'  # Rojo - muy cerca
            elif value < 150: return '#f39c12'  # Naranja - cerca
            else: return '#2ecc71'  # Verde - lejos
        
        self.sensor_front_label.config(text=f"{front:3d}", fg=get_color(front))
        self.sensor_left_label.config(text=f"{left:3d}", fg=get_color(left))
        self.sensor_right_label.config(text=f"{right:3d}", fg=get_color(right))
        
    def update_command_display(self, command):
        """Actualiza el comando actual en español"""
        commands_spanish = {
            "FORWARD": "⬆️ Avanza",
            "TURNL": "↪️ Gira Izquierda",
            "TURNR": "↩️ Gira Derecha",
            "TURNU": "🔄 Media Vuelta",
            "STOP": "⏹️ Detenido",
            "Ninguno": "⏸️ En Espera"
        }
        self.last_command = commands_spanish.get(command, command)
        self.command_label.config(text=self.last_command)

    def do_ping(self):
        self.robot_ip = self.entry_ip.get()
        self.log(f"Intentando conectar a {self.robot_ip}...", "INFO")
        resp = self.send_cmd("STATUS")
        if resp:
            self.log(f"Conectado exitosamente: {resp}", "SUCCESS")
            self.connected = True
            self.status_label.config(text="● Conectado", fg='#2ecc71')
            self.btn_connect.config(bg='#27ae60', text="✓ Conectado")
        else:
            self.log("Sin respuesta del robot", "ERROR")
            self.status_label.config(text="● Error de Conexión", fg='#e74c3c')
            self.connected = False

    def send_cmd(self, cmd):
        try:
            self.sock.sendto(cmd.encode(), (self.robot_ip, ROBOT_PORT))
            data, _ = self.sock.recvfrom(1024)
            return data.decode().strip()
        except socket.timeout:
            return None
        except Exception as e:
            self.log(f"Error Red: {e}")
            return None

    def draw_maze(self):
        self.canvas.delete("all")
        h = MAZE_ROWS * CELL_SIZE
        
        for x in range(MAZE_COLS):
            for y in range(MAZE_ROWS):
                # Coordenadas (0,0 es abajo-izquierda)
                x0 = x * CELL_SIZE
                y0 = h - (y * CELL_SIZE) - CELL_SIZE
                x1 = x0 + CELL_SIZE
                y1 = y0 + CELL_SIZE
                
                cell = self.solver.maze[x][y]
                
                # Color fondo
                color = "white"
                if cell.visited: color = "#e6f3ff"
                
                # Pintar la meta (distancia 0)
                if cell.distance == 0: color = "#ccffcc"
                
                # Pintar robot
                if x==self.rx and y==self.ry: color = "#ffcccc" 
                
                self.canvas.create_rectangle(x0, y0, x1, y1, fill=color, outline="")
                
                # Distancia con mejor formato
                dist_text = str(cell.distance) if cell.distance < 999 else "∞"
                self.canvas.create_text((x0+x1)/2, (y0+y1)/2, text=dist_text, 
                                       fill="#7f8c8d", font=('Arial', 12, 'bold'))
                
                # Paredes más visibles
                w = 5
                wall_color = "#2c3e50"
                if cell.walls['N']: self.canvas.create_line(x0, y0, x1, y0, width=w, fill=wall_color)
                if cell.walls['S']: self.canvas.create_line(x0, y1, x1, y1, width=w, fill=wall_color)
                if cell.walls['E']: self.canvas.create_line(x1, y0, x1, y1, width=w, fill=wall_color)
                if cell.walls['W']: self.canvas.create_line(x0, y0, x0, y1, width=w, fill=wall_color)
        
        # Dibujar robot con flecha direccional
        cx = self.rx * CELL_SIZE + CELL_SIZE/2
        cy = h - (self.ry * CELL_SIZE) - CELL_SIZE/2
        
        # Círculo del robot más grande
        robot_radius = 14
        self.canvas.create_oval(cx-robot_radius, cy-robot_radius, cx+robot_radius, cy+robot_radius, 
                               fill="#e74c3c", outline="#c0392b", width=3)
        
        # Flecha indicando dirección más grande
        arrow_len = 18
        directions = [(0, -arrow_len), (arrow_len, 0), (0, arrow_len), (-arrow_len, 0)]
        dx, dy = directions[self.facing]
        self.canvas.create_line(cx, cy, cx+dx, cy+dy, arrow=tk.LAST, 
                               fill="white", width=4, arrowshape=(10,12,5))

    def start_auto(self):
        if not self.connected:
            messagebox.showwarning("Advertencia", "Primero debes conectar con el robot")
            return
        if not self.running:
            self.running = True
            self.btn_start.config(state=tk.DISABLED, bg='#95a5a6')
            self.log("===================", "INFO")
            self.log("Iniciando modo automático...", "SUCCESS")
            self.log("===================", "INFO")
            threading.Thread(target=self.logic_loop, daemon=True).start()

    def emergency_stop(self):
        self.running = False
        self.calibrating = False
        self.send_cmd("STOP")
        self.update_command_display("STOP")
        self.btn_start.config(state=tk.NORMAL, bg='#27ae60')
        self.btn_calibrate.config(text="🔧 Calibrar/Monitorear Sensores", bg='#9b59b6')
        self.log("===================", "WARNING")
        self.log("PARADA DE EMERGENCIA ACTIVADA", "WARNING")
        self.log("===================", "WARNING")
    
    def reset_exploration(self):
        """Reinicia el recorrido: robot vuelve a posición (0,0) y limpia el mapa"""
        if self.running or self.calibrating:
            messagebox.showwarning("Advertencia", "Detén el proceso actual antes de reiniciar")
            return
        
        # Confirmar reinicio
        confirm = messagebox.askyesno("Confirmar Reinicio", 
                                      "¿Deseas reiniciar el recorrido?\n\n" +
                                      "• El robot volverá a la posición (0,0)\n" +
                                      "• El mapa se limpiará\n" +
                                      "• Los datos de exploración se perderán")
        if not confirm:
            return
        
        self.log("===================", "INFO")
        self.log("🔄 REINICIANDO RECORRIDO", "SUCCESS")
        
        # Reiniciar posición del robot
        old_pos = (self.rx, self.ry, DIRS[self.facing])
        self.rx = 0
        self.ry = 0
        self.facing = NORTH
        
        # Recrear el solver (limpia el mapa)
        self.solver = FloodFillSolver()
        
        # Reiniciar estado de ejecución
        self.exploration_phase = True
        self.goal_found = False
        self.optimal_path = []
        
        # Limpiar display de sensores
        self.sensor_front_label.config(text="---", fg='#2ecc71')
        self.sensor_left_label.config(text="---", fg='#2ecc71')
        self.sensor_right_label.config(text="---", fg='#2ecc71')
        self.update_command_display("Ninguno")
        
        # Actualizar canvas
        self.draw_maze()
        
        self.log(f"Posición anterior: {old_pos}", "INFO")
        self.log(f"Nueva posición: (0, 0, N)", "INFO")
        self.log("Mapa limpiado exitosamente", "SUCCESS")
        self.log("===================", "INFO")
        
        # Enviar comando de stop al robot por seguridad
        self.send_cmd("STOP")
    
    def manual_move(self, command):
        """Ejecuta un movimiento manual del robot para calibración de motores"""
        if self.running or self.calibrating:
            messagebox.showwarning("Advertencia", "Detén los procesos automáticos antes de usar control manual")
            return
        
        if not self.connected:
            messagebox.showwarning("Advertencia", "Conecta el robot antes de usar control manual")
            return
        
        # Diccionario de comandos en español
        commands_spanish = {
            "CELL_FORWARD": "Avanzar 1 celda completa",
            "TURN_LEFT": "Girar 90° a la izquierda",
            "TURN_RIGHT": "Girar 90° a la derecha",
            "TURN_180": "Girar 180° (media vuelta)",
            "CELL_BACKWARD": "Retroceder 1 celda completa"
        }
        
        # Mapeo de comandos a comandos del protocolo
        # Asumiendo que el robot acepta estos comandos
        protocol_commands = {
            "CELL_FORWARD": "FORWARD",
            "TURN_LEFT": "TURNL",
            "TURN_RIGHT": "TURNR",
            "TURN_180": "TURNU",
            "CELL_BACKWARD": "BACKWARD"  # Nuevo comando que el robot debe implementar
        }
        
        cmd_name = commands_spanish.get(command, command)
        protocol_cmd = protocol_commands.get(command, command)
        
        self.log(f"🎮 Control Manual: {cmd_name}", "COMMAND")
        self.manual_status_label.config(text=f"⚙️ Ejecutando: {cmd_name}", fg='#f39c12')
        
        # Enviar comando
        ack = self.send_cmd(protocol_cmd)
        
        if ack != "ACK":
            self.log(f"❌ Error: Comando no reconocido o rechazado ({ack})", "ERROR")
            self.manual_status_label.config(text=f"❌ Error en comando", fg='#e74c3c')
            time.sleep(2)
            self.manual_status_label.config(text="⚪ Listo para comando manual", fg='#95a5a6')
            return
        
        self.log("✓ Comando enviado - Esperando ejecución...", "INFO")
        
        # Esperar a que el robot complete el movimiento
        timeout = 10  # 10 segundos de timeout
        start_time = time.time()
        
        while True:
            if time.time() - start_time > timeout:
                self.log("⚠️ Timeout: El robot no respondió en tiempo", "WARNING")
                self.manual_status_label.config(text="⚠️ Timeout", fg='#e67e22')
                time.sleep(2)
                self.manual_status_label.config(text="⚪ Listo para comando manual", fg='#95a5a6')
                return
            
            time.sleep(0.2)
            status = self.send_cmd("STATUS")
            
            if status and status.startswith("IDLE"):
                self.log(f"✅ Movimiento completado: {cmd_name}", "SUCCESS")
                self.manual_status_label.config(text="✅ Comando completado", fg='#2ecc71')
                
                # Actualizar posición virtual si es un movimiento de avance
                if command == "CELL_FORWARD":
                    dx, dy = [(0,1), (1,0), (0,-1), (-1,0)][self.facing]
                    old_pos = (self.rx, self.ry)
                    self.rx += dx
                    self.ry += dy
                    self.log(f"Posición virtual: {old_pos} → ({self.rx},{self.ry})", "INFO")
                    self.draw_maze()
                elif command == "CELL_BACKWARD":
                    dx, dy = [(0,-1), (-1,0), (0,1), (1,0)][self.facing]
                    old_pos = (self.rx, self.ry)
                    self.rx += dx
                    self.ry += dy
                    self.log(f"Posición virtual: {old_pos} → ({self.rx},{self.ry})", "INFO")
                    self.draw_maze()
                elif command == "TURN_LEFT":
                    self.facing = (self.facing - 1) % 4
                    self.log(f"Orientación: {DIRS[self.facing]}", "INFO")
                    self.draw_maze()
                elif command == "TURN_RIGHT":
                    self.facing = (self.facing + 1) % 4
                    self.log(f"Orientación: {DIRS[self.facing]}", "INFO")
                    self.draw_maze()
                elif command == "TURN_180":
                    self.facing = (self.facing + 2) % 4
                    self.log(f"Orientación: {DIRS[self.facing]}", "INFO")
                    self.draw_maze()
                
                time.sleep(1)
                self.manual_status_label.config(text="⚪ Listo para comando manual", fg='#95a5a6')
                break
    
    def test_motors_encoders(self):
        """Test de diagnóstico de motores y encoders"""
        if self.running or self.calibrating:
            messagebox.showwarning("Advertencia", "Detén los procesos activos primero")
            return
        
        if not self.connected:
            messagebox.showwarning("Advertencia", "Conecta el robot antes de hacer el test")
            return
        
        self.log("===================", "INFO")
        self.log("🧪 INICIANDO TEST DE HARDWARE", "SUCCESS")
        self.log("===================", "INFO")
        self.log("El robot avanzará durante 5 segundos", "INFO")
        self.log("Observa si ambas ruedas giran a velocidad similar", "INFO")
        self.log("===================", "INFO")
        
        # Crear un comando de test especial para el ESP32
        # Enviaremos FORWARD y monitorizaremos
        ack = self.send_cmd("FORWARD")
        
        if ack != "ACK":
            self.log("❌ Error al iniciar test", "ERROR")
            return
        
        self.log("⚙️ Robot en movimiento - Monitoreando encoders...", "INFO")
        
        # Monitorizar durante 5 segundos
        start_time = time.time()
        samples = []
        
        while time.time() - start_time < 5:
            # Simular lectura de encoders pidiendo STATUS
            resp = self.send_cmd("STATUS")
            
            if resp and ":" in resp:
                try:
                    # Parsear respuesta (aunque sea BUSY, nos sirve para monitoreo)
                    parts = resp.split(":")
                    if len(parts) > 1:
                        sensor_data = parts[1].split(",")
                        # En un futuro podrías pedir ENCODERS si implementas ese comando
                        
                        # Por ahora solo mostramos que está funcionando
                        elapsed = time.time() - start_time
                        self.log(f"t={elapsed:.1f}s - Robot avanzando...", "SENSOR")
                        samples.append(elapsed)
                except:
                    pass
            
            time.sleep(0.5)
        
        # Detener el robot
        self.send_cmd("STOP")
        
        self.log("===================", "SUCCESS")
        self.log("🔍 ANÁLISIS DEL TEST:", "SUCCESS")
        self.log("===================", "SUCCESS")
        self.log("", "INFO")
        self.log("Si observaste que:", "INFO")
        self.log("", "INFO")
        self.log("✓ Ambas ruedas giran igual:", "SUCCESS")
        self.log("  → Hardware OK, problema es de calibración PID", "INFO")
        self.log("", "INFO")
        self.log("✗ Rueda IZQUIERDA va MÁS LENTA:", "ERROR")
        self.log("  → Posibles causas:", "INFO")
        self.log("  1. Motor izquierdo con POLARIDAD INVERTIDA", "WARNING")
        self.log("  2. Encoder izquierdo cuenta AL REVÉS", "WARNING")
        self.log("  3. Mayor fricción en rueda izquierda", "WARNING")
        self.log("", "INFO")
        self.log("✗ Rueda DERECHA va MÁS LENTA:", "ERROR")
        self.log("  → Posibles causas:", "INFO")
        self.log("  1. Motor derecho con POLARIDAD INVERTIDA", "WARNING")
        self.log("  2. Encoder derecho cuenta AL REVÉS", "WARNING")
        self.log("  3. Mayor fricción en rueda derecha", "WARNING")
        self.log("", "INFO")
        self.log("💡 SOLUCIÓN RECOMENDADA:", "SUCCESS")
        self.log("En el código del ESP32 (main.cpp):", "INFO")
        self.log("", "INFO")
        self.log("Si rueda IZQUIERDA va lenta, invierte:", "INFO")
        self.log("  SimpleMotor motorLeft(AIN2, AIN1, PWMA);", "COMMAND")
        self.log("  (intercambia AIN1 con AIN2)", "INFO")
        self.log("", "INFO")
        self.log("Si rueda DERECHA va lenta, invierte:", "INFO")
        self.log("  SimpleMotor motorRight(BIN2, BIN1, PWMB);", "COMMAND")
        self.log("  (intercambia BIN1 con BIN2)", "INFO")
        self.log("", "INFO")
        self.log("O también puedes invertir los encoders:", "INFO")
        self.log("  encoderLeft.attachHalfQuad(ENC_L_B, ENC_L_A);", "COMMAND")
        self.log("  (intercambia el pin A con B)", "INFO")
        self.log("===================", "SUCCESS")
        
        # Pequeño delay antes de permitir otro comando
        time.sleep(2)
    
    def toggle_calibration(self):
        """Activa/desactiva el modo de calibración de sensores"""
        if self.running:
            messagebox.showwarning("Advertencia", "Detén el modo automático primero")
            return
        
        if not self.connected:
            messagebox.showwarning("Advertencia", "Conecta el robot antes de calibrar")
            return
        
        if not self.calibrating:
            # Iniciar calibración
            self.calibrating = True
            self.btn_calibrate.config(text="⏹️ Detener Monitoreo", bg='#e74c3c')
            self.btn_start.config(state=tk.DISABLED, bg='#95a5a6')
            self.log("===================", "INFO")
            self.log("🔧 MODO CALIBRACIÓN ACTIVADO", "SUCCESS")
            self.log("Monitoreando sensores en tiempo real...", "INFO")
            self.log("===================", "INFO")
            threading.Thread(target=self.calibration_loop, daemon=True).start()
        else:
            # Detener calibración
            self.calibrating = False
            self.btn_calibrate.config(text="🔧 Calibrar/Monitorear Sensores", bg='#9b59b6')
            self.btn_start.config(state=tk.NORMAL, bg='#27ae60')
            self.log("Modo calibración detenido", "INFO")
    
    def calibration_loop(self):
        """Loop de monitoreo continuo de sensores para calibración"""
        sample_count = 0
        # Estadísticas para calibración
        stats = {
            'front': {'min': 9999, 'max': 0, 'sum': 0, 'count': 0},
            'left': {'min': 9999, 'max': 0, 'sum': 0, 'count': 0},
            'right': {'min': 9999, 'max': 0, 'sum': 0, 'count': 0}
        }
        
        while self.calibrating:
            sample_count += 1
            
            # Leer sensores
            resp = self.send_cmd("SENSORS")
            if not resp or not resp.startswith("IDLE"):
                self.log("Error leyendo sensores", "ERROR")
                time.sleep(0.5)
                continue
            
            try:
                parts = resp.split(":")[1].split(",")
                f_mm, l_mm, r_mm = int(parts[0]), int(parts[1]), int(parts[2])
            except Exception as e:
                self.log(f"Error parseando datos: {e}", "ERROR")
                time.sleep(0.5)
                continue
            
            # Actualizar display
            self.root.after(0, lambda: self.update_sensor_display(f_mm, l_mm, r_mm))
            
            # Actualizar estadísticas
            for sensor, value in [('front', f_mm), ('left', l_mm), ('right', r_mm)]:
                stats[sensor]['min'] = min(stats[sensor]['min'], value)
                stats[sensor]['max'] = max(stats[sensor]['max'], value)
                stats[sensor]['sum'] += value
                stats[sensor]['count'] += 1
            
            # Log detallado cada 10 muestras
            if sample_count % 10 == 0:
                avg_f = stats['front']['sum'] / stats['front']['count']
                avg_l = stats['left']['sum'] / stats['left']['count']
                avg_r = stats['right']['sum'] / stats['right']['count']
                
                self.log(f"📊 Muestra #{sample_count}", "SENSOR")
                self.log(f"  Frontal: {f_mm}mm (Min:{stats['front']['min']}, Max:{stats['front']['max']}, Avg:{avg_f:.1f})", "SENSOR")
                self.log(f"  Izquierdo: {l_mm}mm (Min:{stats['left']['min']}, Max:{stats['left']['max']}, Avg:{avg_l:.1f})", "SENSOR")
                self.log(f"  Derecho: {r_mm}mm (Min:{stats['right']['min']}, Max:{stats['right']['max']}, Avg:{avg_r:.1f})", "SENSOR")
                
                # Sugerencias de calibración
                if sample_count % 50 == 0:
                    self.log("💡 SUGERENCIAS DE CALIBRACIÓN:", "INFO")
                    threshold = 150
                    self.log(f"  • Umbral actual de pared: {threshold}mm", "INFO")
                    if stats['front']['max'] > threshold + 50:
                        self.log(f"  • Sensor frontal detecta objetos lejanos ({stats['front']['max']}mm)", "INFO")
                    if stats['left']['max'] > threshold + 50:
                        self.log(f"  • Sensor izquierdo detecta objetos lejanos ({stats['left']['max']}mm)", "INFO")
                    if stats['right']['max'] > threshold + 50:
                        self.log(f"  • Sensor derecho detecta objetos lejanos ({stats['right']['max']}mm)", "INFO")
            
            time.sleep(0.3)  # 3-4 lecturas por segundo
        
        # Resumen final
        if sample_count > 0:
            self.log("===================", "SUCCESS")
            self.log(f"📋 RESUMEN DE CALIBRACIÓN ({sample_count} muestras)", "SUCCESS")
            for sensor_name, sensor_key in [("Frontal", 'front'), ("Izquierdo", 'left'), ("Derecho", 'right')]:
                avg = stats[sensor_key]['sum'] / stats[sensor_key]['count']
                self.log(f"{sensor_name}: Min={stats[sensor_key]['min']}mm, Max={stats[sensor_key]['max']}mm, Promedio={avg:.1f}mm", "INFO")
            self.log("===================", "SUCCESS")

    def logic_loop(self):
        self.log("Algoritmo Flood Fill iniciado", "SUCCESS")
        self.log(f"Posición inicial: ({self.rx}, {self.ry})", "INFO")
        self.log(f"Orientación: {DIRS[self.facing]}", "INFO")
        
        step_count = 0
        
        while self.running:
            step_count += 1
            
            # 1. Leer Sensores
            resp = self.send_cmd("SENSORS") # Esperamos "IDLE:F,L,R"
            if not resp or not resp.startswith("IDLE"):
                self.log("Esperando respuesta del robot...", "WARNING")
                time.sleep(0.5)
                continue
            
            try:
                parts = resp.split(":")[1].split(",")
                f_mm, l_mm, r_mm = int(parts[0]), int(parts[1]), int(parts[2])
            except Exception as e:
                self.log(f"Error al parsear sensores: {e}", "ERROR")
                continue

            # Actualizar display de sensores
            self.root.after(0, lambda: self.update_sensor_display(f_mm, l_mm, r_mm))
            
            # Log de diagnóstico de sensores
            wall_front = "🧱" if f_mm < 150 else "⬜"
            wall_left = "🧱" if l_mm < 150 else "⬜"
            wall_right = "🧱" if r_mm < 150 else "⬜"
            self.log(f"Paso {step_count} - Sensores: F={f_mm}mm {wall_front}, L={l_mm}mm {wall_left}, R={r_mm}mm {wall_right}", "SENSOR")

            # 2. Actualizar Muro y Mapa
            self.solver.update_walls(self.rx, self.ry, self.facing, f_mm, l_mm, r_mm)
            self.log(f"Mapa actualizado en ({self.rx}, {self.ry})", "INFO")
            
            # --- DETECCIÓN DE META DINÁMICA ---
            if self.exploration_phase and self.solver.check_center_found():
                self.log("===================", "SUCCESS")
                self.log("🏆 ¡META (4 CASILLAS) ENCONTRADA!", "SUCCESS")
                self.log(f"Total de pasos exploración: {step_count}", "SUCCESS")
                self.log("===================", "SUCCESS")
                self.root.after(0, self.draw_maze)
                
                # Cambiar a fase de retorno
                self.goal_found = True
                self.exploration_phase = False
                
                self.log("🔄 INICIANDO RETORNO AL INICIO...", "INFO")
                self.log("Calculando ruta óptima de regreso...", "INFO")
                
                # Actualizar distancias desde la posición inicial
                self.solver.update_distances_from_start()
                self.root.after(0, self.draw_maze)
                
                # Continuar el loop para retornar al inicio
                time.sleep(1)
                continue
            # ----------------------------------
            
            # --- DETECCIÓN DE RETORNO AL INICIO ---
            if not self.exploration_phase and self.rx == 0 and self.ry == 0:
                self.log("===================", "SUCCESS")
                self.log("🏠 ¡RETORNO AL INICIO COMPLETADO!", "SUCCESS")
                self.log("===================", "SUCCESS")
                
                # Recalcular distancias hacia la meta
                self.solver.update_distances()
                
                # Calcular la ruta óptima
                self.log("🧠 CALCULANDO RUTA ÓPTIMA...", "INFO")
                self.optimal_path = self.solver.calculate_optimal_path(self.rx, self.ry, self.facing)
                
                self.log(f"✅ Ruta óptima calculada: {len(self.optimal_path)} comandos", "SUCCESS")
                self.log(f"📋 Comandos: {' → '.join(self.optimal_path[:20])}{'...' if len(self.optimal_path) > 20 else ''}", "INFO")
                
                # Preguntar al usuario si desea ejecutar la ruta óptima
                self.root.after(0, self.draw_maze)
                response = messagebox.askyesno(
                    "Ruta Óptima Calculada",
                    f"¡El robot ha regresado al inicio!\n\n" +
                    f"Ruta óptima: {len(self.optimal_path)} comandos\n\n" +
                    f"¿Deseas ejecutar la ruta óptima ahora?"
                )
                
                if response:
                    self.log("===================", "SUCCESS")
                    self.log("🚀 EJECUTANDO RUTA ÓPTIMA", "SUCCESS")
                    self.log("===================", "SUCCESS")
                    self.execute_optimal_path()
                else:
                    self.log("Ejecución de ruta óptima cancelada por el usuario", "INFO")
                
                self.running = False
                self.send_cmd("STOP")
                self.root.after(0, lambda: self.btn_start.config(state=tk.NORMAL, bg='#27ae60'))
                break
            # ----------------------------------
            
            self.root.after(0, self.draw_maze) # Actualizar GUI

            # 4. Calcular Siguiente Movimiento
            move = self.solver.get_next_move(self.rx, self.ry, self.facing, exploration_mode=self.exploration_phase)
            
            if not move:
                phase_name = "exploración" if self.exploration_phase else "retorno"
                self.log(f"Sin salida disponible - Fin de {phase_name}", "ERROR")
                self.running = False
                self.root.after(0, lambda: self.btn_start.config(state=tk.NORMAL, bg='#27ae60'))
                break
            
            # Traducir comando a español
            commands_spanish = {
                "FORWARD": "Avanza",
                "TURNL": "Gira Izquierda",
                "TURNR": "Gira Derecha",
                "TURNU": "Media Vuelta"
            }
            self.log(f"Comando: {commands_spanish.get(move, move)}", "COMMAND")
            self.root.after(0, lambda m=move: self.update_command_display(m))
            
            # 5. Enviar Comando y Esperar
            ack = self.send_cmd(move)
            if ack != "ACK":
                self.log(f"Error en ACK: {ack}", "ERROR")
                continue
            else:
                self.log("ACK recibido ✓", "INFO")
                
            # 6. Esperar a que termine (Polling STATUS)
            while True:
                time.sleep(0.2)
                status = self.send_cmd("STATUS")
                if status and status.startswith("IDLE"):
                    self.log("Movimiento completado", "INFO")
                    break
            
            # 7. Actualizar Coordenadas Virtuales
            old_x, old_y = self.rx, self.ry
            if move == "FORWARD":
                dx, dy = [(0,1), (1,0), (0,-1), (-1,0)][self.facing]
                self.rx += dx; self.ry += dy
                self.log(f"Posición actualizada: ({old_x},{old_y}) → ({self.rx},{self.ry})", "INFO")
            elif move == "TURNL":
                self.facing = (self.facing - 1) % 4
                self.log(f"Orientación: {DIRS[self.facing]}", "INFO")
            elif move == "TURNR":
                self.facing = (self.facing + 1) % 4
                self.log(f"Orientación: {DIRS[self.facing]}", "INFO")
            elif move == "TURNU":
                self.facing = (self.facing + 2) % 4
                self.log(f"Orientación: {DIRS[self.facing]}", "INFO")
            
            time.sleep(0.2) # Pequeña pausa para estabilizar
        
        self.log("Algoritmo finalizado", "INFO")
        self.root.after(0, lambda: self.update_command_display("Ninguno"))
    
    def execute_optimal_path(self):
        """
        Ejecuta la ruta óptima calculada previamente.
        """
        if not self.optimal_path:
            self.log("❌ No hay ruta óptima calculada", "ERROR")
            return
        
        total_commands = len(self.optimal_path)
        
        for idx, command in enumerate(self.optimal_path, 1):
            if not self.running:
                self.log("⏹️ Ejecución de ruta óptima detenida", "WARNING")
                return
            
            # Traducir comando a español
            commands_spanish = {
                "FORWARD": "Avanza",
                "TURNL": "Gira Izquierda",
                "TURNR": "Gira Derecha",
                "TURNU": "Media Vuelta"
            }
            
            self.log(f"[{idx}/{total_commands}] {commands_spanish.get(command, command)}", "COMMAND")
            self.root.after(0, lambda c=command: self.update_command_display(c))
            
            # Enviar comando
            ack = self.send_cmd(command)
            if ack != "ACK":
                self.log(f"❌ Error en ACK: {ack}", "ERROR")
                self.log("⏹️ Ejecución de ruta óptima abortada", "ERROR")
                return
            
            self.log("ACK recibido ✓", "INFO")
            
            # Esperar a que termine
            while True:
                time.sleep(0.2)
                status = self.send_cmd("STATUS")
                if status and status.startswith("IDLE"):
                    self.log("Movimiento completado", "INFO")
                    break
            
            # Actualizar posición virtual
            if command == "FORWARD":
                dx, dy = [(0,1), (1,0), (0,-1), (-1,0)][self.facing]
                old_x, old_y = self.rx, self.ry
                self.rx += dx
                self.ry += dy
                self.log(f"Posición: ({old_x},{old_y}) → ({self.rx},{self.ry})", "INFO")
            elif command == "TURNL":
                self.facing = (self.facing - 1) % 4
                self.log(f"Orientación: {DIRS[self.facing]}", "INFO")
            elif command == "TURNR":
                self.facing = (self.facing + 1) % 4
                self.log(f"Orientación: {DIRS[self.facing]}", "INFO")
            elif command == "TURNU":
                self.facing = (self.facing + 2) % 4
                self.log(f"Orientación: {DIRS[self.facing]}", "INFO")
            
            self.root.after(0, self.draw_maze)
            time.sleep(0.2)
        
        self.log("===================", "SUCCESS")
        self.log("🎉 ¡RUTA ÓPTIMA COMPLETADA!", "SUCCESS")
        self.log(f"Total de comandos ejecutados: {total_commands}", "SUCCESS")
        self.log(f"Posición final: ({self.rx}, {self.ry})", "SUCCESS")
        self.log("===================", "SUCCESS")
        
        messagebox.showinfo(
            "¡Éxito Total!",
            f"Ruta óptima completada exitosamente\n\n" +
            f"Comandos ejecutados: {total_commands}\n" +
            f"Posición final: ({self.rx}, {self.ry})"
        )

# ====== EJECUCIÓN PRINCIPAL ======
if __name__ == "__main__":
    try:
        # 1. Crear la ventana raíz de Tkinter
        root = tk.Tk()
        
        # 2. Instanciar tu aplicación
        app = MicromouseApp(root)
        
        # 3. Arrancar el bucle infinito que mantiene la ventana abierta
        root.mainloop()
        
    except Exception as e:
        print(f"Error iniciando la aplicación: {e}")
        input("Presiona Enter para salir...")