"""
Interfaz Gráfica para Control de Robot Micromouse
Implementa algoritmo Flood Fill y comunicación WiFi
"""

import tkinter as tk
from tkinter import ttk, messagebox, scrolledtext
import socket
import threading
import queue
import time
from collections import deque

# ====== CONSTANTES ======
ROBOT_IP = "192.168.1.100"  # <<<< CAMBIAR a la IP del ESP32
ROBOT_PORT = 12345
MAZE_COLS = 12  # Laberinto 12 columnas
MAZE_ROWS = 7   # Laberinto 7 filas
CELL_SIZE = 50  # Tamaño de celda en píxeles para visualización
CELL_SIZE_MM = 160  # Tamaño real de celda en mm (16cm × 16cm)
TARGET_X, TARGET_Y = 5, 3  # Centro del laberinto (meta)

# Direcciones
NORTH, EAST, SOUTH, WEST = 0, 1, 2, 3
DIRECTION_NAMES = ['Norte', 'Este', 'Sur', 'Oeste']
DIRECTION_ARROWS = ['↑', '→', '↓', '←']

class MazeCell:
    """Representa una celda del laberinto"""
    def __init__(self):
        self.walls = {'N': True, 'E': True, 'S': True, 'W': True}
        self.visited = False
        self.distance = 999


class FloodFill:
    """Algoritmo Flood Fill para resolver el laberinto"""
    def __init__(self, cols, rows, target_x, target_y):
        self.cols = cols  # Columnas (ancho)
        self.rows = rows  # Filas (alto)
        self.target_x = target_x
        self.target_y = target_y
        self.maze = [[MazeCell() for _ in range(rows)] for _ in range(cols)]
        
        # Inicializar paredes externas
        for x in range(cols):
            for y in range(rows):
                if x == 0:
                    self.maze[x][y].walls['W'] = True
                if x == cols - 1:
                    self.maze[x][y].walls['E'] = True
                if y == 0:
                    self.maze[x][y].walls['S'] = True
                if y == rows - 1:
                    self.maze[x][y].walls['N'] = True
        
        self.calculate_distances()
    
    def calculate_distances(self):
        """Calcula distancias Manhattan desde la meta usando BFS"""
        # Reiniciar distancias
        for x in range(self.cols):
            for y in range(self.rows):
                self.maze[x][y].distance = 999
        
        # BFS desde la meta
        queue_bfs = deque()
        self.maze[self.target_x][self.target_y].distance = 0
        queue_bfs.append((self.target_x, self.target_y))
        
        while queue_bfs:
            x, y = queue_bfs.popleft()
            current_dist = self.maze[x][y].distance
            
            # Revisar vecinos
            neighbors = [
                (x, y + 1, 'N'),  # Norte
                (x + 1, y, 'E'),  # Este
                (x, y - 1, 'S'),  # Sur
                (x - 1, y, 'W')   # Oeste
            ]
            
            for nx, ny, wall_dir in neighbors:
                if 0 <= nx < self.cols and 0 <= ny < self.rows:
                    if not self.maze[x][y].walls[wall_dir]:
                        if self.maze[nx][ny].distance > current_dist + 1:
                            self.maze[nx][ny].distance = current_dist + 1
                            queue_bfs.append((nx, ny))
    
    def update_walls(self, x, y, front, left, right, direction):
        """Actualiza las paredes detectadas por los sensores"""
        wall_threshold = 100  # mm - si la distancia es menor, hay pared
        
        # Mapear sensores a direcciones absolutas
        dirs = {
            NORTH: {'F': 'N', 'L': 'W', 'R': 'E'},
            EAST:  {'F': 'E', 'L': 'N', 'R': 'S'},
            SOUTH: {'F': 'S', 'L': 'E', 'R': 'W'},
            WEST:  {'F': 'W', 'L': 'S', 'R': 'N'}
        }
        
        mapping = dirs[direction]
        
        # Actualizar pared frontal
        self.maze[x][y].walls[mapping['F']] = (front < wall_threshold)
        
        # Actualizar pared izquierda
        self.maze[x][y].walls[mapping['L']] = (left < wall_threshold)
        
        # Actualizar pared derecha
        self.maze[x][y].walls[mapping['R']] = (right < wall_threshold)
        
        # Actualizar paredes de celdas adyacentes
        adjacent = {
            'N': (x, y + 1, 'S'),
            'E': (x + 1, y, 'W'),
            'S': (x, y - 1, 'N'),
            'W': (x - 1, y, 'E')
        }
        
        for wall_dir in ['F', 'L', 'R']:
            abs_dir = mapping[wall_dir]
            if abs_dir in adjacent:
                ax, ay, opposite = adjacent[abs_dir]
                if 0 <= ax < self.cols and 0 <= ay < self.rows:
                    self.maze[ax][ay].walls[opposite] = self.maze[x][y].walls[abs_dir]
        
        # Marcar como visitada
        self.maze[x][y].visited = True
        
        # Recalcular distancias
        self.calculate_distances()
    
    def get_best_move(self, x, y, direction):
        """Determina el mejor movimiento basado en Flood Fill"""
        current_dist = self.maze[x][y].distance
        
        # Direcciones posibles (prioridad: adelante, izquierda, derecha, atrás)
        moves = [
            (direction, 'FORWARD'),
            ((direction - 1) % 4, 'TURNL'),
            ((direction + 1) % 4, 'TURNR'),
            ((direction + 2) % 4, 'TURNU')
        ]
        
        best_move = None
        best_dist = 999
        
        for new_dir, command in moves:
            # Calcular nueva posición
            dx, dy = [(0, 1), (1, 0), (0, -1), (-1, 0)][new_dir]
            nx, ny = x + dx, y + dy
            
            # Verificar si es válido
            if 0 <= nx < self.cols and 0 <= ny < self.rows:
                wall_map = {0: 'N', 1: 'E', 2: 'S', 3: 'W'}
                if not self.maze[x][y].walls[wall_map[new_dir]]:
                    if self.maze[nx][ny].distance < best_dist:
                        best_dist = self.maze[nx][ny].distance
                        best_move = command
        
        return best_move


class RobotController:
    """Controlador de comunicación con el robot"""
    def __init__(self, ip, port):
        self.ip = ip
        self.port = port
        self.socket = None
        self.connected = False
        self.response_queue = queue.Queue()
    
    def connect(self):
        """Conectar al robot"""
        try:
            self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            self.socket.settimeout(2.0)
            self.connected = True
            return True
        except Exception as e:
            print(f"Error conectando: {e}")
            return False
    
    def send_command(self, command):
        """Enviar comando al robot"""
        if not self.connected:
            return None
        
        try:
            self.socket.sendto(command.encode(), (self.ip, self.port))
            data, _ = self.socket.recvfrom(1024)
            response = data.decode().strip()
            return response
        except socket.timeout:
            return "TIMEOUT"
        except Exception as e:
            print(f"Error enviando comando: {e}")
            return None
    
    def get_sensors(self):
        """Obtener lecturas de sensores"""
        response = self.send_command("SENSORS")
        if response and response.startswith("SENSORS:"):
            values = response.split(":")[1].split(",")
            return int(values[0]), int(values[1]), int(values[2])
        return None, None, None
    
    def disconnect(self):
        """Desconectar del robot"""
        if self.socket:
            self.socket.close()
        self.connected = False


class MicromouseGUI:
    """Interfaz gráfica principal"""
    def __init__(self, root):
        self.root = root
        self.root.title("🤖 Control Micromouse - Flood Fill WiFi")
        self.root.geometry("1400x900")
        self.root.configure(bg='#1e1e2e')
        
        # Estilo
        self.setup_style()
        
        # Variables
        self.robot_x = 0
        self.robot_y = 0
        self.robot_dir = NORTH
        self.flood_fill = FloodFill(MAZE_COLS, MAZE_ROWS, TARGET_X, TARGET_Y)
        self.robot_controller = RobotController(ROBOT_IP, ROBOT_PORT)
        self.running = False
        self.paused = False
        
        # Crear interfaz
        self.create_widgets()
        
        # Protocolo de cierre
        self.root.protocol("WM_DELETE_WINDOW", self.on_closing)
    
    def setup_style(self):
        """Configurar estilos de la interfaz"""
        style = ttk.Style()
        style.theme_use('clam')
        
        # Colores
        bg_dark = '#1e1e2e'
        bg_light = '#2d2d44'
        accent = '#7aa2f7'
        success = '#9ece6a'
        warning = '#e0af68'
        error = '#f7768e'
        
        style.configure('TFrame', background=bg_dark)
        style.configure('Card.TFrame', background=bg_light, relief='raised')
        style.configure('TLabel', background=bg_dark, foreground='#c0caf5', font=('Helvetica', 10))
        style.configure('Title.TLabel', font=('Helvetica', 14, 'bold'), foreground=accent)
        style.configure('Status.TLabel', font=('Helvetica', 12), foreground=success)
        
        style.configure('TButton', font=('Helvetica', 10), padding=10)
        style.map('TButton', background=[('active', accent)])
        
        style.configure('Success.TButton', background=success, foreground='black')
        style.configure('Danger.TButton', background=error, foreground='white')
        style.configure('Primary.TButton', background=accent, foreground='white')
    
    def create_widgets(self):
        """Crear widgets de la interfaz"""
        # Layout principal
        main_container = ttk.Frame(self.root)
        main_container.pack(fill='both', expand=True, padx=10, pady=10)
        
        # Panel izquierdo - Laberinto
        left_panel = ttk.Frame(main_container, style='Card.TFrame')
        left_panel.pack(side='left', fill='both', expand=True, padx=(0, 5))
        
        # Título del laberinto
        title_frame = ttk.Frame(left_panel)
        title_frame.pack(pady=10)
        ttk.Label(title_frame, text="🗺️  Mapa del Laberinto", 
                 style='Title.TLabel').pack()
        
        # Canvas para el laberinto
        canvas_frame = ttk.Frame(left_panel)
        canvas_frame.pack(fill='both', expand=True, padx=10, pady=10)
        
        self.canvas = tk.Canvas(canvas_frame, bg='#1a1a2e', 
                               width=MAZE_COLS*CELL_SIZE+20, 
                               height=MAZE_ROWS*CELL_SIZE+20,
                               highlightthickness=2,
                               highlightbackground='#7aa2f7')
        self.canvas.pack()
        
        # Panel derecho - Controles
        right_panel = ttk.Frame(main_container, style='Card.TFrame')
        right_panel.pack(side='right', fill='both', padx=(5, 0))
        
        # === Sección de Conexión ===
        conn_frame = ttk.LabelFrame(right_panel, text="📡 Conexión", padding=10)
        conn_frame.pack(fill='x', padx=10, pady=10)
        
        ttk.Label(conn_frame, text=f"IP: {ROBOT_IP}:{ROBOT_PORT}").pack()
        
        btn_frame1 = ttk.Frame(conn_frame)
        btn_frame1.pack(pady=5)
        
        self.btn_connect = ttk.Button(btn_frame1, text="Conectar", 
                                      command=self.connect_robot,
                                      style='Success.TButton', width=15)
        self.btn_connect.pack(side='left', padx=2)
        
        self.btn_disconnect = ttk.Button(btn_frame1, text="Desconectar", 
                                        command=self.disconnect_robot,
                                        style='Danger.TButton', width=15, state='disabled')
        self.btn_disconnect.pack(side='left', padx=2)
        
        self.status_label = ttk.Label(conn_frame, text="⚫ Desconectado", 
                                     style='Status.TLabel')
        self.status_label.pack(pady=5)
        
        # === Sección de Control ===
        control_frame = ttk.LabelFrame(right_panel, text="🎮 Control", padding=10)
        control_frame.pack(fill='x', padx=10, pady=10)
        
        self.btn_start = ttk.Button(control_frame, text="▶️ Iniciar Exploración", 
                                    command=self.start_exploration,
                                    style='Primary.TButton', state='disabled')
        self.btn_start.pack(fill='x', pady=2)
        
        self.btn_pause = ttk.Button(control_frame, text="⏸️ Pausar", 
                                   command=self.pause_exploration, state='disabled')
        self.btn_pause.pack(fill='x', pady=2)
        
        self.btn_stop = ttk.Button(control_frame, text="⏹️ Detener", 
                                  command=self.stop_exploration,
                                  style='Danger.TButton', state='disabled')
        self.btn_stop.pack(fill='x', pady=2)
        
        self.btn_reset = ttk.Button(control_frame, text="🔄 Reiniciar Mapa", 
                                   command=self.reset_maze)
        self.btn_reset.pack(fill='x', pady=2)
        
        # === Información del Robot ===
        info_frame = ttk.LabelFrame(right_panel, text="📊 Información", padding=10)
        info_frame.pack(fill='x', padx=10, pady=10)
        
        self.info_position = ttk.Label(info_frame, text="Posición: (0, 0)")
        self.info_position.pack(anchor='w', pady=2)
        
        self.info_direction = ttk.Label(info_frame, text="Dirección: Norte ↑")
        self.info_direction.pack(anchor='w', pady=2)
        
        self.info_distance = ttk.Label(info_frame, text="Distancia a meta: 14")
        self.info_distance.pack(anchor='w', pady=2)
        
        # === Sensores ===
        sensor_frame = ttk.LabelFrame(right_panel, text="📡 Sensores (mm)", padding=10)
        sensor_frame.pack(fill='x', padx=10, pady=10)
        
        self.sensor_front = ttk.Label(sensor_frame, text="Frente: ---")
        self.sensor_front.pack(anchor='w', pady=2)
        
        self.sensor_left = ttk.Label(sensor_frame, text="Izquierda: ---")
        self.sensor_left.pack(anchor='w', pady=2)
        
        self.sensor_right = ttk.Label(sensor_frame, text="Derecha: ---")
        self.sensor_right.pack(anchor='w', pady=2)
        
        # === Log ===
        log_frame = ttk.LabelFrame(right_panel, text="📝 Log", padding=10)
        log_frame.pack(fill='both', expand=True, padx=10, pady=10)
        
        self.log_text = scrolledtext.ScrolledText(log_frame, height=15, 
                                                  bg='#1a1a2e', fg='#c0caf5',
                                                  font=('Consolas', 9))
        self.log_text.pack(fill='both', expand=True)
        
        # Dibujar laberinto inicial
        self.draw_maze()
    
    def draw_maze(self):
        """Dibujar el laberinto en el canvas"""
        self.canvas.delete('all')
        
        offset = 10
        
        for x in range(MAZE_COLS):
            for y in range(MAZE_ROWS):
                cell = self.flood_fill.maze[x][y]
                
                # Coordenadas (invertir Y para que (0,0) esté abajo-izquierda)
                cx = offset + x * CELL_SIZE
                cy = offset + (MAZE_ROWS - 1 - y) * CELL_SIZE
                
                # Color de fondo
                if (x, y) == (TARGET_X, TARGET_Y):
                    color = '#9ece6a'  # Verde para meta
                elif cell.visited:
                    color = '#414868'  # Visitada
                else:
                    color = '#24283b'  # No visitada
                
                # Dibujar celda
                self.canvas.create_rectangle(cx, cy, cx+CELL_SIZE, cy+CELL_SIZE, 
                                            fill=color, outline='')
                
                # Dibujar distancia
                if cell.distance < 999:
                    self.canvas.create_text(cx+CELL_SIZE//2, cy+CELL_SIZE//2,
                                          text=str(cell.distance), fill='#7aa2f7',
                                          font=('Helvetica', 8))
                
                # Dibujar paredes
                wall_color = '#c0caf5'
                wall_width = 2
                
                if cell.walls['N']:  # Norte (arriba)
                    self.canvas.create_line(cx, cy, cx+CELL_SIZE, cy, 
                                          fill=wall_color, width=wall_width)
                if cell.walls['E']:  # Este (derecha)
                    self.canvas.create_line(cx+CELL_SIZE, cy, cx+CELL_SIZE, cy+CELL_SIZE,
                                          fill=wall_color, width=wall_width)
                if cell.walls['S']:  # Sur (abajo)
                    self.canvas.create_line(cx, cy+CELL_SIZE, cx+CELL_SIZE, cy+CELL_SIZE,
                                          fill=wall_color, width=wall_width)
                if cell.walls['W']:  # Oeste (izquierda)
                    self.canvas.create_line(cx, cy, cx, cy+CELL_SIZE,
                                          fill=wall_color, width=wall_width)
        
        # Dibujar robot
        rx = offset + self.robot_x * CELL_SIZE + CELL_SIZE // 2
        ry = offset + (MAZE_ROWS - 1 - self.robot_y) * CELL_SIZE + CELL_SIZE // 2
        
        # Triángulo apuntando en la dirección
        arrow_angles = [90, 0, 270, 180]  # Norte, Este, Sur, Oeste
        angle = arrow_angles[self.robot_dir]
        
        self.canvas.create_oval(rx-12, ry-12, rx+12, ry+12, 
                               fill='#f7768e', outline='#c0caf5', width=2)
        self.canvas.create_text(rx, ry, text=DIRECTION_ARROWS[self.robot_dir],
                               fill='white', font=('Helvetica', 16, 'bold'))
    
    def log(self, message):
        """Agregar mensaje al log"""
        timestamp = time.strftime("%H:%M:%S")
        self.log_text.insert('end', f"[{timestamp}] {message}\n")
        self.log_text.see('end')
    
    def connect_robot(self):
        """Conectar al robot"""
        self.log("Conectando al robot...")
        if self.robot_controller.connect():
            self.status_label.config(text="🟢 Conectado", foreground='#9ece6a')
            self.btn_connect.config(state='disabled')
            self.btn_disconnect.config(state='normal')
            self.btn_start.config(state='normal')
            self.log("✓ Conexión establecida")
            
            # Verificar estado
            response = self.robot_controller.send_command("STATUS")
            self.log(f"Robot responde: {response}")
        else:
            self.status_label.config(text="🔴 Error", foreground='#f7768e')
            messagebox.showerror("Error", "No se pudo conectar al robot")
            self.log("✗ Error de conexión")
    
    def disconnect_robot(self):
        """Desconectar del robot"""
        self.robot_controller.disconnect()
        self.status_label.config(text="⚫ Desconectado")
        self.btn_connect.config(state='normal')
        self.btn_disconnect.config(state='disabled')
        self.btn_start.config(state='disabled')
        self.log("Desconectado del robot")
    
    def start_exploration(self):
        """Iniciar exploración del laberinto"""
        if not self.running:
            self.running = True
            self.paused = False
            self.btn_start.config(state='disabled')
            self.btn_pause.config(state='normal')
            self.btn_stop.config(state='normal')
            self.log("▶️ Iniciando exploración con Flood Fill...")
            
            # Iniciar thread de exploración
            thread = threading.Thread(target=self.exploration_loop, daemon=True)
            thread.start()
    
    def pause_exploration(self):
        """Pausar/reanudar exploración"""
        self.paused = not self.paused
        if self.paused:
            self.btn_pause.config(text="▶️ Reanudar")
            self.log("⏸️ Exploración pausada")
        else:
            self.btn_pause.config(text="⏸️ Pausar")
            self.log("▶️ Exploración reanudada")
    
    def stop_exploration(self):
        """Detener exploración"""
        self.running = False
        self.paused = False
        self.btn_start.config(state='normal')
        self.btn_pause.config(state='disabled')
        self.btn_stop.config(state='disabled')
        self.robot_controller.send_command("STOP")
        self.log("⏹️ Exploración detenida")
    
    def reset_maze(self):
        """Reiniciar el mapa"""
        self.robot_x = 0
        self.robot_y = 0
        self.robot_dir = NORTH
        self.flood_fill = FloodFill(MAZE_COLS, MAZE_ROWS, TARGET_X, TARGET_Y)
        self.draw_maze()
        self.update_info()
        self.log("🔄 Mapa reiniciado")
    
    def update_info(self):
        """Actualizar información del robot"""
        self.info_position.config(text=f"Posición: ({self.robot_x}, {self.robot_y})")
        self.info_direction.config(text=f"Dirección: {DIRECTION_NAMES[self.robot_dir]} {DIRECTION_ARROWS[self.robot_dir]}")
        dist = self.flood_fill.maze[self.robot_x][self.robot_y].distance
        self.info_distance.config(text=f"Distancia a meta: {dist}")
    
    def exploration_loop(self):
        """Loop principal de exploración"""
        while self.running:
            if self.paused:
                time.sleep(0.1)
                continue
            
            # Leer sensores
            front, left, right = self.robot_controller.get_sensors()
            
            if front is None:
                self.root.after(0, lambda: self.log("✗ Error leyendo sensores"))
                time.sleep(0.5)
                continue
            
            # Actualizar UI con sensores
            self.root.after(0, lambda f=front, l=left, r=right: self.update_sensors(f, l, r))
            
            # Actualizar mapa con paredes detectadas
            self.flood_fill.update_walls(self.robot_x, self.robot_y, 
                                        front, left, right, self.robot_dir)
            
            # Redibujar laberinto
            self.root.after(0, self.draw_maze)
            
            # Verificar si llegamos a la meta
            if self.robot_x == TARGET_X and self.robot_y == TARGET_Y:
                self.root.after(0, lambda: self.log("🎉 ¡META ALCANZADA!"))
                self.root.after(0, lambda: messagebox.showinfo("¡Éxito!", "¡El robot llegó a la meta!"))
                self.running = False
                break
            
            # Obtener mejor movimiento
            move = self.flood_fill.get_best_move(self.robot_x, self.robot_y, self.robot_dir)
            
            if move is None:
                self.root.after(0, lambda: self.log("⚠️ Sin movimientos válidos"))
                self.running = False
                break
            
            self.root.after(0, lambda m=move: self.log(f"Comando: {m}"))
            
            # Enviar comando
            response = self.robot_controller.send_command(move)
            
            if response == "ACK":
                # Esperar confirmación de OK
                time.sleep(0.5)
                while True:
                    status = self.robot_controller.send_command("STATUS")
                    if status and not status.startswith("BUSY"):
                        break
                    time.sleep(0.1)
                
                # Actualizar posición del robot
                if move == "FORWARD":
                    dx, dy = [(0, 1), (1, 0), (0, -1), (-1, 0)][self.robot_dir]
                    self.robot_x += dx
                    self.robot_y += dy
                elif move == "TURNL":
                    self.robot_dir = (self.robot_dir - 1) % 4
                elif move == "TURNR":
                    self.robot_dir = (self.robot_dir + 1) % 4
                elif move == "TURNU":
                    self.robot_dir = (self.robot_dir + 2) % 4
                
                self.root.after(0, self.update_info)
                self.root.after(0, self.draw_maze)
            else:
                self.root.after(0, lambda: self.log(f"✗ Error en comando: {response}"))
            
            time.sleep(0.3)  # Pausa entre movimientos
        
        # Finalizado
        self.root.after(0, self.stop_exploration)
    
    def update_sensors(self, front, left, right):
        """Actualizar lectura de sensores en UI"""
        self.sensor_front.config(text=f"Frente: {front} mm")
        self.sensor_left.config(text=f"Izquierda: {left} mm")
        self.sensor_right.config(text=f"Derecha: {right} mm")
    
    def on_closing(self):
        """Manejar cierre de ventana"""
        if self.running:
            if messagebox.askokcancel("Salir", "¿Detener exploración y salir?"):
                self.running = False
                self.robot_controller.send_command("STOP")
                self.robot_controller.disconnect()
                self.root.destroy()
        else:
            if self.robot_controller.connected:
                self.robot_controller.disconnect()
            self.root.destroy()


# ====== MAIN ======
if __name__ == "__main__":
    root = tk.Tk()
    app = MicromouseGUI(root)
    root.mainloop()
