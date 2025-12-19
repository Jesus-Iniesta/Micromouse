import tkinter as tk
from tkinter import ttk, messagebox, scrolledtext
import socket
import threading
import queue
import time
from collections import deque

# ====== CONFIGURACIÓN ======
DEFAULT_IP = "192.168.1.50"  # Pon aquí la IP que te dé el monitor serie de Arduino
ROBOT_PORT = 12345

MAZE_COLS = 12
MAZE_ROWS = 7
CELL_SIZE = 50
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

    def get_next_move(self, rx, ry, facing):
        # Buscar vecino accesible con menor distancia
        best_dist = 999
        best_move = None
        
        # Movimientos posibles: (nuevo_facing, comando_robot)
        moves = [
            (facing, 'FORWARD'),
            ((facing - 1) % 4, 'TURNL'),
            ((facing + 1) % 4, 'TURNR'),
            ((facing + 2) % 4, 'TURNU')
        ]
        
        for new_face, cmd in moves:
            # Coordenadas a las que llegariamos
            dx, dy = [(0,1), (1,0), (0,-1), (-1,0)][new_face]
            nx, ny = rx+dx, ry+dy
            
            # Verificar límites
            if 0 <= nx < MAZE_COLS and 0 <= ny < MAZE_ROWS:
                # Verificar pared
                wall_dir = ['N', 'E', 'S', 'W'][new_face]
                if not self.maze[rx][ry].walls[wall_dir]:
                    dist = self.maze[nx][ny].distance
                    # Preferir casillas no visitadas para explorar
                    if not self.maze[nx][ny].visited:
                        dist -= 0.5 
                        
                    if dist < best_dist:
                        best_dist = dist
                        best_move = cmd
                        
        return best_move

class MicromouseApp:
    def __init__(self, root):
        self.root = root
        self.root.title("Control Micromouse - Flood Fill")
        
        # Estado del Robot
        self.rx = 0
        self.ry = 0
        self.facing = NORTH # 0=N
        self.solver = FloodFillSolver()
        
        # Networking
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.settimeout(0.5)
        self.robot_ip = DEFAULT_IP
        self.connected = False
        
        # GUI Layout
        self.create_widgets()
        self.running = False
        
    def create_widgets(self):
        frame_ctrl = ttk.Frame(self.root, padding=10)
        frame_ctrl.pack(side=tk.RIGHT, fill=tk.Y)
        
        ttk.Label(frame_ctrl, text="IP Robot:").pack()
        self.entry_ip = ttk.Entry(frame_ctrl)
        self.entry_ip.insert(0, DEFAULT_IP)
        self.entry_ip.pack()
        
        ttk.Button(frame_ctrl, text="Conectar / Ping", command=self.do_ping).pack(pady=5)
        ttk.Separator(frame_ctrl, orient='horizontal').pack(fill='x', pady=10)
        
        self.btn_start = ttk.Button(frame_ctrl, text="INICIAR AUTO", command=self.start_auto)
        self.btn_start.pack(pady=5)
        
        ttk.Button(frame_ctrl, text="PARADA EMERGENCIA", command=self.emergency_stop).pack(pady=20)
        
        self.log_area = scrolledtext.ScrolledText(frame_ctrl, width=30, height=20)
        self.log_area.pack()
        
        # Canvas Laberinto
        self.canvas = tk.Canvas(self.root, width=MAZE_COLS*CELL_SIZE, height=MAZE_ROWS*CELL_SIZE, bg="white")
        self.canvas.pack(side=tk.LEFT, padx=10, pady=10)
        self.draw_maze()

    def log(self, msg):
        self.log_area.insert(tk.END, msg + "\n")
        self.log_area.see(tk.END)

    def do_ping(self):
        self.robot_ip = self.entry_ip.get()
        resp = self.send_cmd("STATUS")
        if resp:
            self.log(f"Conectado: {resp}")
            self.connected = True
        else:
            self.log("Sin respuesta del robot.")

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
                
                # Distancia
                self.canvas.create_text((x0+x1)/2, (y0+y1)/2, text=str(cell.distance), fill="gray")
                
                # Paredes
                w = 3
                if cell.walls['N']: self.canvas.create_line(x0, y0, x1, y0, width=w)
                if cell.walls['S']: self.canvas.create_line(x0, y1, x1, y1, width=w)
                if cell.walls['E']: self.canvas.create_line(x1, y0, x1, y1, width=w)
                if cell.walls['W']: self.canvas.create_line(x0, y0, x0, y1, width=w)
        
        # Dibujar flecha robot
        cx = self.rx * CELL_SIZE + CELL_SIZE/2
        cy = h - (self.ry * CELL_SIZE) - CELL_SIZE/2
        self.canvas.create_oval(cx-5, cy-5, cx+5, cy+5, fill="red")

    def start_auto(self):
        if not self.running:
            self.running = True
            threading.Thread(target=self.logic_loop, daemon=True).start()

    def emergency_stop(self):
        self.running = False
        self.send_cmd("STOP")
        self.log("!!! PARADA !!!")

    def logic_loop(self):
        self.log("Iniciando Flood Fill...")
        
        while self.running:
            # 1. Leer Sensores
            resp = self.send_cmd("SENSORS") # Esperamos "IDLE:F,L,R"
            if not resp or not resp.startswith("IDLE"):
                self.log("Esperando robot...")
                time.sleep(0.5)
                continue
            
            try:
                parts = resp.split(":")[1].split(",")
                f_mm, l_mm, r_mm = int(parts[0]), int(parts[1]), int(parts[2])
            except:
                continue

            # 2. Actualizar Muro y Mapa
            self.solver.update_walls(self.rx, self.ry, self.facing, f_mm, l_mm, r_mm)
            
            # --- DETECCIÓN DE META DINÁMICA ---
            if self.solver.check_center_found():
                self.log("🏆 ¡META (4 CASILLAS) ENCONTRADA!")
                self.root.after(0, self.draw_maze)
                # Opcional: Detenerse aquí o seguir explorando para mapear todo
                self.running = False
                self.send_cmd("STOP")
                break
            # ----------------------------------
            
            self.root.after(0, self.draw_maze) # Actualizar GUI

            # 4. Calcular Siguiente Movimiento
            move = self.solver.get_next_move(self.rx, self.ry, self.facing)
            
            if not move:
                self.log("Sin salida o error de ruta.")
                self.running = False
                break
                
            self.log(f"Ejecutando: {move}")
            
            # 5. Enviar Comando y Esperar
            ack = self.send_cmd(move)
            if ack != "ACK":
                self.log(f"Error ACK: {ack}")
                continue
                
            # 6. Esperar a que termine (Polling STATUS)
            while True:
                time.sleep(0.2)
                status = self.send_cmd("STATUS")
                if status and status.startswith("IDLE"):
                    break
            
            # 7. Actualizar Coordenadas Virtuales
            if move == "FORWARD":
                dx, dy = [(0,1), (1,0), (0,-1), (-1,0)][self.facing]
                self.rx += dx; self.ry += dy
            elif move == "TURNL":
                self.facing = (self.facing - 1) % 4
            elif move == "TURNR":
                self.facing = (self.facing + 1) % 4
            elif move == "TURNU":
                self.facing = (self.facing + 2) % 4
            
            time.sleep(0.2) # Pequeña pausa para estabilizar

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