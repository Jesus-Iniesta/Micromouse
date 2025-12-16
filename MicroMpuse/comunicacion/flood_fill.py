"""
Implementación del algoritmo Flood Fill para resolver laberintos
Matriz de 12x7 celdas
"""
import numpy as np
from collections import deque

class FloodFill:
    """Algoritmo Flood Fill para navegación del robot"""
    
    # Direcciones: Norte, Este, Sur, Oeste
    NORTE = 0
    ESTE = 1
    SUR = 2
    OESTE = 3
    
    # Deltas de movimiento [fila, columna] para cada dirección
    DELTAS = {
        NORTE: (-1, 0),
        ESTE: (0, 1),
        SUR: (1, 0),
        OESTE: (0, -1)
    }
    
    def __init__(self, filas=7, columnas=12):
        """
        Inicializa el laberinto
        
        Args:
            filas: Número de filas del laberinto (7)
            columnas: Número de columnas del laberinto (12)
        """
        self.filas = filas
        self.columnas = columnas
        
        # Matriz de distancias (inicialmente todas con valor alto)
        self.distancias = np.full((filas, columnas), 999, dtype=int)
        
        # Matriz de paredes: [Norte, Este, Sur, Oeste]
        # True = hay pared, False = no hay pared
        self.paredes = np.zeros((filas, columnas, 4), dtype=bool)
        
        # Paredes del perímetro exterior
        self._inicializar_paredes_exteriores()
        
        # Posición actual del robot
        self.pos_actual = [filas - 1, 0]  # Esquina inferior izquierda
        self.direccion_actual = self.NORTE  # Mirando hacia el norte
        
        # Objetivo (centro del laberinto)
        self.objetivo = [(filas // 2, columnas // 2),
                        (filas // 2, columnas // 2 - 1),
                        (filas // 2 - 1, columnas // 2),
                        (filas // 2 - 1, columnas // 2 - 1)]
        
        # Historial de visitas
        self.visitado = np.zeros((filas, columnas), dtype=bool)
        self.visitado[self.pos_actual[0], self.pos_actual[1]] = True
        
    def _inicializar_paredes_exteriores(self):
        """Establece las paredes del perímetro del laberinto"""
        # Pared norte (primera fila)
        self.paredes[0, :, self.NORTE] = True
        
        # Pared sur (última fila)
        self.paredes[-1, :, self.SUR] = True
        
        # Pared oeste (primera columna)
        self.paredes[:, 0, self.OESTE] = True
        
        # Pared este (última columna)
        self.paredes[:, -1, self.ESTE] = True
    
    def actualizar_paredes(self, fila, col, paredes_detectadas):
        """
        Actualiza las paredes detectadas en una celda
        
        Args:
            fila, col: Posición de la celda
            paredes_detectadas: dict {'frontal': bool, 'izquierdo': bool, 'derecho': bool}
        """
        direccion = self.direccion_actual
        
        # Pared frontal
        if paredes_detectadas['frontal']:
            self.paredes[fila, col, direccion] = True
            # Actualizar celda adyacente
            df, dc = self.DELTAS[direccion]
            nueva_fila, nueva_col = fila + df, col + dc
            if 0 <= nueva_fila < self.filas and 0 <= nueva_col < self.columnas:
                self.paredes[nueva_fila, nueva_col, (direccion + 2) % 4] = True
        
        # Pared izquierda
        dir_izq = (direccion - 1) % 4
        if paredes_detectadas['izquierdo']:
            self.paredes[fila, col, dir_izq] = True
            df, dc = self.DELTAS[dir_izq]
            nueva_fila, nueva_col = fila + df, col + dc
            if 0 <= nueva_fila < self.filas and 0 <= nueva_col < self.columnas:
                self.paredes[nueva_fila, nueva_col, (dir_izq + 2) % 4] = True
        
        # Pared derecha
        dir_der = (direccion + 1) % 4
        if paredes_detectadas['derecho']:
            self.paredes[fila, col, dir_der] = True
            df, dc = self.DELTAS[dir_der]
            nueva_fila, nueva_col = fila + df, col + dc
            if 0 <= nueva_fila < self.filas and 0 <= nueva_col < self.columnas:
                self.paredes[nueva_fila, nueva_col, (dir_der + 2) % 4] = True
    
    def calcular_distancias(self):
        """Calcula las distancias desde cada celda al objetivo usando BFS"""
        # Reiniciar distancias
        self.distancias.fill(999)
        
        # Cola para BFS
        cola = deque()
        
        # Inicializar objetivo con distancia 0
        for obj_fila, obj_col in self.objetivo:
            self.distancias[obj_fila, obj_col] = 0
            cola.append((obj_fila, obj_col))
        
        # BFS
        while cola:
            fila, col = cola.popleft()
            dist_actual = self.distancias[fila, col]
            
            # Revisar vecinos en las 4 direcciones
            for direccion in range(4):
                # Si no hay pared en esta dirección
                if not self.paredes[fila, col, direccion]:
                    df, dc = self.DELTAS[direccion]
                    nueva_fila, nueva_col = fila + df, col + dc
                    
                    # Si está dentro del laberinto
                    if 0 <= nueva_fila < self.filas and 0 <= nueva_col < self.columnas:
                        # Si encontramos un camino más corto
                        if self.distancias[nueva_fila, nueva_col] > dist_actual + 1:
                            self.distancias[nueva_fila, nueva_col] = dist_actual + 1
                            cola.append((nueva_fila, nueva_col))
    
    def obtener_mejor_direccion(self):
        """
        Determina la mejor dirección a tomar desde la posición actual
        
        Returns:
            Dirección (NORTE, ESTE, SUR, OESTE) o None si no hay camino
        """
        fila, col = self.pos_actual
        mejor_dist = self.distancias[fila, col]
        mejor_dir = None
        
        # Revisar todas las direcciones accesibles
        for direccion in range(4):
            if not self.paredes[fila, col, direccion]:
                df, dc = self.DELTAS[direccion]
                nueva_fila, nueva_col = fila + df, col + dc
                
                if 0 <= nueva_fila < self.filas and 0 <= nueva_col < self.columnas:
                    if self.distancias[nueva_fila, nueva_col] < mejor_dist:
                        mejor_dist = self.distancias[nueva_fila, nueva_col]
                        mejor_dir = direccion
        
        return mejor_dir
    
    def obtener_comandos_movimiento(self, direccion_objetivo):
        """
        Calcula los comandos necesarios para orientarse hacia la dirección objetivo
        
        Args:
            direccion_objetivo: Dirección hacia la que queremos movernos
            
        Returns:
            Lista de comandos: 'L' (izquierda), 'R' (derecha), 'F' (avanzar)
        """
        if direccion_objetivo is None:
            return []
        
        comandos = []
        
        # Calcular giros necesarios
        diferencia = (direccion_objetivo - self.direccion_actual) % 4
        
        if diferencia == 1:  # Girar derecha
            comandos.append('R')
            self.direccion_actual = direccion_objetivo
        elif diferencia == 2:  # Dar media vuelta
            comandos.extend(['R', 'R'])
            self.direccion_actual = direccion_objetivo
        elif diferencia == 3:  # Girar izquierda
            comandos.append('L')
            self.direccion_actual = direccion_objetivo
        
        # Avanzar
        comandos.append('F')
        
        # Actualizar posición
        df, dc = self.DELTAS[direccion_objetivo]
        self.pos_actual[0] += df
        self.pos_actual[1] += dc
        self.visitado[self.pos_actual[0], self.pos_actual[1]] = True
        
        return comandos
    
    def en_objetivo(self):
        """Verifica si el robot llegó al objetivo"""
        return tuple(self.pos_actual) in [tuple(obj) for obj in self.objetivo]
    
    def detectar_paredes_desde_sensores(self, sensores, umbral_pared=120):
        """
        Convierte lecturas de sensores en detección de paredes
        
        Args:
            sensores: dict con {'frontal': mm, 'izquierdo': mm, 'derecho': mm}
            umbral_pared: Distancia en mm para considerar que hay pared
            
        Returns:
            dict con {'frontal': bool, 'izquierdo': bool, 'derecho': bool}
        """
        return {
            'frontal': sensores['frontal'] < umbral_pared,
            'izquierdo': sensores['izquierdo'] < umbral_pared,
            'derecho': sensores['derecho'] < umbral_pared
        }
    
    def obtener_nombre_direccion(self, direccion):
        """Convierte número de dirección a nombre"""
        nombres = {0: 'Norte', 1: 'Este', 2: 'Sur', 3: 'Oeste'}
        return nombres.get(direccion, 'Desconocido')
