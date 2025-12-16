"""
Programa principal de control del Micromouse
Integra comunicación serial, Flood Fill e interfaz gráfica
"""
import time
import sys
from protocolo_serial import ProtocoloSerial
from flood_fill import FloodFill
from interfaz_grafica import InterfazLaberinto

class ControlMicromouse:
    """Controlador principal del robot Micromouse"""
    
    def __init__(self, puerto='/dev/ttyUSB0', baudrate=115200):
        """
        Inicializa el sistema de control
        
        Args:
            puerto: Puerto serial del ESP32
            baudrate: Velocidad de comunicación
        """
        # Inicializar componentes
        self.serial = ProtocoloSerial(puerto, baudrate)
        self.flood_fill = FloodFill(filas=7, columnas=12)
        self.interfaz = InterfazLaberinto(self.flood_fill)
        
        # Estado
        self.ejecutando = False
        self.fase = "exploracion"  # "exploracion" o "carrera_rapida"
        self.movimientos = 0
        
    def conectar_robot(self):
        """Conecta con el ESP32"""
        print("Conectando con el robot...")
        if self.serial.conectar():
            print("¡Conectado exitosamente!")
            time.sleep(1)
            return True
        else:
            print("Error: No se pudo conectar con el robot")
            return False
    
    def leer_y_actualizar_sensores(self):
        """Lee los sensores y actualiza el mapa de paredes"""
        # Leer sensores
        sensores = self.serial.leer_sensores()
        
        if sensores is None:
            print("Error al leer sensores")
            return False
        
        print(f"Sensores - F:{sensores['frontal']} L:{sensores['izquierdo']} R:{sensores['derecho']}")
        
        # Detectar paredes
        paredes = self.flood_fill.detectar_paredes_desde_sensores(sensores)
        
        # Actualizar mapa
        fila, col = self.flood_fill.pos_actual
        self.flood_fill.actualizar_paredes(fila, col, paredes)
        
        return True
    
    def ejecutar_comando(self, comando):
        """
        Ejecuta un comando en el robot
        
        Args:
            comando: 'F', 'L', 'R'
            
        Returns:
            True si se ejecutó correctamente
        """
        if comando == 'F':
            print("→ Avanzando...")
            return self.serial.avanzar()
        elif comando == 'L':
            print("↺ Girando izquierda...")
            return self.serial.girar_izquierda()
        elif comando == 'R':
            print("↻ Girando derecha...")
            return self.serial.girar_derecha()
        else:
            print(f"Comando desconocido: {comando}")
            return False
    
    def paso_exploracion(self):
        """Ejecuta un paso del algoritmo de exploración"""
        print(f"\n=== Movimiento {self.movimientos + 1} ===")
        
        # Leer sensores y actualizar mapa
        if not self.leer_y_actualizar_sensores():
            return False
        
        # Recalcular distancias con el mapa actualizado
        self.flood_fill.calcular_distancias()
        
        # Obtener mejor dirección
        mejor_dir = self.flood_fill.obtener_mejor_direccion()
        
        if mejor_dir is None:
            print("¡No hay camino disponible!")
            return False
        
        # Obtener comandos necesarios
        comandos = self.flood_fill.obtener_comandos_movimiento(mejor_dir)
        
        # Ejecutar comandos
        for comando in comandos:
            if not self.ejecutar_comando(comando):
                print(f"Error al ejecutar comando: {comando}")
                return False
            time.sleep(0.1)  # Pequeña pausa entre comandos
        
        self.movimientos += 1
        
        # Verificar si llegamos al objetivo
        if self.flood_fill.en_objetivo():
            print("\n¡¡¡OBJETIVO ALCANZADO!!!")
            return False
        
        return True
    
    def explorar_laberinto(self):
        """Ejecuta la fase de exploración del laberinto"""
        print("\n" + "="*50)
        print("INICIANDO EXPLORACIÓN DEL LABERINTO")
        print("="*50)
        
        self.fase = "exploracion"
        self.movimientos = 0
        self.ejecutando = True
        
        # Leer sensores iniciales
        if not self.leer_y_actualizar_sensores():
            print("Error al leer sensores iniciales")
            return
        
        # Calcular distancias iniciales
        self.flood_fill.calcular_distancias()
        
        # Loop principal de exploración
        while self.ejecutando:
            # Actualizar interfaz
            info = f"Fase: {self.fase} | Movimientos: {self.movimientos}"
            self.interfaz.actualizar(info)
            
            # Procesar eventos de interfaz
            if not self.interfaz.procesar_eventos():
                self.detener()
                break
            
            # Ejecutar un paso
            if not self.paso_exploracion():
                self.ejecutando = False
            
            time.sleep(0.2)  # Pausa entre movimientos
        
        print(f"\nExploración finalizada en {self.movimientos} movimientos")
    
    def modo_manual(self):
        """Modo de control manual para pruebas"""
        print("\n" + "="*50)
        print("MODO MANUAL")
        print("="*50)
        print("Comandos: F=Avanzar, L=Izquierda, R=Derecha, S=Sensores, X=Salir")
        
        while True:
            # Actualizar interfaz
            self.interfaz.actualizar("Modo Manual - Esperando comando...")
            
            if not self.interfaz.procesar_eventos():
                break
            
            comando = input("\nComando: ").upper().strip()
            
            if comando == 'X':
                break
            elif comando == 'S':
                self.leer_y_actualizar_sensores()
            elif comando in ['F', 'L', 'R']:
                self.ejecutar_comando(comando)
                if comando == 'F':
                    # Simular movimiento en flood_fill para visualización
                    df, dc = FloodFill.DELTAS[self.flood_fill.direccion_actual]
                    nueva_fila = self.flood_fill.pos_actual[0] + df
                    nueva_col = self.flood_fill.pos_actual[1] + dc
                    if (0 <= nueva_fila < self.flood_fill.filas and 
                        0 <= nueva_col < self.flood_fill.columnas):
                        self.flood_fill.pos_actual = [nueva_fila, nueva_col]
                elif comando == 'L':
                    self.flood_fill.direccion_actual = (self.flood_fill.direccion_actual - 1) % 4
                elif comando == 'R':
                    self.flood_fill.direccion_actual = (self.flood_fill.direccion_actual + 1) % 4
            else:
                print("Comando no reconocido")
    
    def detener(self):
        """Detiene el robot y limpia recursos"""
        print("\nDeteniendo robot...")
        self.ejecutando = False
        self.serial.detener()
        time.sleep(0.5)
    
    def cerrar(self):
        """Cierra todas las conexiones y ventanas"""
        self.detener()
        self.serial.desconectar()
        self.interfaz.cerrar()
        print("Sistema cerrado")

def main():
    """Función principal"""
    # Configurar puerto serial (ajustar según tu sistema)
    # Linux/Mac: '/dev/ttyUSB0' o '/dev/ttyACM0'
    # Windows: 'COM3', 'COM4', etc.
    puerto = '/dev/ttyUSB0'
    
    print("="*50)
    print("MICROMOUSE - SISTEMA DE CONTROL")
    print("="*50)
    
    # Crear controlador
    control = ControlMicromouse(puerto=puerto, baudrate=115200)
    
    try:
        # Conectar con el robot
        if not control.conectar_robot():
            print("Saliendo...")
            return
        
        # Menú principal
        while True:
            print("\n" + "="*50)
            print("MENÚ PRINCIPAL")
            print("="*50)
            print("1. Explorar laberinto (automático)")
            print("2. Modo manual")
            print("3. Leer sensores")
            print("4. Calibrar robot")
            print("5. Salir")
            
            opcion = input("\nSelecciona una opción: ").strip()
            
            if opcion == '1':
                control.explorar_laberinto()
            elif opcion == '2':
                control.modo_manual()
            elif opcion == '3':
                sensores = control.serial.leer_sensores()
                if sensores:
                    print(f"\nSensores:")
                    print(f"  Frontal: {sensores['frontal']} mm")
                    print(f"  Izquierdo: {sensores['izquierdo']} mm")
                    print(f"  Derecho: {sensores['derecho']} mm")
            elif opcion == '4':
                print("Calibrando...")
                if control.serial.calibrar():
                    print("¡Calibración exitosa!")
                else:
                    print("Error en calibración")
            elif opcion == '5':
                break
            else:
                print("Opción no válida")
        
    except KeyboardInterrupt:
        print("\n\nInterrumpido por el usuario")
    except Exception as e:
        print(f"\nError: {e}")
    finally:
        control.cerrar()

if __name__ == "__main__":
    main()
