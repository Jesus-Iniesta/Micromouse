"""
Protocolo de comunicación serial entre ESP32 y computadora
Maneja el envío de comandos y recepción de datos del sensor
"""
import serial
import time
import struct

class ProtocoloSerial:
    """Maneja la comunicación serial con el ESP32"""
    
    # Comandos hacia ESP32
    CMD_AVANZAR = 'F'      # Forward - avanzar una celda
    CMD_RETROCEDER = 'B'   # Back - retroceder una celda
    CMD_GIRAR_IZQ = 'L'    # Left - girar 90° izquierda
    CMD_GIRAR_DER = 'R'    # Right - girar 90° derecha
    CMD_LEER_SENSORES = 'S' # Sensors - leer sensores
    CMD_DETENER = 'X'      # Stop - detener motores
    CMD_CALIBRAR = 'C'     # Calibrate - calibrar posición
    
    # Respuestas del ESP32
    RESP_OK = 'OK'
    RESP_ERROR = 'ERROR'
    RESP_SENSORES = 'SENS'
    
    def __init__(self, puerto='/dev/ttyUSB0', baudrate=115200):
        """
        Inicializa la conexión serial
        
        Args:
            puerto: Puerto serial (ej: '/dev/ttyUSB0' en Linux, 'COM3' en Windows)
            baudrate: Velocidad de comunicación
        """
        self.puerto = puerto
        self.baudrate = baudrate
        self.serial = None
        self.conectado = False
        
    def conectar(self):
        """Establece conexión con el ESP32"""
        try:
            self.serial = serial.Serial(
                port=self.puerto,
                baudrate=self.baudrate,
                timeout=1.0,
                write_timeout=1.0
            )
            time.sleep(2)  # Esperar reset del ESP32
            self.conectado = True
            print(f"Conectado a {self.puerto} a {self.baudrate} baudios")
            return True
        except serial.SerialException as e:
            print(f"Error al conectar: {e}")
            self.conectado = False
            return False
    
    def desconectar(self):
        """Cierra la conexión serial"""
        if self.serial and self.serial.is_open:
            self.serial.close()
            self.conectado = False
            print("Desconectado")
    
    def enviar_comando(self, comando):
        """
        Envía un comando al ESP32
        
        Args:
            comando: Carácter del comando a enviar
            
        Returns:
            True si se envió correctamente
        """
        if not self.conectado:
            print("No hay conexión")
            return False
            
        try:
            self.serial.write(comando.encode())
            self.serial.flush()
            return True
        except Exception as e:
            print(f"Error al enviar comando: {e}")
            return False
    
    def leer_respuesta(self, timeout=2.0):
        """
        Lee respuesta del ESP32
        
        Args:
            timeout: Tiempo máximo de espera en segundos
            
        Returns:
            String con la respuesta o None si hay error
        """
        if not self.conectado:
            return None
            
        self.serial.timeout = timeout
        try:
            linea = self.serial.readline().decode('utf-8').strip()
            return linea
        except Exception as e:
            print(f"Error al leer respuesta: {e}")
            return None
    
    def leer_sensores(self):
        """
        Lee los datos de los sensores de distancia
        
        Returns:
            dict con {'frontal': mm, 'izquierdo': mm, 'derecho': mm}
            o None si hay error
        """
        if not self.enviar_comando(self.CMD_LEER_SENSORES):
            return None
            
        respuesta = self.leer_respuesta()
        if not respuesta:
            return None
            
        # Formato esperado: "SENS:F=123,L=456,R=789"
        try:
            if respuesta.startswith('SENS:'):
                datos = respuesta[5:].split(',')
                sensores = {}
                for dato in datos:
                    clave, valor = dato.split('=')
                    sensores[clave] = int(valor)
                
                return {
                    'frontal': sensores.get('F', 0),
                    'izquierdo': sensores.get('L', 0),
                    'derecho': sensores.get('R', 0)
                }
        except Exception as e:
            print(f"Error al parsear sensores: {e}")
            return None
    
    def avanzar(self):
        """Avanza una celda y espera confirmación"""
        if self.enviar_comando(self.CMD_AVANZAR):
            respuesta = self.leer_respuesta(timeout=5.0)
            return respuesta == self.RESP_OK
        return False
    
    def retroceder(self):
        """Retrocede una celda y espera confirmación"""
        if self.enviar_comando(self.CMD_RETROCEDER):
            respuesta = self.leer_respuesta(timeout=5.0)
            return respuesta == self.RESP_OK
        return False
    
    def girar_izquierda(self):
        """Gira 90° a la izquierda y espera confirmación"""
        if self.enviar_comando(self.CMD_GIRAR_IZQ):
            respuesta = self.leer_respuesta(timeout=3.0)
            return respuesta == self.RESP_OK
        return False
    
    def girar_derecha(self):
        """Gira 90° a la derecha y espera confirmación"""
        if self.enviar_comando(self.CMD_GIRAR_DER):
            respuesta = self.leer_respuesta(timeout=3.0)
            return respuesta == self.RESP_OK
        return False
    
    def detener(self):
        """Detiene los motores inmediatamente"""
        return self.enviar_comando(self.CMD_DETENER)
    
    def calibrar(self):
        """Calibra la posición del robot"""
        if self.enviar_comando(self.CMD_CALIBRAR):
            respuesta = self.leer_respuesta(timeout=5.0)
            return respuesta == self.RESP_OK
        return False
