import socket
import threading
import cv2
from gpiozero import DistanceSensor, Motor, PWMOutputDevice
import time

# Configuración de cámaras
cap1 = cv2.VideoCapture(0)
cap2 = cv2.VideoCapture(2)
cap1.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
cap1.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
cap2.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
cap2.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

# Definición de motores
motor_1 = Motor(forward=17, backward=27)
enable_motor_1 = PWMOutputDevice(13)
motor_2 = Motor(forward=1, backward=7)
enable_motor_2 = PWMOutputDevice(12)

# Definición de sensores ultrasónicos
#sensor_1 = DistanceSensor(echo=24, trigger=23)
sensor_2 = DistanceSensor(echo=8, trigger=25)
sensor_3 = DistanceSensor(echo=20, trigger=21)

# Configuración de socket
bufferSize = 8192 * 4
serverPort = 2223
serverIP = '192.168.0.4'
RPIsocket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
RPIsocket.bind((serverIP, serverPort))

class CameraStreamer(threading.Thread):
    def __init__(self, camera, camera_id, client_address):
        threading.Thread.__init__(self)
        self.camera = camera
        self.camera_id = camera_id
        self.client_address = client_address
        self.running = True

    def run(self):
        while self.running:
            ret, frame = self.camera.read()
            if ret:
                gray_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
                _, img_encoded = cv2.imencode('.jpg', gray_frame, [int(cv2.IMWRITE_JPEG_QUALITY), 80])
                img_bytes = img_encoded.tobytes()

                RPIsocket.sendto(bytes([self.camera_id]), self.client_address)
                for i in range(0, len(img_bytes), bufferSize):
                    chunk = img_bytes[i:i+bufferSize]
                    RPIsocket.sendto(chunk, self.client_address)
                RPIsocket.sendto(b'END', self.client_address)
                time.sleep(0.03)

    def stop(self):
        self.running = False

def medir_distancias():
    """Mide la distancia de los tres sensores y devuelve una cadena con los valores en cm."""
    d1 = 100 #sensor_1.distance * 100
    d2 = sensor_2.distance * 100
    d3 = sensor_3.distance * 100
    return f"{d1:.2f},{d2:.2f},{d3:.2f}"

def controlar_motores(vel_1, vel_2):
    """Controla los motores en función de los valores recibidos."""
    try:
        vel_1, vel_2 = float(vel_1), float(vel_2)
        enable_motor_1.value, enable_motor_2.value = abs(vel_1), abs(vel_2)
        motor_1.forward() if vel_1 > 0 else motor_1.backward() if vel_1 < 0 else motor_1.stop()
        motor_2.forward() if vel_2 > 0 else motor_2.backward() if vel_2 < 0 else motor_2.stop()
    except ValueError:
        print("Comando inválido")

def manejar_cliente(address):
    """Crea hilos para enviar datos de sensores y streams de cámaras a un cliente específico."""
    def enviar_datos_sensores():
        while True:
            distancia_str = medir_distancias().encode('utf-8')
            RPIsocket.sendto(b'SENSORES:' + distancia_str, address)
            time.sleep(0.1)

    threading.Thread(target=enviar_datos_sensores, daemon=True).start()

    streamer1 = CameraStreamer(cap1, 1, address)
    streamer2 = CameraStreamer(cap2, 2, address)
    streamer1.start()
    streamer2.start()

    return streamer1, streamer2

print('Servidor en ejecución...')
client_streams = {}

try:
    while True:
        message, address = RPIsocket.recvfrom(bufferSize)

        if address not in client_streams:
            streamers = manejar_cliente(address)
            client_streams[address] = streamers

        message = message.decode('utf-8')
        if ',' in message:
            controlar_motores(*message.split(','))

except KeyboardInterrupt:
    print("Servidor detenido.")
finally:
    for streamers in client_streams.values():
        for streamer in streamers:
            streamer.stop()
            streamer.join()
    RPIsocket.close()
    cap1.release()
    cap2.release()