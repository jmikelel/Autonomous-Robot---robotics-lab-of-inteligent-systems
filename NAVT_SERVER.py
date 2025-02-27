# Código del Servidor
import socket
import threading
import cv2
import numpy as np
from gpiozero import DistanceSensor, Motor, PWMOutputDevice

"""
Authors: 
    Jose Miguel Gonzalez Zaragoza
    Ricardo de Jesus Cepeda Varela
    David Sebastian Izaguirre Garza
Teacher: Gilberto Alfonso Montez Ramirez
Class: Laboratorio de Robotica y Sistemas Inteligentes
Date: 02/26/2025
Description: 
Robot/Server code. It sends via WiFi the information of various sensors
and recieves instructions to control the DC motors.
"""

cap = cv2.VideoCapture(0)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

# Definición de motores
motor_1 = Motor(forward=17, backward=27)
enable_motor_1 = PWMOutputDevice(13)
motor_2 = Motor(forward=12, backward=7)
enable_motor_2 = PWMOutputDevice(1)

# Definición de sensores ultrasónicos
sensor_1 = DistanceSensor(echo=24, trigger=23)
sensor_2 = DistanceSensor(echo=8, trigger=25)
sensor_3 = DistanceSensor(echo=21, trigger=20)

# Configuración de socket
bufferSize = 8192 * 4
serverPort = 2223
serverIP = '192.168.0.63'
RPIsocket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
RPIsocket.bind((serverIP, serverPort))

def medir_distancias():
    """Mide la distancia de los tres sensores y devuelve una cadena con los valores en cm."""
    d1 = sensor_1.distance * 100
    d2 = sensor_2.distance * 100
    d3 = sensor_3.distance * 100
    return f"{d1:.2f},{d2:.2f},{d3:.2f} cm"

def capturar_imagen_grises():
    """Captura una imagen en escala de grises y la convierte a formato JPEG en bytes."""
    ret, frame = cap.read()
    if ret:
        gray_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        _, img_encoded = cv2.imencode('.jpg', gray_frame)
        return img_encoded.tobytes()
    return None

def enviar_fragmentos(data, address):
    """Envía los datos en fragmentos para evitar pérdidas por el tamaño del buffer."""
    for i in range(0, len(data), bufferSize):
        RPIsocket.sendto(data[i:i + bufferSize], address)

def enviar_datos(address):
    """Envía continuamente los datos de los sensores y la imagen en escala de grises."""
    while True:
        distancia_str = medir_distancias().encode('utf-8')
        img_bytes = capturar_imagen_grises()
        if img_bytes:
            enviar_fragmentos(distancia_str + b'@@' + img_bytes, address)

def manejar_cliente(address):
    """Crea un hilo para enviar datos a un cliente específico."""
    threading.Thread(target=enviar_datos, args=(address,), daemon=True).start()

def controlar_motores(vel_1, vel_2):
    """Controla los motores en función de los valores recibidos."""
    try:
        vel_1, vel_2 = float(vel_1), float(vel_2)
        enable_motor_1.value, enable_motor_2.value = abs(vel_1), abs(vel_2)
        motor_1.forward() if vel_1 > 0 else motor_1.backward() if vel_1 < 0 else motor_1.stop()
        motor_2.forward() if vel_2 > 0 else motor_2.backward() if vel_2 < 0 else motor_2.stop()
    except ValueError:
        print("Comando inválido")

print('Servidor en ejecución...')
try:
    while True:
        message, address = RPIsocket.recvfrom(bufferSize)
        message = message.decode('utf-8')
        if ',' in message:
            controlar_motores(*message.split(','))
        else:
            manejar_cliente(address)
except KeyboardInterrupt:
    print("Servidor detenido.")
finally:
    RPIsocket.close()
    cap.release()
