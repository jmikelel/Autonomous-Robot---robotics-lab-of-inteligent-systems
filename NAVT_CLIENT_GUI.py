import socket
import threading
import numpy as np
import cv2
import time
import tkinter as tk
from tkinter import Label, Button
from PIL import Image, ImageTk
from pynput import keyboard


"""
Author: Jose Miguel Gonzalez Zaragoza
Teacher: Gilberto Alfonso Montez Ramirez
Class: Laboratorio de Robotica y Sistemas Inteligentes
Date: 02/26/2025
Description: 
Controller of autonomous robot (Kiva like). It has a GUI that shows the data the robot
sends via WiFi (Sensors and Camera) and processes that information.
"""

serverAddress = ('192.168.0.4', 2223)
bufferSize = 8192 * 4

UDPClient = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
msgFromClient = 'Hola desde el cliente, listo para recibir datos'
UDPClient.sendto(msgFromClient.encode('utf-8'), serverAddress)

navt_interval = 0.5
alpha = 0.2
min_interval = 0.1
max_interval = 1.0

root = tk.Tk()
root.title("Interfaz de Procesamiento de Imágenes")

frame_original = Label(root)
frame_original.grid(row=0, column=0)
frame_filtro = Label(root)
frame_filtro.grid(row=0, column=1)
frame_hough = Label(root)
frame_hough.grid(row=1, column=0)
frame_datos = Label(root, text="Datos del Sensor: \nTecla presionada: ", 
                    font=("Arial", 12), anchor='w', justify='left')
frame_datos.grid(row=1, column=1)

button_exit = Button(root, text="Cerrar", command=root.quit)
button_exit.grid(row=2, column=1)

tecla_presionada = ""

def recibir_fragmentos():
    data = b''
    while True:
        fragment, _ = UDPClient.recvfrom(bufferSize)
        data += fragment
        if len(fragment) < bufferSize:
            break
    return data

def procesar_datos(data):
    try:
        partes = data.split(b'@@', 1)
        if len(partes) == 2:
            frame_datos["text"] = f"Datos del Sensor: {partes[0].decode('utf-8')}\nTecla presionada: {tecla_presionada}"
            img_bytes = partes[1]
            np_arr = np.frombuffer(img_bytes, np.uint8)
            img = cv2.imdecode(np_arr, cv2.IMREAD_GRAYSCALE)
            if img is not None:
                edgesI, rI = aplicar_transformada_hough(img)
                mostrar_imagen(img, frame_original)
                mostrar_imagen(edgesI, frame_filtro)
                mostrar_imagen(rI, frame_hough)
    except Exception as e:
        print(f"Error al procesar datos: {e}")

def recibir_datos():
    global navt_interval
    while True:
        start_time = time.time()
        UDPClient.sendto(b'?', serverAddress)
        data = recibir_fragmentos()
        procesar_datos(data)
        rtt = time.time() - start_time
        navt_interval = max(min((1 - alpha) * navt_interval + alpha * rtt, 
                                max_interval), min_interval)
        time.sleep(navt_interval)

def calculate_distance(p1, p2):
    return np.sqrt((p2[0] - p1[0])**2 + (p2[1] - p1[1])**2)

def aplicar_transformada_hough(img):
    blurImage = cv2.GaussianBlur(img, (5, 5), 0)
    edgesImage = cv2.Canny(blurImage, 100, 200)
    resultImage = cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)
    linesP = cv2.HoughLinesP(edgesImage, 
                             1, np.pi/180, threshold=21, 
                             minLineLength=100, 
                             maxLineGap=10)
    
    if linesP is not None:
        for i in range(len(linesP)):
            for j in range(i + 1, len(linesP)):
                x1a, y1a, x2a, y2a = linesP[i][0]
                x1b, y1b, x2b, y2b = linesP[j][0]
                distance1 = calculate_distance((x1a, y1a), (x1b, y1b))
                distance2 = calculate_distance((x2a, y2a), (x2b, y2b))
                
                if distance1 < 100 and distance2 < 100:
                    cv2.line(resultImage, (x1a, y1a), (x2a, y2a), (0, 255, 0), 2)
                    cv2.line(resultImage, (x1b, y1b), (x2b, y2b), (0, 255, 0), 2)
                    xm1, ym1 = (x1a + x1b) // 2, (y1a + y1b) // 2
                    xm2, ym2 = (x2a + x2b) // 2, (y2a + y2b) // 2
                    cv2.line(resultImage, (xm1, ym1), (xm2, ym2), (0, 0, 255), 2)
    
    return edgesImage, resultImage

def mostrar_imagen(img, frame):
    img = cv2.resize(img, (300, 300))
    img = Image.fromarray(img)
    img = ImageTk.PhotoImage(img)
    frame.configure(image=img)
    frame.image = img

def enviar_comandos(key):
    global tecla_presionada
    try:
        if key.char == 'w':
            UDPClient.sendto(b'1,1', serverAddress)
        elif key.char == 's':
            UDPClient.sendto(b'-1,-1', serverAddress)
        elif key.char == 'a':
            UDPClient.sendto(b'1,-1', serverAddress)
        elif key.char == 'd':
            UDPClient.sendto(b'-1,1', serverAddress)
        elif key.char == 'q':
            UDPClient.sendto(b'0,0', serverAddress)
        tecla_presionada = key.char
        frame_datos["text"] = f"Datos del Sensor:\nTecla presionada: {tecla_presionada}"
    except AttributeError:
        pass

listener = keyboard.Listener(on_press=enviar_comandos)
listener.start()

hilo_recepcion = threading.Thread(target=recibir_datos, daemon=True)
hilo_recepcion.start()

root.mainloop()

try:
    listener.join()
except KeyboardInterrupt:
    print("Cliente detenido.")
    UDPClient.close()
    cv2.destroyAllWindows()
