import socket
import threading
import numpy as np
import cv2
from cv2 import aruco
import tkinter as tk
from tkinter import ttk
from PIL import Image, ImageTk
from pynput import keyboard

# Configuración de conexión
SERVER_IP = '192.168.0.4'
SERVER_PORT = 2223
BUFFER_SIZE = 65536

# Crear socket UDP
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.sendto(b'INIT', (SERVER_IP, SERVER_PORT))

# Configuración ArUco para OpenCV 4.10
aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_6X6_250)
parameters = aruco.DetectorParameters()
detector = aruco.ArucoDetector(aruco_dict, parameters)

comandos_aruco = {
    1: "Adelante",
    2: "Atrás",
    3: "Izquierda",
    4: "Derecha",
    5: "Stop"
}

# Variables globales
current_camera = None
current_frame = b''
frames = {1: None, 2: None}
running = True
alignment_active = False
forward_active = False
speed_multiplier = 0.3
aruco_active = False
last_aruco_command = None

# Crear ventana principal
root = tk.Tk()
root.title("Control del Robot")
root.geometry("1200x800")

# Variables para los sliders
sensor_vars = {
    "izquierda": tk.DoubleVar(value=0),
    "centro": tk.DoubleVar(value=0),
    "derecha": tk.DoubleVar(value=0)
}

# Frame para las cámaras
cam_frame = tk.Frame(root)
cam_frame.pack(pady=10)

# Labels para las cámaras
label_cam1 = tk.Label(cam_frame)
label_cam1.grid(row=0, column=0, padx=10)
label_cam2 = tk.Label(cam_frame)
label_cam2.grid(row=0, column=1, padx=10)

# Frame para controles
control_frame = tk.Frame(root)
control_frame.pack(pady=10)

# Botones de control
btn_adelante = ttk.Button(control_frame, text="Adelante (W)", command=lambda: enviar_comando_manual('w'))
btn_adelante.grid(row=0, column=1, padx=5, pady=5)

btn_atras = ttk.Button(control_frame, text="Atrás (S)", command=lambda: enviar_comando_manual('s'))
btn_atras.grid(row=2, column=1, padx=5, pady=5)

btn_izquierda = ttk.Button(control_frame, text="Izquierda (A)", command=lambda: enviar_comando_manual('a'))
btn_izquierda.grid(row=1, column=0, padx=5, pady=5)

btn_derecha = ttk.Button(control_frame, text="Derecha (D)", command=lambda: enviar_comando_manual('d'))
btn_derecha.grid(row=1, column=2, padx=5, pady=5)

btn_stop = ttk.Button(control_frame, text="Detener (Q)", command=lambda: enviar_comando_manual('q'))
btn_stop.grid(row=1, column=1, padx=5, pady=5)

# Frame para sensores
sensor_frame = tk.Frame(root)
sensor_frame.pack(pady=10)

# Sliders para sensores
sensor_labels = {}
sensor_sliders = {}

for i, (name, var) in enumerate(sensor_vars.items()):
    tk.Label(sensor_frame, text=f"Sensor {name.capitalize()}:").grid(row=0, column=i, padx=5)
    sensor_labels[name] = tk.Label(sensor_frame, text="0.0 cm")
    sensor_labels[name].grid(row=1, column=i, padx=5)
    
    slider = tk.Scale(sensor_frame, from_=0, to=100, orient=tk.HORIZONTAL, 
                     variable=var, state='disabled', length=200)
    slider.grid(row=2, column=i, padx=5)
    sensor_sliders[name] = slider

# Label para estado de velocidad
speed_label = tk.Label(root, text="Velocidad actual: 30%", font=('Arial', 12))
speed_label.pack(pady=5)

# Label para estado ArUco
aruco_label = tk.Label(root, text="Comando ArUco: Ninguno", font=('Arial', 12))
aruco_label.pack(pady=5)

def actualizar_gui():
    """Actualiza la GUI con los frames de las cámaras."""
    if frames[1] is not None:
        img = cv2.cvtColor(frames[1], cv2.COLOR_BGR2RGB)
        img = Image.fromarray(img)
        img = ImageTk.PhotoImage(image=img)
        label_cam1.imgtk = img
        label_cam1.configure(image=img)
    
    if frames[2] is not None:
        img = cv2.cvtColor(frames[2], cv2.COLOR_BGR2RGB)
        img = Image.fromarray(img)
        img = ImageTk.PhotoImage(image=img)
        label_cam2.imgtk = img
        label_cam2.configure(image=img)
    
    root.after(50, actualizar_gui)

def calcular_distancia(p1, p2):
    return np.sqrt((p2[0] - p1[0])**2 + (p2[1] - p1[1])**2)

def procesar_lineas_hough(frame):
    if frame is None:
        return None, None
    
    frame = cv2.resize(frame, (frame.shape[1] // 2, frame.shape[0] // 2))
    height, width = frame.shape[:2]
    center_x = width // 2
    
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    blurred = cv2.GaussianBlur(gray, (5, 5), 0)
    edges = cv2.Canny(blurred, 100, 200)
    
    lines = cv2.HoughLinesP(edges, 1, np.pi/180, threshold=21, 
                          minLineLength=100, maxLineGap=10)
    
    result_image = frame.copy()
    center_line = None
    deviation = 0
    
    if lines is not None:
        path_lines = []
        for i in range(len(lines)):
            for j in range(i+1, len(lines)):
                x1a, y1a, x2a, y2a = lines[i][0]
                x1b, y1b, x2b, y2b = lines[j][0]
                
                dist_start = calcular_distancia((x1a, y1a), (x1b, y1b))
                dist_end = calcular_distancia((x2a, y2a), (x2b, y2b))
                
                if dist_start < 100 and dist_end < 100:
                    cv2.line(result_image, (x1a, y1a), (x2a, y2a), (0, 255, 0), 2)
                    cv2.line(result_image, (x1b, y1b), (x2b, y2b), (0, 255, 0), 2)
                    
                    xm1 = (x1a + x1b) // 2
                    ym1 = (y1a + y1b) // 2
                    xm2 = (x2a + x2b) // 2
                    ym2 = (y2a + y2b) // 2
                    
                    cv2.line(result_image, (xm1, ym1), (xm2, ym2), (0, 0, 255), 2)
                    center_line = ((xm1, ym1), (xm2, ym2))
                    
                    center_line_mid_x = (xm1 + xm2) // 2
                    deviation = center_line_mid_x - center_x
                    
                    cv2.line(result_image, (center_x, 0), (center_x, height), (255, 0, 0), 1)
                    cv2.putText(result_image, f"Desviacion: {deviation}", (10, 30), 
                               cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
    
    if alignment_active and forward_active and center_line is not None and speed_multiplier > 0 and not aruco_active:
        ajustar_posicion(deviation)
    
    return cv2.resize(result_image, (width*2, height*2)), deviation

def detectar_arucos(frame):
    global last_aruco_command, aruco_active
    
    if frame is None:
        return frame
    
    # Convertir a escala de grises
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    
    # Detectar marcadores ArUco
    corners, ids, rejected = detector.detectMarkers(gray)
    
    if ids is not None:
        # Dibujar los marcadores detectados
        frame = aruco.drawDetectedMarkers(frame, corners, ids)
        
        for i in range(len(ids)):
            id_marcador = ids[i][0]
            comando = comandos_aruco.get(id_marcador, None)
            
            if comando:
                aruco_active = True
                last_aruco_command = comando
                aruco_label.config(text=f"Comando ArUco: {comando}")
                
                # Dibujar texto en el frame
                cv2.putText(frame, f"ArUco: {comando}", (50, 50), 
                           cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
                
                # Enviar comando al servidor
                sock.sendto(f"ARUCO:{comando}".encode(), (SERVER_IP, SERVER_PORT))
    else:
        if aruco_active:
            sock.sendto(b"CLEAR_ARUCO", (SERVER_IP, SERVER_PORT))
            aruco_active = False
            aruco_label.config(text="Comando ArUco: Ninguno")
    
    return frame

def ajustar_posicion(deviation):
    threshold = 30
    
    if deviation < -threshold:
        sock.sendto(f"{0.7*speed_multiplier},{1*speed_multiplier}".encode(), (SERVER_IP, SERVER_PORT))
    elif deviation > threshold:
        sock.sendto(f"{1*speed_multiplier},{0.7*speed_multiplier}".encode(), (SERVER_IP, SERVER_PORT))
    else:
        sock.sendto(f"{1*speed_multiplier},{1*speed_multiplier}".encode(), (SERVER_IP, SERVER_PORT))

def recibir_datos():
    global current_camera, current_frame, frames, speed_multiplier
    
    while running:
        try:
            data, _ = sock.recvfrom(BUFFER_SIZE)
            
            if data.startswith(b'SENSORES:'):
                parts = data[9:].split(b'|')
                distancias = parts[0].decode('utf-8')
                estado = parts[1].decode('utf-8') if len(parts) > 1 else ""
                
                d1, d2, d3 = distancias.split(',')
                
                # Actualizar sliders y labels
                sensor_vars["izquierda"].set(float(d1))
                sensor_vars["centro"].set(float(d2))
                sensor_vars["derecha"].set(float(d3))
                
                sensor_labels["izquierda"].config(text=f"{d1} cm")
                sensor_labels["centro"].config(text=f"{d2} cm")
                sensor_labels["derecha"].config(text=f"{d3} cm")
                
                # Cambiar color según peligro
                for name, dist in zip(["izquierda", "centro", "derecha"], [d1, d2, d3]):
                    dist_float = float(dist)
                    if dist_float < 10:
                        sensor_labels[name].config(fg='red')
                    elif dist_float < 25:
                        sensor_labels[name].config(fg='orange')
                    else:
                        sensor_labels[name].config(fg='green')
                
                if estado.startswith("SPEED:"):
                    new_speed = float(estado.split(':')[1])
                    if new_speed != speed_multiplier:
                        speed_multiplier = new_speed
                        speed_label.config(text=f"Velocidad actual: {speed_multiplier*100:.0f}%")
                
            elif data == b'END':
                if current_camera is not None and current_frame:
                    np_arr = np.frombuffer(current_frame, np.uint8)
                    img = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
                    if img is not None:
                        if current_camera == 1:
                            img = detectar_arucos(img)
                        elif current_camera == 2:
                            img, _ = procesar_lineas_hough(img)
                        frames[current_camera] = img
                current_camera = None
                current_frame = b''
                
            elif len(data) == 1:
                current_camera = int.from_bytes(data, byteorder='big')
            else:
                if current_camera is not None:
                    current_frame += data
                    
        except Exception as e:
            if running:
                print(f"Error en recepción: {e}")

def enviar_comando_manual(tecla):
    global alignment_active, forward_active
    
    if tecla == 'w':
        forward_active = True
        alignment_active = True
        sock.sendto(f"{1*speed_multiplier},{1*speed_multiplier}".encode(), (SERVER_IP, SERVER_PORT))
    elif tecla == 's':
        forward_active = False
        alignment_active = False
        sock.sendto(f"{-1*speed_multiplier},{-1*speed_multiplier}".encode(), (SERVER_IP, SERVER_PORT))
    elif tecla == 'a':
        forward_active = False
        alignment_active = False
        sock.sendto(f"{1*speed_multiplier},{-1*speed_multiplier}".encode(), (SERVER_IP, SERVER_PORT))
    elif tecla == 'd':
        forward_active = False
        alignment_active = False
        sock.sendto(f"{-1*speed_multiplier},{1*speed_multiplier}".encode(), (SERVER_IP, SERVER_PORT))
    elif tecla == 'q':
        forward_active = False
        alignment_active = False
        sock.sendto(b'0,0', (SERVER_IP, SERVER_PORT))

def on_key_press(key):
    try:
        if key.char.lower() in ['w', 'a', 's', 'd', 'q']:
            enviar_comando_manual(key.char.lower())
    except AttributeError:
        pass

def cerrar_aplicacion():
    global running
    running = False
    sock.sendto(b'0,0', (SERVER_IP, SERVER_PORT))
    sock.close()
    root.destroy()

# Configurar cierre limpio
root.protocol("WM_DELETE_WINDOW", cerrar_aplicacion)

# Iniciar hilos
recv_thread = threading.Thread(target=recibir_datos, daemon=True)
recv_thread.start()

keyboard_listener = keyboard.Listener(on_press=on_key_press)
keyboard_listener.start()

# Iniciar actualización de GUI
actualizar_gui()

# Iniciar bucle principal
root.mainloop()

# Limpieza final
running = False
sock.sendto(b'0,0', (SERVER_IP, SERVER_PORT))
sock.close()