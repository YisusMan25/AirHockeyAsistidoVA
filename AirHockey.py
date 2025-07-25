import cv2
import numpy as np
import serial
import time

# Configura el puerto serial
SERIAL_PORT = 'COM11'
BAUD_RATE = 115200
cap = cv2.VideoCapture(0)

# Punto físico inicial real del robot
PUNTO_FISICO_INICIAL = (0.0, 0.0)
ORIGEN_VIRTUAL = (0.0, 0.0)
robot_fis = (0.0, 0.0)

# Rango de color HSV para disco rojo
lower_red1 = np.array([0, 70, 50])
upper_red1 = np.array([10, 255, 255])
lower_red2 = np.array([170, 70, 50])
upper_red2 = np.array([180, 255, 255])

trayectoria = []
max_trayectoria = 100
prev_center = None
zona_vision = []
zona_robot = []
zona_actual = []
seleccionando = 0
robot_pos = None
ultima_posicion_enviada = None

# Conexión con GRBL (una vez)
def conectar_grbl(puerto, baud):
    try:
        ser = serial.Serial(puerto, baud, timeout=1)
        time.sleep(2)
        ser.flushInput()
        print("[✓] Conectado a GRBL")
        return ser
    except Exception as e:
        print("[✗] Error de conexión:", e)
        return None

def enviar_gcode(ser, comando):
    comando = comando.strip() + '\n'
    ser.write(comando.encode())
    while ser.in_waiting:
        print(f"[←] {ser.readline().decode().strip()}")

def seleccionar_zonas(event, x, y, flags, param):
    global zona_actual, seleccionando, zona_vision, zona_robot
    if event == cv2.EVENT_LBUTTONDOWN:
        zona_actual = [(x, y)]
    elif event == cv2.EVENT_LBUTTONUP:
        zona_actual.append((x, y))
        if seleccionando == 0:
            zona_vision = zona_actual
            seleccionando = 1
        elif seleccionando == 1:
            zona_robot = zona_actual
            cv2.destroyAllWindows()


ret, frame = cap.read()
if not ret:
    print("No se pudo acceder a la cámara")
    cap.release()
    exit()

clone = frame.copy()
cv2.namedWindow("Selecciona zonas")
cv2.setMouseCallback("Selecciona zonas", seleccionar_zonas)

while True:
    temp = clone.copy()
    if len(zona_vision) == 2:
        cv2.rectangle(temp, zona_vision[0], zona_vision[1], (0, 255, 0), 2)
        cv2.putText(temp, "Zona de vision", (zona_vision[0][0], zona_vision[0][1] - 10),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
    if len(zona_robot) == 2:
        cv2.rectangle(temp, zona_robot[0], zona_robot[1], (255, 0, 0), 2)
        cv2.putText(temp, "Zona XY Robot", (zona_robot[0][0], zona_robot[0][1] - 10),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 0, 0), 2)

    cv2.imshow("Selecciona zonas", temp)
    if seleccionando > 1:
        break
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

x1, y1 = min(zona_vision[0][0], zona_vision[1][0]), min(zona_vision[0][1], zona_vision[1][1])
x2, y2 = max(zona_vision[0][0], zona_vision[1][0]), max(zona_vision[0][1], zona_vision[1][1])
xr1, yr1 = min(zona_robot[0][0], zona_robot[1][0]), min(zona_robot[0][1], zona_robot[1][1])
xr2, yr2 = max(zona_robot[0][0], zona_robot[1][0]), max(zona_robot[0][1], zona_robot[1][1])
robot_pos = (0, 0)


def predecir_trayectoria_rebote(puntos, pasos=5, rebotes=3):
    if len(puntos) < 2:
        return []

    p1 = puntos[-2]
    p2 = puntos[-1]
    dx = p2[0] - p1[0]
    dy = p2[1] - p1[1]

    if abs(dx) < 2 and abs(dy) < 2:
        return []

    norm = np.sqrt(dx**2 + dy**2)
    if norm == 0:
        return []

    step_size = 20
    vx = dx / norm * step_size
    vy = dy / norm * step_size

    pred_points = [p2]
    x, y = p2

    for _ in range(rebotes * pasos):
        x += vx
        y += vy

        if x <= 0 or x >= (x2 - x1):
            vx = -vx
            x = max(0, min(x, x2 - x1))
        if y <= 0 or y >= (y2 - y1):
            vy = -vy
            y = max(0, min(y, y2 - y1))

        pred_points.append((int(x), int(y)))

    return pred_points


def mapear_punto_robot(punto, zona_vision, zona_robot, limites_robot):
    x, y = punto
    (vx1, vy1), (vx2, vy2) = zona_vision
    (rx1, ry1), (rx2, ry2) = zona_robot
    xmin, xmax, ymin, ymax = limites_robot

    fx = (x - vx1) / (vx2 - vx1)
    fy = (y - vy1) / (vy2 - vy1)

    x_fis = xmin + fx * (xmax - xmin)
    y_fis = ymin + fy * (ymax - ymin)

    x_px = int(rx1 + fx * (rx2 - rx1))
    y_px = int(ry1 + fy * (ry2 - ry1))

    return (x_px, y_px), (x_fis, y_fis)

# Solo una vez, al inicio del seguimiento
if 'grbl' not in globals():
    grbl = conectar_grbl(SERIAL_PORT, BAUD_RATE)
    if grbl:
        enviar_gcode(grbl, "$X")    # Desbloqueo
        enviar_gcode(grbl, "G20")   # Unidades en pulgadas
        enviar_gcode(grbl, "G92 X0 Y0")  # Fija posición actual como (0,0)
        enviar_gcode(grbl, "G90")   # Modo absoluto

while True:
    ret, frame = cap.read()
    if not ret:
        break

    zona_trabajo = frame[y1:y2, x1:x2]
    hsv = cv2.cvtColor(zona_trabajo, cv2.COLOR_BGR2HSV)

    mask1 = cv2.inRange(hsv, lower_red1, upper_red1)
    mask2 = cv2.inRange(hsv, lower_red2, upper_red2)
    mask = cv2.bitwise_or(mask1, mask2)
    mask = cv2.GaussianBlur(mask, (9, 9), 2)

    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    direction_text = "Esperando disco..."

    for cnt in contours:
        area = cv2.contourArea(cnt)
        if area > 200:
            (x, y), radius = cv2.minEnclosingCircle(cnt)
            center = (int(x), int(y))
            radius = int(radius)

            if 10 < radius < 100:
                center_global = (center[0] + x1, center[1] + y1)
                cv2.circle(frame, center_global, radius, (0, 255, 0), 2)
                cv2.circle(frame, center_global, 3, (0, 0, 255), -1)

                if prev_center is not None:
                    dx = center[0] - prev_center[0]
                    dy = center[1] - prev_center[1]
                    if abs(dx) > abs(dy):
                        if dx > 5:
                            direction_text = "→ Derecha"
                        elif dx < -5:
                            direction_text = "← Izquierda"
                    else:
                        if dy > 5:
                            direction_text = "↓ Abajo"
                        elif dy < -5:
                            direction_text = "↑ Arriba"

                prev_center = center
                trayectoria.append(center)
                if len(trayectoria) > max_trayectoria:
                    trayectoria.pop(0)

                robot_pos, robot_fis = mapear_punto_robot((center[0] + x1, center[1] + y1),
                                                          (zona_vision[0], zona_vision[1]),
                                                          (zona_robot[0], zona_robot[1]),
                                                          (0, 0.180, 0, 0.09))
                break

    for i in range(1, len(trayectoria)):
        pt1 = (trayectoria[i - 1][0] + x1, trayectoria[i - 1][1] + y1)
        pt2 = (trayectoria[i][0] + x1, trayectoria[i][1] + y1)
        cv2.line(frame, pt1, pt2, (255, 0, 0), 2)

    puntos_pred = predecir_trayectoria_rebote(trayectoria)
    for i in range(1, len(puntos_pred)):
        pt1 = (puntos_pred[i - 1][0] + x1, puntos_pred[i - 1][1] + y1)
        pt2 = (puntos_pred[i][0] + x1, puntos_pred[i][1] + y1)
        cv2.line(frame, pt1, pt2, (0, 255, 255), 2)

    cv2.rectangle(frame, (xr1, yr1), (xr2, yr2), (255, 0, 0), 2)
    cv2.putText(frame, "Zona Robot", (xr1, yr1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 0, 0), 2)
    if robot_pos is not None:
        cv2.circle(frame, robot_pos, 6, (0, 0, 255), -1)
        cv2.putText(frame, "Robot", (robot_pos[0] + 10, robot_pos[1]),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)
        if robot_fis is not None:
            texto_fis = f"X={robot_fis[0]:.3f} pulg, Y={robot_fis[1]:.3f} pulg"
            cv2.putText(frame, texto_fis, (robot_pos[0] + 10, robot_pos[1] + 20),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (200, 255, 200), 1)

    cv2.putText(frame, f"Direccion: {direction_text}", (10, 30),
                cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2)

    cv2.imshow("Deteccion de Disco", frame)
    cv2.imshow("Mascara", mask)

    # Enviar a GRBL cuando se detecta un nuevo punto
    if robot_pos and robot_fis:
        x_envio, y_envio = robot_fis
        gcode = f"G1 X{x_envio:.3f} Y{y_envio:.3f} F50"
        enviar_gcode(grbl, gcode)
        print(f"[✓] Enviado a GRBL: {gcode}")
        ultima_posicion_enviada = (x_envio, y_envio)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break
cap.release()
cv2.destroyAllWindows()