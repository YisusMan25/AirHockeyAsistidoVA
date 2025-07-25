import serial
import time

# Configura el puerto serial
SERIAL_PORT = 'COM13'
BAUD_RATE = 115200

# Punto físico inicial real del robot
PUNTO_FISICO_INICIAL = (0.0, 0.0)
ORIGEN_VIRTUAL = (0.0, 0.0)

# Conexión al GRBL
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

# Enviar comando al GRBL
def enviar_gcode(ser, comando):
    comando = comando.strip() + '\n'
    ser.write(comando.encode())
    respuesta = ser.readline().decode().strip()
    print(f"[→] Enviado: {comando.strip()} | [←] Respuesta: {respuesta}")

# Programa principal
def main():
    ser = conectar_grbl(SERIAL_PORT, BAUD_RATE)
    if ser is None:
        return

    enviar_gcode(ser, "$X")    # Desbloquea
    enviar_gcode(ser, "G20")   # Pulgadas
    enviar_gcode(ser, "G91")   # Relativo

    print("\n[!] Punto físico inicial =", PUNTO_FISICO_INICIAL, "→ Considerado como (0, 0) virtual.")
    print("[!] Las coordenadas que escribas serán ABSOLUTAS desde ese nuevo origen virtual.\n")

    # Posición actual virtual del robot (empieza en el origen virtual)
    x_actual = 0.0
    y_actual = 0.0

    while True:
        try:
            entrada = input("Ingresa coordenadas X Y (o 'salir'): ").strip()
            if entrada.lower() == 'salir':
                break

            x_destino, y_destino = map(float, entrada.split())

            # Calcular delta relativo al punto actual virtual
            delta_x = x_destino - x_actual
            delta_y = y_destino - y_actual

            # Enviar comando relativo
            gcode = f"G1 X{delta_x:.4f} Y{delta_y:.4f} F500"
            enviar_gcode(ser, gcode)

            # Actualizar posición virtual actual
            x_actual = x_destino
            y_actual = y_destino

        except ValueError:
            print("[!] Entrada inválida. Usa el formato: X Y")
        except KeyboardInterrupt:
            print("\n[!] Interrumpido por el usuario.")
            break

    ser.close()
    print("[✓] Conexión cerrada.")

if __name__ == "__main__":
    main()
