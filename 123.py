import RPi.GPIO as GPIO
import time
import serial

# --- Configuración inicial ---
GPIO.setwarnings(False)  # Ignorar advertencias de GPIO
GPIO.cleanup()           # Limpiar configuraciones previas

# --- Pines GPIO ---
TRIG_PIN = 23        # Trigger del sensor ultrasónico (garaje)
ECHO_PIN = 24        # Echo del sensor ultrasónico (garaje)

# --- Configuración UART (comunicación con Tiva) ---
try:
    ser = serial.Serial('/dev/ttyACM0', 115200, timeout=1)  
    ser.flush()
except Exception as e:
    print(f"Error al conectar con la Tiva: {e}")
    # Simulador para pruebas
    class DummySerial:
        def write(self, data):
            print(f"Simulando UART: {data}")
        def readline(self):
            return b""
    ser = DummySerial()

# --- Configuración de GPIO ---
GPIO.setmode(GPIO.BCM)
GPIO.setup(TRIG_PIN, GPIO.OUT)
GPIO.setup(ECHO_PIN, GPIO.IN)

# --- Variables globales ---
temp = 20.0  # Temperatura inicial (se actualiza desde controller.txt)

# --- Funciones principales ---
def measure_distance():
    """Mide la distancia con el sensor ultrasónico (en cm)."""
    GPIO.output(TRIG_PIN, True)
    time.sleep(0.00001)  # Pulso de 10µs
    GPIO.output(TRIG_PIN, False)

    start_time = time.time()
    end_time = time.time()

    # Espera el inicio del eco
    while GPIO.input(ECHO_PIN) == 0:
        start_time = time.time()

    # Espera el fin del eco
    while GPIO.input(ECHO_PIN) == 1:
        end_time = time.time()

    pulse_duration = end_time - start_time
    distance = (pulse_duration * 34300) / 2  # Distancia en cm
    return distance if distance > 0 else 999  # Retorna 999 si hay error

def read_temperature():
    """Lee la temperatura desde controller.txt (puede ser en °C o °F)."""
    global temp
    try:
        with open("controller.txt", "r") as file:
            for line in file:
                if "Temp(C)=" in line:
                    temp = float(line.split("=")[1])  # Lee temperatura en °C
                elif "Temp(F)=" in line:
                    temp = (float(line.split("=")[1]) - 32) * 5/9  # Convierte °F a °C
    except Exception as e:
        print(f"Error al leer temperatura: {e}")

def control_temperature():
    """Envía comandos a la Tiva según la temperatura."""
    if temp > 20:
        ser.write(b"cooler\n")  # Enciende ventilador
    elif temp < 2:
        ser.write(b"heater\n")  # Enciende LED rojo (calentador)
    else:
        ser.write(b"off\n")     # Apaga ambos

# --- Bucle principal ---
try:
    while True:
        # 1. Garaje automático
        distance = measure_distance()
        if distance < 7:
            ser.write(b"garaje\n")  # Levanta la puerta
            print(f"Objeto detectado: {distance:.2f} cm")

        # 2. Control de temperatura
        read_temperature()
        control_temperature()
        print(f"Temperatura actual: {temp:.2f} °C")

        time.sleep(1)  # Espera 1 segundo entre iteraciones

except KeyboardInterrupt:
    print("\nPrograma terminado por el usuario")
    ser.close()
    GPIO.cleanup()