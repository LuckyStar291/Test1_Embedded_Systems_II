import RPi.GPIO as GPIO
import time
import serial
import smtplib
from datetime import datetime

# --- Configuración inicial ---
GPIO.setwarnings(False)
GPIO.cleanup()

# --- Pines GPIO ---
BUTTON_PIN = 17      # Botón para la bombilla
TRIG_PIN = 23        # Sensor ultrasónico (garaje)
ECHO_PIN = 24
TRIG2_PIN = 25       # Sensor ultrasónico (alarma)
ECHO2_PIN = 8

# --- Variables globales ---
temp = 20.0          # Temperatura inicial
pwmLed1 = 50         # Intensidad LED 1 (0-100%)
pwmLed2 = 50         # Intensidad LED 2 (0-100%)
led_state = 0        # Estado de los LEDs (0=apagados, 1=LED1, 2=LED2, 3=ambos)
alarm_state = False
alarm_start_time = None

# --- Configuración UART ---
try:
    ser = serial.Serial('/dev/ttyACM0', 9600, timeout=1)
    ser.flush()
except Exception as e:
    print(f"Error UART: {e}")
    class DummySerial:
        def write(self, data): print(f"UART Simulado: {data}")
        def readline(self): return b""
    ser = DummySerial()

# --- Setup GPIO ---
GPIO.setmode(GPIO.BCM)
GPIO.setup(BUTTON_PIN, GPIO.IN, pull_up_down=GPIO.PUD_UP)
GPIO.setup(TRIG_PIN, GPIO.OUT)
GPIO.setup(ECHO_PIN, GPIO.IN)
GPIO.setup(TRIG2_PIN, GPIO.OUT)
GPIO.setup(ECHO2_PIN, GPIO.IN)

# --- Funciones principales ---
def measure_distance(trig, echo):
    """Mide distancia con sensor ultrasónico."""
    GPIO.output(trig, True)
    time.sleep(0.00001)
    GPIO.output(trig, False)

    start = time.time()
    while GPIO.input(echo) == 0: start = time.time()
    while GPIO.input(echo) == 1: end = time.time()

    return (end - start) * 34300 / 2  # Distancia en cm

def read_controller():
    """Lee temperatura desde archivo (modificable via SSH)."""
    global temp, pwmLed1, pwmLed2
    try:
        with open("controller.txt", "r") as f:
            for line in f:
                if "Temp(C)=" in line: temp = float(line.split("=")[1])
                elif "Temp(F)=" in line: temp = (float(line.split("=")[1]) - 32) * 5/9
                elif "led1(%)=" in line: pwmLed1 = int(line.split("=")[1])
                elif "led2(%)=" in line: pwmLed2 = int(line.split("=")[1])
    except Exception as e:
        print(f"Error leyendo archivo: {e}")

def control_temperature():
    """Controla ventilador y calentador según temperatura."""
    if temp > 20: ser.write(b"cooler\n")
    elif temp < 2: ser.write(b"heater\n")
    else: ser.write(b"off\n")

def handle_button():
    """Controla bombilla inteligente con botón."""
    global led_state
    if GPIO.input(BUTTON_PIN) == GPIO.LOW:
        time.sleep(0.3)  # Antirrebote
        led_state = (led_state + 1) % 4
        # Envía estado de LEDs e intensidad PWM
        ser.write(f"leds,{led_state},{pwmLed1},{pwmLed2}\n".encode())

def control_alarm():
    """Activa/desactiva alarma con zumbador."""
    global alarm_state, alarm_start_time
    distance = measure_distance(TRIG2_PIN, ECHO2_PIN)
    
    if distance < 7 and not alarm_state:
        alarm_state = True
        alarm_start_time = time.time()
        ser.write(b"alarm_start\n")
    
    if alarm_state:
        elapsed = time.time() - alarm_start_time
        if elapsed >= 10:  # Apagar después de 10 segundos
            alarm_state = False
            ser.write(b"alarm_stop\n")

# --- Bucle principal ---
try:
    while True:
        # 1. Garaje automático
        dist = measure_distance(TRIG_PIN, ECHO_PIN)
        if dist < 7: ser.write(b"garaje_open\n")

        # 2. Control de temperatura
        read_controller()
        control_temperature()

        # 3. Bombilla inteligente
        handle_button()

        # 4. Sistema de alarma
        control_alarm()

        time.sleep(0.1)  # Pequeña pausa

except KeyboardInterrupt:
    print("\nPrograma terminado")
    ser.close()
    GPIO.cleanup()