import RPi.GPIO as GPIO
GPIO.setwarnings(False)  # Ignora advertencias de pines GPIO ya en uso (evita mensajes molestos)
import time
import serial            # Para comunicación UART con la Tiva
import smtplib          # Para enviar emails (alarma)
from datetime import datetime  # Para registrar tiempos de activación

# --- Configuración de pines ---
GPIO.cleanup()  # Limpia configuraciones previas de GPIO (evita conflictos)
BUTTON_PIN = 17      # Pin GPIO para el botón de la bombilla inteligente
TRIG_PIN = 23        # Pin GPIO para el trigger del sensor ultrasónico (garaje)
ECHO_PIN = 24        # Pin GPIO para el echo del sensor ultrasónico (garaje)
TRIG2_PIN = 25       # Pin GPIO para el trigger del sensor de alarma (¡diferente al garaje!)
ECHO2_PIN = 8        # Pin GPIO para el echo del sensor de alarma

# --- Variables globales ---
flag_led = 0          # Bandera para cambios en LEDs
flag_temp = None      # Guarda el último estado de temperatura (evita enviar comandos repetidos)
temp = 0              # Temperatura actual (leída desde controller.txt)
pwmLed1 = 0           # Intensidad del LED 1 (0-100%)
pwmLed2 = 0           # Intensidad del LED 2 (0-100%)
led1 = 0              # Estado del LED 1 (0=apagado, 1=encendido)
led2 = 0              # Estado del LED 2 (0=apagado, 1=encendido)
pwmLed1_prev = 0      # Valor anterior de pwmLed1 (para detectar cambios)
pwmLed2_prev = 0      # Valor anterior de pwmLed2 (para detectar cambios)
estado_leds = 0       # Estado combinado de LEDs (0=apagados, 1=LED1, 2=LED2, 3=ambos)
ALARM_STATE = "OFF"   # Estado de la alarma ("ON" o "OFF")
timestamp = None      # Marca de tiempo cuando se activa la alarma
tiempo_activado = 0   # Duración de la alarma en segundos
metodo = ""           # Cómo se desactivó la alarma (ej: "SSH" o "objeto alejado")
changes1 = 0          # Bandera para indicar cambios en los LEDs

# --- Configuración de GPIO ---
GPIO.setmode(GPIO.BCM)  # Usar numeración BCM (Broadcom) para los pines
GPIO.setup(BUTTON_PIN, GPIO.IN, pull_up_down=GPIO.PUD_UP)  # Botón con resistencia pull-up
GPIO.setup(TRIG_PIN, GPIO.OUT)    # Configura TRIG_PIN como salida (sensor garaje)
GPIO.setup(ECHO_PIN, GPIO.IN)     # Configura ECHO_PIN como entrada (sensor garaje)
GPIO.setup(TRIG2_PIN, GPIO.OUT)   # Configura TRIG2_PIN como salida (sensor alarma)
GPIO.setup(ECHO2_PIN, GPIO.IN)    # Configura ECHO2_PIN como entrada (sensor alarma)

# --- Configuración UART (Comunicación con Tiva) ---
try:
    ser = serial.Serial('/dev/ttyACM0', 9600, timeout=1)  # Intenta conectar con la Tiva
    ser.flush()  # Limpia el buffer de serial
except Exception as e:
    print(f"NO SE PUDO CONECTAR A LA TIVA: {e}")
    # Simulador de UART para pruebas (no bloquea el programa si la Tiva no está conectada)
    class DummySerial:
        def write(self, data):
            print(f"Simulando envío UART: {data}")
        def readline(self):
            return b""
        def close(self):
            pass
    ser = DummySerial()

# --- Funciones ---
def read_controller():
    """Lee el archivo 'controller.txt' para actualizar variables como temperatura y estados de LEDs."""
    global temp, pwmLed1, pwmLed2, estado_leds, ALARM_STATE
    try:
        with open("controller.txt", "r") as file:
            for line in file:
                if "Temp(C)=" in line:
                    temp = float(line.strip().split("=")[1])  # Lee temperatura en Celsius
                elif "Temp(F)=" in line:
                    temp = (float(line.strip().split("=")[1]) - 32) * 5/9  # Convierte Fahrenheit a Celsius
                elif "led1(%)=" in line:
                    pwmLed1 = float(line.strip().split("=")[1])  # Lee intensidad LED1
                elif "led2(%)=" in line:
                    pwmLed2 = float(line.strip().split("=")[1])  # Lee intensidad LED2
                elif "ALARM=" in line:
                    ALARM_STATE = line.strip().split("=")[1]  # Lee estado de la alarma
    except Exception as e:
        print("Error al leer el archivo:", e)

def validate_values():
    """Asegura que los valores de los LEDs estén en rangos válidos (0-100%)."""
    global pwmLed1, pwmLed2, changes1, pwmLed1_prev, pwmLed2_prev
    try:
        pwmLed1 = max(0, min(100, int(pwmLed1)))  # Fuerza el rango 0-100%
        pwmLed2 = max(0, min(100, int(pwmLed2)))
        if pwmLed1 != pwmLed1_prev or pwmLed2 != pwmLed2_prev:
            changes1 = 1  # Marca que hubo cambios
            pwmLed1_prev, pwmLed2_prev = pwmLed1, pwmLed2  # Guarda valores actuales
    except ValueError:
        print("Valores inválidos en el archivo")

def measure_distance(trig_pin, echo_pin):
    """Mide distancia con sensor ultrasónico (en cm)."""
    GPIO.output(trig_pin, True)
    time.sleep(0.00001)  # Pulso de 10µs
    GPIO.output(trig_pin, False)
    
    pulse_start, pulse_end = time.time(), time.time()
    timeout = time.time() + 0.1  # Timeout para evitar bucles infinitos
    
    while GPIO.input(echo_pin) == 0 and time.time() < timeout:
        pulse_start = time.time()  # Espera el inicio del eco
    while GPIO.input(echo_pin) == 1 and time.time() < timeout:
        pulse_end = time.time()    # Espera el fin del eco
    
    pulse_duration = pulse_end - pulse_start
    distance = (pulse_duration * 34300) / 2  # Calcula distancia (velocidad del sonido)
    return distance if distance > 0 else 999  # Retorna 999 si hay error

def check_temperature():
    """Envía comandos a la Tiva según la temperatura (cooler/heater/off)."""
    global flag_temp
    if temp > 20:
        message = "cooler"  # Enciende ventilador si >20°C
    elif temp < 2:
        message = "heater"  # Enciende calentador si <2°C
    else:
        message = "off"     # Apaga ambos
    if message != flag_temp:  # Solo envía si hubo cambio
        ser.write((message + "\n").encode())
        flag_temp = message

def button_callback():
    """Cambia el estado de los LEDs con el botón (sin interrupciones)."""
    global estado_leds, changes1, led1, led2
    button_state = GPIO.input(BUTTON_PIN)
    if button_state == GPIO.LOW:  # Botón presionado (LOW por pull-up)
        time.sleep(0.3)  # Antirrebote
        estado_leds = (estado_leds + 1) % 4  # Cicla entre 0-3
        if estado_leds == 0:
            led1, led2 = 0, 0  # Apaga ambos
        elif estado_leds == 1:
            led1, led2 = 1, 0   # Enciende LED1
        elif estado_leds == 2:
            led1, led2 = 0, 1   # Enciende LED2
        elif estado_leds == 3:
            led1, led2 = 1, 1   # Enciende ambos
        changes1 = 1  # Indica que hubo cambios

def control_alarm():
    """Activa/desactiva la alarma según el sensor de proximidad."""
    global ALARM_STATE, timestamp, tiempo_activado, metodo
    distance = measure_distance(TRIG2_PIN, ECHO2_PIN)
    
    if distance < 7 and ALARM_STATE == "OFF":
        timestamp = datetime.now()  # Registra hora de activación
        ALARM_STATE = "ON"
        ser.write(b"alarm\n")       # Envía comando a la Tiva
        update_alarm_state("ON")    # Actualiza archivo
    
    elif distance >= 7 and ALARM_STATE == "ON":
        tiempo_activado = (datetime.now() - timestamp).total_seconds()
        metodo = "El objeto fue alejado"
        detener_alarma()

def update_alarm_state(state):
    """Actualiza el estado de la alarma en 'controller.txt'."""
    with open("controller.txt", "r+") as file:
        lines = file.readlines()
        file.seek(0)
        for line in lines:
            if "ALARM=" in line:
                file.write(f"ALARM={state}\n")  # Escribe nuevo estado
            else:
                file.write(line)
        file.truncate()

def detener_alarma():
    """Apaga la alarma y envía email."""
    global ALARM_STATE
    ALARM_STATE = "OFF"
    ser.write(b"alarmoff\n")  # Envía comando a la Tiva
    update_alarm_state("OFF") # Actualiza archivo
    enviar_email()            # Notifica por correo

def enviar_email():
    """Envía un correo con detalles de la alarma."""
    try:
        s = smtplib.SMTP('smtp.gmail.com', 587)
        s.starttls()
        s.login("luz.soria@ucb.edu.bo", "twaouqgmekpjvtul")  # Credenciales (¡cambiar en producción!)
        message = f"Alarma activada a las {timestamp}. Duración: {tiempo_activado:.2f}s. Apagada por: {metodo}."
        s.sendmail("luz.soria@ucb.edu.bo", "dluz.soria@ucb.edu.bo", message)
        s.quit()
        print("Correo enviado!")
    except Exception as e:
        print("Error al enviar email:", e)

# --- Bucle principal ---
try:
    while True:
        read_controller()       # Lee temperatura y estados desde archivo
        validate_values()       # Valida rangos de valores
        button_callback()       # Verifica botón de bombilla
        control_alarm()         # Controla alarma
        
        # Garaje automático
        distance_garaje = measure_distance(TRIG_PIN, ECHO_PIN)
        if distance_garaje < 7:
            ser.write(b"garaje\n")  # Levanta puerta si objeto está cerca
        
        check_temperature()     # Controla cooler/heater
        
        # Envia cambios de LEDs a Tiva
        if changes1:
            ser.write(f"{led1},{led2},{pwmLed1},{pwmLed2}\n".encode())
            changes1 = 0
        
        time.sleep(0.5)  # Evita sobrecarga de CPU

except KeyboardInterrupt:
    print("\nPrograma terminado por el usuario")
    ser.close()    # Cierra conexión UART
    GPIO.cleanup() # Limpia pines GPIO