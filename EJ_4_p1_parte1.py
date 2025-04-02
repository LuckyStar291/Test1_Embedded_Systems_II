import serial
import RPi.GPIO as GPIO
import time

# Configuración del puerto serial
ser = serial.Serial("/dev/ttyACM0", 9600)
ser.reset_input_buffer() 

# Pines GPIO para controlar los motores

buzzer = 16

# Configuración de pines GPIO
GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)

# Pines de control de dirección

GPIO.setup(buzzer, GPIO.OUT)
GPIO.output(buzzer, GPIO.LOW)



# Bucle principal
pwm_A = 0
pwm_B = 0
value = "0"


while True:
    
    if ser.in_waiting > 0:
        value = ser.readline().decode('utf-8').rstrip()
        print(f"Valor recibido: {value}")
    if value == '1':
        print("buzzer activado")
        GPIO.output(buzzer, GPIO.HIGH)
        time.sleep(0.5)
        
        
    if value == '0':
        print("buzzer desactivado")
        GPIO.output(buzzer, GPIO.LOW)
        time.sleep(0.5)

 
    
    
    
