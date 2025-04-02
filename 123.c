#include <stdint.h>
#include <stdbool.h>
#include "inc/hw_memmap.h"
#include "driverlib/sysctl.h"
#include "driverlib/gpio.h"
#include "driverlib/uart.h"
#include "driverlib/pwm.h"

// Variables globales
uint32_t ui32SysClock;  // Almacena la frecuencia del reloj del sistema

// Prototipos de funciones
static void configureSysClock(void);
static void configGPIO(void);
static void configUART(void);
static void InitPWM(void);
void ProcesarCadena(void);

int main(void) {
    // 1. Configuraciones iniciales
    configureSysClock();  // Configura el reloj del sistema a 120 MHz
    configGPIO();         // Configura los pines GPIO
    configUART();         // Configura la UART para comunicación
    InitPWM();            // Configura PWM para el ventilador

    UARTprintf("\033[2J\n\nSistema listo\r\n");

    // 2. Bucle principal
    while (1) {     
        ProcesarCadena();  // Procesa los comandos recibidos por UART
    }
}

// =============================================
// FUNCIONES DE CONFIGURACIÓN
// =============================================

static void configureSysClock(void) {
    // Configura el sistema a 120 MHz usando PLL
    ui32SysClock = SysCtlClockFreqSet(
        SYSCTL_XTAL_25MHZ | SYSCTL_OSC_MAIN | SYSCTL_USE_PLL | SYSCTL_CFG_VCO_240, 
        120000000
    );
}

static void configGPIO(void) {
    // Habilita los periféricos GPIO necesarios
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPION);  // Para LEDs/garaje
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);  // Para PWM/ventilador

    // Configura los pines:
    // - PN4 y PN5 para controlar el garaje
    // - PF1 para PWM del ventilador
    GPIOPinTypeGPIOOutput(GPIO_PORTN_BASE, GPIO_PIN_4 | GPIO_PIN_5);
    GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, GPIO_PIN_1);
}

static void configUART(void) {
    // Configura UART0 a 115200 baudios
    SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
    
    GPIOPinConfigure(GPIO_PA0_U0RX);
    GPIOPinConfigure(GPIO_PA1_U0TX);
    GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);
    
    UARTConfigSetExpClk(
        UART0_BASE, 
        ui32SysClock, 
        115200, 
        UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE | UART_CONFIG_PAR_NONE
    );
    UARTEnable(UART0_BASE);
}

static void InitPWM(void) {
    // Configura PWM para el ventilador (PF1)
    SysCtlPeripheralEnable(SYSCTL_PERIPH_PWM0);
    GPIOPinConfigure(GPIO_PF1_M0PWM1);
    GPIOPinTypePWM(GPIO_PORTF_BASE, GPIO_PIN_1);
    
    PWMGenConfigure(
        PWM0_BASE, 
        PWM_GEN_0, 
        PWM_GEN_MODE_DOWN | PWM_GEN_MODE_NO_SYNC
    );
    PWMGenPeriodSet(PWM0_BASE, PWM_GEN_0, 1000);  // Frecuencia de 1 kHz
    PWMPulseWidthSet(PWM0_BASE, PWM_OUT_1, 500); // 50% de ciclo de trabajo inicial
    PWMGenEnable(PWM0_BASE, PWM_GEN_0);
    PWMOutputState(PWM0_BASE, PWM_OUT_1_BIT, true);
}

// =============================================
// LÓGICA PRINCIPAL
// =============================================

void ProcesarCadena(void) {
    char data[50];
    
    if (UARTCharsAvail(UART0_BASE)) {
        UARTgets(data, sizeof(data));  // Lee el comando UART
        
        // 1. Control del garaje
        if (strcmp(data, "garaje") == 0) {
            GPIOPinWrite(GPIO_PORTN_BASE, GPIO_PIN_4, GPIO_PIN_4);  // Activa motor
            UARTprintf("Puerta del garaje abierta\n");
            SysCtlDelay(ui32SysClock / 3);  // Espera 1 segundo
            GPIOPinWrite(GPIO_PORTN_BASE, GPIO_PIN_4, 0);  // Desactiva motor
        }
        // 2. Control de temperatura
        else if (strcmp(data, "cooler") == 0) {
            PWMPulseWidthSet(PWM0_BASE, PWM_OUT_1, 800);  // Ventilador al 80%
            UARTprintf("Ventilador activado\n");
        }
        else if (strcmp(data, "heater") == 0) {
            GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_2, GPIO_PIN_2);  // LED rojo (calentador)
            UARTprintf("Calentador activado\n");
        }
        else if (strcmp(data, "off") == 0) {
            PWMPulseWidthSet(PWM0_BASE, PWM_OUT_1, 0);  // Apaga ventilador
            GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_2, 0);  // Apaga LED
            UARTprintf("Sistema apagado\n");
        }
    }
}