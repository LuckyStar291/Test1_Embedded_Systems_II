#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include <ctype.h>  // Para verificar caracteres numéricos
#include "inc/hw_memmap.h"
#include "inc/hw_ints.h"
#include "inc/hw_types.h"
#include "driverlib/sysctl.h"
#include "driverlib/gpio.h"
#include "driverlib/pin_map.h"
#include "driverlib/uart.h"
#include "driverlib/interrupt.h"
#include "driverlib/rom.h"
#include "driverlib/rom_map.h"
#include "driverlib/pwm.h"
#include "driverlib/adc.h"       // Funciones para controlar el ADC (configuración, inicio de conversiones)

#include "inc/hw_nvic.h"
#include "utils/uartstdio.h"
#include "driverlib/timer.h"

// PIN ventilador 

uint32_t ui32SysClock;
uint8_t counter = 0;
volatile uint8_t ledState = 0;
char data[50]; // Buffer para almacenar los datos recibidos por UART
#define duty 4095 // ciclo de trabajo maximo del pwm en este caso debe de ser el tamaño de la resolucion de bist del adc 12bits
uint32_t PWMCycle1;
uint32_t PWMCycle2;
uint32_t width;        // Variable para almacenar el ancho del pulso de PWM
uint32_t adcValue;
uint32_t g_ui32Flags;
uint32_t g_ui32States;
uint32_t count =0;


// Prototipos
static void configureSysClock(void);
static void configGPIO(void);
static void configUART(void);
static void configInterrupt(void);
static void configTimer0(void);
void ProcesarCadena(void);
void ButtonISR(void);
static void InitPWM1(void);
static void InitPWM2(void);
static void config_adc(void);        // Inicializa el ADC
void Timer0IntHandler(void);
void UART0IntHandler(void);
uint32_t ReadADC(void);               //lectura del adc



int main(void) {
    configureSysClock();
    configGPIO();
    configUART();
    configTimer0();
    InitPWM1();
    InitPWM2();
    config_adc();
    width = 0;
    configInterrupt(); // Configurar interrupción

    UARTprintf("\033[2J\n\nUART Listo\r\n");

    while (1) {     
        ProcesarCadena();
        /*
        adcValue = ReadADC(); // Llamar a la función para leer el ADC
        width=adcValue;
        UARTprintf("ADC Value: %d\r\n", adcValue); // Enviar el valor del ADC
        PWMPulseWidthSet(PWM0_BASE, PWM_OUT_1, PWMCycle1 * width/4300);
        
        if(adcValue <= 10)
        {
            PWMPulseWidthSet(PWM0_BASE, PWM_OUT_1, 5);
        }
        SysCtlDelay(ui32SysClock*0.2); // Ajustar el valor de delay según sea necesario
        */
        
    }
}

// ===============================
//      CONFIGURACIONES
// ===============================

static void configureSysClock(void) {
    ui32SysClock = SysCtlClockFreqSet((SYSCTL_XTAL_25MHZ |
                                       SYSCTL_OSC_MAIN |
                                       SYSCTL_USE_PLL |
                                       SYSCTL_CFG_VCO_240), 120000000);
}

//CONFIG GPIO Y PERIPHERALS *******************
static void configGPIO(void) {
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOJ); // Pulsadores
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPION); // LEDs
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF); // LEDs
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA); // UART
    SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0); // UART

    while (!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOJ)) {}
    while (!SysCtlPeripheralReady(SYSCTL_PERIPH_GPION)) {}
    while (!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOA)) {}
    while (!SysCtlPeripheralReady(SYSCTL_PERIPH_UART0)) {}
    // Configurar User LEDs (PORTN, PIN_0 y PIN_1)
    GPIOPinTypeGPIOOutput(GPIO_PORTN_BASE, 0x03); //user led PN 0 1
    GPIOPinTypeGPIOOutput(GPIO_PORTN_BASE, 0x3C); // PN 2,3,4,5
    GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, 0x0E); // PF 1,2,3

    // Configurar User Switch (PORTJ, pin 0 y 1
    GPIOPinTypeGPIOInput(GPIO_PORTJ_BASE, 0x03);
    GPIOPadConfigSet(GPIO_PORTJ_BASE, 0x03, GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD_WPU); // Pull-up interno
}

//CONFIGURAR UART***************************************
static void configUART(void) {
    GPIOPinConfigure(GPIO_PA0_U0RX);
    GPIOPinConfigure(GPIO_PA1_U0TX);
    GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);
    
    UARTConfigSetExpClk(UART0_BASE, ui32SysClock, 115200,
                        (UART_CONFIG_WLEN_8 |
                         UART_CONFIG_STOP_ONE |
                         UART_CONFIG_PAR_NONE));

    UARTEnable(UART0_BASE);
    UARTStdioConfig(0, 115200, ui32SysClock);
}
// ===============================
//    CONFIGURACIÓN DE INTERRUPCIÓN
// ===============================
static void configInterrupt(void) {
    GPIOIntDisable(GPIO_PORTJ_BASE, GPIO_PIN_0); // Deshabilitar interrupciones mientras se configuran
    GPIOIntClear(GPIO_PORTJ_BASE, GPIO_PIN_0); // Limpiar interrupciones previas
    GPIOIntRegister(GPIO_PORTJ_BASE, ButtonISR); // Registrar la ISR (función de interrupción)
    GPIOIntTypeSet(GPIO_PORTJ_BASE, GPIO_PIN_0, GPIO_FALLING_EDGE); // Configurar para detectar flanco de bajada
    GPIOIntEnable(GPIO_PORTJ_BASE, GPIO_PIN_0); // Habilitar interrupción en el pin
    MAP_IntEnable(INT_GPIOJ); // Habilitar interrupción global en PORTJ
    MAP_IntMasterEnable(); // Habilitar interrupciones globales
}

// ===============================
//    FUNCIÓN DE INTERRUPCIÓN
// ===============================

void ButtonISR(void) {
    GPIOIntClear(GPIO_PORTJ_BASE, GPIO_PIN_0); // Limpiar bandera de interrupción

    // Alternar LEDs
    ledState = !ledState;
    if (ledState) {
        GPIOPinWrite(GPIO_PORTN_BASE, GPIO_PIN_0 | GPIO_PIN_1, GPIO_PIN_0); // Enciende LED1
    } else {
        GPIOPinWrite(GPIO_PORTN_BASE, GPIO_PIN_0 | GPIO_PIN_1, GPIO_PIN_1); // Enciende LED2
    }

    UARTprintf("Botón presionado. LED cambiado.\r\n");
}

void ProcesarCadena(void) {
    if (UARTCharsAvail(UART0_BASE)) {
        UARTgets(data, sizeof(data)); // Leer los datos recibidos
        UARTprintf("data recibida: %s\n", data);
        char *token;
        char *tokens[4];
        uint32_t valor = 0;
        int8_t pwmA = 0, pwmB = 0;
        int i = 0;

        if (strcmp(data, "garaje") == 0) {
            GPIOPinWrite(GPIO_PORTN_BASE, 0x30, 0x10); // pins N 4 y 5
            UARTprintf("Garaje abierto\n");
            SysCtlDelay(120000000 / 3 * 2); // Aproximadamente 1 segundo
            GPIOPinWrite(GPIO_PORTN_BASE, 0x30, 0x00); //

        }
        else if (strcmp(data, "cooler") == 0) {
            GPIOPinWrite(GPIO_PORTN_BASE, 0x0C, 0x08); // 2,3, prende 3
            UARTprintf("Ventilador prendido\n");
            SysCtlDelay(120000000 / 3 * 2); // Aproximadamente 1 segundo
        }
        else if (strcmp(data, "heater") == 0) {
            GPIOPinWrite(GPIO_PORTN_BASE, 0x0C, 0x04); // 2,3 prende 2
            UARTprintf("heater prendido\n");
            SysCtlDelay(120000000 / 3 * 2); // Aproximadamente 1 segundo
        }
        else if (strcmp(data, "nada") == 0) {
            UARTprintf("todo apagado\n");
            GPIOPinWrite(GPIO_PORTN_BASE, 0x0C, 0x00); // apaga todo
        }
        else if (strcmp(data, "alarm") == 0) {
            UARTprintf("alarma prendida\n");
            //GPIOPinWrite(GPIO_PORTF_BASE, 0x08, 0x08); // PF3
            HWREGBITW(&g_ui32Flags, 0) = 1;
        }
        else if (strcmp(data, "alarmoff") == 0) {
            UARTprintf("alarma apagada\n");
            count = 0;
            HWREGBITW(&g_ui32Flags, 0) = 0;
            GPIOPinWrite(GPIO_PORTF_BASE, 0x08, 0x00); // apaga buzzer PF3
        }

        else{
            // Separar la cadena por comas
            token = strtok(data, ",");
            while (token != NULL && i < 4) {
                tokens[i++] = token;
                token = strtok(NULL, ",");
            }

            // Validar si hay exactamente 4 valores
            if (i == 4) {
                // Validar los primeros 2 valores (deben ser '0' o '1')
                for (i = 0; i < 2; i++) {
                    if (tokens[i][0] != '0' && tokens[i][0] != '1') {
                        UARTprintf("error flag1");
                        goto error; // Si no es válido, ir a error
                    }
                    valor |= (tokens[i][0] - '0') << (3 - i); // Mapear bits en uint32_t
                }

                pwmA = atoi(tokens[2]);
                pwmB = atoi(tokens[3]);

                UARTprintf("pwmA y B: %d, %d \n", pwmA, pwmB);
                
                if (tokens[0][0]=='1'){
                    PWMPulseWidthSet(PWM0_BASE, PWM_OUT_1, PWMCycle1 * pwmA*0.01);
                }
                else{
                    PWMPulseWidthSet(PWM0_BASE, PWM_OUT_1, PWMCycle1 * 0+1);   
                }

                if (tokens[1][0]=='1'){
                    PWMPulseWidthSet(PWM0_BASE, PWM_OUT_2, PWMCycle2 * pwmB*0.01);
                }
                else{
                    PWMPulseWidthSet(PWM0_BASE, PWM_OUT_2, PWMCycle2 * 0+1);  
                }            
                
            }
         
        else {
            error: // Si falla la validación
            strcat(data, " desde Tiva\n"); // Agregar mensaje de error
            UARTprintf(data);
        }
        }
    }
}


// Inicializar PWM
static void InitPWM1(void) {
    /*
- **BASE:** `PWM0_BASE` (Usa el módulo PWM0)
- **GEN:** `PWM_GEN_0` (Usa el generador 0 dentro de PWM0)
- **PIN:** `PF1` (Usa el pin PF1 como salida de PWM)
- **PWM Output:** `PWM_OUT_1` (Salida PWM asociada a PF1)
*/
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF); // Habilitar el reloj para GPIO F
    SysCtlPeripheralEnable(SYSCTL_PERIPH_PWM0);  // Habilitar el reloj para el módulo PWM0

    // Configurar PF1 como salida de PWM
    GPIOPinConfigure(GPIO_PF1_M0PWM1);  // PF1 -> Módulo 0 PWM1
    GPIOPinTypePWM(GPIO_PORTF_BASE, GPIO_PIN_1); // Configurar PF1 como PWM

    PWMClockSet(PWM0_BASE, PWM_SYSCLK_DIV_8);
    uint32_t ui32PWMClockRate = ui32SysClock / 8;
    PWMCycle1 = ui32PWMClockRate / 250;

    // Configurar el generador PWM0
    PWMGenConfigure(PWM0_BASE, PWM_GEN_0, PWM_GEN_MODE_DOWN | PWM_GEN_MODE_NO_SYNC);
    PWMGenPeriodSet(PWM0_BASE, PWM_GEN_0, PWMCycle1); // Establecer el período del PWM (frecuencia de 250 Hz)

    PWMPulseWidthSet(PWM0_BASE, PWM_OUT_1, 0); // Establecer el ciclo de trabajo inicial
    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_PWM0)) { }
    PWMGenEnable(PWM0_BASE, PWM_GEN_0); // Habilitar el generador PWM
    PWMOutputState(PWM0_BASE, PWM_OUT_1_BIT, true); // Habilitar la salida PWM
}

// Inicializar segundo PWM
static void InitPWM2(void) {
    /*
- **BASE:** `PWM0_BASE` (Usa el módulo PWM0)
- **GEN:** `PWM_GEN_1` (Usa el generador 1 dentro de PWM0)
- **PIN:** `PF2` (Usa el pin PF2 como salida de PWM)
- **PWM Output:** `PWM_OUT_2` (Salida PWM asociada a PF2)
*/
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF); // Habilitar el reloj para GPIO F
    SysCtlPeripheralEnable(SYSCTL_PERIPH_PWM0);  // Habilitar el reloj para el módulo PWM0

    // Configurar PF2 como salida de PWM
    GPIOPinConfigure(GPIO_PF2_M0PWM2);  // PF2 -> Módulo 0 PWM2
    GPIOPinTypePWM(GPIO_PORTF_BASE, GPIO_PIN_2); // Configurar PF2 como PWM

    PWMClockSet(PWM0_BASE, PWM_SYSCLK_DIV_8);
    uint32_t ui32PWMClockRate = ui32SysClock / 8;
    PWMCycle2 = ui32PWMClockRate / 250;

    // Configurar el generador PWM1
    PWMGenConfigure(PWM0_BASE, PWM_GEN_1, PWM_GEN_MODE_DOWN | PWM_GEN_MODE_NO_SYNC);
    PWMGenPeriodSet(PWM0_BASE, PWM_GEN_1, PWMCycle2); // Establecer el período del PWM

    PWMPulseWidthSet(PWM0_BASE, PWM_OUT_2, 0); // Establecer el ciclo de trabajo inicial
    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_PWM0)) { }
    PWMGenEnable(PWM0_BASE, PWM_GEN_1); // Habilitar el generador PWM
    PWMOutputState(PWM0_BASE, PWM_OUT_2_BIT, true); // Habilitar la salida PWM
}

void Timer0IntHandler(void)
{
    //TimerIntClear(TIMER0_BASE, TIMER_A); 
    UARTprintf("aaaa\n");

    if (HWREGBITW(&g_ui32Flags, 0)==1){
        count++;
        if (count <= 10 && count%2==0){
                HWREGBITW(&g_ui32States, 0) ^= 1;
                GPIOPinWrite(GPIO_PORTF_BASE, 0x08, g_ui32States<<3);
        }
        else if (count <= 20){
            HWREGBITW(&g_ui32States, 0) ^= 1;
            GPIOPinWrite(GPIO_PORTF_BASE, 0x08, g_ui32States<<3);
        }
        else{
            count = 0;
            HWREGBITW(&g_ui32Flags, 0) = 0; 
        }
    }
}

void UART0IntHandler(void){

}

void config_adc(void) {
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE); // Habilitar el reloj para el puerto GPIO E (donde está PE3)
    SysCtlPeripheralEnable(SYSCTL_PERIPH_ADC0); // Habilitar el reloj para el módulo ADC0

    // Configurar el pin PE3 como entrada analógica para el ADC
    GPIOPinTypeADC(GPIO_PORTE_BASE, GPIO_PIN_3); // Configurar el pin como entrada de ADC

    // Esperar a que el módulo ADC esté listo
    while (!SysCtlPeripheralReady(SYSCTL_PERIPH_ADC0)) {}

    // Configurar el secuenciador 3 para el ADC
    ADCSequenceConfigure(ADC0_BASE, 3, ADC_TRIGGER_PROCESSOR, 0); // Configurar secuenciador 3
    ADCSequenceStepConfigure(ADC0_BASE, 3, 0, ADC_CTL_CH0 | ADC_CTL_IE | ADC_CTL_END); // Configurar paso 0
    ADCSequenceEnable(ADC0_BASE, 3); // Habilitar el secuenciador 3
    ADCIntClear(ADC0_BASE, 3); // Limpiar interrupciones del secuenciador 3
}

// Leer valor del ADC
uint32_t ReadADC(void) {
    // Disparar el ADC
    ADCProcessorTrigger(ADC0_BASE, 3);

    // Esperar a que la conversión esté completa
    while (!ADCIntStatus(ADC0_BASE, 3, false)) {}

    // Leer el valor del ADC
    ADCSequenceDataGet(ADC0_BASE, 3, &adcValue);

    // Limpiar la bandera de interrupción
    ADCIntClear(ADC0_BASE, 3);

    return adcValue; // Retornar el valor leído del ADC
}

static void configTimer0(void){
    SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER0);
    IntMasterEnable();
    TimerConfigure(TIMER0_BASE, TIMER_CFG_PERIODIC);
    TimerLoadSet(TIMER0_BASE, TIMER_A, ui32SysClock/2);
    IntEnable(INT_TIMER0A);
    TimerIntEnable(TIMER0_BASE, TIMER_TIMA_TIMEOUT);
    TimerEnable(TIMER0_BASE, TIMER_A);
}