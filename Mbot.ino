#define F_CPU 16000000UL
#include <util/delay.h>
#include <string.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <Arduino_FreeRTOS.h>
#include <task.h>

// Definiciones para la dirección del movimiento
#define FORWARD 1
#define BACKWARD 0

#define USART_BAUDRATE 9600
#define UBRR_VALUE (((F_CPU / (USART_BAUDRATE * 16UL))) - 1)

// Configuración de PWM para los motores
void setupPWM() {
    // Configurar Timer1 para Motor 1 y Motor 2
    TCCR1A = (1 << WGM10) | (1 << COM1A1) | (1 << COM1B1); // Modo PWM de 8 bits, Clear OC1A/OC1B on Compare Match
    TCCR1B = (1 << WGM12) | (1 << CS11); // Prescaler 8
    OCR1A = 102; // 40% de 255 (8-bit PWM)
    OCR1B = 102; // 40% de 255 (8-bit PWM)

    // Configurar Timer3 para Motor 3 y Motor 4
    TCCR3A = (1 << WGM30) | (1 << COM3A1) | (1 << COM3B1); // Modo PWM de 8 bits, Clear OC3A/OC3B on Compare Match
    TCCR3B = (1 << WGM32) | (1 << CS31); // Prescaler 8
    OCR3A = 102; // 40% de 255 (8-bit PWM)
    OCR3B = 102; // 40% de 255 (8-bit PWM)
}

// Inicialización de UART2
void initUART2(void) {
    // Configurar pines RX2 y TX2
    DDRH &= ~(1 << PH0); // RX2 como entrada
    DDRH |= (1 << PH1);  // TX2 como salida

    // Configuración de UART2: 9600 baud, 8N1
    uint16_t ubrr = UBRR_VALUE; // Valor calculado para 9600 baud con 16 MHz de reloj
    UBRR2H = (unsigned char)(ubrr >> 8);
    UBRR2L = (unsigned char)ubrr;
    UCSR2B = (1 << RXEN2) | (1 << TXEN2); // Habilitar RX, TX
    UCSR2C = (1 << UCSZ21) | (1 << UCSZ20); // Configurar 8 bits de datos, 1 bit de parada
}

// Enviar datos por UART2
void sendUART2(char data) {
    while (!(UCSR2A & (1 << UDRE2))); // Esperar a que el registro de datos esté vacío
    UDR2 = data;  // Enviar el dato
}

void sendUART2String(const char *str) {
    while (*str) {
        sendUART2(*str++);
    }
}

char receiveUART2(void) {
    while (!(UCSR2A & (1 << RXC2))); // Esperar a que haya datos recibidos
    return UDR2; // Leer el dato recibido
}

// Funciones de movimiento para cada motor
void runForwardM1() {
    OCR1A = 102; // Ajustar el ciclo de trabajo de PWM para el motor 1
    PORTC |= (1 << PC4);  // Configurar dirección hacia adelante
    PORTC &= ~(1 << PC5); // Apagar la dirección opuesta
}

void runBackwardM1() {
    OCR1A = 102; // Ajustar el ciclo de trabajo de PWM para el motor 1
    PORTC |= (1 << PC5);  // Configurar dirección hacia atrás
    PORTC &= ~(1 << PC4); // Apagar la dirección opuesta
}

void runForwardM2() {
    OCR1B = 102; // Ajustar el ciclo de trabajo de PWM para el motor 2
    PORTC |= (1 << PC2);  // Configurar dirección hacia adelante
    PORTC &= ~(1 << PC3); // Apagar la dirección opuesta
}

void runBackwardM2() {
    OCR1B = 102; // Ajustar el ciclo de trabajo de PWM para el motor 2
    PORTC |= (1 << PC3);  // Configurar dirección hacia atrás
    PORTC &= ~(1 << PC2); // Apagar la dirección opuesta
}

void runForwardM3() {
    OCR3A = 102; // Ajustar el ciclo de trabajo de PWM para el motor 3
    PORTG |= (1 << PG0);  // Configurar dirección hacia adelante
    PORTG &= ~(1 << PG1); // Apagar la dirección opuesta
}

void runBackwardM3() {
    OCR3A = 102; // Ajustar el ciclo de trabajo de PWM para el motor 3
    PORTG |= (1 << PG1);  // Configurar dirección hacia atrás
    PORTG &= ~(1 << PG0); // Apagar la dirección opuesta
}

void runForwardM4() {
    OCR3B = 102; // Ajustar el ciclo de trabajo de PWM para el motor 4
    PORTC |= (1 << PC0);  // Configurar dirección hacia adelante
    PORTC &= ~(1 << PC1); // Apagar la dirección opuesta
}

void runBackwardM4() {
    OCR3B = 102; // Ajustar el ciclo de trabajo de PWM para el motor 4
    PORTC |= (1 << PC1);  // Configurar dirección hacia atrás
    PORTC &= ~(1 << PC0); // Apagar la dirección opuesta
}

// Tareas de FreeRTOS para cada función de movimiento
void runForwardTask(void *pvParameters) {
    for (;;) {
        runForwardM1();
        runForwardM2();
        runForwardM3();
        runForwardM4();
        vTaskDelay(pdMS_TO_TICKS(100)); // Ajustar el tiempo de retardo según sea necesario
    }
}

void runBackwardTask(void *pvParameters) {
    for (;;) {
        runBackwardM1();
        runBackwardM2();
        runBackwardM3();
        runBackwardM4();
        vTaskDelay(pdMS_TO_TICKS(100)); // Ajustar el tiempo de retardo según sea necesario
    }
}

void turnRightTask(void *pvParameters) {
    for (;;) {
        runForwardM3();
        runForwardM4();
        runBackwardM1();
        runBackwardM2();
        vTaskDelay(pdMS_TO_TICKS(1500)); // Ajustar el tiempo de retardo según sea necesario
        stopMotor();
        vTaskDelay(pdMS_TO_TICKS(100)); // Ajustar el tiempo de retardo según sea necesario
    }
}

void turnLeftTask(void *pvParameters) {
    for (;;) {
        runForwardM1();
        runForwardM2();
        runBackwardM3();
        runBackwardM4();
        vTaskDelay(pdMS_TO_TICKS(1500)); // Ajustar el tiempo de retardo según sea necesario
        stopMotor();
        vTaskDelay(pdMS_TO_TICKS(100)); // Ajustar el tiempo de retardo según sea necesario
    }
}

// Funciones de movimiento para todos los motores
void stopMotor() {
    OCR1A = 0;
    OCR1B = 0;
    OCR3A = 0;
    OCR3B = 0;
}

// Tareas de FreeRTOS
void manualControlTask(void *pvParameters) {
    while (1) {
        if (UCSR2A & (1 << RXC2)) { // Si hay datos disponibles en UART2
            char command = receiveUART2(); // Leer el comando
            if (command == 'F' || command == 'X' || command == 'R' || command == 'L' || command == 'G') {
                switch (command) {
                    case 'F':
                        xTaskCreate(runForwardTask, "RunForward", configMINIMAL_STACK_SIZE, NULL, 1, NULL);
                        break;
                    case 'X':
                        stopMotor();
                        vTaskDelete(NULL); // Eliminar la tarea actual
                        break;
                    case 'R':
                        xTaskCreate(turnRightTask, "TurnRight", configMINIMAL_STACK_SIZE, NULL, 1, NULL);
                        break;
                    case 'L':
                        xTaskCreate(turnLeftTask, "TurnLeft", configMINIMAL_STACK_SIZE, NULL, 1, NULL);
                        break;
                    case 'G':
                        xTaskCreate(runBackwardTask, "RunBackward", configMINIMAL_STACK_SIZE, NULL, 1, NULL);
                        break;
                }
            }
        }
        vTaskDelay(pdMS_TO_TICKS(10)); // Retardo de 10 ms
    }
}

// Configuración inicial
void setup() {
    // Configuración de pines para los motores
    DDRB |= (1 << PB5) | (1 << PB6);
    DDRC |= (1 << PC0) | (1 << PC1) | (1 << PC2) | (1 << PC3) | (1 << PC4) | (1 << PC5);
    DDRH |= (1 << PH4) | (1 << PH5);
    DDRG |= (1 << PG0) | (1 << PG1);
    PORTB &= ~((1 << PB5) | (1 << PB6));
    PORTH &= ~((1 << PH4) | (1 << PH5));

    // Inicialización de comunicaciones serie
    Serial.begin(9600); // Comunicación para depuración a 9600 baudios
    initUART2(); // Inicializar UART2

    setupPWM(); // Configuración de PWM para los motores

    // Crear tarea para controlar el Mbot en modo manual
    xTaskCreate(manualControlTask, "ManualControl", configMINIMAL_STACK_SIZE, NULL, 1, NULL);

    // Habilitar interrupciones globales
    sei();
}

void loop() {
}
