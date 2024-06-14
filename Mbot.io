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

int currentDirection = FORWARD; // Variable para mantener la dirección actual
bool manualMode = false; // Variable para controlar el modo manual o automático

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
    OCR4B = 102; // 40% de 255 (8-bit PWM)
    OCR4C = 102; // 40% de 255 (8-bit PWM)
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
    PORTB |= (1 << PB5);  // Encender Motor 1 (OC1A)
    PORTC |= (1 << PC4);  // Configurar dirección hacia adelante
    PORTC &= ~(1 << PC5); // Apagar la dirección opuesta
}

void runBackwardM1() {
    PORTB |= (1 << PB5);  // Encender Motor 1 (OC1A)
    PORTC |= (1 << PC5);  // Configurar dirección hacia atrás
    PORTC &= ~(1 << PC4); // Apagar la dirección opuesta
}

void runForwardM2() {
    PORTB |= (1 << PB6);  // Encender Motor 2 (OC1B)
    PORTC |= (1 << PC2);  // Configurar dirección hacia adelante
    PORTC &= ~(1 << PC3); // Apagar la dirección opuesta
}

void runBackwardM2() {
    PORTB |= (1 << PB6);  // Encender Motor 2 (OC1B)
    PORTC |= (1 << PC3);  // Configurar dirección hacia atrás
    PORTC &= ~(1 << PC2); // Apagar la dirección opuesta
}

void runForwardM3() {
    PORTH |= (1 << PH4);  // Encender Motor 3 (OC3A)
    PORTG |= (1 << PG0);  // Configurar dirección hacia adelante
    PORTG &= ~(1 << PG1); // Apagar la dirección opuesta
}

void runBackwardM3() {
    PORTH |= (1 << PH4);  // Encender Motor 3 (OC3A)
    PORTG |= (1 << PG1);  // Configurar dirección hacia atrás
    PORTG &= ~(1 << PG0); // Apagar la dirección opuesta
}

void runForwardM4() {
    PORTH |= (1 << PH5);  // Encender Motor 4 (OC3B)
    PORTC |= (1 << PC0);  // Configurar dirección hacia adelante
    PORTC &= ~(1 << PC1); // Apagar la dirección opuesta
}

void runBackwardM4() {
    PORTH |= (1 << PH5);  // Encender Motor 4 (OC3B)
    PORTC |= (1 << PC1);  // Configurar dirección hacia atrás
    PORTC &= ~(1 << PC0); // Apagar la dirección opuesta
}

// Funciones de movimiento para todos los motores
void runForward() {
    TCCR1A |= (1<<COM1A1)| (1<<COM1B1);
    TCCR4A |= (1<<COM4B1) | (1<<COM4C1);
    runForwardM1();
    runForwardM2();
    runForwardM3();
    runForwardM4();
}

void runBackward() {
    TCCR1A |= (1<<COM1A1)| (1<<COM1B1);
    TCCR4A |= (1<<COM4B1) | (1<<COM4C1);
    runBackwardM1();
    runBackwardM2();
    runBackwardM3();
    runBackwardM4();
}

void turnRight() {
    TCCR1A |= (1<<COM1A1)| (1<<COM1B1);
    TCCR4A |= (1<<COM4B1) | (1<<COM4C1);
    runForwardM3();
    runForwardM4();
    runBackwardM1();
    runBackwardM2();
}

void turnLeft() {
    TCCR1A |= (1<<COM1A1)| (1<<COM1B1);
    TCCR4A |= (1<<COM4B1) | (1<<COM4C1);
    runForwardM1();
    runForwardM2();
    runBackwardM3();
    runBackwardM4();
}

void stopMotor() {
    TCCR1A &= ~((1<<COM1A1) | (1<<COM1B1));
    TCCR4A &= ~((1<<COM4B1) | (1<<COM4C1));
    PORTB &= ~((1 << PB5) | (1 << PB6));
    PORTH &= ~((1 << PH4) | (1 << PH5));
}

// Tareas de FreeRTOS
void manualControlTask(void *pvParameters) {
    while (1) {
        if (UCSR2A & (1 << RXC2)) { // Si hay datos disponibles en UART2
            char command = receiveUART2(); // Leer el comando
            if (command == 'F' || command == 'X' || command == 'R' || command == 'L' || command == 'G' || command == 'M' || command == 'Q') {
                switch (command) {
                    case 'F':
                        if (manualMode) runForward(); // Ejecutar comando hacia adelante en modo manual
                        break;
                    case 'X':
                        if (manualMode) runBackward(); // Ejecutar comando hacia atrás en modo manual
                        break;
                    case 'R':
                        if (manualMode) {
                            turnRight(); // Ejecutar comando para girar a la derecha en modo manual
                            vTaskDelay(pdMS_TO_TICKS(1500)); // Duración de 1.5 segundos
                            stopMotor(); // Detener motores
                        }
                        break;
                    case 'L':
                        if (manualMode) {
                            turnLeft(); // Ejecutar comando para girar a la izquierda en modo manual
                            vTaskDelay(pdMS_TO_TICKS(1500)); // Duración de 1.5 segundos
                            stopMotor(); // Detener motores
                        }
                        break;
                    case 'G':
                        if (manualMode) stopMotor(); // Detener motores
                        break;
                    case 'M':
                        if (!manualMode) {
                            manualMode = true;
                            stopMotor(); // Detener motores al cambiar a modo manual
                        }
                        break;
                    case 'Q':
                        if (manualMode) {
                            manualMode = false;
                            stopMotor(); // Detener motores al cambiar a modo automático
                            currentDirection = FORWARD; // Asegurar que la dirección actual es hacia adelante
                        }
                        break;
                }
            }
        }
        vTaskDelay(pdMS_TO_TICKS(10)); // Retardo de 10 ms
    }
}

void collisionTask(void *pvParameters) {
    while (1) {
        if (!manualMode) { // Solo ejecutar la lógica de colisión en modo automático
            if (currentDirection == FORWARD) {
                if (PINF & (1 << PF6)) { // Sensor frontal detecta un obstáculo
                    runForward();
                } else {
                    currentDirection = BACKWARD;
                    runBackward(); // Cambiar a marcha atrás
                    sendUART2('W'); // Envía el comando para atrasar canción
                }
            } else if (currentDirection == BACKWARD) {
                if ((PINK & (1 << PK3))) { // Sensor trasero detecta un obstáculo
                    runBackward();
                } else {
                    currentDirection = FORWARD;
                    runForward(); // Cambiar a marcha adelante
                    sendUART2('S'); // Envía el comando para adelantar canción
                }
            }
        }
        vTaskDelay(pdMS_TO_TICKS(100)); // Retardo de 100 ms
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

    // Configuración de pines para los sensores
    DDRF &= ~(1 << PF6); // PF6 como entrada (sensor frontal)
    DDRK &= ~(1 << PK3); // PK3 como entrada (sensor trasero)

    // Inicialización de comunicaciones serie
    Serial.begin(9600); // Comunicación para depuración a 9600 baudios
    initUART2(); // Inicializar UART2

    setupPWM(); // Configuración de PWM para los motores

    // Crear tarea para controlar el Mbot en modo manual
    xTaskCreate(manualControlTask, "ManualControl", configMINIMAL_STACK_SIZE, NULL, 1, NULL);
    // Crear tarea para manejar colisiones en modo automático
    xTaskCreate(collisionTask, "CollisionTask", configMINIMAL_STACK_SIZE, NULL, 1, NULL);

    // Habilitar interrupciones globales
    sei();
}

void loop() {
}
