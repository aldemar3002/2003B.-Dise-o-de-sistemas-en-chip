#include <Keypad.h>
#include <WiFi.h>
#include <Firebase_ESP_Client.h>
#include <addons/TokenHelper.h>
#include "addons/RTDBHelper.h"


// Definición de pines para UART2
#define RXD2 16
#define TXD2 17

// Credenciales de Firebase
#define API_KEY "AIzaSyB_iod5nDkJu5AN15YVuGlPGvVa2bedcQE"  // Reemplaza con tu API key
#define DATABASE_URL "https://prueba2-94ed4-default-rtdb.firebaseio.com"  // Reemplaza con tu URL de Firebase

char ssid[] = "AldemariPhone";
char pass[] = "perrosalchicha";
FirebaseData fbdo;
FirebaseAuth auth;
FirebaseConfig config;
unsigned long lastKeyPressTime = 0;
unsigned long debounceDelay = 3000;
bool signupOK = false;
bool sendToRaspberry = true;

const uint8_t ROWS = 4;
const uint8_t COLS = 4;
char keys[ROWS][COLS] = {
  { '1', '2', '3', 'A' },
  { '4', '5', '6', 'B' },
  { '7', '8', '9', 'C' },
  { '*', '0', '#', 'D' }
};

uint8_t colPins[COLS] = { 26, 25, 5, 18 };
uint8_t rowPins[ROWS] = { 13, 12, 14, 27 };
Keypad keypad = Keypad(makeKeymap(keys), rowPins, colPins, ROWS, COLS);

bool manualMode = true;
char inputBuffer[4] = "";
int digitCount = 0;

HardwareSerial MySerial(2);

// Variables para almacenar los valores actuales de estado y movimiento
String currentMovement = "";

void setup() {
  Serial.begin(9600);
  MySerial.begin(9600, SERIAL_8N1, RXD2, TXD2);

  WiFi.begin(ssid, pass);
  connectToWiFi();

  config.api_key = API_KEY;
  config.database_url = DATABASE_URL;

  if (Firebase.signUp(&config, &auth, "", "")) {
    Serial.println("Sign up exitoso");
    signupOK = true;
  } else {
    Serial.printf("Error en sign up: %s\n", config.signer.signupError.message.c_str());
  }

  if (signupOK) {
    Firebase.begin(&config, &auth);
  }

  Serial.println("Configuración completa. Esperando respuesta del teclado...");
}

void connectToWiFi() {
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("");
  Serial.println("WiFi connected");
}

void reconnectFirebase() {
  if (!Firebase.ready()) {
    Serial.println("Intentando reconectar a Firebase...");
    if (Firebase.signUp(&config, &auth, "", "")) {
      Serial.println("Reconexión exitosa a Firebase");
      Firebase.begin(&config, &auth);
    } else {
      Serial.printf("Error en reconexión a Firebase: %s\n", config.signer.signupError.message.c_str());
    }
  }
}

void sendMovementToFirebase(const char* movement) {
  if (Firebase.RTDB.setString(&fbdo, "/movimiento_mbot/movimiento", movement)) {
    Serial.print("Movimiento enviado a Firebase correctamente: ");
    Serial.println(movement);
  } else {
    Serial.println("Error al enviar movimiento a Firebase");
    Serial.println(fbdo.errorReason());
    reconnectFirebase();
  }
  fbdo.clear(); // Liberar memoria
}

void handleSongChange(int change) {
  if (Firebase.RTDB.getInt(&fbdo, "/numero_cancion/numero")) {
    int currentSong = fbdo.intData();
    int newSong = currentSong + change;
    if (Firebase.RTDB.setInt(&fbdo, "/numero_cancion/numero", newSong)) {
      Serial.print("Número de canción actualizado a: ");
      Serial.println(newSong);
    } else {
      Serial.println("Error al actualizar el número de canción en Firebase");
      Serial.println(fbdo.errorReason());
    }
  } else {
    Serial.println("Error al leer el número de canción de Firebase");
    Serial.println(fbdo.errorReason());
  }
  fbdo.clear(); // Liberar memoria
}

void simulateKeyPress(char key) {
  Serial.print("Simulated Key Press: ");
  Serial.println(key);
  if (key == '*') {
    manualMode = !manualMode;
    Serial.println(manualMode ? "Switched to manual mode" : "Switched to automatic mode");
    MySerial.print(manualMode ? 'M' : 'Q');
  } else if (manualMode) {
    if (key == 'A' || key == 'B' || key == 'C' || key == 'D' || key == '#') {
      switch (key) {
        case 'A':
          sendMovementToFirebase("A");
          MySerial.print('F');
          Serial.println("Sent command: F");
          break;
        case 'B':
          sendMovementToFirebase("B");
          MySerial.print('X');
          Serial.println("Sent command: X");
          break;
        case 'C':
          sendMovementToFirebase("C");
          MySerial.print('R');
          Serial.println("Sent command: R");
          break;
        case 'D':
          sendMovementToFirebase("D");
          MySerial.print('L');
          Serial.println("Sent command: L");
          break;
        case '#':
          sendMovementToFirebase("#");
          MySerial.print('G');
          Serial.println("Sent command: G");
          break;
      }
    } else if (key >= '0' && key <= '9') {
      inputBuffer[digitCount] = key;
      digitCount++;
      if (digitCount == 3) {
        inputBuffer[3] = '\0';
        int music_n = atoi(inputBuffer);
        Serial.print("Número de música ingresado: ");
        Serial.println(music_n);
        // Envia el número a Firebase
        if (Firebase.RTDB.setInt(&fbdo, "/numero_cancion/numero", music_n)) {
          Serial.print("Número enviado a Firebase correctamente: ");
          Serial.println(music_n);
        } else {
          Serial.println("Error al enviar número a Firebase");
          Serial.println(fbdo.errorReason());
          reconnectFirebase();
        }
        digitCount = 0;
        memset(inputBuffer, 0, sizeof(inputBuffer));
      }
    }
  }
}

void loop() {
  static unsigned long lastWiFiReconnectAttempt = 0;
  static unsigned long lastFirebaseCheck = 0;
  unsigned long currentMillis = millis();

  // Verificar conexión Wi-Fi periódicamente
  if (WiFi.status() != WL_CONNECTED && currentMillis - lastWiFiReconnectAttempt >= 10000) { // Cada 10 segundos
    Serial.println("Conexión Wi-Fi perdida. Intentando reconectar...");
    WiFi.begin(ssid, pass);
    lastWiFiReconnectAttempt = currentMillis;
  }

  char key = keypad.getKey();
  if (key) {
    simulateKeyPress(key);
  }

if (MySerial.available() > 0) {
    char sensorCommand = MySerial.read();
    Serial.print("Comando recibido: ");
    Serial.println(sensorCommand);
    if(sensorCommand == 'W' || sensorCommand == 'S'){
      Serial.print("Comando de canción recibido: ");
      Serial.println(sensorCommand);
      switch (sensorCommand){
        case 'W':
          if (!manualMode){
           Serial.println("Sent command: W");
          handleSongChange(-1); // Cambia la canción hacia adelante
          }
          break;
        case 'S':
         if (!manualMode){
          Serial.println("Sent command: S");
          handleSongChange(1); // Cambia la canción hacia adelante
         }
         break;
      }
    }
    else {
      Serial.print("Otro comando recibido: ");
      Serial.println(sensorCommand);
    }
  }

  // Verificar Firebase periódicamente
  if (currentMillis - lastFirebaseCheck >= 5000) { // Cada 5 segundos
    // Recibir movimiento de Firebase
    if (Firebase.RTDB.getString(&fbdo, "/movimiento_mbot/movimiento")) {
      String movimiento = fbdo.stringData();
      if (movimiento != currentMovement) {
        currentMovement = movimiento;
        Serial.print("Movimiento recibido de Firebase: ");
        Serial.println(movimiento); // Asegúrate de que el valor es correcto
        simulateKeyPress(movimiento[0]); // Simula la tecla presionada
      }
    } else {
      Serial.println("Error al recibir movimiento de Firebase");
      Serial.println(fbdo.errorReason());
    }

    fbdo.clear(); // Liberar memoria
    lastFirebaseCheck = currentMillis;
  }
}
