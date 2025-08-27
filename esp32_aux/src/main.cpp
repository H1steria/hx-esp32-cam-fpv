#include <Arduino.h>
#include <Wire.h>

// Definiciones de comandos I2C
#define I2C_CMD_FORWARD 1
#define I2C_CMD_BACKWARD 2
#define I2C_CMD_RIGHT 3
#define I2C_CMD_LEFT 4
#define I2C_CMD_FLASH 5

// Dirección I2C del esclavo (debe coincidir con la del maestro)
#define I2C_SLAVE_ADDR 0x08

void setup() {
  // Inicializar comunicación serial
  Serial.begin(115200);
  Serial.println("I2C Slave Iniciado");
  
  // Inicializar comunicación I2C
  Wire.begin(I2C_SLAVE_ADDR);
  Wire.onReceive(receiveEvent);
  
  Serial.println("Esperando comandos I2C...");
}

void loop() {
  // El loop puede estar vacío ya que la comunicación I2C es por interrupciones
  delay(100);
}

// Función que se ejecuta cuando se recibe datos por I2C
void receiveEvent(int howMany) {
  while (Wire.available()) {
    byte command = Wire.read(); // Leer el comando
    
    switch (command) {
      case I2C_CMD_FORWARD:
        Serial.println("Comando recibido: FORWARD");
        // Aquí puedes añadir el código para mover el robot hacia adelante
        break;
        
      case I2C_CMD_BACKWARD:
        Serial.println("Comando recibido: BACKWARD");
        // Aquí puedes añadir el código para mover el robot hacia atrás
        break;
        
      case I2C_CMD_RIGHT:
        Serial.println("Comando recibido: RIGHT");
        // Aquí puedes añadir el código para girar el robot a la derecha
        break;
        
      case I2C_CMD_LEFT:
        Serial.println("Comando recibido: LEFT");
        // Aquí puedes añadir el código para girar el robot a la izquierda
        break;
        
      case I2C_CMD_FLASH:
        Serial.println("Comando recibido: FLASH");
        // Aquí puedes añadir el código para activar el flash
        break;
        
      default:
        Serial.print("Comando desconocido recibido: ");
        Serial.println(command);
        break;
    }
  }
}
