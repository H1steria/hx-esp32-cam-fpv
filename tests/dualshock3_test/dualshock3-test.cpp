#include <iostream>
#include <fcntl.h>
#include <unistd.h>
#include <linux/joystick.h>
#include <cstring>
#include <csignal>
#include <iomanip>
#include <cerrno> // Necesario para 'errno'

bool running = true;
int joystick_fd = -1;
const char* device = "/dev/input/js0";

void signalHandler(int signal) {
    if (signal == SIGINT) {
        std::cout << "\nPrograma terminado." << std::endl;
        running = false;
    }
}

bool openJoystick() {
    if (joystick_fd >= 0) {
        close(joystick_fd);
    }
    
    joystick_fd = open(device, O_RDONLY | O_NONBLOCK);
    return joystick_fd >= 0;
}

void closeJoystick() {
    if (joystick_fd >= 0) {
        close(joystick_fd);
        joystick_fd = -1;
    }
}

bool isJoystickConnected() {
    return access(device, F_OK) == 0;
}

int main() {
    // Registrar manejador de señal para Ctrl+C
    signal(SIGINT, signalHandler);
    
    std::cout << "Iniciando test de DualShock3..." << std::endl;
    
    // Intentar abrir el dispositivo del joystick
    if (!openJoystick()) {
        std::cout << "Control no encontrado. Esperando conexión..." << std::endl;
    } else {
        std::cout << "Control inicializado desde " << device << std::endl;
    }
    
    std::cout << "Presiona Ctrl+C para salir..." << std::endl;
    
    struct js_event event;
    bool was_connected = (joystick_fd >= 0);
    
    while (running) {
        // Verificar si el control está conectado
        bool is_connected = isJoystickConnected();
        
        if (is_connected && !was_connected) {
            // Control recién conectado
            if (openJoystick()) {
                std::cout << "Control conectado y reinicializado desde " << device << std::endl;
            } else {
                std::cout << "Error al reinicializar el control." << std::endl;
            }
        } else if (!is_connected && was_connected) {
            // Control desconectado
            closeJoystick();
            std::cout << "Control desconectado. Esperando reconexión..." << std::endl;
        }
        
        was_connected = is_connected;
        
        // Si el control está conectado, leer eventos
        if (joystick_fd >= 0) {
            ssize_t bytes = read(joystick_fd, &event, sizeof(event));
            
            if (bytes == sizeof(event)) {
                // Procesar el evento según su tipo
                // **CORRECCIÓN:** No ignorar JS_EVENT_INIT para procesar el estado inicial
                switch (event.type) {
                    case JS_EVENT_BUTTON:
                        std::cout << "Botón " << (int)event.number 
                                  << (event.value ? " presionado" : " liberado") 
                                  << " (valor: " << event.value << ")" << std::endl;
                        break;
                        
                    case JS_EVENT_AXIS:
                        // **CORRECCIÓN:** Imprimir el valor sin procesar del eje
                        std::cout << "Eje " << (int)event.number 
                                  << " movido a " << event.value << std::endl;
                        break;

                    // Manejar explícitamente los eventos de inicialización si se desea
                    case JS_EVENT_INIT | JS_EVENT_BUTTON:
                        std::cout << "Botón " << (int)event.number 
                                  << " estado inicial: " << (event.value ? "presionado" : "liberado")
                                  << " (valor: " << event.value << ")" << std::endl;
                        break;

                    case JS_EVENT_INIT | JS_EVENT_AXIS:
                        std::cout << "Eje " << (int)event.number 
                                  << " posición inicial: " << event.value << std::endl;
                        break;
                }
            } else if (bytes < 0 && errno != EAGAIN) {
                // Error de lectura (posible desconexión)
                closeJoystick();
                std::cout << "Error de lectura. Control desconectado. Esperando reconexión..." << std::endl;
                was_connected = false;
            }
        }

        // Pequeña pausa para no saturar la CPU
        usleep(16000); // ~60 lecturas por segundo
    }
    
    closeJoystick();
    return 0;
}