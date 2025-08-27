# Documentación Técnica: air_firmware_esp32cam

## 1. Introducción

El firmware `air_firmware_esp32cam` es una solución de transmisión de video en tiempo real diseñada para dispositivos ESP32-CAM. Este firmware permite capturar video desde un módulo de cámara OV2640 u OV5640, comprimirlo en formato JPEG y transmitirlo a través de una conexión Wi-Fi a una estación receptora (ground station).

## 2. Arquitectura General del Sistema

El sistema está basado en el ESP-IDF (Espressif IoT Development Framework) y utiliza múltiples componentes para manejar diferentes aspectos del funcionamiento:

- **Captura de Video**: Utiliza el sensor de cámara conectado al ESP32-CAM
- **Procesamiento de Imagen**: Aplicación de configuraciones de calidad, brillo, contraste, etc.
- **Compresión JPEG**: Compresión de los fotogramas capturados
- **Transmisión Wi-Fi**: Envío de datos a través de una conexión Wi-Fi punto a punto
- **Codificación FEC**: Aplicación de Forward Error Correction para mejorar la robustez de la transmisión
- **OSD (On-Screen Display)**: Superposición de información en el video transmitido
- **Control y Configuración**: Manejo de comandos de configuración desde la estación base

## 3. Componentes Principales

### 3.1 Captura y Procesamiento de Video

El firmware utiliza la biblioteca `esp32-camera` para interactuar con el sensor de imagen. La captura se realiza en modo JPEG directamente desde el sensor, lo que reduce la carga de procesamiento en el ESP32.

**Características clave:**
- Soporte para sensores OV2640 y OV5640
- Resoluciones configurables desde QVGA (320x240) hasta UXGA (1600x1200)
- Control de parámetros de imagen: brillo, contraste, saturación, nitidez, balance de blancos, etc.
- Control de tasa de fotogramas (FPS) configurable

### 3.2 Transmisión Wi-Fi y Protocolo de Paquetes

La comunicación se realiza a través de una conexión Wi-Fi en modo estación (STA) o punto de acceso (AP), utilizando paquetes personalizados para la transmisión de datos.

**Tipos de Paquetes:**
1. **Paquetes de Video (Air2Ground_Video_Packet)**
   - Contienen fragmentos de datos JPEG
   - Incluyen información de índice de fotograma y parte
   - Señalan el final de un fotograma

2. **Paquetes de OSD (Air2Ground_OSD_Packet)**
   - Contienen información de superposición en pantalla
   - Incluyen estadísticas del sistema
   - Se actualizan periódicamente

3. **Paquetes de Configuración (Air2Ground_Config_Packet)**
   - Contienen la configuración actual de la cámara y transmisión
   - Se envían para sincronizar la configuración entre aire y tierra

4. **Paquetes de Telemetría (Air2Ground_Data_Packet)**
   - Contienen datos de telemetría del sistema
   - Se utilizan para enviar información adicional como datos de control

### 3.3 Sistema FEC (Forward Error Correction)

Para mejorar la robustez de la transmisión en entornos con interferencias, el firmware implementa un sistema FEC basado en el algoritmo Reed-Solomon.

**Funcionamiento:**
- Los datos se dividen en bloques de tamaño K
- Se generan N-K paquetes de paridad adicionales
- En el receptor, se pueden recuperar los datos originales incluso si se pierden algunos paquetes
- Los parámetros K y N son configurables

### 3.4 OSD (On-Screen Display)

El sistema OSD permite superponer información en el video transmitido, como estadísticas del sistema, estado de la conexión, etc.

**Información mostrada:**
- Calidad de la señal Wi-Fi (RSSI)
- Tasa de fotogramas actual
- Estado de grabación
- Temperatura del sensor
- Estadísticas de transmisión
- Espacio disponible en tarjeta SD (si está disponible)

### 3.5 Configuración y Control

El firmware permite la configuración remota a través de paquetes de configuración enviados desde la estación base.

**Parámetros configurables:**
- Resolución de video
- Calidad de imagen
- Brillo, contraste, saturación
- Canal y potencia Wi-Fi
- Parámetros FEC
- Tasa de fotogramas límite

### 3.6 GPIO y Control Externo

El firmware incluye soporte para controlar pines GPIO externos, lo que permite integrar funciones adicionales como control de relés, LEDs, etc.

**Configuración:**
- El pin GPIO4 se utiliza por defecto para control externo
- Se puede activar/desactivar mediante comandos desde la estación base

### 3.7 Manejo de Temperatura

El firmware incluye un sistema de monitoreo de temperatura del sensor de imagen para prevenir el sobrecalentamiento.

**Funciones:**
- Lectura periódica de la temperatura del sensor
- Reporte de temperatura en los paquetes OSD
- Posible implementación de trottling térmico (no implementado en la versión actual)

## 4. Diagrama de Flujo de Datos

```
[Cámara] → JPEG → [Callback camera_data_available]
                            ↓
                    [Procesamiento de datos JPEG]
                            ↓
              [División en paquetes Air2Ground_Video_Packet]
                            ↓
                    [Codificación FEC (s_fec_encoder)]
                            ↓
                [Cola de transmisión Wi-Fi (s_wlan_outgoing_queue)]
                            ↓
                        [Transmisión Wi-Fi]
                            ↓
                    [Recepción en estación base]
                            ↓
                [Decodificación FEC (s_fec_decoder)]
                            ↓
                    [Procesamiento de paquetes]
                            ↓
                        [Visualización]
```

## 5. Descripción de los Paquetes de Transmisión

### 5.1 Paquetes Air2Ground_Video_Packet

Estos paquetes contienen los datos de video JPEG fragmentados.

**Estructura:**
- `type`: Tipo de paquete (Video)
- `resolution`: Resolución del video
- `frame_index`: Índice del fotograma
- `part_index`: Índice de la parte dentro del fotograma
- `last_part`: Indicador de última parte del fotograma
- `size`: Tamaño total del paquete
- `pong`: Utilizado para medir latencia
- `version`: Versión del protocolo
- `airDeviceId`: ID del dispositivo aire
- `gsDeviceId`: ID del dispositivo estación base
- `crc`: Código de verificación

### 5.2 Paquetes Air2Ground_OSD_Packet

Estos paquetes contienen información de OSD y estadísticas del sistema.

**Estructura:**
- `type`: Tipo de paquete (OSD)
- `size`: Tamaño total del paquete
- `pong`: Utilizado para medir latencia
- `version`: Versión del protocolo
- `airDeviceId`: ID del dispositivo aire
- `gsDeviceId`: ID del dispositivo estación base
- `stats`: Estadísticas del sistema (ver estructura AirStats)
- `buffer`: Datos del buffer OSD
- `crc`: Código de verificación

### 5.3 Paquetes Air2Ground_Config_Packet

Estos paquetes contienen la configuración actual del dispositivo.

**Estructura:**
- `type`: Tipo de paquete (Config)
- `size`: Tamaño total del paquete
- `version`: Versión del protocolo
- `airDeviceId`: ID del dispositivo aire
- `gsDeviceId`: ID del dispositivo estación base
- `camera`: Configuración de la cámara (ver estructura CameraConfig)
- `dataChannel`: Configuración del canal de datos (ver estructura DataChannelConfig)
- `crc`: Código de verificación

### 5.4 Paquetes Ground2Air

Estos paquetes se envían desde la estación base al dispositivo aire.

**Tipos:**
- `Config`: Configuración del dispositivo
- `Connect`: Inicialización de conexión
- `Control`: Comandos de control
- `Telemetry`: Datos de telemetría

## 6. Asignación de Pines GPIO

La asignación de pines GPIO sigue el esquema típico del módulo ESP32-CAM:

**Pines de la Cámara (OV2640/OV5640):**
- `Y2_GPIO_NUM`: GPIO5
- `Y3_GPIO_NUM`: GPIO18
- `Y4_GPIO_NUM`: GPIO19
- `Y5_GPIO_NUM`: GPIO21
- `Y6_GPIO_NUM`: GPIO36
- `Y7_GPIO_NUM`: GPIO39
- `Y8_GPIO_NUM`: GPIO34
- `Y9_GPIO_NUM`: GPIO35
- `XCLK_GPIO_NUM`: GPIO0
- `PCLK_GPIO_NUM`: GPIO22
- `VSYNC_GPIO_NUM`: GPIO25
- `HREF_GPIO_NUM`: GPIO23
- `SIOD_GPIO_NUM`: GPIO26
- `SIOC_GPIO_NUM`: GPIO27
- `PWDN_GPIO_NUM`: GPIO32
- `RESET_GPIO_NUM`: -1 (no utilizado)

**Pines de Control:**
- `GPIO_CONTROL_PIN`: GPIO4 (configurable para control externo)

**Pines UART:**
- `TXD0_PIN`: GPIO1
- `RXD0_PIN`: GPIO3

## 7. Procedimiento de Inicialización

1. **Inicialización del Sistema:**
   - Configuración de UART para depuración
   - Inicialización de NVS (Non-Volatile Storage)
   - Lectura de configuración almacenada

2. **Configuración Wi-Fi:**
   - Inicialización del subsistema Wi-Fi
   - Configuración del canal, tasa y potencia
   - Configuración del modo promiscuo para recepción de paquetes

3. **Inicialización de FEC:**
   - Configuración del codificador y decodificador FEC
   - Inicialización de colas de transmisión y recepción

4. **Inicialización de la Cámara:**
   - Configuración del sensor de imagen
   - Aplicación de parámetros de configuración
   - Inicio de la captura de video

5. **Creación de Tareas:**
   - Tarea de transmisión Wi-Fi
   - Tarea de recepción Wi-Fi
   - Bucle principal de procesamiento

## 8. Captura y Transmisión de Video

### 8.1 Proceso de Captura

1. El sensor de imagen captura fotogramas en formato JPEG
2. Los datos JPEG se envían al callback `camera_data_available`
3. En el callback, los datos se procesan y se dividen en paquetes
4. Los paquetes se codifican con FEC y se añaden a la cola de transmisión

### 8.2 Control de Calidad Adaptativo

El firmware implementa un sistema de control de calidad adaptativo para mantener una transmisión estable:

1. **Medición de Ancho de Banda:**
   - Monitoreo del uso de la cola Wi-Fi
   - Conteo de errores de transmisión
   - Medición de la tasa de fotogramas efectiva

2. **Ajuste de Calidad:**
   - Modificación dinámica de la calidad JPEG
   - Ajuste de nitidez según la calidad
   - Control de tamaño máximo de fotograma

3. **Prevención de Sobrecarga:**
   - Detección de sobrecarga en la cola Wi-Fi
   - Reducción de calidad cuando se detecta congestión
   - Parada temporal de la transmisión si es necesario

## 9. Procesamiento de Paquetes Recibidos

### 9.1 Recepción de Paquetes

1. Los paquetes se reciben a través del callback `packet_received_cb`
2. Se verifica el canal Wi-Fi y la dirección MAC
3. Los paquetes se filtran según el ID del dispositivo
4. Los datos se decodifican con FEC y se añaden a la cola de recepción

### 9.2 Manejo de Paquetes de Configuración

1. Los paquetes de configuración se procesan en `handle_ground2air_config_packet`
2. Se actualizan los parámetros de configuración
3. Se aplican cambios en tiempo real cuando es posible
4. Algunos cambios requieren reinicio o reconfiguración de la cámara

### 9.3 Manejo de Paquetes de Control

1. Los paquetes de control se procesan en `handle_ground2air_control_packet`
2. Se ejecutan acciones específicas como activación de GPIO
3. Se envían confirmaciones cuando es necesario

## 10. Recopilación de Estadísticas

El firmware recopila y transmite estadísticas del sistema periódicamente:

### 10.1 Estadísticas de Transmisión
- Datos enviados y recibidos
- Errores de transmisión
- Tasa de fotogramas actual y esperada
- Uso de la cola Wi-Fi

### 10.2 Estadísticas de la Cámara
- Tamaño mínimo y máximo de fotogramas
- Conteo de sobrecargas de la cámara
- Temperatura del sensor

### 10.3 Estadísticas de Sistema
- Uso de memoria
- Estado de la tarjeta SD (si está disponible)
- Estado de grabación

## 11. Funcionamiento de la Transmisión Unidireccional

El sistema está diseñado principalmente como una transmisión unidireccional desde el dispositivo aire a la estación base:

1. **Transmisión de Video:**
   - Los fotogramas JPEG se transmiten continuamente
   - No se espera confirmación de recepción
   - La calidad se ajusta dinámicamente según las condiciones

2. **Transmisión de OSD:**
   - Información de estado se envía periódicamente
   - Incluye estadísticas del sistema y estado de la conexión

3. **Transmisión de Configuración:**
   - La configuración actual se envía periódicamente
   - Permite a la estación base mantener sincronización

4. **Recepción de Comandos:**
   - La estación base puede enviar comandos de configuración
   - Los comandos se procesan y aplican en tiempo real

## 12. Medidas de Integridad de Datos

### 12.1 Verificación de CRC

Todos los paquetes incluyen un código CRC-8 para verificar la integridad de los datos:

1. Antes de enviar, se calcula el CRC del paquete
2. Al recibir, se verifica el CRC contra el contenido
3. Los paquetes con CRC inválido se descartan

### 12.2 Filtrado de Paquetes

El sistema implementa un mecanismo de filtrado de paquetes:

1. **Filtrado por ID de Dispositivo:**
   - Solo se procesan paquetes dirigidos al dispositivo
   - Se pueden aceptar conexiones de dispositivos específicos

2. **Filtrado por Canal:**
   - Solo se procesan paquetes en el canal configurado
   - Se ignoran paquetes de otros canales

### 12.3 Forward Error Correction

El sistema FEC ayuda a mantener la integridad de los datos en entornos con interferencias:

1. **Codificación en el Transmisor:**
   - Los datos se dividen en bloques
   - Se generan paquetes de paridad adicionales

2. **Decodificación en el Receptor:**
   - Se pueden recuperar datos perdidos usando paquetes de paridad
   - Se requiere un número mínimo de paquetes para la recuperación

## 13. Optimizaciones y Consideraciones de Rendimiento

### 13.1 Uso de Memoria

- Uso extensivo de PSRAM para colas de transmisión
- Optimización de estructuras de datos para minimizar el uso de memoria
- Uso de buffers circulares para manejo eficiente de datos

### 13.2 Optimización de CPU

- Uso de funciones IRAM_ATTR para código crítico
- Minimización de operaciones en callbacks de interrupción
- Uso de tareas separadas para procesamiento intensivo

### 13.3 Optimización de Wi-Fi

- Configuración de tasa y potencia fijas para maximizar el rendimiento
- Uso de modo HT20 para mejor estabilidad
- Monitoreo continuo del estado de la conexión

## 14. Diagnóstico y Depuración

### 14.1 Salida de Depuración

El firmware puede generar salida de depuración a través del UART0:

- Estadísticas del sistema cada segundo
- Información de errores y advertencias
- Información de estado de componentes

### 14.2 Monitoreo de Rendimiento

- Medición de uso de CPU por tarea
- Monitoreo de uso de memoria
- Estadísticas detalladas de transmisión

## 15. Consideraciones de Seguridad

### 15.1 Identificación de Dispositivos

- Cada dispositivo tiene un ID único generado a partir de su dirección MAC
- Los paquetes se filtran según los IDs de origen y destino

### 15.2 Privacidad

- No se implementa encriptación de datos
- Las transmisiones pueden ser interceptadas si no se protegen a nivel de red

## 16. Posibles Mejoras Futuras

1. **Encriptación de Datos:**
   - Implementar encriptación AES para proteger la transmisión

2. **Protocolo de Control Bidireccional:**
   - Mejorar el protocolo de control para comunicación más robusta

3. **Soporte para Más Sensores:**
   - Ampliar el soporte a más modelos de sensores de imagen

4. **Mejoras en OSD:**
   - Implementar más opciones de personalización del OSD

5. **Optimización de FEC:**
   - Implementar algoritmos FEC más eficientes

6. **Soporte para Audio:**
   - Añadir capacidad de transmisión de audio

## 17. Conclusión

El firmware `air_firmware_esp32cam` proporciona una solución completa para la transmisión de video en tiempo real utilizando un ESP32-CAM. Con su sistema de control de calidad adaptativo, FEC para robustez, y capacidad de configuración remota, es ideal para aplicaciones de FPV (First Person View) y otros usos donde se requiere una transmisión de video confiable en entornos con posibles interferencias.

La arquitectura modular del firmware permite fácil mantenimiento y extensión, mientras que las optimizaciones de rendimiento aseguran una transmisión fluida incluso en condiciones desafiantes.
