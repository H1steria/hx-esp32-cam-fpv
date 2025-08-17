# Análisis del Envío de Comandos de Control (GS hacia ESP32)

Este documento explica cómo se envían los comandos de control desde la Ground Station (GS) al ESP32 y cómo se puede modificar este comportamiento para usar botones en la interfaz gráfica en lugar de botones físicos de una Raspberry Pi.

## Resumen del Mecanismo de Control

El sistema no envía "comandos" individuales como "inicia grabación". En su lugar, funciona de la siguiente manera:

1.  **Paquete de Estado Constante:** La Ground Station (GS) envía continuamente un paquete de datos llamado `Ground2Air_Config_Packet` al ESP32. Este paquete contiene toda la configuración de la cámara y del canal de comunicación.
2.  **Contadores de Botones:** Dentro de ese paquete, en la estructura anidada `DataChannelConfig`, existen campos especiales que actúan como contadores, por ejemplo, `air_record_btn`.
3.  **Interacción en la GS:** Cuando presionas un botón en la interfaz gráfica de la GS, el valor de uno de estos contadores se incrementa en 1.
4.  **Envío Periódico:** Un hilo de comunicación separado (`comms_thread_proc`) toma la versión más reciente del paquete de configuración (con el contador ya incrementado) y lo envía por WiFi al ESP32. Esto ocurre aproximadamente cada 500 milisegundos.
5.  **Detección en el ESP32:** El firmware del ESP32 recibe este paquete, guarda el último valor conocido de los contadores y, en cada nuevo paquete, comprueba si el valor ha cambiado. Si el contador se ha incrementado, el ESP32 ejecuta la acción asociada (como iniciar o detener la grabación).

Este método es robusto porque no importa si un paquete se pierde; mientras uno con el valor actualizado llegue eventualmente, la acción se ejecutará.

## Flujo Detallado en el Código

Aquí están las partes clave del código que implementan este flujo:

#### 1. La Estructura de Datos (El "Comando")

El paquete principal que se envía se define en [`components/common/packets.h`](components/common/packets.h).

*   `Ground2Air_Config_Packet`: Es la estructura que contiene toda la información que viaja de la GS al ESP32.
*   `DataChannelConfig`: Dentro del paquete anterior, esta estructura contiene los contadores de los botones que nos interesan:
    *   `uint8_t air_record_btn = 0;`
    *   `uint8_t profile1_btn = 0;`
    *   `uint8_t profile2_btn = 0;`

    El comentario en el código es clave: `//incremented each time button is pressed on gs`.

#### 2. El Hilo de Envío (Dónde se envía)

En [`gs/src/main.cpp`](gs/src/main.cpp), la función `comms_thread_proc()` se ejecuta en un hilo separado y es la responsable de toda la comunicación.

*   **Líneas 535-549:** Dentro de su bucle infinito, este bloque de código se ejecuta cada 500ms. Prepara y envía el paquete de configuración global `s_ground2air_config_packet`.

    ```cpp
    // gs/src/main.cpp:537
    if ( s_got_config_packet )
    {
        std::lock_guard<std::mutex> lg(s_ground2air_config_packet_mutex);
        auto& config = s_ground2air_config_packet;
        // ... (prepara el paquete)
        s_comms.send(&config, sizeof(config), true);
    }
    ```

#### 3. La Interfaz Gráfica (Cómo se modifica el comando)

La interfaz se dibuja en la función `run()` dentro de [`gs/src/main.cpp`](gs/src/main.cpp). La lógica de los botones que ya existen nos da el patrón a seguir.

*   **Líneas 2131-2134:** Aquí puedes ver cómo el botón "Air Record" de la ventana de depuración implementa este mecanismo.

    ```cpp
    // gs/src/main.cpp:2131
    if ( ImGui::Button("Air Record") )
    {
        config.dataChannel.air_record_btn++;
    }
    ```

    Cuando se hace clic en el botón, simplemente se incrementa el contador `air_record_btn` en la variable local `config`.

*   **Línea 2268:** Al final del bucle de la interfaz, la configuración local modificada (`config`) se copia de nuevo a la variable global `s_ground2air_config_packet`, dejándola lista para que el hilo de comunicación la envíe.

    ```cpp
    // gs/src/main.cpp:2268
    s_ground2air_config_packet = config;
    ```

## Cómo Implementar Tus Propios Botones

Ahora que entiendes el flujo, puedes añadir tus propios botones fácilmente.

1.  **Ve al archivo** [`gs/src/main.cpp`](gs/src/main.cpp).
2.  **Busca la función `run()`** y la expresión lambda `auto f = [&config,&argv]` que se define dentro (alrededor de la línea 1182). Este es el corazón de la interfaz gráfica.
3.  **Añade una nueva ventana de ImGui** para tus controles. Puedes poner este código junto a las otras ventanas, por ejemplo, después del bloque `//------------ debug window`.

    ```cpp
    // Pon esto dentro de la lambda 'f', por ejemplo, antes de la línea 2167

    ImGui::Begin("Mis Controles"); // Crea una nueva ventana llamada "Mis Controles"

    // Botón para activar el perfil 1
    if (ImGui::Button("Activar Perfil 1"))
    {
        // Al hacer clic, incrementamos el contador del botón de perfil 1.
        // El ESP32 debe estar programado para reaccionar a este cambio.
        config.dataChannel.profile1_btn++;
    }

    ImGui::SameLine(); // Coloca el siguiente botón en la misma línea

    // Botón para activar el perfil 2
    if (ImGui::Button("Activar Perfil 2"))
    {
        config.dataChannel.profile2_btn++;
    }

    // Puedes añadir más lógica aquí si es necesario

    ImGui::End(); // Cierra la ventana
    ```

4.  **¡Listo!** Al compilar y ejecutar, verás una nueva ventana con tus botones. Al hacer clic, estarás modificando el paquete de configuración que se envía al ESP32, replicando exactamente la funcionalidad que tenían los botones físicos de la Raspberry Pi.

## Diagrama de Flujo

```mermaid
graph TD
    subgraph Ground Station (GS)
        A[Bucle de UI en main.cpp] --> B{ImGui::Button("Mi Botón")};
        B -- Clic del Usuario --> C{config.dataChannel.profile1_btn++};
        C --> D[Copia a s_ground2air_config_packet global];
        
        E[Hilo comms_thread_proc] -- Cada 500ms --> F{Lee s_ground2air_config_packet};
        F --> G[Envía paquete por WiFi];
    end

    subgraph ESP32 (Air Unit)
        G --> H[Recibe Ground2Air_Config_Packet];
        H --> I{Compara valor de profile1_btn con el anterior};
        I -- Si ha cambiado --> J[Ejecuta Acción del Perfil 1];
    end