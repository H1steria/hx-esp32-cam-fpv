import pygame

# Inicializa Pygame y el módulo de joystick
pygame.init()
pygame.joystick.init()

# Verifica si hay joysticks conectados
joystick_count = pygame.joystick.get_count()
if joystick_count == 0:
    print("No se encontraron controles.")
else:
    # Usa el primer joystick encontrado
    joystick = pygame.joystick.Joystick(0)
    joystick.init()
    print(f"Control inicializado: {joystick.get_name()}")

    # Bucle principal para leer los eventos del control
    try:
        while True:
            # Procesa la cola de eventos de Pygame
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    pygame.quit()
                    exit()

                # Eventos de botones
                if event.type == pygame.JOYBUTTONDOWN:
                    print(f"Botón {event.button} presionado")
                if event.type == pygame.JOYBUTTONUP:
                    print(f"Botón {event.button} liberado")

                # Eventos de los ejes (sticks analógicos)
                if event.type == pygame.JOYAXISMOTION:
                    # Los valores de los ejes van de -1.0 a 1.0
                    print(f"Eje {event.axis} movido a {event.value:.2f}")

                # Eventos del D-pad (hat)
                if event.type == pygame.JOYHATMOTION:
                    print(f"Hat {event.hat} movido a {event.value}")

    except KeyboardInterrupt:
        print("\nPrograma terminado.")
    finally:
        pygame.quit()
