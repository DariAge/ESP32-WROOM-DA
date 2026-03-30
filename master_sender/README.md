Master Sender: HMI Control System for ESP32

Este proyecto implementa un Maestro de Control HMI basado en ESP-IDF v5.x para un sistema industrial de 
lazo cerrado (calefacción y velocidad). Utiliza un modelo de tareas concurrentes con FreeRTOS para 
gestionar entradas de usuario, comunicación UART con un esclavo y visualización en tiempo real mediante
un display OLED SSD1306.

🚀 Características Principales

Arquitectura Multitarea: 3 tareas principales (uart_rx_task, input_task, oled_task) con prioridades balanceadas.

Gestión de Estados: Uso de Event Groups para sincronizar estados RUNNING, STOP y ERROR.

Sincronización: Acceso a memoria compartida protegido mediante Mutexes y comunicación asincrónica vía Queues.

Interfaz Fluida: Renderizado optimizado del OLED para evitar el parpadeo (flicker) mediante actualización por página y padding de caracteres.

🛠 Configuración de Hardware (Pinout)

| Periférico 			 | Pin ESP32 | Función		   	   | Notas 					 	  |
| :--- 					 | :---:     | :---: 			   | :--- |
| **Display OLED (SDA)** | GPIO 21   | I2C_NUM_0 		   | Datos I2C (Bus Maestro) 	  |
| **Display OLED (SCL)** | GPIO 22   | I2C_NUM_0 		   | Reloj I2C (400kHz) 		  |
| **UART RX** 			 | GPIO 18   | UART_NUM_2 	   	   | Recepción desde el Esclavo   |
| **UART TX** 			 | GPIO 17   | UART_NUM_2 	   	   | Transmisión hacia el Esclavo |
| **Botón START** 		 | GPIO 4    | Entrada | Pull-up   | Inicia la marcha del motor   |
| **Botón STOP** 		 | GPIO 5    | Entrada | Pull-up   | Detiene el sistema 		  |
| **Botón INC RPM** 	 | GPIO 27   | Entrada | Pull-up   | +10 RPM (Solo en STOP)       |
| **Botón DEC RPM** 	 | GPIO 19   | Entrada | Pull-up   | -10 RPM (Solo en STOP)       |
| **Botón INC SLOPE** 	 | GPIO 23   | Entrada | Pull-up   | +5.0 R/s (Rampa)             |
| **Botón DEC SLOPE** 	 | GPIO 33   | Entrada | Pull-up   | -5.0 R/s (Rampa)             |


📂 Preparación del Entorno (IMPORTANTE)

Para compilar este proyecto, es necesario incluir el driver del OLED como un componente local.
En la raíz de tu proyecto, crea una carpeta llamada components.
Clona o descarga la librería de nopnop2002 para SSD1306:
Referencia: esp-idf-ssd1306 por nopnop2002.

Corrección de Compatibilidad (v5.3+):
Para evitar conflictos entre el driver I2C antiguo y el nuevo (driver_ng), debes modificar el archivo 
components/ssd1306/CMakeLists.txt para que solo compile los archivos legacy:

CMake:

set(component_srcs "ssd1306.c" "ssd1306_i2c_legacy.c" "ssd1306_spi.c")
set(component_requires "driver esp_driver_spi esp_driver_i2c esp_driver_gpio")

idf_component_register(SRCS "${component_srcs}" 
                    REQUIRES "${component_requires}" 
                    INCLUDE_DIRS ".")
                    

📂 Preparación del Entorno (IMPORTANTE)
Para compilar este proyecto, es necesario incluir el driver del OLED como un componente local.

En la raíz de tu proyecto, crea una carpeta llamada components.

Clona o descarga la librería de nopnop2002 para SSD1306:

Referencia: esp-idf-ssd1306 por nopnop2002.

Corrección de Compatibilidad (v5.3+):
Para evitar conflictos entre el driver I2C antiguo y el nuevo (driver_ng), debes modificar el archivo components/ssd1306/CMakeLists.txt para que solo compile los archivos legacy:

CMake
set(component_srcs "ssd1306.c" "ssd1306_i2c_legacy.c" "ssd1306_spi.c")
set(component_requires "driver esp_driver_spi esp_driver_i2c esp_driver_gpio")

idf_component_register(SRCS "${component_srcs}" 
                    REQUIRES "${component_requires}" 
                    INCLUDE_DIRS ".")
                    
                    
💻 Compilación y Ejecución
Asegúrate de tener configurado tu entorno de ESP-IDF (recomendado v5.3.1).

Bash
# Limpiar compilaciones previas si hubo cambios en componentes
idf.py fullclean

# Compilar y flashear
idf.py build flash monitor


📡 Protocolo de Comunicación (UART)
El sistema se comunica con el esclavo mediante tramas ASCII:

Maestro -> Esclavo: $START#, $STOP#, $SET_RPM:1000#, $SET_SLOPE:50#.

Esclavo -> Maestro: $DAT:<REAL_RPM>,<RAMP_RPM>,<CURR_MA>,<STATE>#.


✍️ Créditos y Referencias

Driver OLED: Basado en la librería de nopnop2002, adaptada para compatibilidad 
con el driver I2C legacy en versiones recientes de ESP-IDF.

