/**
 **master_sender
 * 
 *@brief: Maestro encargado de HMI para sistema a lazo cerrado de calefacción y velocidad.
 *        Envía comandos y valores por UART y recibe valores medidos de periféricos por el Esclavo.
 */

#include <stdio.h>
#include <string.h>
#include <math.h>
#include <inttypes.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/timers.h"
#include "freertos/event_groups.h"

#include "esp_log.h"
#include "portmacro.h"

#include "driver/gpio.h"
#include "driver/uart.h"
#include "driver/i2c.h"
 
#include "ssd1306.h"

// --- CONFIGURACIÓN DE PINES ---
#define GPIO_START_BUTTON    4
#define GPIO_STOP_BUTTON     5
#define GPIO_INC_BUTTON      27
#define GPIO_DEC_BUTTON      19
#define GPIO_SLOPE_INC       23
#define GPIO_SLOPE_DEC       33

#define RX_PIN 16 
#define TX_PIN 17
#define UART_PORT UART_NUM_2

// --- PARÁMETROS DE CONTROL ---
#define CONTROL_LOOP_MS      100
#define DEFAULT_SETPOINT	 500
#define MAX_SETPOINT		 1500
#define MIN_SETPOINT		 50
#define SETPOINT_STEP		 10

#define DEFAULT_SLOPE        50.0f
#define MIN_SLOPE            5.0f
#define MAX_SLOPE            200.0f
#define SLOPE_STEP           5.0f

#define EVENT_BIT_RUNNING      (1 << 0) // El sistema está habilitado para girar
#define EVENT_BIT_STOP       (1 << 1) // El sistema debe detenerse
#define EVENT_BIT_ERROR_OC   (1 << 2) // Hay una falla de sobrecorriente

#define BUF_SIZE_RX 1024
#define BUF_SIZE_TX 64

#define I2C_MASTER_SCL_IO           22      // Pin SCL
#define I2C_MASTER_SDA_IO           21      // Pin SDA
#define I2C_MASTER_NUM              I2C_NUM_0 
#define I2C_MASTER_FREQ_HZ          400000  // 400kHz

//Estructuras de datos para colas

//Estructura para trabajar con recepcion uart y envio por *arg a oled_task
typedef struct {
    uint16_t setpoint_rpm;
    float current_ramp_rpm;
    float current_real_rpm;
    float current_ma; 
    uint8_t system_state;// 0: STOP, 1: RUN, 2: ERROR OVER-CURRENT
    SemaphoreHandle_t xSlaveParamsMutex;
} SlaveStatus_t;

//Estructura para trabajar con input_Task en el ingreso por pulsadores y envío por uart a esclavo
typedef struct {
    uint16_t setpoint_rpm;
    float ramp_slope;
    SemaphoreHandle_t xMotorParamsMutex;
    EventGroupHandle_t event_group;
} MotorParams_t;

//Estructura para comunicar input_task con oled_task mediante oled_ui_queue
typedef struct {
    uint16_t target_setpoint;
    float target_slope;
    EventBits_t control_bits; // Para saber si estamos en START, STOP o ERROR
} OledUiMsg_t;

// --- VARIABLES GLOBALES ---

static EventGroupHandle_t s_system_event_group; //handler para EventGroup en maq de estados
static QueueHandle_t gpio_evt_queue = NULL; //ISR a input_task
static QueueHandle_t oled_ui_queue = NULL; //input_task a oled_task


// --- PROTOTIPOS Y FUNCIONES
static void IRAM_ATTR gpio_isr_handler(void* arg) {
    uint32_t gpio_num = (uint32_t) arg;
    xQueueSendFromISR(gpio_evt_queue, &gpio_num, NULL);
}

void init_uart() {
    uart_config_t uart_config = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
    };
    uart_driver_install(UART_PORT, BUF_SIZE_RX * 2, 0, 0, NULL, 0);
    uart_param_config(UART_PORT, &uart_config);
    uart_set_pin(UART_PORT, TX_PIN, RX_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
}

void i2c_init_bus() {
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ,
    };
    i2c_param_config(I2C_MASTER_NUM, &conf);
    i2c_driver_install(I2C_MASTER_NUM, conf.mode, 0, 0, 0);
}

//------------------------------TAREAS-----------------------------------------
//-----------------------------------------------------------------------------


// --- TAREA: UART_TASK ---
/** 		 
 *@brief:  Recibe cada 200ms cadena con formato $DAT:<REAL_RPM>,<CURRENT_RAMP_RPM>,<I>,<STATE_NUMBER>#		  
 */
 void uart_rx_task(void *arg) {
    uint8_t rx_byte;
    char rx_buffer[BUF_SIZE_TX];
    int rx_idx = 0;
    SlaveStatus_t *slave_data = (SlaveStatus_t *)arg;

    while(1) {
        // Leemos de a un byte para no perder la sincronía
        if (uart_read_bytes(UART_PORT, &rx_byte, 1, portMAX_DELAY) > 0) {
            
            if (rx_byte == '$') { // Inicio de trama
                rx_idx = 0;
                rx_buffer[rx_idx++] = rx_byte;
            } 
            else if (rx_byte == '#') { // Fin de trama
                rx_buffer[rx_idx++] = '\0';
                
                // Procesar trama completa: rx_buffer debe ser "$DAT:..."
                if (strncmp(rx_buffer, "$DAT:", 5) == 0) {
                    float f_real, f_ramp, f_curr;
                    uint8_t u_state;
                    if (sscanf(rx_buffer + 5, "%f,%f,%f,%hhu", &f_real, &f_ramp, &f_curr, &u_state) == 4) {
                        if (xSemaphoreTake(slave_data->xSlaveParamsMutex, pdMS_TO_TICKS(10))) {
                            slave_data->current_real_rpm = f_real;
                            slave_data->current_ramp_rpm = f_ramp;
                            slave_data->current_ma = f_curr;
                            slave_data->system_state = u_state;
                            xSemaphoreGive(slave_data->xSlaveParamsMutex);
                        }
                    }
                }
                rx_idx = 0; // Reset para la próxima
            } 
            else if (rx_idx > 0 && rx_idx < BUF_SIZE_TX - 1) {
                rx_buffer[rx_idx++] = rx_byte;
            }
        }
    }
}
/*
void uart_rx_task(void *arg) {
    uint8_t data [BUF_SIZE_RX];
    //SlaveStatus_t datos_motor;
    SlaveStatus_t *slave_data = (SlaveStatus_t *)arg;
    
    while(1) {
		// Usamos portMAX_DELAY para que la tarea no consuma nada de CPU hasta que lleguen bytes
		int len = uart_read_bytes(UART_PORT, data, BUF_SIZE_RX - 1, portMAX_DELAY);
        if (len > 0)
        {
			data[len] = '\0';
            char* str_data = (char*)data;
            
            // Buscar inicio y fin de trama: $...#
            char* start = strchr(str_data, '$');
            char* end = strchr(str_data, '#');
            if (start && end && (end > start)) {
                *end = '\0'; // Cortamos el final
                char* payload = start + 1; // Saltamos el '$'
                
            	// Verificamos que sea una trama de datos (empieza con "DAT:")
            	if (strncmp(payload, "DAT:", 4) == 0) {
               		char* valores = payload + 4; // Saltamos el "DAT:"
                    
                    float f_real_rpm, f_ramp_rpm, f_curr_ma;
                    uint8_t u_state;
                    
               		// --- PARSER ---
               		// Formato esperado: %f,%f,%f,%d
               		int items = sscanf(valores, "%f,%f,%f,%hhu", 
                                       &f_real_rpm, 
                                       &f_ramp_rpm, 
                                       &f_curr_ma, 
                                       &u_state);
                    
               		if (items == 4) {
                  		
                  		// SECCIÓN CRÍTICA: Protegemos el acceso a la memoria compartida
                        if (xSemaphoreTake(slave_data->xSlaveParamsMutex, pdMS_TO_TICKS(10))) {
                            slave_data->current_real_rpm = f_real_rpm;
                            slave_data->current_ramp_rpm = f_ramp_rpm;
                            slave_data->current_ma = f_curr_ma;
                            slave_data->system_state = u_state;
                            xSemaphoreGive(slave_data->xSlaveParamsMutex);
                  		}
        
    				}
    			}
			}
		}
	}//fin while(1)
}
*/

// --- TAREA: PROCESAMIENTO DE INPUTS ---
/**
 *@brief: Escribe en la cola oled_ui_queue cuando recibe de la ISR el valor de pulsador 
 *     	  que corresponde a START, STOP, INC_RPM, DEC_RPM, INC_SLOPE, DEC_SLOPE. 
 *		  Modifica variables locales a enviar 
 *		  Envía comandos por UART en función de pulsadores presionados.
 *        Comandos a enviar: START - STOP - SET_RPM:<payload> - SET_SLOPE:<payload>
 */
void input_task(void *arg) {
	
	uint16_t local_setpoint = DEFAULT_SETPOINT; // Valor inicial
	uint16_t local_slope = DEFAULT_SLOPE; //valor inicial 
	char uart_buffer[32]; //para enviar cadena de texto a esclavo asincronicamente
	OledUiMsg_t ui_msg; //mensaje para enviar a oled_task
	
    uint32_t io_num;
    
    for(;;) {
        if(xQueueReceive(gpio_evt_queue, &io_num, portMAX_DELAY)) {
            
            vTaskDelay(pdMS_TO_TICKS(50)); // Debounce
            
            if(gpio_get_level(io_num) == 0) {
               
                // Solo permitimos cambios de parámetros si el sistema está detenido
                EventBits_t status = xEventGroupGetBits(s_system_event_group);
                bool is_stopped = (status & EVENT_BIT_STOP) || (status & EVENT_BIT_ERROR_OC);              
             
                switch(io_num) {
                    case GPIO_START_BUTTON: 
                    	uart_write_bytes(UART_PORT, "$START#", 7);
                    	xEventGroupClearBits(s_system_event_group, EVENT_BIT_STOP);
            			status = xEventGroupSetBits(s_system_event_group, EVENT_BIT_RUNNING);
                        break;
                        
                    case GPIO_STOP_BUTTON: 
                        uart_write_bytes(UART_PORT, "$STOP#", 6);
                        xEventGroupClearBits(s_system_event_group, EVENT_BIT_RUNNING);
            			status = xEventGroupSetBits(s_system_event_group, EVENT_BIT_STOP);
                        break;
                        
                    case GPIO_INC_BUTTON:
                        
							if(local_setpoint <= MAX_SETPOINT) local_setpoint += SETPOINT_STEP;
							snprintf(uart_buffer, sizeof(uart_buffer), "$SET_RPM:%u#", local_setpoint);
                        	uart_write_bytes(UART_PORT, uart_buffer, strlen(uart_buffer));
                        
                        break;

                    case GPIO_DEC_BUTTON:
                        
                            if(local_setpoint >= MIN_SETPOINT) local_setpoint -= SETPOINT_STEP;
                            snprintf(uart_buffer, sizeof(uart_buffer), "$SET_RPM:%u#", local_setpoint);
                       		uart_write_bytes(UART_PORT, uart_buffer, strlen(uart_buffer));
                        
                        break;

                    case GPIO_SLOPE_INC:
                        if(is_stopped) {
                            if(local_slope <= MAX_SLOPE) local_slope += SLOPE_STEP;
							snprintf(uart_buffer, sizeof(uart_buffer), "$SET_SLOPE:%u#", local_slope);
                        	uart_write_bytes(UART_PORT, uart_buffer, strlen(uart_buffer));
                        }
                        break;

                    case GPIO_SLOPE_DEC:
                        if(is_stopped) {
                            if(local_slope >= MIN_SLOPE) local_slope -= SLOPE_STEP;
							snprintf(uart_buffer, sizeof(uart_buffer), "$SET_SLOPE:%u#", local_slope);
                        	uart_write_bytes(UART_PORT, uart_buffer, strlen(uart_buffer));
             			}
             			break;
                }
                
                // --- ACTUALIZACIÓN ASINCRÓNICA DEL OLED ---
                ui_msg.target_setpoint = local_setpoint;
                ui_msg.target_slope = local_slope;
                ui_msg.control_bits = status; // Mandamos la foto actual del EventGroup
                xQueueSend(oled_ui_queue, &ui_msg, 0);
                 
                //Debounce por vTaskDelay de 50ms se suma a los 10ms, que si o si, tarda cuando no se presiona un pulsador. 
                while(gpio_get_level(io_num) == 0) vTaskDelay(10);
            }
        }
    }
}


// --- TAREA: OLED TASK ---
/**
*@brief: recibe *arg con telemetría uart del dispositivo esclavo cada 200ms. A su vez, 
* 		 tiene una cola con timeout de 200ms para recibir asincrónicamente una cola con
*		 los valores de setpoint, slope y estado, que son modificados en input_task
*/
void oled_task(void *arg) {
    
	SlaveStatus_t *slave_data = (SlaveStatus_t *)arg; //recepcion de datos de uart_task por *arg
    OledUiMsg_t ui_info = { .target_setpoint = DEFAULT_SETPOINT, 
    					    .target_slope = DEFAULT_SLOPE, 
    					    .control_bits = EVENT_BIT_STOP }; //var recepcion de input_task
    SlaveStatus_t local_slave_copy;
    char buf[32];
    SSD1306_t dev;
    
    //Configuramos los campos mínimos necesarios
    dev._address = 0x3C;
    dev._i2c_num = I2C_NUM_0;
    dev._flip = false;      // ¿Pantalla rotada 180°?
    
    ESP_LOGI("OLED_task", "Inicializando SSD1306 en puerto %d...", dev._i2c_num);
    
    ssd1306_init(&dev, 128, 64);
    ssd1306_clear_screen(&dev, false);
    
    //Borrado de pantalla inicial
    ssd1306_clear_screen(&dev, false);
    // Leemos el estado inicial por única vez antes del loop, inicializado anteriormente
    ui_info.control_bits = xEventGroupGetBits(s_system_event_group);
    
    for(;;) 
    { 
		
      	// Esperamos una actualización de la UI o timeout de 200ms para refrescar telemetría
        // Esto permite que el OLED reaccione al instante a un botón o se refresque solo
        //modifico a no timeout
        if (xQueueReceive(oled_ui_queue, &ui_info, 0)) {
            // Se actualizó ui_info con los datos de la cola
        }
		
		//actualizo el eventgroupbits para saber estado actual
		ui_info.control_bits = xEventGroupGetBits(s_system_event_group);
		
        //Copiamos los datos del Esclavo cargado en uart_task (Telemetría) usando el Mutex
        if (xSemaphoreTake(slave_data->xSlaveParamsMutex, pdMS_TO_TICKS(20))) {
            local_slave_copy = *slave_data;
            xSemaphoreGive(slave_data->xSlaveParamsMutex);
        }

        //Dibujamos en pantalla
       
        // LÍNEA 0: ESTADO
        if (ui_info.control_bits & EVENT_BIT_RUNNING) 
            ssd1306_display_text(&dev, 0, "ESTADO: RUNNING ", 16, false);
        else if (ui_info.control_bits & EVENT_BIT_ERROR_OC)
            ssd1306_display_text(&dev, 0, "ESTADO: !ERROR! ", 16, true); 
        else
            ssd1306_display_text(&dev, 0, "ESTADO: STOPPED ", 16, false);

        // LÍNEA 2: SETPOINT (Etiqueta + Valor)
        snprintf(buf, sizeof(buf), "SET: %4u RPM   ", ui_info.target_setpoint);
        ssd1306_display_text(&dev, 2, buf, 16, false);
        
        // LÍNEA 3: SLOPE
        snprintf(buf, sizeof(buf), "SLP: %4.1f R/s  ", ui_info.target_slope);
        ssd1306_display_text(&dev, 3, buf, 16, false);

        // LÍNEA 5: SEPARADOR (Lo re-escribimos para asegurar que esté ahí)
        ssd1306_display_text(&dev, 5, "--- MOTOR ---   ", 16, false);

        // LÍNEA 6: VELOCIDAD REAL
        snprintf(buf, sizeof(buf), "VEL: %4.1f RPM  ", local_slave_copy.current_real_rpm);
        ssd1306_display_text(&dev, 6, buf, 16, false);

        // LÍNEA 7: CORRIENTE
        snprintf(buf, sizeof(buf), "COR: %4.2f mA   ", local_slave_copy.current_ma);
        ssd1306_display_text(&dev, 7, buf, 16, false);
        
		// 4. EL SECRETO: Delay obligatorio
        // Esto permite que el procesador respire y el IDLE task no dispare el Watchdog
        vTaskDelay(pdMS_TO_TICKS(200));
    }
}

void app_main(void) {
    // Inicializar Periféricos
    init_uart();
    i2c_init_bus();
    //init_ssd1306 en oled_task
    
    // Configurar GPIOs
    gpio_config_t btn_conf = {
        .intr_type = GPIO_INTR_NEGEDGE,
        .mode = GPIO_MODE_INPUT,
        .pin_bit_mask = (1ULL<<GPIO_START_BUTTON) | (1ULL<<GPIO_STOP_BUTTON) | 
                        (1ULL<<GPIO_INC_BUTTON) | (1ULL<<GPIO_DEC_BUTTON) |
                        (1ULL<<GPIO_SLOPE_INC) | (1ULL<<GPIO_SLOPE_DEC),
        .pull_up_en = 1
    };
    gpio_config(&btn_conf);
    
    
    //Recursos de FreeRTOS
    
    //Semaforos y colas
    gpio_evt_queue = xQueueCreate(10, sizeof(uint32_t));
    oled_ui_queue = xQueueCreate(10, sizeof(OledUiMsg_t)); 
    
    //EventGroups
    s_system_event_group = xEventGroupCreate();

    if (s_system_event_group == NULL) {
        // Error crítico: no hay memoria para el grupo de eventos
        return;
    }    
    
    xEventGroupSetBits(s_system_event_group, EVENT_BIT_STOP); // Por defecto, arrancamos en estado STOP
    
    //Variables argumento de tarea
    //Creamos la instancia de configuración (única "global" técnica, pero encapsulada)
    
	static SlaveStatus_t motor_info;
	//arranco todas las variables en 0 pese a su sobreescritura por esclavo como buena práctica
	motor_info.xSlaveParamsMutex= xSemaphoreCreateMutex();
	motor_info.system_state = 0;
	motor_info.setpoint_rpm = 0U;
	motor_info.current_real_rpm = 0.0f;
	motor_info.current_ramp_rpm = 0.0f;
	motor_info.current_ma = 0.0f;
    
    //Interrupiones por hardware
    gpio_install_isr_service(0);
    gpio_isr_handler_add(GPIO_START_BUTTON, gpio_isr_handler, (void*)GPIO_START_BUTTON);
    gpio_isr_handler_add(GPIO_STOP_BUTTON, gpio_isr_handler, (void*)GPIO_STOP_BUTTON);
    gpio_isr_handler_add(GPIO_INC_BUTTON, gpio_isr_handler, (void*)GPIO_INC_BUTTON);
    gpio_isr_handler_add(GPIO_DEC_BUTTON, gpio_isr_handler, (void*)GPIO_DEC_BUTTON);
    gpio_isr_handler_add(GPIO_SLOPE_INC, gpio_isr_handler, (void*)GPIO_SLOPE_INC);
    gpio_isr_handler_add(GPIO_SLOPE_DEC, gpio_isr_handler, (void*)GPIO_SLOPE_DEC);

    // Crear Tareas
    xTaskCreate(uart_rx_task, "uart", 4096, (void *)&motor_info, 10, NULL);
    xTaskCreate(input_task, "input", 4096, NULL, 9, NULL);
    xTaskCreate(oled_task, "oled", 4096, (void *)&motor_info, 1, NULL);
}