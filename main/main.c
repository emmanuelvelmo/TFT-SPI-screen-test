#include <stdio.h> // Funciones de entrada/salida estándar
#include "freertos/FreeRTOS.h" // Núcleo del sistema operativo FreeRTOS
#include "freertos/task.h" // Manejo de tareas y delays
#include "driver/gpio.h" // Control de pines GPIO
#include "driver/spi_master.h" // Control del bus SPI

// PLACA Y MÓDULOS
// ESPWROOM32 XX5R69 ← ILI9341: 5V, GND, G14 (SCLK), G13 (MOSI), G2 (DC), G15 (CS), G26 (RST), G21 (BLK)

// PINES GPIO (CONFIGURACIÓN SPI PARA ILI9341)
#define pin_sclk GPIO_NUM_14 // Pin de reloj SPI (Serial Clock)
#define pin_mosi GPIO_NUM_13 // Pin de datos MOSI (Master Out Slave In)
#define pin_cs GPIO_NUM_15 // Pin de selección de chip (Chip Select)
#define pin_dc GPIO_NUM_2 // Pin de control dato/comando (Data/Command)
#define pin_rst GPIO_NUM_26 // Pin de reinicio (Reset)
#define pin_blk GPIO_NUM_21 // Pin de retroiluminación (Backlight)

// CONFIGURACIÓN SPI
#define spi_host_usado SPI2_HOST // Controlador SPI a utilizar (SPI2)
#define velocidad_spi 1000000 // 1 MHz (velocidad segura para pruebas)
#define cola_spi 7 // Tamaño de cola de transacciones SPI
#define max_transfer 4096 // Tamaño máximo válido con DMA

// PARÁMETROS DE PANTALLA ILI9341 (ORIENTACIÓN VERTICAL)
#define ancho_pantalla 320 // Ancho de pantalla en píxeles (320)
#define alto_pantalla 240 // Alto de pantalla en píxeles (240)
#define bytes_pixel 2 // Bytes por píxel (formato RGB565)
#define pixeles_totales (ancho_pantalla * alto_pantalla) // Total de píxeles en pantalla

// COMANDOS ILI9341
#define cmd_swreset 0x01 // Comando: software reset (reinicio por software)
#define cmd_slpout 0x11 // Comando: sleep out (salir de modo reposo)
#define cmd_colmod 0x3A // Comando: color mode (configurar modo de color)
#define cmd_madctl 0x36 // Comando: memory access control (control de acceso a memoria)
#define cmd_dispon 0x29 // Comando: display on (encender pantalla)
#define cmd_caset 0x2A // Comando: column address set (definir dirección de columna)
#define cmd_paset 0x2B // Comando: page address set (definir dirección de página)
#define cmd_ramwr 0x2C // Comando: memory write (escritura en memoria)

// VARIABLES GLOBALES
static spi_device_handle_t dispositivo_spi; // Manejador del dispositivo SPI

// FUNCIONES BÁSICAS (ÚLTIMAS EN USARSE)

// Enviar comando a la pantalla (modo comando)
static void enviar_comando(uint8_t comando)
{
    gpio_set_level(pin_dc, 0); // Establece pin DC en bajo (modo comando)

    spi_transaction_t transaccion = { // Estructura para transacción SPI
        .length = 8, // 8 bits de longitud
        .tx_buffer = &comando // Buffer de transmisión con el comando
    };

    spi_device_transmit(dispositivo_spi, &transaccion); // Transmite comando por SPI
}

// Enviar datos a la pantalla (modo datos, con soporte DMA)
static void enviar_datos(const uint8_t *datos, uint32_t longitud)
{
    gpio_set_level(pin_dc, 1); // Establece pin DC en alto (modo datos)

    if (longitud == 0) // Si no hay datos que enviar
    {
        return; // Termina función
    }

    uint32_t offset = 0; // Desplazamiento actual en el buffer

    while (offset < longitud) // Procesa datos en bloques
    {
        uint32_t bytes_bloque = longitud - offset; // Calcula bytes restantes

        if (bytes_bloque > max_transfer) // Si excede tamaño máximo de transferencia
        {
            bytes_bloque = max_transfer; // Limita al tamaño máximo
        }

        spi_transaction_t transaccion = { // Estructura para transacción SPI
            .length = bytes_bloque * 8, // Convierte bytes a bits (cada byte = 8 bits)
            .tx_buffer = datos + offset // Puntero al inicio del bloque actual
        };

        spi_device_transmit(dispositivo_spi, &transaccion); // Transmite bloque por SPI

        offset += bytes_bloque; // Avanza al siguiente bloque
    }
}

// Enviar un solo byte como dato
static void enviar_dato(uint8_t dato)
{
    enviar_datos(&dato, 1); // Llama a función general con longitud 1
}

// FUNCIONES INTERMEDIAS

// Define la ventana activa de escritura en la pantalla
static void establecer_ventana(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1)
{
    uint8_t datos_coord[4]; // Array para almacenar coordenadas como bytes

    enviar_comando(cmd_caset); // Envía comando para definir dirección de columna

    datos_coord[0] = x0 >> 8; // Byte alto de coordenada X inicial
    datos_coord[1] = x0 & 0xFF; // Byte bajo de coordenada X inicial
    datos_coord[2] = x1 >> 8; // Byte alto de coordenada X final
    datos_coord[3] = x1 & 0xFF; // Byte bajo de coordenada X final

    enviar_datos(datos_coord, 4); // Envía los 4 bytes de coordenadas X

    enviar_comando(cmd_paset); // Envía comando para definir dirección de página

    datos_coord[0] = y0 >> 8; // Byte alto de coordenada Y inicial
    datos_coord[1] = y0 & 0xFF; // Byte bajo de coordenada Y inicial
    datos_coord[2] = y1 >> 8; // Byte alto de coordenada Y final
    datos_coord[3] = y1 & 0xFF; // Byte bajo de coordenada Y final

    enviar_datos(datos_coord, 4); // Envía los 4 bytes de coordenadas Y

    enviar_comando(cmd_ramwr); // Establece modo de escritura en memoria RAM
}

// Llena completamente la pantalla con un color RGB565
static void llenar_pantalla_color(uint16_t color)
{
    establecer_ventana(0, 0, ancho_pantalla - 1, alto_pantalla - 1); // Define ventana completa

    uint8_t color_bytes[2] = { color >> 8, color & 0xFF }; // Separa color en bytes alto y bajo

    const uint32_t pixeles_bloque = 1024; // Número de píxeles por bloque de envío
    uint8_t buffer[pixeles_bloque * bytes_pixel]; // Buffer para almacenar datos de píxeles

    for (uint32_t i = 0; i < sizeof(buffer); i += 2) // Prellena buffer con el color
    {
        buffer[i]     = color_bytes[0]; // Byte alto del color
        buffer[i + 1] = color_bytes[1]; // Byte bajo del color
    }

    uint32_t restantes = pixeles_totales; // Píxeles totales que faltan por enviar

    while (restantes > 0) // Envía datos en bloques hasta completar pantalla
    {
        uint32_t enviar = restantes > pixeles_bloque ? pixeles_bloque : restantes; // Calcula tamaño del bloque actual

        enviar_datos(buffer, enviar * bytes_pixel); // Envía bloque de píxeles

        restantes -= enviar; // Reduce contador de píxeles restantes
    }
}

// FUNCIONES DE INICIALIZACIÓN (MÁS COMPLEJAS)

// Inicializa pines GPIO utilizados por la pantalla
static void inicializar_gpio(void)
{
    gpio_config_t config = { // Estructura de configuración GPIO
        .mode = GPIO_MODE_OUTPUT, // Configuración como salida digital
        .pull_up_en = GPIO_PULLUP_DISABLE, // Deshabilita resistencia pull-up interna
        .pull_down_en = GPIO_PULLDOWN_DISABLE, // Deshabilita resistencia pull-down interna
        .intr_type = GPIO_INTR_DISABLE // Deshabilita interrupciones en el pin
    };

    config.pin_bit_mask = (1ULL << pin_rst); // Configura máscara para pin de reset
    gpio_config(&config); // Aplica configuración al pin de reset

    config.pin_bit_mask = (1ULL << pin_blk); // Configura máscara para pin de retroiluminación
    gpio_config(&config); // Aplica configuración al pin de retroiluminación

    config.pin_bit_mask = (1ULL << pin_dc); // Configura máscara para pin de data/command
    gpio_config(&config); // Aplica configuración al pin DC

    gpio_set_level(pin_blk, 0); // Apaga retroiluminación inicialmente
    gpio_set_level(pin_dc, 0); // Establece modo comando inicialmente
}

// Inicializa bus SPI CON DMA (necesario para transferencias grandes)
static void inicializar_spi(void)
{
    spi_bus_config_t bus_config = { // Configuración del bus SPI
        .mosi_io_num = pin_mosi, // Pin de datos MOSI (Master Out Slave In)
        .miso_io_num = -1, // No se usa pin MISO (sólo escritura a pantalla)
        .sclk_io_num = pin_sclk, // Pin de reloj SPI (Serial Clock)
        .quadwp_io_num = -1, // No se usa modo Quad SPI (pin para escritura)
        .quadhd_io_num = -1, // No se usa modo Quad SPI (pin para retención)
        .max_transfer_sz = max_transfer // Tamaño máximo de transferencia con DMA
    };

    spi_bus_initialize( // Inicializa el bus SPI
        spi_host_usado, // Controlador SPI a utilizar
        &bus_config, // Configuración del bus
        SPI_DMA_CH_AUTO // Canal DMA automático
    );

    spi_device_interface_config_t dev_config = { // Configuración del dispositivo SPI
        .clock_speed_hz = velocidad_spi, // Velocidad de reloj en Hz (1 MHz)
        .mode = 0, // Modo SPI 0 (CPOL=0, CPHA=0)
        .spics_io_num = pin_cs, // Pin de selección de chip
        .queue_size = cola_spi // Tamaño de cola de transacciones SPI
    };

    spi_bus_add_device(spi_host_usado, &dev_config, &dispositivo_spi); // Añade dispositivo al bus SPI
}

// Secuencia mínima de inicialización del ILI9341
static void inicializar_pantalla(void)
{
    gpio_set_level(pin_rst, 0); // Activa reset (nivel bajo)
    vTaskDelay(100 / portTICK_PERIOD_MS); // Espera 100ms para reinicio efectivo

    gpio_set_level(pin_rst, 1); // Libera reset (nivel alto)
    vTaskDelay(150 / portTICK_PERIOD_MS); // Espera 150ms para estabilización

    enviar_comando(cmd_swreset); // Envía comando de reset por software
    vTaskDelay(150 / portTICK_PERIOD_MS); // Espera 150ms después del reset

    enviar_comando(cmd_slpout); // Envía comando para salir de modo sleep
    vTaskDelay(150 / portTICK_PERIOD_MS); // Espera 150ms después de despertar

    enviar_comando(cmd_colmod); // Envía comando para configurar modo de color
    enviar_dato(0x55); // Configura formato RGB565 (16 bits por píxel)

    enviar_comando(cmd_madctl); // Envía comando de control de acceso a memoria
    enviar_dato(0x28); // Configura orientación vertical (320x240)

    enviar_comando(cmd_dispon); // Envía comando para encender pantalla
    vTaskDelay(100 / portTICK_PERIOD_MS); // Espera 100ms después de encender

    gpio_set_level(pin_blk, 1); // Enciende retroiluminación
}

// TAREA PRINCIPAL: CICLO DE CAMBIO DE COLORES
static void ciclo_colores(void *param)
{
    uint16_t colores[] = { 0xF800, 0x07E0, 0x001F }; // Rojo, verde, azul en formato RGB565
    uint8_t indice = 0; // Índice del color actual en el array

    vTaskDelay(500 / portTICK_PERIOD_MS); // Espera inicial de 500ms

    while (1) // Bucle infinito
    {
        llenar_pantalla_color(colores[indice]); // Llena pantalla con color actual

        indice = (indice + 1) % 3; // Avanza al siguiente color (cíclico)

        vTaskDelay(1000 / portTICK_PERIOD_MS); // Espera 1 segundo entre cambios de color
    }
}

// PUNTO DE PARTIDA
void app_main(void)
{
    inicializar_gpio(); // Inicializa pines GPIO
    inicializar_spi(); // Inicializa bus SPI
    inicializar_pantalla(); // Inicializa controlador de pantalla

    xTaskCreate( // Crea tarea para ciclo de colores
        ciclo_colores, // Función a ejecutar en la tarea
        "ciclo_colores", // Nombre descriptivo de la tarea
        4096, // Tamaño de pila en bytes
        NULL, // Parámetros pasados a la tarea (ninguno)
        1, // Prioridad de tarea (baja)
        NULL // Manejador de tarea (no usado)
    );

    while (1) // Bucle principal del programa
    {
        vTaskDelay(1000 / portTICK_PERIOD_MS); // Espera 1 segundo
    }
}
