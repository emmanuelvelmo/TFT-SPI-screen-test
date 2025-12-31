#include <stdio.h> // Funciones de entrada/salida estándar
#include "freertos/FreeRTOS.h" // Núcleo de sistema operativo FreeRTOS
#include "freertos/task.h" // Tareas y delays
#include "driver/gpio.h" // Control de pines GPIO
#include "TFT_eSPI.h" // Librería para pantalla ILI9341

// PLACA Y MÓDULOS
// ESPWROOM32 XX5R69 ← ILI9341: 5V, GND, G14 (SCLK), G13 (MOSI), G2 (DC), G15 (CS), G26 (RST), G21 (BLK)

// PINES DE LA PLACA
#define PIN_NUM_BLK_TFT GPIO_NUM_21 // Pin para backlight de pantalla

// VARIABLES DE CONFIGURACIÓN
#define TIEMPO_CAMBIO_COLOR 1000 // Tiempo en milisegundos para cada cambio de color (1 segundo)

// LISTA DE COLORES PARA EL CICLO
static const uint16_t lista_colores[] = {
    TFT_RED,    // Color rojo (primer color)
    TFT_GREEN,  // Color verde (segundo color)
    TFT_BLUE    // Color azul (tercer color)
};

#define NUMERO_COLORES (sizeof(lista_colores) / sizeof(lista_colores[0])) // Número total de colores en la lista

// FUNCIONES
// Inicializa pantalla ILI9341 con TFT_eSPI y configura pines
static TFT_eSPI* inicializar_pantalla_tft(void)
{
    // Crea objeto de pantalla en memoria dinámica
    TFT_eSPI *pantalla_tft = new TFT_eSPI();

    // Inicializa hardware de pantalla (configura pines SPI, etc.)
    pantalla_tft->init();
    
    // Configura rotación de pantalla (0 = orientación normal)
    pantalla_tft->setRotation(0);
    
    // Establece color de fondo inicial (negro)
    pantalla_tft->fillScreen(TFT_BLACK);
    
    // Configura pin de backlight como salida digital
    gpio_set_direction(PIN_NUM_BLK_TFT, GPIO_MODE_OUTPUT);
    
    // Habilita backlight de pantalla (nivel alto = encendido)
    gpio_set_level(PIN_NUM_BLK_TFT, 1);

    // Retorna puntero al objeto de pantalla inicializado
    return pantalla_tft;
}

// Ciclo principal que cambia colores de pantalla cada segundo
static void ciclo_cambio_colores(void *parametros)
{
    // Inicializa pantalla y obtiene objeto para control
    TFT_eSPI *pantalla_tft = inicializar_pantalla_tft();
    
    // Índice para recorrer lista de colores
    int indice_color_actual = 0;

    // Bucle infinito para cambio continuo de colores
    while (1)
    {
        // Obtiene color actual de la lista según índice
        uint16_t color_actual = lista_colores[indice_color_actual];
        
        // Aplica color a toda la pantalla (fondo completo)
        pantalla_tft->fillScreen(color_actual);
        
        // Incrementa índice para siguiente color
        indice_color_actual++;
        
        // Reinicia índice si supera el número de colores disponibles
        if (indice_color_actual >= NUMERO_COLORES)
        {
            indice_color_actual = 0;
        }
        
        // Espera tiempo definido antes de siguiente cambio de color
        vTaskDelay(TIEMPO_CAMBIO_COLOR / portTICK_PERIOD_MS);
    }
}

// PUNTO DE PARTIDA (FUNCIÓN PRINCIPAL)
void app_main(void)
{
    // Crea tarea para ciclo de cambio de colores en pantalla
    xTaskCreate(
        ciclo_cambio_colores, // Función que se ejecutará en la tarea
        "ciclo_colores", // Nombre descriptivo para la tarea
        4096, // Tamaño de pila en bytes (suficiente para esta tarea simple)
        NULL, // Parámetros pasados a la función (ninguno)
        1, // Prioridad de tarea (baja, ya que no es crítica)
        NULL // Manejador de tarea (no necesario en este caso)
    );

    // Bucle principal mantiene programa en ejecución
    while (1)
    {
        // Espera indefinidamente (la tarea maneja toda la funcionalidad)
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}