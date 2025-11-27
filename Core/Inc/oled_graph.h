#ifndef __OLED_GRAPH_H
#define __OLED_GRAPH_H

#ifdef __cplusplus
extern "C" {
#endif

#include "ssd1306.h"
#include "ssd1306_conf.h"
#include "ssd1306_fonts.h"
#include "ssd1306_tests.h"

// Ring buffer structure for sensor data
typedef struct {
    float* data;        // Array of sensor values
    uint16_t size;      // Total capacity of the buffer
    uint16_t head;      // Index of newest element
    uint16_t count;     // Number of elements currently in buffer
} ring_buffer_t;

// Extern declarations for ring buffers
extern ring_buffer_t sensor_ring_buffer_1;
extern ring_buffer_t sensor_ring_buffer_2;
extern ring_buffer_t sensor_ring_buffer_3;
extern ring_buffer_t sensor_ring_buffer_4;

// Function to add a value to the ring buffer
void ring_buffer_push(ring_buffer_t* buffer, float value);

// Function to get value at a specific index (oldest to newest)
float ring_buffer_get(ring_buffer_t* buffer, uint16_t index);

// Function to plot a graph on the OLED display
void graph_plot(ring_buffer_t* buffer, uint8_t graph_x, uint8_t graph_y,
                uint8_t graph_width, uint8_t graph_height,
                uint8_t dot_size, float min_y, float max_y);

// New graph plotting functions for different arrangements
void graph_plots_1t1b(ring_buffer_t* buffer1, ring_buffer_t* buffer2,
                      uint8_t graph_x, uint8_t graph_y, uint8_t graph_width, uint8_t graph_height,
                      uint8_t margin);

void graph_plots_1l1r(ring_buffer_t* buffer1, ring_buffer_t* buffer2,
                      uint8_t graph_x, uint8_t graph_y, uint8_t graph_width, uint8_t graph_height,
                      uint8_t margin);

void graph_plots_1t2b(ring_buffer_t* buffer1, ring_buffer_t* buffer2, ring_buffer_t* buffer3,
                     uint8_t graph_x, uint8_t graph_y, uint8_t graph_width, uint8_t graph_height,
                     uint8_t margin);

void graph_plots_2t1b(ring_buffer_t* buffer1, ring_buffer_t* buffer2, ring_buffer_t* buffer3,
                     uint8_t graph_x, uint8_t graph_y, uint8_t graph_width, uint8_t graph_height,
                     uint8_t margin);

void graph_plots_1l2r(ring_buffer_t* buffer1, ring_buffer_t* buffer2, ring_buffer_t* buffer3,
                     uint8_t graph_x, uint8_t graph_y, uint8_t graph_width, uint8_t graph_height,
                     uint8_t margin);

void graph_plots_2l1r(ring_buffer_t* buffer1, ring_buffer_t* buffer2, ring_buffer_t* buffer3,
                     uint8_t graph_x, uint8_t graph_y, uint8_t graph_width, uint8_t graph_height,
                     uint8_t margin);

void graph_plots_2t2b(ring_buffer_t* buffer1, ring_buffer_t* buffer2,
                     ring_buffer_t* buffer3, ring_buffer_t* buffer4,
                     uint8_t graph_x, uint8_t graph_y, uint8_t graph_width, uint8_t graph_height,
                     uint8_t margin);


// Function to arrange 4 graphs (2x2 grid)
void arrange_four_grid(uint8_t graph_x, uint8_t graph_y, uint8_t graph_width, uint8_t graph_height);


#ifdef __cplusplus
}
#endif

#endif /* __OLED_GRAPH_H */