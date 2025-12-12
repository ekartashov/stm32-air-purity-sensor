/**
 * @file oled_graph.h
 * @brief OLED graph plotting functions for sensor data visualization
 *
 * This header file provides functions for plotting sensor data on an OLED display
 * using ring buffers to store historical data. It supports various graph arrangements
 * and provides utilities for displaying sensor trends.
 */

#ifndef __OLED_GRAPH_H
#define __OLED_GRAPH_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include "ssd1306.h"
#include "ssd1306_conf.h"
#include "ssd1306_fonts.h"
#include "ssd1306_tests.h"

/**
 * @brief Graph buffer size
 *
 * Defines the size of the ring buffers used for storing sensor data.
 * This value determines how many historical data points are stored for each graph.
 */
#define GRAPH_BUFFER_SIZE SSD1306_WIDTH

/* Special value: no blinking label */
#define GRAPH_BLINK_NONE   0xFFu

/* Hidden-graph bitmask helpers (index → bit) */
#define GRAPH_HIDE_0       (1u << 0)
#define GRAPH_HIDE_1       (1u << 1)
#define GRAPH_HIDE_2       (1u << 2)
#define GRAPH_HIDE_3       (1u << 3)

/**
 * @brief Ring buffer structure for sensor data storage
 */
typedef struct {
    float*    data;
    uint16_t  size;
    uint16_t  head;
    uint16_t  count;
} ring_buffer_t;

/* External ring buffers */
extern ring_buffer_t sensor_ring_buffer_1;
extern ring_buffer_t sensor_ring_buffer_2;
extern ring_buffer_t sensor_ring_buffer_3;
extern ring_buffer_t sensor_ring_buffer_4;

/* Ring buffer helpers */
void  ring_buffer_push(ring_buffer_t* buffer, float value);
float ring_buffer_get(ring_buffer_t* buffer, uint16_t index);

/* Basic single-graph plot */
void graph_plot(ring_buffer_t* buffer, uint8_t graph_x, uint8_t graph_y,
                uint8_t graph_width, uint8_t graph_height,
                uint8_t dot_size, float min_y, float max_y, const char* legend_text);

/* Two-buffer overlay helper */
void graph_plot_two_buffers(ring_buffer_t* buffer1, ring_buffer_t* buffer2,
                            uint8_t graph_x, uint8_t graph_y,
                            uint8_t graph_width, uint8_t graph_height,
                            uint8_t dot_size, float min_y1, float max_y1,
                            float min_y2, float max_y2);

/* Layout helpers (non-extended, simple API) */
void graph_plots_1t1b(ring_buffer_t* buffer1, ring_buffer_t* buffer2,
                      uint8_t graph_x, uint8_t graph_y, uint8_t graph_width, uint8_t graph_height,
                      uint8_t margin, const char* legend_text1, const char* legend_text2);

void graph_plots_1l1r(ring_buffer_t* buffer1, ring_buffer_t* buffer2,
                      uint8_t graph_x, uint8_t graph_y, uint8_t graph_width, uint8_t graph_height,
                      uint8_t margin, const char* legend_text1, const char* legend_text2);

void graph_plots_1t2b(ring_buffer_t* buffer1, ring_buffer_t* buffer2, ring_buffer_t* buffer3,
                      uint8_t graph_x, uint8_t graph_y, uint8_t graph_width, uint8_t graph_height,
                      uint8_t margin, const char* legend_text1, const char* legend_text2, const char* legend_text3);

void graph_plots_2t1b(ring_buffer_t* buffer1, ring_buffer_t* buffer2, ring_buffer_t* buffer3,
                      uint8_t graph_x, uint8_t graph_y, uint8_t graph_width, uint8_t graph_height,
                      uint8_t margin, const char* legend_text1, const char* legend_text2, const char* legend_text3);

void graph_plots_1l2r(ring_buffer_t* buffer1, ring_buffer_t* buffer2, ring_buffer_t* buffer3,
                      uint8_t graph_x, uint8_t graph_y, uint8_t graph_width, uint8_t graph_height,
                      uint8_t margin, const char* legend_text1, const char* legend_text2, const char* legend_text3);

void graph_plots_2l1r(ring_buffer_t* buffer1, ring_buffer_t* buffer2, ring_buffer_t* buffer3,
                      uint8_t graph_x, uint8_t graph_y, uint8_t graph_width, uint8_t graph_height,
                      uint8_t margin, const char* legend_text1, const char* legend_text2, const char* legend_text3);

void graph_plots_2t2b(ring_buffer_t* buffer1, ring_buffer_t* buffer2,
                      ring_buffer_t* buffer3, ring_buffer_t* buffer4,
                      uint8_t graph_x, uint8_t graph_y, uint8_t graph_width, uint8_t graph_height,
                      uint8_t margin, const char* legend_text1, const char* legend_text2,
                      const char* legend_text3, const char* legend_text4);

/* Debug helpers */
void debug_screen_edges(void);
void debug_fullscreen_graph(void);

/* --- Extended / blinking / hide-aware helpers --- */

/**
 * Extended single-graph plot with optional blinking label and hide flag.
 *
 * NOTE: If @p buffer is NULL, this is treated exactly like @p hide_graph != 0
 *       and nothing is drawn for this slot.
 */
void graph_plot_ex(ring_buffer_t* buffer,
                   uint8_t graph_x, uint8_t graph_y,
                   uint8_t graph_width, uint8_t graph_height,
                   uint8_t dot_size, float min_y, float max_y,
                   const char* legend_text,
                   uint8_t blink_label,
                   uint8_t hide_graph);

/**
 * Extended multi-graph helpers:
 *
 *  - blinking_index: 0-based index of the graph whose label should blink;
 *                    use GRAPH_BLINK_NONE for no blinking.
 *  - hidden_mask   : OR of GRAPH_HIDE_n bits to completely suppress a graph.
 *
 * Additionally, if a buffer pointer for a given slot is NULL, that slot is
 * also treated as hidden and nothing is drawn for it (Option 1 behaviour).
 */

/* 2 graphs: top (index 0), bottom (index 1) */
void graph_plots_1t1b_ex(ring_buffer_t* buffer1, ring_buffer_t* buffer2,
                         uint8_t graph_x, uint8_t graph_y,
                         uint8_t graph_width, uint8_t graph_height,
                         uint8_t margin,
                         const char* legend_text1, const char* legend_text2,
                         uint8_t blinking_index,
                         uint8_t hidden_mask);

/* 2 graphs: left (index 0), right (index 1) */
void graph_plots_1l1r_ex(ring_buffer_t* buffer1, ring_buffer_t* buffer2,
                         uint8_t graph_x, uint8_t graph_y,
                         uint8_t graph_width, uint8_t graph_height,
                         uint8_t margin,
                         const char* legend_text1, const char* legend_text2,
                         uint8_t blinking_index,
                         uint8_t hidden_mask);

/* 3 graphs: top (0), bottom-left (1), bottom-right (2) */
void graph_plots_1t2b_ex(ring_buffer_t* buffer1, ring_buffer_t* buffer2, ring_buffer_t* buffer3,
                         uint8_t graph_x, uint8_t graph_y,
                         uint8_t graph_width, uint8_t graph_height,
                         uint8_t margin,
                         const char* legend_text1, const char* legend_text2, const char* legend_text3,
                         uint8_t blinking_index,
                         uint8_t hidden_mask);

/* 3 graphs: top-left (0), top-right (1), bottom (2) */
void graph_plots_2t1b_ex(ring_buffer_t* buffer1, ring_buffer_t* buffer2, ring_buffer_t* buffer3,
                         uint8_t graph_x, uint8_t graph_y,
                         uint8_t graph_width, uint8_t graph_height,
                         uint8_t margin,
                         const char* legend_text1, const char* legend_text2, const char* legend_text3,
                         uint8_t blinking_index,
                         uint8_t hidden_mask);

/* 3 graphs: left (0), right-top (1), right-bottom (2) */
void graph_plots_1l2r_ex(ring_buffer_t* buffer1, ring_buffer_t* buffer2, ring_buffer_t* buffer3,
                         uint8_t graph_x, uint8_t graph_y,
                         uint8_t graph_width, uint8_t graph_height,
                         uint8_t margin,
                         const char* legend_text1, const char* legend_text2, const char* legend_text3,
                         uint8_t blinking_index,
                         uint8_t hidden_mask);

/* 3 graphs: left-top (0), left-bottom (1), right (2) */
void graph_plots_2l1r_ex(ring_buffer_t* buffer1, ring_buffer_t* buffer2, ring_buffer_t* buffer3,
                         uint8_t graph_x, uint8_t graph_y,
                         uint8_t graph_width, uint8_t graph_height,
                         uint8_t margin,
                         const char* legend_text1, const char* legend_text2, const char* legend_text3,
                         uint8_t blinking_index,
                         uint8_t hidden_mask);

/* 4 graphs: indices: top-left (0), top-right (1), bottom-left (2), bottom-right (3) */
void graph_plots_2t2b_ex(ring_buffer_t* buffer1, ring_buffer_t* buffer2,
                         ring_buffer_t* buffer3, ring_buffer_t* buffer4,
                         uint8_t graph_x, uint8_t graph_y,
                         uint8_t graph_width, uint8_t graph_height,
                         uint8_t margin,
                         const char* legend_text1, const char* legend_text2,
                         const char* legend_text3, const char* legend_text4,
                         uint8_t blinking_index,
                         uint8_t hidden_mask);

#ifdef __cplusplus
}
#endif

#endif /* __OLED_GRAPH_H */
