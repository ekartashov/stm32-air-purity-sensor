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

/**
 * @brief Ring buffer structure for sensor data storage
 *
 * This structure implements a circular buffer for storing sensor readings.
 * It allows efficient storage and retrieval of historical data for graphing.
 */
typedef struct {
    float* data;        /**< Array of sensor values */
    uint16_t size;      /**< Total capacity of the buffer */
    uint16_t head;      /**< Index of newest element */
    uint16_t count;     /**< Number of elements currently in buffer */
} ring_buffer_t;

/**
 * @brief External ring buffers for sensor data
 *
 * Four external ring buffers are declared for storing sensor data from different sources.
 * Each buffer can hold up to GRAPH_BUFFER_SIZE sensor readings.
 */
extern ring_buffer_t sensor_ring_buffer_1;
extern ring_buffer_t sensor_ring_buffer_2;
extern ring_buffer_t sensor_ring_buffer_3;
extern ring_buffer_t sensor_ring_buffer_4;

/**
 * @brief Add a value to the ring buffer
 * @param buffer Pointer to the ring buffer structure
 * @param value Value to be added to the buffer
 */
void ring_buffer_push(ring_buffer_t* buffer, float value);

/**
 * @brief Get value at a specific index in the ring buffer
 * @param buffer Pointer to the ring buffer structure
 * @param index Index of the value to retrieve (0 = oldest, count-1 = newest)
 * @return Value at the specified index, or 0.0f if index is invalid
 */
float ring_buffer_get(ring_buffer_t* buffer, uint16_t index);

/**
 * @brief Plot a graph on the OLED display
 *
 * Plots sensor data from a ring buffer onto the OLED display within the specified
 * rectangular area. The graph scales the data to fit within the provided dimensions.
 *
 * @param buffer Pointer to the ring buffer containing sensor data
 * @param graph_x X coordinate of the top-left corner of the graph area
 * @param graph_y Y coordinate of the top-left corner of the graph area
 * @param graph_width Width of the graph area in pixels
 * @param graph_height Height of the graph area in pixels
 * @param dot_size Size of data point markers in pixels
 * @param min_y Minimum Y value for scaling (0.0f to use automatic scaling)
 * @param max_y Maximum Y value for scaling (0.0f to use automatic scaling)
 * @param legend_text Text to display as legend (NULL for no legend)
 */
void graph_plot(ring_buffer_t* buffer, uint8_t graph_x, uint8_t graph_y,
                uint8_t graph_width, uint8_t graph_height,
                uint8_t dot_size, float min_y, float max_y, const char* legend_text);

/**
 * @brief Plot two graphs arranged vertically (1 top, 1 bottom)
 *
 * Plots two graphs with one on top and one on bottom, sharing the same horizontal space.
 *
 * @param buffer1 Pointer to the ring buffer for the top graph
 * @param buffer2 Pointer to the ring buffer for the bottom graph
 * @param graph_x X coordinate of the top-left corner of the graph area
 * @param graph_y Y coordinate of the top-left corner of the graph area
 * @param graph_width Width of the graph area in pixels
 * @param graph_height Height of the graph area in pixels
 * @param margin Space between the two graphs in pixels
 * @param legend_text1 Text to display as legend for the top graph (NULL for no legend)
 * @param legend_text2 Text to display as legend for the bottom graph (NULL for no legend)
 */
void graph_plots_1t1b(ring_buffer_t* buffer1, ring_buffer_t* buffer2,
                      uint8_t graph_x, uint8_t graph_y, uint8_t graph_width, uint8_t graph_height,
                      uint8_t margin, const char* legend_text1, const char* legend_text2);

/**
 * @brief Plot two graphs arranged horizontally (1 left, 1 right)
 *
 * Plots two graphs with one on the left and one on the right, sharing the same vertical space.
 *
 * @param buffer1 Pointer to the ring buffer for the left graph
 * @param buffer2 Pointer to the ring buffer for the right graph
 * @param graph_x X coordinate of the top-left corner of the graph area
 * @param graph_y Y coordinate of the top-left corner of the graph area
 * @param graph_width Width of the graph area in pixels
 * @param graph_height Height of the graph area in pixels
 * @param margin Space between the two graphs in pixels
 * @param legend_text1 Text to display as legend for the left graph (NULL for no legend)
 * @param legend_text2 Text to display as legend for the right graph (NULL for no legend)
 */
void graph_plots_1l1r(ring_buffer_t* buffer1, ring_buffer_t* buffer2,
                      uint8_t graph_x, uint8_t graph_y, uint8_t graph_width, uint8_t graph_height,
                      uint8_t margin, const char* legend_text1, const char* legend_text2);

/**
 * @brief Plot three graphs arranged with one wide on top and two shallow on bottom
 *
 * Plots three graphs with one wide graph on top and two smaller graphs on the bottom.
 *
 * @param buffer1 Pointer to the ring buffer for the top graph
 * @param buffer2 Pointer to the ring buffer for the first bottom graph
 * @param buffer3 Pointer to the ring buffer for the second bottom graph
 * @param graph_x X coordinate of the top-left corner of the graph area
 * @param graph_y Y coordinate of the top-left corner of the graph area
 * @param graph_width Width of the graph area in pixels
 * @param graph_height Height of the graph area in pixels
 * @param margin Space between the graphs in pixels
 * @param legend_text1 Text to display as legend for the top graph (NULL for no legend)
 * @param legend_text2 Text to display as legend for the first bottom graph (NULL for no legend)
 * @param legend_text3 Text to display as legend for the second bottom graph (NULL for no legend)
 */
void graph_plots_1t2b(ring_buffer_t* buffer1, ring_buffer_t* buffer2, ring_buffer_t* buffer3,
                     uint8_t graph_x, uint8_t graph_y, uint8_t graph_width, uint8_t graph_height,
                     uint8_t margin, const char* legend_text1, const char* legend_text2, const char* legend_text3);

/**
 * @brief Plot three graphs arranged with two shallow on top and one wide on bottom
 *
 * Plots three graphs with two smaller graphs on top and one wide graph on the bottom.
 *
 * @param buffer1 Pointer to the ring buffer for the first top graph
 * @param buffer2 Pointer to the ring buffer for the second top graph
 * @param buffer3 Pointer to the ring buffer for the bottom graph
 * @param graph_x X coordinate of the top-left corner of the graph area
 * @param graph_y Y coordinate of the top-left corner of the graph area
 * @param graph_width Width of the graph area in pixels
 * @param graph_height Height of the graph area in pixels
 * @param margin Space between the graphs in pixels
 * @param legend_text1 Text to display as legend for the first top graph (NULL for no legend)
 * @param legend_text2 Text to display as legend for the second top graph (NULL for no legend)
 * @param legend_text3 Text to display as legend for the bottom graph (NULL for no legend)
 */
void graph_plots_2t1b(ring_buffer_t* buffer1, ring_buffer_t* buffer2, ring_buffer_t* buffer3,
                     uint8_t graph_x, uint8_t graph_y, uint8_t graph_width, uint8_t graph_height,
                     uint8_t margin, const char* legend_text1, const char* legend_text2, const char* legend_text3);

/**
 * @brief Plot three graphs arranged with one tall on left and two short on right
 *
 * Plots three graphs with one tall graph on the left and two shorter graphs on the right.
 *
 * @param buffer1 Pointer to the ring buffer for the left graph
 * @param buffer2 Pointer to the ring buffer for the first right graph
 * @param buffer3 Pointer to the ring buffer for the second right graph
 * @param graph_x X coordinate of the top-left corner of the graph area
 * @param graph_y Y coordinate of the top-left corner of the graph area
 * @param graph_width Width of the graph area in pixels
 * @param graph_height Height of the graph area in pixels
 * @param margin Space between the graphs in pixels
 * @param legend_text1 Text to display as legend for the left graph (NULL for no legend)
 * @param legend_text2 Text to display as legend for the first right graph (NULL for no legend)
 * @param legend_text3 Text to display as legend for the second right graph (NULL for no legend)
 */
void graph_plots_1l2r(ring_buffer_t* buffer1, ring_buffer_t* buffer2, ring_buffer_t* buffer3,
                     uint8_t graph_x, uint8_t graph_y, uint8_t graph_width, uint8_t graph_height,
                     uint8_t margin, const char* legend_text1, const char* legend_text2, const char* legend_text3);

/**
 * @brief Plot three graphs arranged with two short on left and one tall on right
 *
 * Plots three graphs with two shorter graphs on the left and one tall graph on the right.
 *
 * @param buffer1 Pointer to the ring buffer for the first left graph
 * @param buffer2 Pointer to the ring buffer for the second left graph
 * @param buffer3 Pointer to the ring buffer for the right graph
 * @param graph_x X coordinate of the top-left corner of the graph area
 * @param graph_y Y coordinate of the top-left corner of the graph area
 * @param graph_width Width of the graph area in pixels
 * @param graph_height Height of the graph area in pixels
 * @param margin Space between the graphs in pixels
 * @param legend_text1 Text to display as legend for the first left graph (NULL for no legend)
 * @param legend_text2 Text to display as legend for the second left graph (NULL for no legend)
 * @param legend_text3 Text to display as legend for the right graph (NULL for no legend)
 */
void graph_plots_2l1r(ring_buffer_t* buffer1, ring_buffer_t* buffer2, ring_buffer_t* buffer3,
                     uint8_t graph_x, uint8_t graph_y, uint8_t graph_width, uint8_t graph_height,
                     uint8_t margin, const char* legend_text1, const char* legend_text2, const char* legend_text3);

/**
 * @brief Plot four graphs arranged in a 2x2 grid
 *
 * Plots four graphs arranged in a 2x2 grid layout.
 *
 * @param buffer1 Pointer to the ring buffer for the top-left graph
 * @param buffer2 Pointer to the ring buffer for the top-right graph
 * @param buffer3 Pointer to the ring buffer for the bottom-left graph
 * @param buffer4 Pointer to the ring buffer for the bottom-right graph
 * @param graph_x X coordinate of the top-left corner of the graph area
 * @param graph_y Y coordinate of the top-left corner of the graph area
 * @param graph_width Width of the graph area in pixels
 * @param graph_height Height of the graph area in pixels
 * @param margin Space between the graphs in pixels
 * @param legend_text1 Text to display as legend for the top-left graph (NULL for no legend)
 * @param legend_text2 Text to display as legend for the top-right graph (NULL for no legend)
 * @param legend_text3 Text to display as legend for the bottom-left graph (NULL for no legend)
 * @param legend_text4 Text to display as legend for the bottom-right graph (NULL for no legend)
 */
void graph_plots_2t2b(ring_buffer_t* buffer1, ring_buffer_t* buffer2,
                     ring_buffer_t* buffer3, ring_buffer_t* buffer4,
                     uint8_t graph_x, uint8_t graph_y, uint8_t graph_width, uint8_t graph_height,
                     uint8_t margin, const char* legend_text1, const char* legend_text2, const char* legend_text3, const char* legend_text4);

/**
 * @brief Visualize screen edges and vertices for debugging
 *
 * This function provides a visual debugging aid that cycles through different
 * screen visualization modes:
 * 1. Black screen (no drawing)
 * 2. Four corner dots only
 * 3. Rectangle outline around screen
 * 4. Black screen (everything cleared)
 *
 * The visualization cycles through these modes with a blinking effect.
 */
void debug_screen_edges(void);


#ifdef __cplusplus
}
#endif

#endif /* __OLED_GRAPH_H */