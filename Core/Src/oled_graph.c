/**
 * @file oled_graph.c
 * @brief OLED graph plotting implementation for sensor data visualization
 *
 * This file implements functions for plotting sensor data on an OLED display
 * using ring buffers to store historical data. It supports various graph arrangements
 * and provides utilities for displaying sensor trends.
 */

#include "oled_graph.h"
#include "ssd1306.h"

/**
 * @brief Ring buffers for 4 different sensor data streams
 *
 * Four static arrays are defined to store sensor data for each of the four
 * available ring buffers. Each buffer can hold up to GRAPH_BUFFER_SIZE sensor readings.
 */
static float sensor_data_1[GRAPH_BUFFER_SIZE];
static float sensor_data_2[GRAPH_BUFFER_SIZE];
static float sensor_data_3[GRAPH_BUFFER_SIZE];
static float sensor_data_4[GRAPH_BUFFER_SIZE];

/**
 * @brief Ring buffer structures for sensor data
 *
 * Four ring buffer structures initialized with their respective data arrays.
 * These are used to store and manage sensor readings for graphing.
 */
ring_buffer_t sensor_ring_buffer_1 = {
    .data = sensor_data_1,
    .size = GRAPH_BUFFER_SIZE,
    .head = 0,
    .count = 0
};

ring_buffer_t sensor_ring_buffer_2 = {
    .data = sensor_data_2,
    .size = GRAPH_BUFFER_SIZE,
    .head = 0,
    .count = 0
};

ring_buffer_t sensor_ring_buffer_3 = {
    .data = sensor_data_3,
    .size = GRAPH_BUFFER_SIZE,
    .head = 0,
    .count = 0
};

ring_buffer_t sensor_ring_buffer_4 = {
    .data = sensor_data_4,
    .size = GRAPH_BUFFER_SIZE,
    .head = 0,
    .count = 0
};

/**
 * @brief Add a value to the ring buffer
 *
 * Adds a new value to the ring buffer, overwriting the oldest value if the
 * buffer is full. The buffer maintains a circular structure to efficiently
 * store historical data.
 *
 * @param buffer Pointer to the ring buffer structure
 * @param value Value to be added to the buffer
 */
void ring_buffer_push(ring_buffer_t* buffer, float value) {
    if(buffer->count < buffer->size) {
        buffer->count++;
    } else {
        // If buffer is full, overwrite oldest value
        buffer->head = (buffer->head + 1) % buffer->size;
    }

    // Insert new value at head position
    buffer->data[buffer->head] = value;
    buffer->head = (buffer->head + 1) % buffer->size;
}

/**
 * @brief Get a value from the ring buffer at a specific index
 *
 * Retrieves a value from the ring buffer at the specified index. The index
 * represents the position in the buffer history, where 0 is the oldest value
 * and (count-1) is the newest value.
 *
 * @param buffer Pointer to the ring buffer structure
 * @param index Index of the value to retrieve (0 to count-1)
 * @return Value at the specified index, or 0.0f if index is invalid
 */
float ring_buffer_get(ring_buffer_t* buffer, uint16_t index) {
    if(index >= buffer->count) {
        return 0.0f; // Invalid index
    }

    // Calculate actual index in circular buffer
    uint16_t actual_index = (buffer->head - buffer->count + index + buffer->size) % buffer->size;
    return buffer->data[actual_index];
}

/**
 * @brief Calculate tick positions for adaptive tick drawing
 *
 * Calculates optimal positions for X and Y axis tick marks based on the
 * graph dimensions. This function determines the number of ticks and their
 * positions to provide readable axis labels.
 *
 * @param graph_width Width of the graph area in pixels
 * @param graph_height Height of the graph area in pixels
 * @param x_tick_positions Array to store calculated X-axis tick positions
 * @param y_tick_positions Array to store calculated Y-axis tick positions
 * @param num_x_ticks Pointer to store number of X-axis ticks
 * @param num_y_ticks Pointer to store number of Y-axis ticks
 */
static void calculate_tick_positions(uint8_t graph_width, uint8_t graph_height,
                                     uint8_t* x_tick_positions, uint8_t* y_tick_positions,
                                     uint8_t* num_x_ticks, uint8_t* num_y_ticks) {
    // Calculate X-axis tick positions
    if(graph_width > 0) {
        // Determine number of ticks based on graph width (minimum 2 ticks)
        *num_x_ticks = (uint8_t)(graph_width / 20);  // Adjust divisor to control tick density
        if(*num_x_ticks < 2) *num_x_ticks = 2;
        if(*num_x_ticks > 10) *num_x_ticks = 10;  // Cap at 10 ticks

        // Calculate tick interval in pixels
        float x_tick_interval_px = (float)(graph_width - 1) / (*num_x_ticks - 1);

        // Calculate exact positions using interpolation
        for(int i = 0; i < *num_x_ticks; i++) {
            // Calculate exact position using interpolation
            float x_pos_float = i * x_tick_interval_px;

            // Round to nearest integer pixel position
            x_tick_positions[i] = (uint8_t)roundf(x_pos_float);
        }
    } else {
        *num_x_ticks = 0;
    }

    // Calculate Y-axis tick positions
    if(graph_height > 0) {
        // Determine number of ticks based on graph height (minimum 2 ticks)
        *num_y_ticks = (uint8_t)(graph_height / 20);  // Adjust divisor to control tick density
        if(*num_y_ticks < 2) *num_y_ticks = 2;
        if(*num_y_ticks > 10) *num_y_ticks = 10;  // Cap at 10 ticks

        // Calculate tick interval in pixels (graph_height - 1 due to graph box requirement < graph_height)
        float y_tick_interval_px = (float)(graph_height - 1) / (*num_y_ticks - 1);

        // Calculate exact positions using interpolation
        for(int i = 0; i < *num_y_ticks; i++) {
            // Calculate exact position using interpolation
            float y_pos_float = i * y_tick_interval_px;

            // Round to nearest integer pixel position
            y_tick_positions[i] = (uint8_t)roundf(y_pos_float);
        }
    } else {
        *num_y_ticks = 0;
    }
}

/**
 * @brief Plot a graph on the OLED display
 *
 * Plots sensor data from a ring buffer onto the OLED display within the specified
 * rectangular area. The graph scales the data to fit within the provided dimensions.
 * It draws axes, tick marks, and plots data points with configurable dot sizes.
 *
 * @param buffer Pointer to the ring buffer containing sensor data
 * @param graph_x X coordinate of the top-left corner of the graph area
 * @param graph_y Y coordinate of the top-left corner of the graph area
 * @param graph_width Width of the graph area in pixels
 * @param graph_height Height of the graph area in pixels
 * @param dot_size Size of data point markers in pixels
 * @param min_y Minimum Y value for scaling (0.0f to use automatic scaling)
 * @param max_y Maximum Y value for scaling (0.0f to use automatic scaling)
 */
void graph_plot(ring_buffer_t* buffer, uint8_t graph_x, uint8_t graph_y,
                uint8_t graph_width, uint8_t graph_height,
                uint8_t dot_size, float min_y, float max_y) {
    // Validate inputs
    if(!buffer || graph_width == 0 || graph_height == 0 || dot_size == 0) {
        return;
    }

    // DEBUG: Temporarily disable axes for easier debugging
    // Draw axes
    // X axis
    ssd1306_Line(graph_x, graph_y + graph_height - 1,
                 graph_x + graph_width - 1, graph_y + graph_height - 1,
                 White);

    // Y axis
    ssd1306_Line(graph_x, graph_y,
                 graph_x, graph_y + graph_height - 1,
                 White);

    // Draw tick marks on X axis
    // Adaptive tick drawing based on graph width
    if(graph_width > 0) {
        // Pre-calculate tick positions to avoid recalculation
        uint8_t x_tick_positions[10];  // Max 10 ticks
        uint8_t y_tick_positions[10];  // Max 10 ticks
        uint8_t num_x_ticks, num_y_ticks;

        calculate_tick_positions(graph_width, graph_height, x_tick_positions, y_tick_positions, &num_x_ticks, &num_y_ticks);

        // Draw X-axis tick marks
        for(int i = 0; i < num_x_ticks; i++) {
            // Adjust position to graph coordinates
            uint8_t x_pos = graph_x + x_tick_positions[i];

            // Ensure tick marks don't go beyond graph boundaries
            if(x_pos >= graph_x && x_pos < graph_x + graph_width) {
                // Draw horizontal tick mark at bottom of graph area
                ssd1306_Line(x_pos, graph_y + graph_height - 1,
                             x_pos, graph_y + graph_height - 1 - 2,
                             White);
                // ssd1306_UpdateScreen();
            }
        }
    }

    // Draw tick marks on Y axis
    // Adaptive tick drawing based on graph height
    if(graph_height > 0) {
        // Pre-calculate tick positions to avoid recalculation
        uint8_t x_tick_positions[10];  // Max 10 ticks
        uint8_t y_tick_positions[10];  // Max 10 ticks
        uint8_t num_x_ticks, num_y_ticks;

        calculate_tick_positions(graph_width, graph_height, x_tick_positions, y_tick_positions, &num_x_ticks, &num_y_ticks);

        // Draw Y-axis tick marks
        for(int i = 0; i < num_y_ticks; i++) {
            // Adjust position to graph coordinates
            uint8_t y_pos = graph_y + y_tick_positions[i];

            // Ensure tick marks don't go beyond graph boundaries
            if(y_pos >= graph_y && y_pos < graph_y + graph_height) {
                // Draw vertical tick mark at left of graph area
                ssd1306_Line(graph_x, y_pos,
                             graph_x + 2, y_pos,
                             White);
            }
        }
    }

    // If no data, return early
    if(buffer->count == 0) {
        return;
    }

    // Determine min and max values if not provided
    float actual_min_y = (min_y != 0.0f || max_y != 0.0f) ? min_y : buffer->data[0];
    float actual_max_y = (min_y != 0.0f || max_y != 0.0f) ? max_y : buffer->data[0];

    // Find actual min and max values in the buffer
    if(min_y == 0.0f && max_y == 0.0f) {
        // Only scan the buffer if we're using automatic scaling
        for(uint16_t i = 0; i < buffer->count; i++) {
            float val = ring_buffer_get(buffer, i);
            if(val < actual_min_y) {
                actual_min_y = val;
            }
            if(val > actual_max_y) {
                actual_max_y = val;
            }
        }
    }

    // Handle case where all values are the same
    if(actual_min_y == actual_max_y) {
        actual_max_y += 1.0f;
    }

    // Scale factor for mapping values to screen coordinates
    float scale_y = (actual_max_y - actual_min_y) / (float)graph_height;

    // Plot data points
    uint16_t num_points = (buffer->count < graph_width) ? buffer->count : graph_width;

    // Plot from right to left to achieve proper scrolling effect
    // The newest data is at the head of the buffer, so we plot from right to left
    // We iterate from the newest data backwards to the oldest data
    for(uint16_t i = 0; i < num_points; i++) {
        // Access data from newest to oldest (right to left)
        // The newest data is at head-1, then head-2, etc.
        uint16_t index = (buffer->head + buffer->size - 1 - i) % buffer->size;
        float value = buffer->data[index];

        // Map value to screen coordinates - plotting from right to left
        uint8_t x = graph_x + graph_width - 1 - i;
        uint8_t y = graph_y + graph_height - 1 - (uint8_t)((value - actual_min_y) / scale_y);

        // Draw dot
        for(uint8_t dx = 0; dx < dot_size; dx++) {
            for(uint8_t dy = 0; dy < dot_size; dy++) {
                if((x + dx < SSD1306_WIDTH) && (y + dy < SSD1306_HEIGHT)) {
                    ssd1306_DrawPixel(x + dx, y + dy, White);
                }
            }
        }
    }
}

/**
 * @brief Plot two graphs from different buffers on the same display area
 *
 * Plots sensor data from two different ring buffers onto the OLED display within the specified
 * rectangular area. The graphs are scaled independently and displayed side by side or stacked.
 * This function is useful for comparing two different data sets.
 *
 * @param buffer1 Pointer to the first ring buffer containing sensor data
 * @param buffer2 Pointer to the second ring buffer containing sensor data
 * @param graph_x X coordinate of the top-left corner of the graph area
 * @param graph_y Y coordinate of the top-left corner of the graph area
 * @param graph_width Width of the graph area in pixels
 * @param graph_height Height of the graph area in pixels
 * @param dot_size Size of data point markers in pixels
 * @param min_y1 Minimum Y value for scaling of first buffer (0.0f to use automatic scaling)
 * @param max_y1 Maximum Y value for scaling of first buffer (0.0f to use automatic scaling)
 * @param min_y2 Minimum Y value for scaling of second buffer (0.0f to use automatic scaling)
 * @param max_y2 Maximum Y value for scaling of second buffer (0.0f to use automatic scaling)
 */
void graph_plot_two_buffers(ring_buffer_t* buffer1, ring_buffer_t* buffer2,
                           uint8_t graph_x, uint8_t graph_y,
                           uint8_t graph_width, uint8_t graph_height,
                           uint8_t dot_size, float min_y1, float max_y1,
                           float min_y2, float max_y2) {
    // Validate inputs
    if(!buffer1 || !buffer2 || graph_width == 0 || graph_height == 0 || dot_size == 0) {
        return;
    }

    // If no data in either buffer, return early
    if(buffer1->count == 0 && buffer2->count == 0) {
        return;
    }

    // For simplicity, we'll plot both graphs in the same area with different colors
    // (Note: This is a simplified version - in practice, you might want to split the area)

    // Plot first buffer
    if(buffer1->count > 0) {
        // Determine min and max values for first buffer
        float actual_min_y1 = (min_y1 != 0.0f || max_y1 != 0.0f) ? min_y1 : buffer1->data[0];
        float actual_max_y1 = (min_y1 != 0.0f || max_y1 != 0.0f) ? max_y1 : buffer1->data[0];

        // Find actual min and max values in the buffer
        if(min_y1 == 0.0f && max_y1 == 0.0f) {
            for(uint16_t i = 0; i < buffer1->count; i++) {
                float val = ring_buffer_get(buffer1, i);
                if(val < actual_min_y1) {
                    actual_min_y1 = val;
                }
                if(val > actual_max_y1) {
                    actual_max_y1 = val;
                }
            }
        }

        // Handle case where all values are the same
        if(actual_min_y1 == actual_max_y1) {
            actual_max_y1 += 1.0f;
        }

        // Scale factor for mapping values to screen coordinates
        float scale_y1 = (actual_max_y1 - actual_min_y1) / (float)graph_height;

        // Plot data points from first buffer
        uint16_t num_points1 = (buffer1->count < graph_width) ? buffer1->count : graph_width;

        for(uint16_t i = 0; i < num_points1; i++) {
            // Access data from newest to oldest (right to left)
            uint16_t index = (buffer1->head + buffer1->size - 1 - i) % buffer1->size;
            float value = buffer1->data[index];

            // Map value to screen coordinates - plotting from right to left
            uint8_t x = graph_x + graph_width - 1 - i;
            uint8_t y = graph_y + graph_height - 1 - (uint8_t)((value - actual_min_y1) / scale_y1);

            // Draw dot with white color
            for(uint8_t dx = 0; dx < dot_size; dx++) {
                for(uint8_t dy = 0; dy < dot_size; dy++) {
                    if((x + dx < SSD1306_WIDTH) && (y + dy < SSD1306_HEIGHT)) {
                        ssd1306_DrawPixel(x + dx, y + dy, White);
                    }
                }
            }
        }
    }

    // Plot second buffer
    if(buffer2->count > 0) {
        // Determine min and max values for second buffer
        float actual_min_y2 = (min_y2 != 0.0f || max_y2 != 0.0f) ? min_y2 : buffer2->data[0];
        float actual_max_y2 = (min_y2 != 0.0f || max_y2 != 0.0f) ? max_y2 : buffer2->data[0];

        // Find actual min and max values in the buffer
        if(min_y2 == 0.0f && max_y2 == 0.0f) {
            for(uint16_t i = 0; i < buffer2->count; i++) {
                float val = ring_buffer_get(buffer2, i);
                if(val < actual_min_y2) {
                    actual_min_y2 = val;
                }
                if(val > actual_max_y2) {
                    actual_max_y2 = val;
                }
            }
        }

        // Handle case where all values are the same
        if(actual_min_y2 == actual_max_y2) {
            actual_max_y2 += 1.0f;
        }

        // Scale factor for mapping values to screen coordinates
        float scale_y2 = (actual_max_y2 - actual_min_y2) / (float)graph_height;

        // Plot data points from second buffer
        uint16_t num_points2 = (buffer2->count < graph_width) ? buffer2->count : graph_width;

        for(uint16_t i = 0; i < num_points2; i++) {
            // Access data from newest to oldest (right to left)
            uint16_t index = (buffer2->head + buffer2->size - 1 - i) % buffer2->size;
            float value = buffer2->data[index];

            // Map value to screen coordinates - plotting from right to left
            uint8_t x = graph_x + graph_width - 1 - i;
            uint8_t y = graph_y + graph_height - 1 - (uint8_t)((value - actual_min_y2) / scale_y2);

            // Draw dot with different color (for demonstration, we'll use white)
            // Note: In a real implementation, you might want to use different colors
            for(uint8_t dx = 0; dx < dot_size; dx++) {
                for(uint8_t dy = 0; dy < dot_size; dy++) {
                    if((x + dx < SSD1306_WIDTH) && (y + dy < SSD1306_HEIGHT)) {
                        ssd1306_DrawPixel(x + dx, y + dy, White);
                    }
                }
            }
        }
    }
}

/**
 * @brief Helper function to calculate positions for 1 top, 1 bottom graph arrangement
 *
 * Calculates the positions and dimensions for two graphs arranged vertically.
 * The top graph takes half the height with a margin, and the bottom graph takes
 * the remaining space.
 *
 * @param total_width Total width of the available area
 * @param total_height Total height of the available area
 * @param margin Space between the two graphs
 * @param top_x Pointer to store X coordinate of top graph
 * @param top_y Pointer to store Y coordinate of top graph
 * @param top_width Pointer to store width of top graph
 * @param top_height Pointer to store height of top graph
 * @param bottom_x Pointer to store X coordinate of bottom graph
 * @param bottom_y Pointer to store Y coordinate of bottom graph
 * @param bottom_width Pointer to store width of bottom graph
 * @param bottom_height Pointer to store height of bottom graph
 */
void calculate_positions_1t1b(uint8_t total_width, uint8_t total_height, uint8_t margin,
                              uint8_t* top_x, uint8_t* top_y, uint8_t* top_width, uint8_t* top_height,
                              uint8_t* bottom_x, uint8_t* bottom_y, uint8_t* bottom_width, uint8_t* bottom_height) {
    *top_x = 0;
    *top_y = 0;
    *top_width = total_width;
    *top_height = (total_height - margin) / 2;

    *bottom_x = 0;
    *bottom_y = *top_height + margin;
    *bottom_width = total_width;
    *bottom_height = (total_height - margin) / 2;
}

/**
 * @brief Helper function to calculate positions for 1 left, 1 right graph arrangement
 *
 * Calculates the positions and dimensions for two graphs arranged horizontally.
 * The left graph takes half the width with a margin, and the right graph takes
 * the remaining space.
 *
 * @param total_width Total width of the available area
 * @param total_height Total height of the available area
 * @param margin Space between the two graphs
 * @param left_x Pointer to store X coordinate of left graph
 * @param left_y Pointer to store Y coordinate of left graph
 * @param left_width Pointer to store width of left graph
 * @param left_height Pointer to store height of left graph
 * @param right_x Pointer to store X coordinate of right graph
 * @param right_y Pointer to store Y coordinate of right graph
 * @param right_width Pointer to store width of right graph
 * @param right_height Pointer to store height of right graph
 */
void calculate_positions_1l1r(uint8_t total_width, uint8_t total_height, uint8_t margin,
                             uint8_t* left_x, uint8_t* left_y, uint8_t* left_width, uint8_t* left_height,
                             uint8_t* right_x, uint8_t* right_y, uint8_t* right_width, uint8_t* right_height) {
    *left_x = 0;
    *left_y = 0;
    *left_width = (total_width - margin) / 2;
    *left_height = total_height;

    *right_x = *left_width + margin;
    *right_y = 0;
    *right_width = (total_width - margin) / 2;
    *right_height = total_height;
}

/**
 * @brief Helper function to calculate positions for 1 top, 2 bottom graph arrangement
 *
 * Calculates the positions and dimensions for three graphs arranged with one
 * wide graph on top and two smaller graphs on the bottom.
 *
 * @param total_width Total width of the available area
 * @param total_height Total height of the available area
 * @param margin Space between the graphs
 * @param top_x Pointer to store X coordinate of top graph
 * @param top_y Pointer to store Y coordinate of top graph
 * @param top_width Pointer to store width of top graph
 * @param top_height Pointer to store height of top graph
 * @param bottom1_x Pointer to store X coordinate of first bottom graph
 * @param bottom1_y Pointer to store Y coordinate of first bottom graph
 * @param bottom1_width Pointer to store width of first bottom graph
 * @param bottom1_height Pointer to store height of first bottom graph
 * @param bottom2_x Pointer to store X coordinate of second bottom graph
 * @param bottom2_y Pointer to store Y coordinate of second bottom graph
 * @param bottom2_width Pointer to store width of second bottom graph
 * @param bottom2_height Pointer to store height of second bottom graph
 */
void calculate_positions_1t2b(uint8_t total_width, uint8_t total_height, uint8_t margin,
                             uint8_t* top_x, uint8_t* top_y, uint8_t* top_width, uint8_t* top_height,
                             uint8_t* bottom1_x, uint8_t* bottom1_y, uint8_t* bottom1_width, uint8_t* bottom1_height,
                             uint8_t* bottom2_x, uint8_t* bottom2_y, uint8_t* bottom2_width, uint8_t* bottom2_height) {
    *top_x = 0;
    *top_y = 0;
    *top_width = total_width;
    *top_height = (total_height - margin) / 2;

    *bottom1_x = 0;
    *bottom1_y = *top_height + margin;
    *bottom1_width = (total_width - margin) / 2;
    *bottom1_height = (total_height - margin) / 2;

    *bottom2_x = *bottom1_width + margin;
    *bottom2_y = *top_height + margin;
    *bottom2_width = (total_width - margin) / 2;
    *bottom2_height = (total_height - margin) / 2;
}

/**
 * @brief Helper function to calculate positions for 2 top, 1 bottom graph arrangement
 *
 * Calculates the positions and dimensions for three graphs arranged with two
 * smaller graphs on top and one wide graph on the bottom.
 *
 * @param total_width Total width of the available area
 * @param total_height Total height of the available area
 * @param margin Space between the graphs
 * @param top1_x Pointer to store X coordinate of first top graph
 * @param top1_y Pointer to store Y coordinate of first top graph
 * @param top1_width Pointer to store width of first top graph
 * @param top1_height Pointer to store height of first top graph
 * @param top2_x Pointer to store X coordinate of second top graph
 * @param top2_y Pointer to store Y coordinate of second top graph
 * @param top2_width Pointer to store width of second top graph
 * @param top2_height Pointer to store height of second top graph
 * @param bottom_x Pointer to store X coordinate of bottom graph
 * @param bottom_y Pointer to store Y coordinate of bottom graph
 * @param bottom_width Pointer to store width of bottom graph
 * @param bottom_height Pointer to store height of bottom graph
 */
void calculate_positions_2t1b(uint8_t total_width, uint8_t total_height, uint8_t margin,
                             uint8_t* top1_x, uint8_t* top1_y, uint8_t* top1_width, uint8_t* top1_height,
                             uint8_t* top2_x, uint8_t* top2_y, uint8_t* top2_width, uint8_t* top2_height,
                             uint8_t* bottom_x, uint8_t* bottom_y, uint8_t* bottom_width, uint8_t* bottom_height) {
    *top1_x = 0;
    *top1_y = 0;
    *top1_width = (total_width - margin) / 2;
    *top1_height = (total_height - margin) / 2;

    *top2_x = *top1_width + margin;
    *top2_y = 0;
    *top2_width = (total_width - margin) / 2;
    *top2_height = (total_height - margin) / 2;

    *bottom_x = 0;
    *bottom_y = *top1_height + margin;
    *bottom_width = total_width;
    *bottom_height = (total_height - margin) / 2;
}

/**
 * @brief Helper function to calculate positions for 1 left, 2 right graph arrangement
 *
 * Calculates the positions and dimensions for three graphs arranged with one
 * tall graph on the left and two shorter graphs on the right.
 *
 * @param total_width Total width of the available area
 * @param total_height Total height of the available area
 * @param margin Space between the graphs
 * @param left_x Pointer to store X coordinate of left graph
 * @param left_y Pointer to store Y coordinate of left graph
 * @param left_width Pointer to store width of left graph
 * @param left_height Pointer to store height of left graph
 * @param right1_x Pointer to store X coordinate of first right graph
 * @param right1_y Pointer to store Y coordinate of first right graph
 * @param right1_width Pointer to store width of first right graph
 * @param right1_height Pointer to store height of first right graph
 * @param right2_x Pointer to store X coordinate of second right graph
 * @param right2_y Pointer to store Y coordinate of second right graph
 * @param right2_width Pointer to store width of second right graph
 * @param right2_height Pointer to store height of second right graph
 */
void calculate_positions_1l2r(uint8_t total_width, uint8_t total_height, uint8_t margin,
                             uint8_t* left_x, uint8_t* left_y, uint8_t* left_width, uint8_t* left_height,
                             uint8_t* right1_x, uint8_t* right1_y, uint8_t* right1_width, uint8_t* right1_height,
                             uint8_t* right2_x, uint8_t* right2_y, uint8_t* right2_width, uint8_t* right2_height) {
    *left_x = 0;
    *left_y = 0;
    *left_width = (total_width - margin) / 2;
    *left_height = total_height;

    *right1_x = *left_width + margin;
    *right1_y = 0;
    *right1_width = (total_width - margin) / 2;
    *right1_height = (total_height - margin) / 2;

    *right2_x = *left_width + margin;
    *right2_y = *right1_height + margin;
    *right2_width = (total_width - margin) / 2;
    *right2_height = (total_height - margin) / 2;
}

/**
 * @brief Helper function to calculate positions for 2 left, 1 right graph arrangement
 *
 * Calculates the positions and dimensions for three graphs arranged with two
 * shorter graphs on the left and one tall graph on the right.
 *
 * @param total_width Total width of the available area
 * @param total_height Total height of the available area
 * @param margin Space between the graphs
 * @param left1_x Pointer to store X coordinate of first left graph
 * @param left1_y Pointer to store Y coordinate of first left graph
 * @param left1_width Pointer to store width of first left graph
 * @param left1_height Pointer to store height of first left graph
 * @param left2_x Pointer to store X coordinate of second left graph
 * @param left2_y Pointer to store Y coordinate of second left graph
 * @param left2_width Pointer to store width of second left graph
 * @param left2_height Pointer to store height of second left graph
 * @param right_x Pointer to store X coordinate of right graph
 * @param right_y Pointer to store Y coordinate of right graph
 * @param right_width Pointer to store width of right graph
 * @param right_height Pointer to store height of right graph
 */
void calculate_positions_2l1r(uint8_t total_width, uint8_t total_height, uint8_t margin,
                              uint8_t* left1_x, uint8_t* left1_y, uint8_t* left1_width, uint8_t* left1_height,
                              uint8_t* left2_x, uint8_t* left2_y, uint8_t* left2_width, uint8_t* left2_height,
                              uint8_t* right_x, uint8_t* right_y, uint8_t* right_width, uint8_t* right_height) {
    *left1_x = 0;
    *left1_y = 0;
    *left1_width = (total_width - margin) / 2;
    *left1_height = (total_height - margin) / 2;

    *left2_x = 0;
    *left2_y = *left1_height + margin;
    *left2_width = (total_width - margin) / 2;
    *left2_height = (total_height - margin) / 2;

    *right_x = *left1_width + margin;
    *right_y = 0;
    *right_width = (total_width - margin) / 2;
    *right_height = total_height;
}

/**
 * @brief Helper function to calculate positions for 2 top, 2 bottom graph arrangement
 *
 * Calculates the positions and dimensions for four graphs arranged in a 2x2 grid.
 *
 * @param total_width Total width of the available area
 * @param total_height Total height of the available area
 * @param margin Space between the graphs
 * @param top1_x Pointer to store X coordinate of top-left graph
 * @param top1_y Pointer to store Y coordinate of top-left graph
 * @param top1_width Pointer to store width of top-left graph
 * @param top1_height Pointer to store height of top-left graph
 * @param top2_x Pointer to store X coordinate of top-right graph
 * @param top2_y Pointer to store Y coordinate of top-right graph
 * @param top2_width Pointer to store width of top-right graph
 * @param top2_height Pointer to store height of top-right graph
 * @param bottom1_x Pointer to store X coordinate of bottom-left graph
 * @param bottom1_y Pointer to store Y coordinate of bottom-left graph
 * @param bottom1_width Pointer to store width of bottom-left graph
 * @param bottom1_height Pointer to store height of bottom-left graph
 * @param bottom2_x Pointer to store X coordinate of bottom-right graph
 * @param bottom2_y Pointer to store Y coordinate of bottom-right graph
 * @param bottom2_width Pointer to store width of bottom-right graph
 * @param bottom2_height Pointer to store height of bottom-right graph
 */
void calculate_positions_2t2b(uint8_t total_width, uint8_t total_height, uint8_t margin,
                              uint8_t* top1_x, uint8_t* top1_y, uint8_t* top1_width, uint8_t* top1_height,
                              uint8_t* top2_x, uint8_t* top2_y, uint8_t* top2_width, uint8_t* top2_height,
                              uint8_t* bottom1_x, uint8_t* bottom1_y, uint8_t* bottom1_width, uint8_t* bottom1_height,
                              uint8_t* bottom2_x, uint8_t* bottom2_y, uint8_t* bottom2_width, uint8_t* bottom2_height) {
    *top1_x = 0;
    *top1_y = 0;
    *top1_width = (total_width - margin) / 2;
    *top1_height = (total_height - margin) / 2;

    *top2_x = *top1_width + margin;
    *top2_y = 0;
    *top2_width = (total_width - margin) / 2;
    *top2_height = (total_height - margin) / 2;

    *bottom1_x = 0;
    *bottom1_y = *top1_height + margin;
    *bottom1_width = (total_width - margin) / 2;
    *bottom1_height = (total_height - margin) / 2;

    *bottom2_x = *bottom1_width + margin;
    *bottom2_y = *top1_height + margin;
    *bottom2_width = (total_width - margin) / 2;
    *bottom2_height = (total_height - margin) / 2;
}

// Function to plot 2 graphs: one wide on top, one wide on bottom
void graph_plots_1t1b(ring_buffer_t* buffer1, ring_buffer_t* buffer2,
                      uint8_t graph_x, uint8_t graph_y, uint8_t graph_width, uint8_t graph_height,
                      uint8_t margin) {
    uint8_t top_x, top_y, top_width, top_height;
    uint8_t bottom_x, bottom_y, bottom_width, bottom_height;

    calculate_positions_1t1b(graph_width, graph_height, margin,
                            &top_x, &top_y, &top_width, &top_height,
                            &bottom_x, &bottom_y, &bottom_width, &bottom_height);

    graph_plot(buffer1, graph_x + top_x, graph_y + top_y, top_width, top_height, 1, 0.0f, 100.0f);
    graph_plot(buffer2, graph_x + bottom_x, graph_y + bottom_y, bottom_width, bottom_height, 1, 0.0f, 100.0f);
}

// Function to plot 2 graphs: one tall on left, one tall on right
void graph_plots_1l1r(ring_buffer_t* buffer1, ring_buffer_t* buffer2,
                      uint8_t graph_x, uint8_t graph_y, uint8_t graph_width, uint8_t graph_height,
                      uint8_t margin) {
    uint8_t left_x, left_y, left_width, left_height;
    uint8_t right_x, right_y, right_width, right_height;

    calculate_positions_1l1r(graph_width, graph_height, margin,
                            &left_x, &left_y, &left_width, &left_height,
                            &right_x, &right_y, &right_width, &right_height);

    graph_plot(buffer1, graph_x + left_x, graph_y + left_y, left_width, left_height, 1, 0.0f, 100.0f);
    graph_plot(buffer2, graph_x + right_x, graph_y + right_y, right_width, right_height, 1, 0.0f, 100.0f);
}

// Function to plot 3 graphs: 1 wide on top, 2 shallow on bottom
void graph_plots_1t2b(ring_buffer_t* buffer1, ring_buffer_t* buffer2, ring_buffer_t* buffer3,
                     uint8_t graph_x, uint8_t graph_y, uint8_t graph_width, uint8_t graph_height,
                     uint8_t margin) {
    uint8_t top_x, top_y, top_width, top_height;
    uint8_t bottom1_x, bottom1_y, bottom1_width, bottom1_height;
    uint8_t bottom2_x, bottom2_y, bottom2_width, bottom2_height;

    calculate_positions_1t2b(graph_width, graph_height, margin,
                            &top_x, &top_y, &top_width, &top_height,
                            &bottom1_x, &bottom1_y, &bottom1_width, &bottom1_height,
                            &bottom2_x, &bottom2_y, &bottom2_width, &bottom2_height);

    graph_plot(buffer1, graph_x + top_x, graph_y + top_y, top_width, top_height, 1, 0.0f, 100.0f);
    graph_plot(buffer2, graph_x + bottom1_x, graph_y + bottom1_y, bottom1_width, bottom1_height, 1, 0.0f, 100.0f);
    graph_plot(buffer3, graph_x + bottom2_x, graph_y + bottom2_y, bottom2_width, bottom2_height, 1, 0.0f, 100.0f);
}

// Function to plot 3 graphs: 2 shallow on top, 1 wide on bottom
void graph_plots_2t1b(ring_buffer_t* buffer1, ring_buffer_t* buffer2, ring_buffer_t* buffer3,
                     uint8_t graph_x, uint8_t graph_y, uint8_t graph_width, uint8_t graph_height,
                     uint8_t margin) {
    uint8_t top1_x, top1_y, top1_width, top1_height;
    uint8_t top2_x, top2_y, top2_width, top2_height;
    uint8_t bottom_x, bottom_y, bottom_width, bottom_height;

    calculate_positions_2t1b(graph_width, graph_height, margin,
                            &top1_x, &top1_y, &top1_width, &top1_height,
                            &top2_x, &top2_y, &top2_width, &top2_height,
                            &bottom_x, &bottom_y, &bottom_width, &bottom_height);

    graph_plot(buffer1, graph_x + top1_x, graph_y + top1_y, top1_width, top1_height, 1, 0.0f, 100.0f);
    graph_plot(buffer2, graph_x + top2_x, graph_y + top2_y, top2_width, top2_height, 1, 0.0f, 100.0f);
    graph_plot(buffer3, graph_x + bottom_x, graph_y + bottom_y, bottom_width, bottom_height, 1, 0.0f, 100.0f);
}

// Function to plot 3 graphs: 1 tall on left, 2 short on right
void graph_plots_1l2r(ring_buffer_t* buffer1, ring_buffer_t* buffer2, ring_buffer_t* buffer3,
                     uint8_t graph_x, uint8_t graph_y, uint8_t graph_width, uint8_t graph_height,
                     uint8_t margin) {
    uint8_t left_x, left_y, left_width, left_height;
    uint8_t right1_x, right1_y, right1_width, right1_height;
    uint8_t right2_x, right2_y, right2_width, right2_height;

    calculate_positions_1l2r(graph_width, graph_height, margin,
                           &left_x, &left_y, &left_width, &left_height,
                           &right1_x, &right1_y, &right1_width, &right1_height,
                           &right2_x, &right2_y, &right2_width, &right2_height);

    graph_plot(buffer1, graph_x + left_x, graph_y + left_y, left_width, left_height, 1, 0.0f, 100.0f);
    graph_plot(buffer2, graph_x + right1_x, graph_y + right1_y, right1_width, right1_height, 1, 0.0f, 100.0f);
    graph_plot(buffer3, graph_x + right2_x, graph_y + right2_y, right2_width, right2_height, 1, 0.0f, 100.0f);
}

// Function to plot 3 graphs: 2 short on left, 1 tall on right
void graph_plots_2l1r(ring_buffer_t* buffer1, ring_buffer_t* buffer2, ring_buffer_t* buffer3,
                     uint8_t graph_x, uint8_t graph_y, uint8_t graph_width, uint8_t graph_height,
                     uint8_t margin) {
    uint8_t left1_x, left1_y, left1_width, left1_height;
    uint8_t left2_x, left2_y, left2_width, left2_height;
    uint8_t right_x, right_y, right_width, right_height;

    calculate_positions_2l1r(graph_width, graph_height, margin,
                            &left1_x, &left1_y, &left1_width, &left1_height,
                            &left2_x, &left2_y, &left2_width, &left2_height,
                            &right_x, &right_y, &right_width, &right_height);

    graph_plot(buffer1, graph_x + left1_x, graph_y + left1_y, left1_width, left1_height, 1, 0.0f, 100.0f);
    graph_plot(buffer2, graph_x + left2_x, graph_y + left2_y, left2_width, left2_height, 1, 0.0f, 100.0f);
    graph_plot(buffer3, graph_x + right_x, graph_y + right_y, right_width, right_height, 1, 0.0f, 100.0f);
}

// Function to plot 4 graphs: 2 top, 2 bottom
void graph_plots_2t2b(ring_buffer_t* buffer1, ring_buffer_t* buffer2,
                     ring_buffer_t* buffer3, ring_buffer_t* buffer4,
                     uint8_t graph_x, uint8_t graph_y, uint8_t graph_width, uint8_t graph_height,
                     uint8_t margin) {
    uint8_t top1_x, top1_y, top1_width, top1_height;
    uint8_t top2_x, top2_y, top2_width, top2_height;
    uint8_t bottom1_x, bottom1_y, bottom1_width, bottom1_height;
    uint8_t bottom2_x, bottom2_y, bottom2_width, bottom2_height;

    calculate_positions_2t2b(graph_width, graph_height, margin,
                            &top1_x, &top1_y, &top1_width, &top1_height,
                            &top2_x, &top2_y, &top2_width, &top2_height,
                            &bottom1_x, &bottom1_y, &bottom1_width, &bottom1_height,
                            &bottom2_x, &bottom2_y, &bottom2_width, &bottom2_height);

    graph_plot(buffer1, graph_x + top1_x, graph_y + top1_y, top1_width, top1_height, 1, 0.0f, 100.0f);
    graph_plot(buffer2, graph_x + top2_x, graph_y + top2_y, top2_width, top2_height, 1, 0.0f, 100.0f);
    graph_plot(buffer3, graph_x + bottom1_x, graph_y + bottom1_y, bottom1_width, bottom1_height, 1, 0.0f, 100.0f);
    graph_plot(buffer4, graph_x + bottom2_x, graph_y + bottom2_y, bottom2_width, bottom2_height, 1, 0.0f, 100.0f);
}

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
void debug_screen_edges(void) {
    static uint8_t blink_phase = 0;
    static uint8_t phase_frame_count = 0;

    // Each phase lasts 10 frames
    if(phase_frame_count >= 10) {
        // Advance to next phase
        blink_phase = (blink_phase + 1) % 4;
        phase_frame_count = 0;
    }

    // Cycle through 4 phases:
    // 0: Nothing drawn (black screen)
    // 1: 4 corner dots only
    // 2: Rectangle drawn around screen
    // 3: Everything cleared (black screen)

    switch(blink_phase) {
        case 0: // Nothing drawn - black screen
            // Do nothing, screen remains black
            break;

        case 1: // Only 4 corner dots
            // Draw 4 prominent dots at screen vertices
            ssd1306_DrawPixel(0, 0, White);
            ssd1306_DrawPixel(SSD1306_WIDTH - 1, 0, White);
            ssd1306_DrawPixel(0, SSD1306_HEIGHT - 1, White);
            ssd1306_DrawPixel(SSD1306_WIDTH - 1, SSD1306_HEIGHT - 1, White);
            break;

        case 2: // Rectangle drawn around screen
            // Draw 4 dots at screen vertices
            ssd1306_DrawPixel(0, 0, White);
            ssd1306_DrawPixel(SSD1306_WIDTH - 1, 0, White);
            ssd1306_DrawPixel(0, SSD1306_HEIGHT - 1, White);
            ssd1306_DrawPixel(SSD1306_WIDTH - 1, SSD1306_HEIGHT - 1, White);

            // Draw lines to outline the screen edges
            ssd1306_Line(0, 0, SSD1306_WIDTH - 1, 0, White);                    // Top edge
            ssd1306_Line(0, SSD1306_HEIGHT - 1, SSD1306_WIDTH - 1, SSD1306_HEIGHT - 1, White); // Bottom edge
            ssd1306_Line(0, 0, 0, SSD1306_HEIGHT - 1, White);                       // Left edge
            ssd1306_Line(SSD1306_WIDTH - 1, 0, SSD1306_WIDTH - 1, SSD1306_HEIGHT - 1, White); // Right edge
            break;

        case 3: // Everything cleared - black screen
            // Do nothing, screen remains black
            break;
    }

    // Increment frame counter for current phase
    phase_frame_count++;
}

/**
 * @brief Plot a full-screen graph for debugging purposes
 *
 * This function creates a simple sine wave pattern in the sensor buffer and
 * plots it across the entire OLED display. It's intended for debugging purposes
 * to verify that graph plotting functionality works correctly.
 *
 * The function fills the sensor buffer with a known pattern, clears the screen,
 * plots the graph, and then displays a debug visualization of screen edges.
 */
void debug_fullscreen_graph(void) {
    // Create a simple sine wave for debugging purposes
    // Fill the buffer with a known pattern
    for(int i = 0; i < 100; i++) {
        float value = 50.0f + 20.0f * sinf(i * 0.2f);
        ring_buffer_push(&sensor_ring_buffer_1, value);
    }

    // Clear screen
    ssd1306_Fill(Black);

    // Plot a full-screen graph (using full display dimensions)
    graph_plot(&sensor_ring_buffer_1, 0, 0, SSD1306_WIDTH, SSD1306_HEIGHT, 1, 0.0f, 100.0f);

    // Draw debug visualization
    debug_screen_edges();

    // Update the display
    ssd1306_UpdateScreen();
}