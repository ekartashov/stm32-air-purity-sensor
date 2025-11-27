#include "oled_graph.h"
#include "ssd1306.h"

// Ring buffers for 4 different sensor data streams
#define BUFFER_SIZE 100
float sensor_data_1[BUFFER_SIZE];
float sensor_data_2[BUFFER_SIZE];
float sensor_data_3[BUFFER_SIZE];
float sensor_data_4[BUFFER_SIZE];

ring_buffer_t sensor_ring_buffer_1 = {
    .data = sensor_data_1,
    .size = BUFFER_SIZE,
    .head = 0,
    .count = 0
};

ring_buffer_t sensor_ring_buffer_2 = {
    .data = sensor_data_2,
    .size = BUFFER_SIZE,
    .head = 0,
    .count = 0
};

ring_buffer_t sensor_ring_buffer_3 = {
    .data = sensor_data_3,
    .size = BUFFER_SIZE,
    .head = 0,
    .count = 0
};

ring_buffer_t sensor_ring_buffer_4 = {
    .data = sensor_data_4,
    .size = BUFFER_SIZE,
    .head = 0,
    .count = 0
};

// Function to add a value to the ring buffer
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

// Function to get value at a specific index (oldest to newest)
float ring_buffer_get(ring_buffer_t* buffer, uint16_t index) {
    if(index >= buffer->count) {
        return 0.0f; // Invalid index
    }

    // Calculate actual index in circular buffer
    uint16_t actual_index = (buffer->head - buffer->count + index + buffer->size) % buffer->size;
    return buffer->data[actual_index];
}

// Function to calculate tick positions for adaptive tick drawing
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

// Function to plot a graph on the OLED display
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
    // ssd1306_Line(graph_x, graph_y + graph_height - 1,
    //              graph_x + graph_width - 1, graph_y + graph_height - 1,
    //              White);

    // Y axis
    // ssd1306_Line(graph_x, graph_y,
    //              graph_x, graph_y + graph_height - 1,
    //              White);

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

// Helper function to calculate positions for 1 top, 1 bottom graph arrangement
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

// Helper function to calculate positions for 1 left, 1 right graph arrangement
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

// Helper function to calculate positions for 1 top, 2 bottom graph arrangement
void calculate_positions_1t2b(uint8_t total_width, uint8_t total_height, uint8_t margin,
                             uint8_t* top_x, uint8_t* top_y, uint8_t* top_width, uint8_t* top_height,
                             uint8_t* bottom1_x, uint8_t* bottom1_y, uint8_t* bottom1_width, uint8_t* bottom1_height,
                             uint8_t* bottom2_x, uint8_t* bottom2_y, uint8_t* bottom2_width, uint8_t* bottom2_height) {
    *top_x = 0;
    *top_y = 0;
    *top_width = total_width;
    *top_height = (total_height - margin) / 3;

    *bottom1_x = 0;
    *bottom1_y = *top_height + margin;
    *bottom1_width = (total_width - margin) / 2;
    *bottom1_height = (total_height - margin) / 3;

    *bottom2_x = *bottom1_width + margin;
    *bottom2_y = *top_height + margin;
    *bottom2_width = (total_width - margin) / 2;
    *bottom2_height = (total_height - margin) / 3;
}

// Helper function to calculate positions for 2 top, 1 bottom graph arrangement
void calculate_positions_2t1b(uint8_t total_width, uint8_t total_height, uint8_t margin,
                             uint8_t* top1_x, uint8_t* top1_y, uint8_t* top1_width, uint8_t* top1_height,
                             uint8_t* top2_x, uint8_t* top2_y, uint8_t* top2_width, uint8_t* top2_height,
                             uint8_t* bottom_x, uint8_t* bottom_y, uint8_t* bottom_width, uint8_t* bottom_height) {
    *top1_x = 0;
    *top1_y = 0;
    *top1_width = (total_width - margin) / 2;
    *top1_height = (total_height - margin) / 3;

    *top2_x = *top1_width + margin;
    *top2_y = 0;
    *top2_width = (total_width - margin) / 2;
    *top2_height = (total_height - margin) / 3;

    *bottom_x = 0;
    *bottom_y = *top1_height + margin;
    *bottom_width = total_width;
    *bottom_height = (total_height - margin) / 3;
}

// Helper function to calculate positions for 1 left, 2 right graph arrangement
void calculate_positions_1l2r(uint8_t total_width, uint8_t total_height, uint8_t margin,
                             uint8_t* left_x, uint8_t* left_y, uint8_t* left_width, uint8_t* left_height,
                             uint8_t* right1_x, uint8_t* right1_y, uint8_t* right1_width, uint8_t* right1_height,
                             uint8_t* right2_x, uint8_t* right2_y, uint8_t* right2_width, uint8_t* right2_height) {
    *left_x = 0;
    *left_y = 0;
    *left_width = (total_width - margin) / 3;
    *left_height = total_height;

    *right1_x = *left_width + margin;
    *right1_y = 0;
    *right1_width = (total_width - margin) / 3;
    *right1_height = (total_height - margin) / 2;

    *right2_x = *left_width + margin;
    *right2_y = *right1_height + margin;
    *right2_width = (total_width - margin) / 3;
    *right2_height = (total_height - margin) / 2;
}

// Helper function to calculate positions for 2 left, 1 right graph arrangement
void calculate_positions_2l1r(uint8_t total_width, uint8_t total_height, uint8_t margin,
                              uint8_t* left1_x, uint8_t* left1_y, uint8_t* left1_width, uint8_t* left1_height,
                              uint8_t* left2_x, uint8_t* left2_y, uint8_t* left2_width, uint8_t* left2_height,
                              uint8_t* right_x, uint8_t* right_y, uint8_t* right_width, uint8_t* right_height) {
    *left1_x = 0;
    *left1_y = 0;
    *left1_width = (total_width - margin) / 3;
    *left1_height = (total_height - margin) / 2;

    *left2_x = 0;
    *left2_y = *left1_height + margin;
    *left2_width = (total_width - margin) / 3;
    *left2_height = (total_height - margin) / 2;

    *right_x = *left1_width + margin;
    *right_y = 0;
    *right_width = (total_width - margin) / 3;
    *right_height = total_height;
}

// Helper function to calculate positions for 2 top, 2 bottom graph arrangement
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

// Debug function to visualize screen edges and vertices
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

// Simplified function to plot a full-screen graph for debugging
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