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
#include <math.h>
#include <string.h>

/* ---------- Ring buffer storage ---------- */

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
static float sensor_data_5[GRAPH_BUFFER_SIZE];
static float sensor_data_6[GRAPH_BUFFER_SIZE];
static float sensor_data_7[GRAPH_BUFFER_SIZE];

/**
 * @brief Ring buffer structures for sensor data
 *
 * Four ring buffer structures initialized with their respective data arrays.
 * These are used to store and manage sensor readings for graphing.
 */
ring_buffer_t sensor_ring_buffer_1 = {
    .data  = sensor_data_1,
    .size  = GRAPH_BUFFER_SIZE,
    .head  = 0,
    .count = 0
};

ring_buffer_t sensor_ring_buffer_2 = {
    .data  = sensor_data_2,
    .size  = GRAPH_BUFFER_SIZE,
    .head  = 0,
    .count = 0
};

ring_buffer_t sensor_ring_buffer_3 = {
    .data  = sensor_data_3,
    .size  = GRAPH_BUFFER_SIZE,
    .head  = 0,
    .count = 0
};

ring_buffer_t sensor_ring_buffer_4 = {
    .data  = sensor_data_4,
    .size  = GRAPH_BUFFER_SIZE,
    .head  = 0,
    .count = 0
};

ring_buffer_t sensor_ring_buffer_5 = {
    .data  = sensor_data_5,
    .size  = GRAPH_BUFFER_SIZE,
    .head  = 0,
    .count = 0
};

ring_buffer_t sensor_ring_buffer_6 = {
    .data  = sensor_data_6,
    .size  = GRAPH_BUFFER_SIZE,
    .head  = 0,
    .count = 0
};

ring_buffer_t sensor_ring_buffer_7 = {
    .data  = sensor_data_7,
    .size  = GRAPH_BUFFER_SIZE,
    .head  = 0,
    .count = 0
};


/* ---------- Blinking state ---------- */

/* Blink phase: 0 = label OFF, 1 = label ON */
static uint8_t  s_blink_phase   = 1u;
/* Frame counter for blinking */
static uint16_t s_blink_counter = 0u;

/* How many frames between phase toggles (bigger = slower blink) */
#define GRAPH_BLINK_PERIOD_FRAMES 5u

static void graph_blink_tick(void)
{
    if (++s_blink_counter >= GRAPH_BLINK_PERIOD_FRAMES) {
        s_blink_counter = 0u;
        s_blink_phase ^= 1u;
    }
}

/* ---------- Ring buffer helpers ---------- */

void ring_buffer_push(ring_buffer_t* buffer, float value)
{
    if (!buffer || !buffer->data || buffer->size == 0) {
        return;
    }

    buffer->data[buffer->head] = value;

    if (buffer->count < buffer->size) {
        buffer->count++;
    }

    buffer->head = (uint16_t)((buffer->head + 1u) % buffer->size);
}

float ring_buffer_get(ring_buffer_t* buffer, uint16_t index)
{
    if (!buffer || !buffer->data || index >= buffer->count || buffer->size == 0) {
        return 0.0f; // Invalid index
    }

    uint16_t actual_index =
        (uint16_t)((buffer->head + buffer->size - buffer->count + index) % buffer->size);
    return buffer->data[actual_index];
}

/* ---------- Tick positions (axes) ---------- */

static void calculate_tick_positions(uint8_t graph_width, uint8_t graph_height,
                                     uint8_t* x_tick_positions, uint8_t* y_tick_positions,
                                     uint8_t* num_x_ticks, uint8_t* num_y_ticks)
{
    /* X-axis ticks */
    if (graph_width > 0u) {
        *num_x_ticks = (uint8_t)(graph_width / 20u);
        if (*num_x_ticks < 2u)  *num_x_ticks = 2u;
        if (*num_x_ticks > 10u) *num_x_ticks = 10u;

        float x_tick_interval_px = (float)(graph_width - 1u) / (float)(*num_x_ticks - 1u);

        for (uint8_t i = 0u; i < *num_x_ticks; i++) {
            float x_pos_float = i * x_tick_interval_px;
            x_tick_positions[i] = (uint8_t)roundf(x_pos_float);
        }
    } else {
        *num_x_ticks = 0u;
    }

    /* Y-axis ticks */
    if (graph_height > 0u) {
        *num_y_ticks = (uint8_t)(graph_height / 20u);
        if (*num_y_ticks < 2u)  *num_y_ticks = 2u;
        if (*num_y_ticks > 10u) *num_y_ticks = 10u;

        float y_tick_interval_px = (float)(graph_height - 1u) / (float)(*num_y_ticks - 1u);

        for (uint8_t i = 0u; i < *num_y_ticks; i++) {
            float y_pos_float = i * y_tick_interval_px;
            y_tick_positions[i] = (uint8_t)roundf(y_pos_float);
        }
    } else {
        *num_y_ticks = 0u;
    }
}

/* ---------- Core graph plotting (extended) ---------- */

void graph_plot_ex(ring_buffer_t* buffer,
                   uint8_t graph_x, uint8_t graph_y,
                   uint8_t graph_width, uint8_t graph_height,
                   uint8_t dot_size, float min_y, float max_y,
                   const char* legend_text,
                   uint8_t blink_label,
                   uint8_t hide_graph)
{
    /* Option 1 core: if hidden OR buffer is NULL, skip drawing entirely. */
    if (hide_graph ||
        buffer == NULL ||
        graph_width == 0u || graph_height == 0u || dot_size == 0u) {
        return;
    }

    if (buffer->count == 0u || buffer->size == 0u || buffer->data == NULL) {
        return;
    }

    /* Axes */

    /* X axis */
    ssd1306_Line(graph_x,
                 graph_y + graph_height - 1u,
                 graph_x + graph_width - 1u,
                 graph_y + graph_height - 1u,
                 White);

    /* Y axis */
    ssd1306_Line(graph_x,
                 graph_y,
                 graph_x,
                 graph_y + graph_height - 1u,
                 White);

    /* Ticks */
    if (graph_width > 0u || graph_height > 0u) {
        uint8_t x_tick_positions[10];
        uint8_t y_tick_positions[10];
        uint8_t num_x_ticks;
        uint8_t num_y_ticks;

        calculate_tick_positions(graph_width, graph_height,
                                 x_tick_positions, y_tick_positions,
                                 &num_x_ticks, &num_y_ticks);

        /* X-axis tick marks */
        if (graph_width > 0u) {
            for (uint8_t i = 0u; i < num_x_ticks; i++) {
                uint8_t x_pos = (uint8_t)(graph_x + x_tick_positions[i]);
                if (x_pos >= graph_x && x_pos < (uint8_t)(graph_x + graph_width)) {
                    ssd1306_Line(x_pos,
                                 graph_y + graph_height - 1u,
                                 x_pos,
                                 graph_y + graph_height - 3u,
                                 White);
                }
            }
        }

        /* Y-axis tick marks */
        if (graph_height > 0u) {
            for (uint8_t i = 0u; i < num_y_ticks; i++) {
                uint8_t y_pos = (uint8_t)(graph_y + y_tick_positions[i]);
                if (y_pos >= graph_y && y_pos < (uint8_t)(graph_y + graph_height)) {
                    ssd1306_Line(graph_x,
                                 y_pos,
                                 graph_x + 2u,
                                 y_pos,
                                 White);
                }
            }
        }
    }

    /* Determine min/max */
    float actual_min_y;
    float actual_max_y;

    if (min_y != 0.0f || max_y != 0.0f) {
        actual_min_y = min_y;
        actual_max_y = max_y;
    } else {
        actual_min_y = buffer->data[0];
        actual_max_y = buffer->data[0];

        for (uint16_t i = 0u; i < buffer->count; i++) {
            float val = ring_buffer_get(buffer, i);
            if (val < actual_min_y) {
                actual_min_y = val;
            }
            if (val > actual_max_y) {
                actual_max_y = val;
            }
        }
    }

    if (actual_min_y == actual_max_y) {
        actual_max_y += 1.0f;
    }

    float scale_y = (actual_max_y - actual_min_y) / (float)graph_height;
    if (scale_y == 0.0f) {
        scale_y = 1.0f;
    }

    /* Plot data points, newest on the right */

    uint16_t num_points = (buffer->count < graph_width) ? buffer->count : graph_width;

    for (uint16_t i = 0u; i < num_points; i++) {
        uint16_t index =
            (uint16_t)((buffer->head + buffer->size - 1u - i) % buffer->size);
        float value = buffer->data[index];

        uint8_t x = (uint8_t)(graph_x + graph_width - 1u - i);
        uint8_t y = (uint8_t)(graph_y + graph_height - 1u -
                              (uint8_t)((value - actual_min_y) / scale_y));

        for (uint8_t dx = 0u; dx < dot_size; dx++) {
            for (uint8_t dy = 0u; dy < dot_size; dy++) {
                if ((x + dx) < SSD1306_WIDTH && (y + dy) < SSD1306_HEIGHT) {
                    ssd1306_DrawPixel(x + dx, y + dy, White);
                }
            }
        }
    }

    /* Legend with blinking */

    if (legend_text != NULL) {
        /* If blinking is enabled and phase is OFF, skip drawing the label. */
        uint8_t draw_legend = 1u;
        if (blink_label != 0u && s_blink_phase == 0u) {
            draw_legend = 0u;
        }

        if (draw_legend) {
            uint8_t legend_x = (uint8_t)(graph_x + 5u);
            uint8_t legend_y = graph_y;

            uint8_t font_width  = 6u;
            uint8_t font_height = 8u;

            uint8_t text_length = (uint8_t)strlen(legend_text);
            uint8_t bg_width    = (uint8_t)(text_length * font_width + 4u);
            uint8_t bg_height   = (uint8_t)(font_height + 4u);

            uint8_t max_bg_width  = (uint8_t)(graph_width - 4u);
            uint8_t max_bg_height = (uint8_t)(graph_height - 4u);

            if (bg_width > max_bg_width)   bg_width  = max_bg_width;
            if (bg_height > max_bg_height) bg_height = max_bg_height;

            uint8_t max_legend_x = (uint8_t)(graph_x + graph_width  - bg_width);
            uint8_t max_legend_y = (uint8_t)(graph_y + graph_height - bg_height);

            if (legend_x > max_legend_x) legend_x = max_legend_x;
            if (legend_y > max_legend_y) legend_y = max_legend_y;

            if ((legend_x + 2u) < SSD1306_WIDTH && (legend_y + 2u) < SSD1306_HEIGHT) {
                ssd1306_SetCursor(legend_x + 2u, legend_y + 2u);
                ssd1306_WriteString((char*)legend_text, Font_6x8, Black);
            }

            if ((legend_x + bg_width) <= SSD1306_WIDTH &&
                (legend_y + bg_height) <= SSD1306_HEIGHT) {
                ssd1306_DrawRectangle(legend_x,
                                      legend_y,
                                      (uint16_t)(legend_x + bg_width  - 1u),
                                      (uint16_t)(legend_y + bg_height - 1u),
                                      White);
            }
        }
    }
}

/* Simple wrapper: original API, no blinking, no hiding */
void graph_plot(ring_buffer_t* buffer, uint8_t graph_x, uint8_t graph_y,
                uint8_t graph_width, uint8_t graph_height,
                uint8_t dot_size, float min_y, float max_y,
                const char* legend_text)
{
    graph_blink_tick(); /* treat a single graph as a frame for blinking */
    graph_plot_ex(buffer,
                  graph_x, graph_y,
                  graph_width, graph_height,
                  dot_size, min_y, max_y,
                  legend_text,
                  0u, /* no blinking */
                  0u  /* not hidden */
                  );
}

/* ---------- Two-buffer overlay graph (debug/helper) ---------- */

void graph_plot_two_buffers(ring_buffer_t* buffer1, ring_buffer_t* buffer2,
                            uint8_t graph_x, uint8_t graph_y,
                            uint8_t graph_width, uint8_t graph_height,
                            uint8_t dot_size, float min_y1, float max_y1,
                            float min_y2, float max_y2)
{
    if (!buffer1 || !buffer2 || graph_width == 0u || graph_height == 0u || dot_size == 0u) {
        return;
    }

    if (buffer1->count == 0u && buffer2->count == 0u) {
        return;
    }

    /* For simplicity, we just draw both in white, like original code. */

    /* First buffer scaling */
    if (buffer1->count > 0u) {
        float actual_min_y1;
        float actual_max_y1;

        if (min_y1 != 0.0f || max_y1 != 0.0f) {
            actual_min_y1 = min_y1;
            actual_max_y1 = max_y1;
        } else {
            actual_min_y1 = buffer1->data[0];
            actual_max_y1 = buffer1->data[0];

            for (uint16_t i = 0u; i < buffer1->count; i++) {
                float val = ring_buffer_get(buffer1, i);
                if (val < actual_min_y1) {
                    actual_min_y1 = val;
                }
                if (val > actual_max_y1) {
                    actual_max_y1 = val;
                }
            }
        }

        if (actual_min_y1 == actual_max_y1) {
            actual_max_y1 += 1.0f;
        }

        float scale_y1 = (actual_max_y1 - actual_min_y1) / (float)graph_height;
        if (scale_y1 == 0.0f) {
            scale_y1 = 1.0f;
        }

        uint16_t num_points1 = (buffer1->count < graph_width) ? buffer1->count : graph_width;

        for (uint16_t i = 0u; i < num_points1; i++) {
            uint16_t index =
                (uint16_t)((buffer1->head + buffer1->size - 1u - i) % buffer1->size);
            float value = buffer1->data[index];

            uint8_t x = (uint8_t)(graph_x + graph_width - 1u - i);
            uint8_t y = (uint8_t)(graph_y + graph_height - 1u -
                                  (uint8_t)((value - actual_min_y1) / scale_y1));

            for (uint8_t dx = 0u; dx < dot_size; dx++) {
                for (uint8_t dy = 0u; dy < dot_size; dy++) {
                    if ((x + dx) < SSD1306_WIDTH && (y + dy) < SSD1306_HEIGHT) {
                        ssd1306_DrawPixel(x + dx, y + dy, White);
                    }
                }
            }
        }
    }

    /* Second buffer scaling */
    if (buffer2->count > 0u) {
        float actual_min_y2;
        float actual_max_y2;

        if (min_y2 != 0.0f || max_y2 != 0.0f) {
            actual_min_y2 = min_y2;
            actual_max_y2 = max_y2;
        } else {
            actual_min_y2 = buffer2->data[0];
            actual_max_y2 = buffer2->data[0];

            for (uint16_t i = 0u; i < buffer2->count; i++) {
                float val = ring_buffer_get(buffer2, i);
                if (val < actual_min_y2) {
                    actual_min_y2 = val;
                }
                if (val > actual_max_y2) {
                    actual_max_y2 = val;
                }
            }
        }

        if (actual_min_y2 == actual_max_y2) {
            actual_max_y2 += 1.0f;
        }

        float scale_y2 = (actual_max_y2 - actual_min_y2) / (float)graph_height;
        if (scale_y2 == 0.0f) {
            scale_y2 = 1.0f;
        }

        uint16_t num_points2 = (buffer2->count < graph_width) ? buffer2->count : graph_width;

        for (uint16_t i = 0u; i < num_points2; i++) {
            uint16_t index =
                (uint16_t)((buffer2->head + buffer2->size - 1u - i) % buffer2->size);
            float value = buffer2->data[index];

            uint8_t x = (uint8_t)(graph_x + graph_width - 1u - i);
            uint8_t y = (uint8_t)(graph_y + graph_height - 1u -
                                  (uint8_t)((value - actual_min_y2) / scale_y2));

            for (uint8_t dx = 0u; dx < dot_size; dx++) {
                for (uint8_t dy = 0u; dy < dot_size; dy++) {
                    if ((x + dx) < SSD1306_WIDTH && (y + dy) < SSD1306_HEIGHT) {
                        ssd1306_DrawPixel(x + dx, y + dy, White);
                    }
                }
            }
        }
    }
}

/* ---------- Layout helpers (positions only) ---------- */

void calculate_positions_1t1b(uint8_t total_width, uint8_t total_height, uint8_t margin,
                              uint8_t* top_x, uint8_t* top_y, uint8_t* top_width, uint8_t* top_height,
                              uint8_t* bottom_x, uint8_t* bottom_y, uint8_t* bottom_width, uint8_t* bottom_height)
{
    *top_x      = 0u;
    *top_y      = 0u;
    *top_width  = total_width;
    *top_height = (uint8_t)((total_height - margin) / 2u);

    *bottom_x      = 0u;
    *bottom_y      = (uint8_t)(*top_height + margin);
    *bottom_width  = total_width;
    *bottom_height = (uint8_t)((total_height - margin) / 2u);
}

void calculate_positions_1l1r(uint8_t total_width, uint8_t total_height, uint8_t margin,
                              uint8_t* left_x, uint8_t* left_y, uint8_t* left_width, uint8_t* left_height,
                              uint8_t* right_x, uint8_t* right_y, uint8_t* right_width, uint8_t* right_height)
{
    *left_x      = 0u;
    *left_y      = 0u;
    *left_width  = (uint8_t)((total_width - margin) / 2u);
    *left_height = total_height;

    *right_x      = (uint8_t)(*left_width + margin);
    *right_y      = 0u;
    *right_width  = (uint8_t)((total_width - margin) / 2u);
    *right_height = total_height;
}

void calculate_positions_1t2b(uint8_t total_width, uint8_t total_height, uint8_t margin,
                              uint8_t* top_x, uint8_t* top_y, uint8_t* top_width, uint8_t* top_height,
                              uint8_t* bottom1_x, uint8_t* bottom1_y, uint8_t* bottom1_width, uint8_t* bottom1_height,
                              uint8_t* bottom2_x, uint8_t* bottom2_y, uint8_t* bottom2_width, uint8_t* bottom2_height)
{
    *top_x      = 0u;
    *top_y      = 0u;
    *top_width  = total_width;
    *top_height = (uint8_t)((total_height - margin) / 2u);

    *bottom1_x      = 0u;
    *bottom1_y      = (uint8_t)(*top_height + margin);
    *bottom1_width  = (uint8_t)((total_width - margin) / 2u);
    *bottom1_height = (uint8_t)((total_height - margin) / 2u);

    *bottom2_x      = (uint8_t)(*bottom1_width + margin);
    *bottom2_y      = (uint8_t)(*top_height + margin);
    *bottom2_width  = (uint8_t)((total_width - margin) / 2u);
    *bottom2_height = (uint8_t)((total_height - margin) / 2u);
}

void calculate_positions_2t1b(uint8_t total_width, uint8_t total_height, uint8_t margin,
                              uint8_t* top1_x, uint8_t* top1_y, uint8_t* top1_width, uint8_t* top1_height,
                              uint8_t* top2_x, uint8_t* top2_y, uint8_t* top2_width, uint8_t* top2_height,
                              uint8_t* bottom_x, uint8_t* bottom_y, uint8_t* bottom_width, uint8_t* bottom_height)
{
    *top1_x      = 0u;
    *top1_y      = 0u;
    *top1_width  = (uint8_t)((total_width - margin) / 2u);
    *top1_height = (uint8_t)((total_height - margin) / 2u);

    *top2_x      = (uint8_t)(*top1_width + margin);
    *top2_y      = 0u;
    *top2_width  = (uint8_t)((total_width - margin) / 2u);
    *top2_height = (uint8_t)((total_height - margin) / 2u);

    *bottom_x      = 0u;
    *bottom_y      = (uint8_t)(*top1_height + margin);
    *bottom_width  = total_width;
    *bottom_height = (uint8_t)((total_height - margin) / 2u);
}

void calculate_positions_1l2r(uint8_t total_width, uint8_t total_height, uint8_t margin,
                              uint8_t* left_x, uint8_t* left_y, uint8_t* left_width, uint8_t* left_height,
                              uint8_t* right1_x, uint8_t* right1_y, uint8_t* right1_width, uint8_t* right1_height,
                              uint8_t* right2_x, uint8_t* right2_y, uint8_t* right2_width, uint8_t* right2_height)
{
    *left_x      = 0u;
    *left_y      = 0u;
    *left_width  = (uint8_t)((total_width - margin) / 2u);
    *left_height = total_height;

    *right1_x      = (uint8_t)(*left_width + margin);
    *right1_y      = 0u;
    *right1_width  = (uint8_t)((total_width - margin) / 2u);
    *right1_height = (uint8_t)((total_height - margin) / 2u);

    *right2_x      = (uint8_t)(*left_width + margin);
    *right2_y      = (uint8_t)(*right1_height + margin);
    *right2_width  = (uint8_t)((total_width - margin) / 2u);
    *right2_height = (uint8_t)((total_height - margin) / 2u);
}

void calculate_positions_2l1r(uint8_t total_width, uint8_t total_height, uint8_t margin,
                              uint8_t* left1_x, uint8_t* left1_y, uint8_t* left1_width, uint8_t* left1_height,
                              uint8_t* left2_x, uint8_t* left2_y, uint8_t* left2_width, uint8_t* left2_height,
                              uint8_t* right_x, uint8_t* right_y, uint8_t* right_width, uint8_t* right_height)
{
    *left1_x      = 0u;
    *left1_y      = 0u;
    *left1_width  = (uint8_t)((total_width - margin) / 2u);
    *left1_height = (uint8_t)((total_height - margin) / 2u);

    *left2_x      = 0u;
    *left2_y      = (uint8_t)(*left1_height + margin);
    *left2_width  = (uint8_t)((total_width - margin) / 2u);
    *left2_height = (uint8_t)((total_height - margin) / 2u);

    *right_x      = (uint8_t)(*left1_width + margin);
    *right_y      = 0u;
    *right_width  = (uint8_t)((total_width - margin) / 2u);
    *right_height = total_height;
}

void calculate_positions_2t2b(uint8_t total_width, uint8_t total_height, uint8_t margin,
                              uint8_t* top1_x, uint8_t* top1_y, uint8_t* top1_width, uint8_t* top1_height,
                              uint8_t* top2_x, uint8_t* top2_y, uint8_t* top2_width, uint8_t* top2_height,
                              uint8_t* bottom1_x, uint8_t* bottom1_y, uint8_t* bottom1_width, uint8_t* bottom1_height,
                              uint8_t* bottom2_x, uint8_t* bottom2_y, uint8_t* bottom2_width, uint8_t* bottom2_height)
{
    *top1_x      = 0u;
    *top1_y      = 0u;
    *top1_width  = (uint8_t)((total_width - margin) / 2u);
    *top1_height = (uint8_t)((total_height - margin) / 2u);

    *top2_x      = (uint8_t)(*top1_width + margin);
    *top2_y      = 0u;
    *top2_width  = (uint8_t)((total_width - margin) / 2u);
    *top2_height = (uint8_t)((total_height - margin) / 2u);

    *bottom1_x      = 0u;
    *bottom1_y      = (uint8_t)(*top1_height + margin);
    *bottom1_width  = (uint8_t)((total_width - margin) / 2u);
    *bottom1_height = (uint8_t)((total_height - margin) / 2u);

    *bottom2_x      = (uint8_t)(*bottom1_width + margin);
    *bottom2_y      = (uint8_t)(*top1_height + margin);
    *bottom2_width  = (uint8_t)((total_width - margin) / 2u);
    *bottom2_height = (uint8_t)((total_height - margin) / 2u);
}

/* ---------- Extended multi-graph helpers (Option 1 aware) ---------- */

static uint8_t normalize_blink_index(uint8_t blinking_index, uint8_t num_graphs)
{
    if (blinking_index == GRAPH_BLINK_NONE) {
        return GRAPH_BLINK_NONE;
    }
    if (blinking_index >= num_graphs) {
        return GRAPH_BLINK_NONE;
    }
    return blinking_index;
}

/* 2 graphs: top (0), bottom (1) */
void graph_plots_1t1b_ex(ring_buffer_t* buffer1, ring_buffer_t* buffer2,
                         uint8_t graph_x, uint8_t graph_y,
                         uint8_t graph_width, uint8_t graph_height,
                         uint8_t margin,
                         const char* legend_text1, const char* legend_text2,
                         uint8_t blinking_index,
                         uint8_t hidden_mask)
{
    graph_blink_tick();

    uint8_t top_x, top_y, top_w, top_h;
    uint8_t bot_x, bot_y, bot_w, bot_h;

    calculate_positions_1t1b(graph_width, graph_height, margin,
                             &top_x, &top_y, &top_w, &top_h,
                             &bot_x, &bot_y, &bot_w, &bot_h);

    uint8_t blink_idx = normalize_blink_index(blinking_index, 2u);

    uint8_t hide0 = (hidden_mask & GRAPH_HIDE_0) ? 1u : 0u;
    uint8_t hide1 = (hidden_mask & GRAPH_HIDE_1) ? 1u : 0u;

    /* Option 1: NULL buffer also means hidden */
    if (buffer1 == NULL) hide0 = 1u;
    if (buffer2 == NULL) hide1 = 1u;

    uint8_t blink0 = (blink_idx == 0u) ? 1u : 0u;
    uint8_t blink1 = (blink_idx == 1u) ? 1u : 0u;

    graph_plot_ex(buffer1,
                  (uint8_t)(graph_x + top_x), (uint8_t)(graph_y + top_y),
                  top_w, top_h,
                  1u, 0.0f, 100.0f, legend_text1,
                  blink0, hide0);

    graph_plot_ex(buffer2,
                  (uint8_t)(graph_x + bot_x), (uint8_t)(graph_y + bot_y),
                  bot_w, bot_h,
                  1u, 0.0f, 100.0f, legend_text2,
                  blink1, hide1);
}

/* 2 graphs: left (0), right (1) */
void graph_plots_1l1r_ex(ring_buffer_t* buffer1, ring_buffer_t* buffer2,
                         uint8_t graph_x, uint8_t graph_y,
                         uint8_t graph_width, uint8_t graph_height,
                         uint8_t margin,
                         const char* legend_text1, const char* legend_text2,
                         uint8_t blinking_index,
                         uint8_t hidden_mask)
{
    graph_blink_tick();

    uint8_t left_x, left_y, left_w, left_h;
    uint8_t right_x, right_y, right_w, right_h;

    calculate_positions_1l1r(graph_width, graph_height, margin,
                             &left_x, &left_y, &left_w, &left_h,
                             &right_x, &right_y, &right_w, &right_h);

    uint8_t blink_idx = normalize_blink_index(blinking_index, 2u);

    uint8_t hide0 = (hidden_mask & GRAPH_HIDE_0) ? 1u : 0u;
    uint8_t hide1 = (hidden_mask & GRAPH_HIDE_1) ? 1u : 0u;

    if (buffer1 == NULL) hide0 = 1u;
    if (buffer2 == NULL) hide1 = 1u;

    uint8_t blink0 = (blink_idx == 0u) ? 1u : 0u;
    uint8_t blink1 = (blink_idx == 1u) ? 1u : 0u;

    graph_plot_ex(buffer1,
                  (uint8_t)(graph_x + left_x), (uint8_t)(graph_y + left_y),
                  left_w, left_h,
                  1u, 0.0f, 100.0f, legend_text1,
                  blink0, hide0);

    graph_plot_ex(buffer2,
                  (uint8_t)(graph_x + right_x), (uint8_t)(graph_y + right_y),
                  right_w, right_h,
                  1u, 0.0f, 100.0f, legend_text2,
                  blink1, hide1);
}

/* 3 graphs: top (0), bottom-left (1), bottom-right (2) */
void graph_plots_1t2b_ex(ring_buffer_t* buffer1, ring_buffer_t* buffer2, ring_buffer_t* buffer3,
                         uint8_t graph_x, uint8_t graph_y,
                         uint8_t graph_width, uint8_t graph_height,
                         uint8_t margin,
                         const char* legend_text1, const char* legend_text2, const char* legend_text3,
                         uint8_t blinking_index,
                         uint8_t hidden_mask)
{
    graph_blink_tick();

    uint8_t top_x, top_y, top_w, top_h;
    uint8_t b1_x, b1_y, b1_w, b1_h;
    uint8_t b2_x, b2_y, b2_w, b2_h;

    calculate_positions_1t2b(graph_width, graph_height, margin,
                             &top_x, &top_y, &top_w, &top_h,
                             &b1_x, &b1_y, &b1_w, &b1_h,
                             &b2_x, &b2_y, &b2_w, &b2_h);

    uint8_t blink_idx = normalize_blink_index(blinking_index, 3u);

    uint8_t hide0 = (hidden_mask & GRAPH_HIDE_0) ? 1u : 0u;
    uint8_t hide1 = (hidden_mask & GRAPH_HIDE_1) ? 1u : 0u;
    uint8_t hide2 = (hidden_mask & GRAPH_HIDE_2) ? 1u : 0u;

    if (buffer1 == NULL) hide0 = 1u;
    if (buffer2 == NULL) hide1 = 1u;
    if (buffer3 == NULL) hide2 = 1u;

    uint8_t blink0 = (blink_idx == 0u) ? 1u : 0u;
    uint8_t blink1 = (blink_idx == 1u) ? 1u : 0u;
    uint8_t blink2 = (blink_idx == 2u) ? 1u : 0u;

    graph_plot_ex(buffer1,
                  (uint8_t)(graph_x + top_x), (uint8_t)(graph_y + top_y),
                  top_w, top_h,
                  1u, 0.0f, 100.0f, legend_text1,
                  blink0, hide0);

    graph_plot_ex(buffer2,
                  (uint8_t)(graph_x + b1_x), (uint8_t)(graph_y + b1_y),
                  b1_w, b1_h,
                  1u, 0.0f, 100.0f, legend_text2,
                  blink1, hide1);

    graph_plot_ex(buffer3,
                  (uint8_t)(graph_x + b2_x), (uint8_t)(graph_y + b2_y),
                  b2_w, b2_h,
                  1u, 0.0f, 100.0f, legend_text3,
                  blink2, hide2);
}

/* 3 graphs: top-left (0), top-right (1), bottom (2) */
void graph_plots_2t1b_ex(ring_buffer_t* buffer1, ring_buffer_t* buffer2, ring_buffer_t* buffer3,
                         uint8_t graph_x, uint8_t graph_y,
                         uint8_t graph_width, uint8_t graph_height,
                         uint8_t margin,
                         const char* legend_text1, const char* legend_text2, const char* legend_text3,
                         uint8_t blinking_index,
                         uint8_t hidden_mask)
{
    graph_blink_tick();

    uint8_t t1_x, t1_y, t1_w, t1_h;
    uint8_t t2_x, t2_y, t2_w, t2_h;
    uint8_t b_x,  b_y,  b_w,  b_h;

    calculate_positions_2t1b(graph_width, graph_height, margin,
                             &t1_x, &t1_y, &t1_w, &t1_h,
                             &t2_x, &t2_y, &t2_w, &t2_h,
                             &b_x,  &b_y,  &b_w,  &b_h);

    uint8_t blink_idx = normalize_blink_index(blinking_index, 3u);

    uint8_t hide0 = (hidden_mask & GRAPH_HIDE_0) ? 1u : 0u;
    uint8_t hide1 = (hidden_mask & GRAPH_HIDE_1) ? 1u : 0u;
    uint8_t hide2 = (hidden_mask & GRAPH_HIDE_2) ? 1u : 0u;

    if (buffer1 == NULL) hide0 = 1u;
    if (buffer2 == NULL) hide1 = 1u;
    if (buffer3 == NULL) hide2 = 1u;

    uint8_t blink0 = (blink_idx == 0u) ? 1u : 0u;
    uint8_t blink1 = (blink_idx == 1u) ? 1u : 0u;
    uint8_t blink2 = (blink_idx == 2u) ? 1u : 0u;

    graph_plot_ex(buffer1,
                  (uint8_t)(graph_x + t1_x), (uint8_t)(graph_y + t1_y),
                  t1_w, t1_h,
                  1u, 0.0f, 100.0f, legend_text1,
                  blink0, hide0);

    graph_plot_ex(buffer2,
                  (uint8_t)(graph_x + t2_x), (uint8_t)(graph_y + t2_y),
                  t2_w, t2_h,
                  1u, 0.0f, 100.0f, legend_text2,
                  blink1, hide1);

    graph_plot_ex(buffer3,
                  (uint8_t)(graph_x + b_x), (uint8_t)(graph_y + b_y),
                  b_w, b_h,
                  1u, 0.0f, 100.0f, legend_text3,
                  blink2, hide2);
}

/* 3 graphs: left (0), right-top (1), right-bottom (2) */
void graph_plots_1l2r_ex(ring_buffer_t* buffer1, ring_buffer_t* buffer2, ring_buffer_t* buffer3,
                         uint8_t graph_x, uint8_t graph_y,
                         uint8_t graph_width, uint8_t graph_height,
                         uint8_t margin,
                         const char* legend_text1, const char* legend_text2, const char* legend_text3,
                         uint8_t blinking_index,
                         uint8_t hidden_mask)
{
    graph_blink_tick();

    uint8_t l_x, l_y, l_w, l_h;
    uint8_t r1_x, r1_y, r1_w, r1_h;
    uint8_t r2_x, r2_y, r2_w, r2_h;

    calculate_positions_1l2r(graph_width, graph_height, margin,
                             &l_x,  &l_y,  &l_w,  &l_h,
                             &r1_x, &r1_y, &r1_w, &r1_h,
                             &r2_x, &r2_y, &r2_w, &r2_h);

    uint8_t blink_idx = normalize_blink_index(blinking_index, 3u);

    uint8_t hide0 = (hidden_mask & GRAPH_HIDE_0) ? 1u : 0u;
    uint8_t hide1 = (hidden_mask & GRAPH_HIDE_1) ? 1u : 0u;
    uint8_t hide2 = (hidden_mask & GRAPH_HIDE_2) ? 1u : 0u;

    if (buffer1 == NULL) hide0 = 1u;
    if (buffer2 == NULL) hide1 = 1u;
    if (buffer3 == NULL) hide2 = 1u;

    uint8_t blink0 = (blink_idx == 0u) ? 1u : 0u;
    uint8_t blink1 = (blink_idx == 1u) ? 1u : 0u;
    uint8_t blink2 = (blink_idx == 2u) ? 1u : 0u;

    graph_plot_ex(buffer1,
                  (uint8_t)(graph_x + l_x), (uint8_t)(graph_y + l_y),
                  l_w, l_h,
                  1u, 0.0f, 100.0f, legend_text1,
                  blink0, hide0);

    graph_plot_ex(buffer2,
                  (uint8_t)(graph_x + r1_x), (uint8_t)(graph_y + r1_y),
                  r1_w, r1_h,
                  1u, 0.0f, 100.0f, legend_text2,
                  blink1, hide1);

    graph_plot_ex(buffer3,
                  (uint8_t)(graph_x + r2_x), (uint8_t)(graph_y + r2_y),
                  r2_w, r2_h,
                  1u, 0.0f, 100.0f, legend_text3,
                  blink2, hide2);
}

/* 3 graphs: left-top (0), left-bottom (1), right (2) */
void graph_plots_2l1r_ex(ring_buffer_t* buffer1, ring_buffer_t* buffer2, ring_buffer_t* buffer3,
                         uint8_t graph_x, uint8_t graph_y,
                         uint8_t graph_width, uint8_t graph_height,
                         uint8_t margin,
                         const char* legend_text1, const char* legend_text2, const char* legend_text3,
                         uint8_t blinking_index,
                         uint8_t hidden_mask)
{
    graph_blink_tick();

    uint8_t l1_x, l1_y, l1_w, l1_h;
    uint8_t l2_x, l2_y, l2_w, l2_h;
    uint8_t r_x,  r_y,  r_w,  r_h;

    calculate_positions_2l1r(graph_width, graph_height, margin,
                             &l1_x, &l1_y, &l1_w, &l1_h,
                             &l2_x, &l2_y, &l2_w, &l2_h,
                             &r_x,  &r_y,  &r_w,  &r_h);

    uint8_t blink_idx = normalize_blink_index(blinking_index, 3u);

    uint8_t hide0 = (hidden_mask & GRAPH_HIDE_0) ? 1u : 0u;
    uint8_t hide1 = (hidden_mask & GRAPH_HIDE_1) ? 1u : 0u;
    uint8_t hide2 = (hidden_mask & GRAPH_HIDE_2) ? 1u : 0u;

    if (buffer1 == NULL) hide0 = 1u;
    if (buffer2 == NULL) hide1 = 1u;
    if (buffer3 == NULL) hide2 = 1u;

    uint8_t blink0 = (blink_idx == 0u) ? 1u : 0u;
    uint8_t blink1 = (blink_idx == 1u) ? 1u : 0u;
    uint8_t blink2 = (blink_idx == 2u) ? 1u : 0u;

    graph_plot_ex(buffer1,
                  (uint8_t)(graph_x + l1_x), (uint8_t)(graph_y + l1_y),
                  l1_w, l1_h,
                  1u, 0.0f, 100.0f, legend_text1,
                  blink0, hide0);

    graph_plot_ex(buffer2,
                  (uint8_t)(graph_x + l2_x), (uint8_t)(graph_y + l2_y),
                  l2_w, l2_h,
                  1u, 0.0f, 100.0f, legend_text2,
                  blink1, hide1);

    graph_plot_ex(buffer3,
                  (uint8_t)(graph_x + r_x), (uint8_t)(graph_y + r_y),
                  r_w, r_h,
                  1u, 0.0f, 100.0f, legend_text3,
                  blink2, hide2);
}

/* 4 graphs: top-left (0), top-right (1), bottom-left (2), bottom-right (3) */
void graph_plots_2t2b_ex(ring_buffer_t* buffer1, ring_buffer_t* buffer2,
                         ring_buffer_t* buffer3, ring_buffer_t* buffer4,
                         uint8_t graph_x, uint8_t graph_y,
                         uint8_t graph_width, uint8_t graph_height,
                         uint8_t margin,
                         const char* legend_text1, const char* legend_text2,
                         const char* legend_text3, const char* legend_text4,
                         uint8_t blinking_index,
                         uint8_t hidden_mask)
{
    graph_blink_tick();

    uint8_t t1_x, t1_y, t1_w, t1_h;
    uint8_t t2_x, t2_y, t2_w, t2_h;
    uint8_t b1_x, b1_y, b1_w, b1_h;
    uint8_t b2_x, b2_y, b2_w, b2_h;

    calculate_positions_2t2b(graph_width, graph_height, margin,
                             &t1_x, &t1_y, &t1_w, &t1_h,
                             &t2_x, &t2_y, &t2_w, &t2_h,
                             &b1_x, &b1_y, &b1_w, &b1_h,
                             &b2_x, &b2_y, &b2_w, &b2_h);

    uint8_t blink_idx = normalize_blink_index(blinking_index, 4u);

    uint8_t hide0 = (hidden_mask & GRAPH_HIDE_0) ? 1u : 0u;
    uint8_t hide1 = (hidden_mask & GRAPH_HIDE_1) ? 1u : 0u;
    uint8_t hide2 = (hidden_mask & GRAPH_HIDE_2) ? 1u : 0u;
    uint8_t hide3 = (hidden_mask & GRAPH_HIDE_3) ? 1u : 0u;

    if (buffer1 == NULL) hide0 = 1u;
    if (buffer2 == NULL) hide1 = 1u;
    if (buffer3 == NULL) hide2 = 1u;
    if (buffer4 == NULL) hide3 = 1u;

    uint8_t blink0 = (blink_idx == 0u) ? 1u : 0u;
    uint8_t blink1 = (blink_idx == 1u) ? 1u : 0u;
    uint8_t blink2 = (blink_idx == 2u) ? 1u : 0u;
    uint8_t blink3 = (blink_idx == 3u) ? 1u : 0u;

    graph_plot_ex(buffer1,
                  (uint8_t)(graph_x + t1_x), (uint8_t)(graph_y + t1_y),
                  t1_w, t1_h,
                  1u, 0.0f, 100.0f, legend_text1,
                  blink0, hide0);

    graph_plot_ex(buffer2,
                  (uint8_t)(graph_x + t2_x), (uint8_t)(graph_y + t2_y),
                  t2_w, t2_h,
                  1u, 0.0f, 100.0f, legend_text2,
                  blink1, hide1);

    graph_plot_ex(buffer3,
                  (uint8_t)(graph_x + b1_x), (uint8_t)(graph_y + b1_y),
                  b1_w, b1_h,
                  1u, 0.0f, 100.0f, legend_text3,
                  blink2, hide2);

    graph_plot_ex(buffer4,
                  (uint8_t)(graph_x + b2_x), (uint8_t)(graph_y + b2_y),
                  b2_w, b2_h,
                  1u, 0.0f, 100.0f, legend_text4,
                  blink3, hide3);
}

/* ---------- Non-extended wrappers (backwards compatible) ---------- */

void graph_plots_1t1b(ring_buffer_t* buffer1, ring_buffer_t* buffer2,
                      uint8_t graph_x, uint8_t graph_y, uint8_t graph_width, uint8_t graph_height,
                      uint8_t margin, const char* legend_text1, const char* legend_text2)
{
    graph_plots_1t1b_ex(buffer1, buffer2,
                        graph_x, graph_y, graph_width, graph_height,
                        margin,
                        legend_text1, legend_text2,
                        GRAPH_BLINK_NONE,
                        0u);
}

void graph_plots_1l1r(ring_buffer_t* buffer1, ring_buffer_t* buffer2,
                      uint8_t graph_x, uint8_t graph_y, uint8_t graph_width, uint8_t graph_height,
                      uint8_t margin, const char* legend_text1, const char* legend_text2)
{
    graph_plots_1l1r_ex(buffer1, buffer2,
                        graph_x, graph_y, graph_width, graph_height,
                        margin,
                        legend_text1, legend_text2,
                        GRAPH_BLINK_NONE,
                        0u);
}

void graph_plots_1t2b(ring_buffer_t* buffer1, ring_buffer_t* buffer2, ring_buffer_t* buffer3,
                      uint8_t graph_x, uint8_t graph_y, uint8_t graph_width, uint8_t graph_height,
                      uint8_t margin, const char* legend_text1, const char* legend_text2, const char* legend_text3)
{
    graph_plots_1t2b_ex(buffer1, buffer2, buffer3,
                        graph_x, graph_y, graph_width, graph_height,
                        margin,
                        legend_text1, legend_text2, legend_text3,
                        GRAPH_BLINK_NONE,
                        0u);
}

void graph_plots_2t1b(ring_buffer_t* buffer1, ring_buffer_t* buffer2, ring_buffer_t* buffer3,
                      uint8_t graph_x, uint8_t graph_y, uint8_t graph_width, uint8_t graph_height,
                      uint8_t margin, const char* legend_text1, const char* legend_text2, const char* legend_text3)
{
    graph_plots_2t1b_ex(buffer1, buffer2, buffer3,
                        graph_x, graph_y, graph_width, graph_height,
                        margin,
                        legend_text1, legend_text2, legend_text3,
                        GRAPH_BLINK_NONE,
                        0u);
}

void graph_plots_1l2r(ring_buffer_t* buffer1, ring_buffer_t* buffer2, ring_buffer_t* buffer3,
                      uint8_t graph_x, uint8_t graph_y, uint8_t graph_width, uint8_t graph_height,
                      uint8_t margin, const char* legend_text1, const char* legend_text2, const char* legend_text3)
{
    graph_plots_1l2r_ex(buffer1, buffer2, buffer3,
                        graph_x, graph_y, graph_width, graph_height,
                        margin,
                        legend_text1, legend_text2, legend_text3,
                        GRAPH_BLINK_NONE,
                        0u);
}

void graph_plots_2l1r(ring_buffer_t* buffer1, ring_buffer_t* buffer2, ring_buffer_t* buffer3,
                      uint8_t graph_x, uint8_t graph_y, uint8_t graph_width, uint8_t graph_height,
                      uint8_t margin, const char* legend_text1, const char* legend_text2, const char* legend_text3)
{
    graph_plots_2l1r_ex(buffer1, buffer2, buffer3,
                        graph_x, graph_y, graph_width, graph_height,
                        margin,
                        legend_text1, legend_text2, legend_text3,
                        GRAPH_BLINK_NONE,
                        0u);
}

void graph_plots_2t2b(ring_buffer_t* buffer1, ring_buffer_t* buffer2,
                      ring_buffer_t* buffer3, ring_buffer_t* buffer4,
                      uint8_t graph_x, uint8_t graph_y, uint8_t graph_width, uint8_t graph_height,
                      uint8_t margin, const char* legend_text1, const char* legend_text2,
                      const char* legend_text3, const char* legend_text4)
{
    graph_plots_2t2b_ex(buffer1, buffer2, buffer3, buffer4,
                        graph_x, graph_y, graph_width, graph_height,
                        margin,
                        legend_text1, legend_text2, legend_text3, legend_text4,
                        GRAPH_BLINK_NONE,
                        0u);
}

/* ---------- Debug helpers ---------- */

void debug_screen_edges(void)
{
    static uint8_t blink_phase = 0u;
    static uint8_t phase_frame_count = 0u;

    if (phase_frame_count >= 10u) {
        blink_phase = (uint8_t)((blink_phase + 1u) % 4u);
        phase_frame_count = 0u;
    }

    switch (blink_phase) {
        case 0u:
            /* Black screen */
            break;

        case 1u:
            /* 4 corner dots */
            ssd1306_DrawPixel(0, 0, White);
            ssd1306_DrawPixel(SSD1306_WIDTH - 1u, 0, White);
            ssd1306_DrawPixel(0, SSD1306_HEIGHT - 1u, White);
            ssd1306_DrawPixel(SSD1306_WIDTH - 1u, SSD1306_HEIGHT - 1u, White);
            break;

        case 2u:
            /* Rectangle + corner dots */
            ssd1306_DrawPixel(0, 0, White);
            ssd1306_DrawPixel(SSD1306_WIDTH - 1u, 0, White);
            ssd1306_DrawPixel(0, SSD1306_HEIGHT - 1u, White);
            ssd1306_DrawPixel(SSD1306_WIDTH - 1u, SSD1306_HEIGHT - 1u, White);

            ssd1306_Line(0, 0, SSD1306_WIDTH - 1u, 0, White);
            ssd1306_Line(0, SSD1306_HEIGHT - 1u, SSD1306_WIDTH - 1u, SSD1306_HEIGHT - 1u, White);
            ssd1306_Line(0, 0, 0, SSD1306_HEIGHT - 1u, White);
            ssd1306_Line(SSD1306_WIDTH - 1u, 0, SSD1306_WIDTH - 1u, SSD1306_HEIGHT - 1u, White);
            break;

        case 3u:
            /* Black again */
            break;
    }

    phase_frame_count++;
}

void debug_fullscreen_graph(void)
{
    for (int i = 0; i < 100; i++) {
        float value = 50.0f + 20.0f * sinf(i * 0.2f);
        ring_buffer_push(&sensor_ring_buffer_1, value);
    }

    ssd1306_Fill(Black);

    /* Classic single-graph plot, no blinking */
    graph_plot(&sensor_ring_buffer_1, 0, 0,
               SSD1306_WIDTH, SSD1306_HEIGHT,
               1u, 0.0f, 100.0f, NULL);

    debug_screen_edges();
    ssd1306_UpdateScreen();
}
