/* driver_sen5x_debug.c */

#include "driver_sen5x.h"
#include "driver_sen5x_interface.h"
#include "driver_sen5x_debug.h"

#include <string.h>
#include <math.h>
#include <stdlib.h>   /* labs */

/* Global handle for debug use */
static sen5x_handle_t g_sen5x_handle;

/* One-shot guard so you can call sen5x_run_full_test_once() inside while(1) */
static uint8_t g_sen5x_full_test_ran = 0;

/* ---------- Helpers ---------- */

/**
 * @brief Print a float as fixed-point using only integer formatting.
 *
 * This avoids %f, so it works even if printf float support is disabled.
 */
static void sen5x_debug_print_fixed(const char *label,
                                    float value,
                                    const char *unit,
                                    uint8_t decimals)
{
    if (unit == NULL) {
        unit = "";
    }

    /* Handle NaN explicitly if FP_NAN is used in the driver */
    if (isnan(value)) {
        sen5x_interface_debug_print("  %s: NaN %s\r\n", label, unit);
        return;
    }

    long scale = 1;
    for (uint8_t i = 0; i < decimals; ++i) {
        scale *= 10;
    }

    float scaled_f = value * (float)scale;
    long scaled;

    /* Round correctly for positive and negative values */
    if (scaled_f >= 0.0f) {
        scaled = (long)(scaled_f + 0.5f);
    } else {
        scaled = (long)(scaled_f - 0.5f);
    }

    long ipart = scaled / scale;
    long frac  = labs(scaled % scale);

    if (decimals == 0) {
        sen5x_interface_debug_print("  %s: %ld %s\r\n",
                                    label, ipart, unit);
    } else if (decimals == 1) {
        sen5x_interface_debug_print("  %s: %ld.%01ld %s\r\n",
                                    label, ipart, frac, unit);
    } else if (decimals == 2) {
        sen5x_interface_debug_print("  %s: %ld.%02ld %s\r\n",
                                    label, ipart, frac, unit);
    } else {
        /* Clamp to 3 decimals for simplicity */
        sen5x_interface_debug_print("  %s: %ld.%03ld %s\r\n",
                                    label, ipart, frac, unit);
    }
}

/* Retry wrapper for raw data – VOC/NOx are often slower to become “ready” */
static uint8_t sen5x_debug_read_raw_with_retry(sen5x_raw_t *raw,
                                               uint8_t max_retries,
                                               uint16_t delay_ms)
{
    uint8_t res = 0;

    for (uint8_t i = 0; i < max_retries; ++i)
    {
        res = sen5x_read_raw_value(&g_sen5x_handle, raw);
        if (res == 0)
        {
            return 0;
        }

        /* The driver itself prints "sen5x: data not ready." */
        sen5x_interface_delay_ms(delay_ms);
    }

    return res;
}

/* ---------- Public API ---------- */

void sen5x_init_handle_default(void)
{
    /* Wire LibDriver-style function pointers */
    DRIVER_SEN5X_LINK_INIT(&g_sen5x_handle, sen5x_handle_t);
    DRIVER_SEN5X_LINK_IIC_INIT(&g_sen5x_handle, sen5x_interface_iic_init);
    DRIVER_SEN5X_LINK_IIC_DEINIT(&g_sen5x_handle, sen5x_interface_iic_deinit);
    DRIVER_SEN5X_LINK_IIC_WRITE_COMMAND(&g_sen5x_handle, sen5x_interface_iic_write_cmd);
    DRIVER_SEN5X_LINK_IIC_READ_COMMAND(&g_sen5x_handle, sen5x_interface_iic_read_cmd);
    DRIVER_SEN5X_LINK_DELAY_MS(&g_sen5x_handle, sen5x_interface_delay_ms);
    DRIVER_SEN5X_LINK_DEBUG_PRINT(&g_sen5x_handle, sen5x_interface_debug_print);

    /* You’re using SEN55 */
    (void)sen5x_set_type(&g_sen5x_handle, SEN55);
}

void sen5x_run_full_test_once(void)
{
    if (g_sen5x_full_test_ran)
    {
        return;
    }
    g_sen5x_full_test_ran = 1;

    sen5x_info_t   info;
    uint8_t        res;
    sen5x_pm_t     pm_data;
    sen5x_raw_t    raw_data;
    char           product_name[32]  = {0};
    char           serial_number[32] = {0};
    uint32_t       device_status     = 0;
    uint8_t        device_version    = 0;
    sen5x_type_t   sensor_type       = 0;
    uint8_t        raw_buf[3]        = {0};

    memset(&pm_data,  0, sizeof(pm_data));
    memset(&raw_data, 0, sizeof(raw_data));

    sen5x_init_handle_default();

    sen5x_interface_debug_print("===== SEN5X FULL SELF-TEST =====\r\n");

    /* init */
    res = sen5x_init(&g_sen5x_handle);
    if (res != 0)
    {
        sen5x_interface_debug_print("sen5x_init: ERROR %d\r\n", res);
        return;
    }
    sen5x_interface_debug_print("sen5x_init: OK\r\n");

    /* static driver info */
    res = sen5x_info(&info);
    if (res == 0)
    {
        uint32_t ver = info.driver_version;
        uint32_t ver_major = ver / 1000U;
        uint32_t ver_minor = (ver / 100U) % 10U;
        uint32_t ver_patch = ver % 100U;

        sen5x_interface_debug_print(
            "Chip: %s, Manufacturer: %s, IF: %s, Driver v%lu.%lu.%lu\r\n",
            info.chip_name,
            info.manufacturer_name,
            info.interface,
            (unsigned long)ver_major,
            (unsigned long)ver_minor,
            (unsigned long)ver_patch
        );
    }
    else
    {
        sen5x_interface_debug_print("sen5x_info: ERROR %d\r\n", res);
    }

    /* Sensor type (SEN50 / 54 / 55 / 60 ...) */
    res = sen5x_get_type(&g_sen5x_handle, &sensor_type);
    if (res == 0)
    {
        sen5x_interface_debug_print("Sensor type: %u\r\n", (unsigned)sensor_type);
    }
    else
    {
        sen5x_interface_debug_print("sen5x_get_type: ERROR %d\r\n", res);
    }

    /* Product name */
    res = sen5x_get_product_name(&g_sen5x_handle, product_name);
    if (res == 0)
    {
        sen5x_interface_debug_print("Product name: %s\r\n", product_name);
    }
    else
    {
        sen5x_interface_debug_print("sen5x_get_product_name: ERROR %d\r\n", res);
    }

    /* Serial number */
    res = sen5x_get_serial_number(&g_sen5x_handle, serial_number);
    if (res == 0)
    {
        sen5x_interface_debug_print("Serial: %s\r\n", serial_number);
    }
    else
    {
        sen5x_interface_debug_print("sen5x_get_serial_number: ERROR %d\r\n", res);
    }

    /* Device version */
    res = sen5x_get_version(&g_sen5x_handle, &device_version);
    if (res == 0)
    {
        sen5x_interface_debug_print("Version: %u\r\n", (unsigned)device_version);
    }
    else
    {
        sen5x_interface_debug_print("sen5x_get_version: ERROR %d\r\n", res);
    }

    /* Device status */
    res = sen5x_get_device_status(&g_sen5x_handle, &device_status);
    if (res == 0)
    {
        sen5x_interface_debug_print("Device Status: 0x%08X\r\n", (unsigned)device_status);
    }
    else
    {
        sen5x_interface_debug_print("sen5x_get_device_status: ERROR %d\r\n", res);
    }

    /* Persist settings (harmless if unchanged) */
    res = sen5x_persist_settings(&g_sen5x_handle);
    sen5x_interface_debug_print("sen5x_persist_settings: %s\r\n",
                                (res == 0) ? "OK" : "ERROR");

    /* Start measurement */
    res = sen5x_start_measurement(&g_sen5x_handle);
    sen5x_interface_debug_print("sen5x_start_measurement: %s\r\n",
                                (res == 0) ? "OK" : "ERROR");

    if (res == 0)
    {
        /* Give the sensor time to produce the first fully-populated sample. */
        sen5x_interface_debug_print("Waiting for first sample...\r\n");
        sen5x_interface_delay_ms(7000);

        /* --- PM block --- */
        res = sen5x_read_pm_value(&g_sen5x_handle, &pm_data);
        if (res == 0)
        {
            sen5x_interface_debug_print("PM RAW (mass conc. [0.1 ug/m3 units]):\r\n");
            sen5x_interface_debug_print(
                "  PM1.0: %u, PM2.5: %u, PM4.0: %u, PM10: %u\r\n",
                pm_data.mass_concentration_pm1p0_raw,
                pm_data.mass_concentration_pm2p5_raw,
                pm_data.mass_concentration_pm4p0_raw,
                pm_data.mass_concentration_pm10_raw
            );
            sen5x_interface_debug_print(
                "PM RAW (number conc. [0.1 #/cm3 units]):\r\n"
                "  PM0.5: %u, PM1.0: %u, PM2.5: %u, PM4.0: %u, PM10: %u\r\n",
                pm_data.number_concentration_pm0p5_raw,
                pm_data.number_concentration_pm1p0_raw,
                pm_data.number_concentration_pm2p5_raw,
                pm_data.number_concentration_pm4p0_raw,
                pm_data.number_concentration_pm10_raw
            );
            sen5x_interface_debug_print(
                "  Typical particle raw: %u\r\n",
                pm_data.typical_particle_raw
            );

            sen5x_interface_debug_print("PM Converted:\r\n");
            /* 1 decimal place for ug/m3 and #/cm3, 3 for size in um */
            sen5x_debug_print_fixed("PM1.0", pm_data.pm1p0_ug_m3,  "ug/m3", 1);
            sen5x_debug_print_fixed("PM2.5", pm_data.pm2p5_ug_m3,  "ug/m3", 1);
            sen5x_debug_print_fixed("PM4.0", pm_data.pm4p0_ug_m3,  "ug/m3", 1);
            sen5x_debug_print_fixed("PM10",  pm_data.pm10_ug_m3,   "ug/m3", 1);

            sen5x_debug_print_fixed("N0.5",  pm_data.pm0p5_cm3,    "#/cm3", 1);
            sen5x_debug_print_fixed("N1.0",  pm_data.pm1p0_cm3,    "#/cm3", 1);
            sen5x_debug_print_fixed("N2.5",  pm_data.pm2p5_cm3,    "#/cm3", 1);
            sen5x_debug_print_fixed("N4.0",  pm_data.pm4p0_cm3,    "#/cm3", 1);
            sen5x_debug_print_fixed("N10",   pm_data.pm10_cm3,     "#/cm3", 1);

            sen5x_debug_print_fixed("D_typ", pm_data.typical_particle_um, "um", 3);

            sen5x_interface_debug_print("  pm_valid flag: %u\r\n",
                                        (unsigned)pm_data.pm_valid);
        }
        else
        {
            sen5x_interface_debug_print("sen5x_read_pm_value: ERROR %d\r\n", res);
        }

        /* --- Raw block (T, RH, VOC, NOx) --- */
        res = sen5x_debug_read_raw_with_retry(&raw_data, 10, 1000);
        if (res == 0)
        {
            sen5x_interface_debug_print("Raw environmental values (raw):\r\n");
            sen5x_interface_debug_print(
                "  Humidity raw     : %d\r\n"
                "  Temperature raw  : %d\r\n"
                "  VOC raw          : %u\r\n"
                "  NOx raw          : %u\r\n",
                (int)raw_data.humidity_raw,
                (int)raw_data.temperature_raw,
                (unsigned)raw_data.voc_raw,
                (unsigned)raw_data.nox_raw
            );

            sen5x_interface_debug_print("Raw environmental values (converted):\r\n");
            sen5x_debug_print_fixed("Humidity",    raw_data.humidity_percentage,   "%",    2);
            sen5x_debug_print_fixed("Temperature", raw_data.temperature_degree,    "degC", 2);
            sen5x_debug_print_fixed("VOC index",   raw_data.voc,                   "",     1);
            sen5x_debug_print_fixed("NOx index",   raw_data.nox,                   "",     1);
        }
        else
        {
            sen5x_interface_debug_print("sen5x_read_raw_value (with retries) failed, res=%d\r\n",
                                        res);
        }

        /* Stop measurement regardless of whether the above failed */
        res = sen5x_stop_measurement(&g_sen5x_handle);
        sen5x_interface_debug_print("sen5x_stop_measurement: %s\r\n",
                                    (res == 0) ? "OK" : "ERROR");
    }

    /* Fan cleaning (non-fatal) */
    res = sen5x_start_fan_cleaning(&g_sen5x_handle);
    sen5x_interface_debug_print("sen5x_start_fan_cleaning: %s\r\n",
                                (res == 0) ? "OK" : "ERROR");

    /* Clear device status */
    res = sen5x_clear_device_status(&g_sen5x_handle);
    sen5x_interface_debug_print("sen5x_clear_device_status: %s\r\n",
                                (res == 0) ? "OK" : "ERROR");

    /* Raw get_reg for some variant/sanity check */
    res = sen5x_get_reg(&g_sen5x_handle, 0x202F, raw_buf, 3);
    sen5x_interface_debug_print(
        "sen5x_get_reg(0x202F): %s (res=%d, bytes=%02X %02X %02X)\r\n",
        (res == 0) ? "OK" : "ERROR",
        res,
        raw_buf[0], raw_buf[1], raw_buf[2]
    );
    /* NOTE: we do NOT write this register. */

    /* deinit */
    res = sen5x_deinit(&g_sen5x_handle);
    sen5x_interface_debug_print("sen5x_deinit: %s\r\n",
                                (res == 0) ? "OK" : "ERROR");

    sen5x_interface_debug_print("===== SEN5X FULL SELF-TEST DONE =====\r\n");
}
