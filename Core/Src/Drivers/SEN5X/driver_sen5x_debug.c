#include "driver_sen5x.h"
#include "driver_sen5x_interface.h"
#include "main.h"

static sen5x_handle_t g_sen5x_handle;

static void sen5x_init_handle_default(void)
{
    DRIVER_SEN5X_LINK_INIT(&g_sen5x_handle, sen5x_handle_t);
    DRIVER_SEN5X_LINK_IIC_INIT(&g_sen5x_handle, sen5x_interface_iic_init);
    DRIVER_SEN5X_LINK_IIC_DEINIT(&g_sen5x_handle, sen5x_interface_iic_deinit);
    DRIVER_SEN5X_LINK_IIC_WRITE_COMMAND(&g_sen5x_handle, sen5x_interface_iic_write_cmd);
    DRIVER_SEN5X_LINK_IIC_READ_COMMAND(&g_sen5x_handle, sen5x_interface_iic_read_cmd);
    DRIVER_SEN5X_LINK_DELAY_MS(&g_sen5x_handle, sen5x_interface_delay_ms);
    DRIVER_SEN5X_LINK_DEBUG_PRINT(&g_sen5x_handle, sen5x_interface_debug_print);

    /* You have SCD41 */
    (void)sen5x_set_type(&g_sen5x_handle, SCD41);
}

void sen5x_run_full_test_once(void)
{
    sen5x_info_t info;
    sen5x_bool_t ready;
    uint8_t res;

    /* Correct types: co2_ppm must be uint16_t, not float */
    uint16_t co2_raw  = 0;
    uint16_t co2_ppm  = 0;
    uint16_t t_raw    = 0;
    uint16_t rh_raw   = 0;
    float    t_degC   = 0.0f;
    float    rh_pct   = 0.0f;

    // uint16_t offset_reg, altitude_reg, pressure_reg;
    uint16_t variant;
    uint16_t serial[3];
    // sen5x_bool_t asc_enable;
    sen5x_bool_t malfunction;

    uint8_t raw_buf[3] = {0};

    sen5x_init_handle_default();

    sen5x_interface_debug_print("\r\n===== SCD4x FULL SELF-TEST =====\r\n");

    /* init */
    res = sen5x_init(&g_sen5x_handle);
    if (res != 0)
    {
        sen5x_interface_debug_print("sen5x_init: ERROR %d\r\n", res);
        return;
    }
    sen5x_interface_debug_print("sen5x_init: OK\r\n");

    /* info */
    res = sen5x_info(&info);
    if (res == 0)
    {
        sen5x_interface_debug_print("Chip: %s, Manufacturer: %s, IF: %s\r\n",
                                    info.chip_name,
                                    info.manufacturer_name,
                                    info.interface);
    }

    /* sensor variant via high-level API */
    res = sen5x_get_sensor_variant(&g_sen5x_handle, &variant);
    if (res == 0)
    {
        sen5x_interface_debug_print("Sensor variant: 0x%04X\r\n", variant);
    }
    else
    {
        sen5x_interface_debug_print("sen5x_get_sensor_variant: ERROR %d\r\n", res);
    }

    /* read serial number */
    res = sen5x_get_serial_number(&g_sen5x_handle, serial);
    if (res == 0)
    {
        sen5x_interface_debug_print("Serial: %04X-%04X-%04X\r\n",
                                    serial[0], serial[1], serial[2]);
    }

    /* persist settings (harmless if unchanged) */
    res = sen5x_persist_settings(&g_sen5x_handle);
    sen5x_interface_debug_print("sen5x_persist_settings: %s\r\n", (res == 0) ? "OK" : "ERROR");

    /* normal periodic measurement */
    res = sen5x_start_periodic_measurement(&g_sen5x_handle);
    sen5x_interface_debug_print("sen5x_start_periodic_measurement: %s\r\n",
                                (res == 0) ? "OK" : "ERROR");

    if (res == 0)
    {
        /* Wait for data ready (typ 5 s, max 12 s -> we wait up to ~15 s) */
        for (int i = 0; i < 15; ++i)
        {
            sen5x_interface_delay_ms(1000);
            res = sen5x_get_data_ready_status(&g_sen5x_handle, &ready);
            if (res != 0) break;
            if (ready == SEN5X_BOOL_TRUE) break;
        }

        sen5x_interface_debug_print("sen5x_get_data_ready_status: %s\r\n",
                                    (res == 0) ? "OK" : "ERROR");

        if (res == 0 && ready == SEN5X_BOOL_TRUE)
        {
            res = sen5x_read(&g_sen5x_handle,
                             &co2_raw, &co2_ppm,
                             &t_raw, &t_degC,
                             &rh_raw, &rh_pct);

            if (res != 0)
            {
                sen5x_interface_debug_print("sen5x_read (periodic): ERROR %d\r\n", res);
            }
            else
            {
                /* Failsafe: if ppm is 0 but raw isn't, trust raw */
                if (co2_ppm == 0 && co2_raw != 0)
                {
                    co2_ppm = co2_raw;
                }

                long t_int  = (long)t_degC;
                long t_frac = (long)((t_degC - t_int) * 100.0f);
                long rh_int  = (long)rh_pct;
                long rh_frac = (long)((rh_pct - rh_int) * 100.0f);

                sen5x_interface_debug_print(
                    "sen5x_read (periodic): OK\r\n"
                    "Periodic: CO2=%u ppm (raw=%u), "
                    "T=%ld.%02ld C (raw=%u), "
                    "RH=%ld.%02ld %% (raw=%u)\r\n",
                    (unsigned)co2_ppm,
                    (unsigned)co2_raw,
                    t_int, t_frac, t_raw,
                    rh_int, rh_frac, rh_raw
                );
            }
        }

        res = sen5x_stop_periodic_measurement(&g_sen5x_handle);
        sen5x_interface_debug_print("sen5x_stop_periodic_measurement: %s\r\n",
                                    (res == 0) ? "OK" : "ERROR");
        sen5x_interface_delay_ms(500);
    }

    /* low power periodic (optional – may fail depending on state / FW) */
    res = sen5x_start_low_power_periodic_measurement(&g_sen5x_handle);
    if (res != 0)
    {
        sen5x_interface_debug_print(
            "sen5x_start_low_power_periodic_measurement: SKIPPED (res=%d)\r\n", res);
    }
    else
    {
        /* Low power mode has long interval (~30s). We just check readiness once. */
        sen5x_interface_delay_ms(32000);

        res = sen5x_get_data_ready_status(&g_sen5x_handle, &ready);
        sen5x_interface_debug_print("sen5x_get_data_ready_status (LP): %s\r\n",
                                    (res == 0) ? "OK" : "ERROR");

        if (res == 0 && ready == SEN5X_BOOL_TRUE)
        {
            res = sen5x_read(&g_sen5x_handle,
                             &co2_raw, &co2_ppm,
                             &t_raw, &t_degC,
                             &rh_raw, &rh_pct);

            if (res != 0)
            {
                sen5x_interface_debug_print("sen5x_read (LP periodic): ERROR %d\r\n", res);
            }
            else
            {
                if (co2_ppm == 0 && co2_raw != 0)
                {
                    co2_ppm = co2_raw;
                }

                long t_int  = (long)t_degC;
                long t_frac = (long)((t_degC - t_int) * 100.0f);
                long rh_int  = (long)rh_pct;
                long rh_frac = (long)((rh_pct - rh_int) * 100.0f);

                sen5x_interface_debug_print(
                    "sen5x_read (LP periodic): OK\r\n"
                    "LP: CO2=%u ppm (raw=%u), "
                    "T=%ld.%02ld C (raw=%u), "
                    "RH=%ld.%02ld %% (raw=%u)\r\n",
                    (unsigned)co2_ppm,
                    (unsigned)co2_raw,
                    t_int, t_frac, t_raw,
                    rh_int, rh_frac, rh_raw
                );
            }
        }

        res = sen5x_stop_periodic_measurement(&g_sen5x_handle);
        sen5x_interface_debug_print("sen5x_stop_periodic_measurement (after LP): %s\r\n",
                                    (res == 0) ? "OK" : "ERROR");
        sen5x_interface_delay_ms(500);
    }

    /* single shot measurement */
    res = sen5x_measure_single_shot(&g_sen5x_handle);
    sen5x_interface_debug_print("sen5x_measure_single_shot: %s\r\n",
                                (res == 0) ? "OK" : "ERROR");
    if (res == 0)
    {
        sen5x_interface_delay_ms(5000); /* max command duration */
        res = sen5x_read(&g_sen5x_handle,
                         &co2_raw, &co2_ppm,
                         &t_raw, &t_degC,
                         &rh_raw, &rh_pct);
        if (res != 0)
        {
            sen5x_interface_debug_print("sen5x_read (single shot): ERROR %d\r\n", res);
        }
        else
        {
            if (co2_ppm == 0 && co2_raw != 0)
            {
                co2_ppm = co2_raw;
            }

            long t_int  = (long)t_degC;
            long t_frac = (long)((t_degC - t_int) * 100.0f);
            long rh_int  = (long)rh_pct;
            long rh_frac = (long)((rh_pct - rh_int) * 100.0f);

            sen5x_interface_debug_print(
                "sen5x_read (single shot): OK\r\n"
                "Single: CO2=%u ppm (raw=%u), "
                "T=%ld.%02ld C (raw=%u), "
                "RH=%ld.%02ld %% (raw=%u)\r\n",
                (unsigned)co2_ppm,
                (unsigned)co2_raw,
                t_int, t_frac, t_raw,
                rh_int, rh_frac, rh_raw
            );
        }
    }

    /* single shot RHT only */
    res = sen5x_measure_single_shot_rht_only(&g_sen5x_handle);
    sen5x_interface_debug_print("sen5x_measure_single_shot_rht_only: %s\r\n",
                                (res == 0) ? "OK" : "ERROR");
    if (res == 0)
    {
        sen5x_interface_delay_ms(5000);
        res = sen5x_read(&g_sen5x_handle,
                         &co2_raw, &co2_ppm,
                         &t_raw, &t_degC,
                         &rh_raw, &rh_pct);
        if (res != 0)
        {
            sen5x_interface_debug_print("sen5x_read (single shot RHT only): ERROR %d\r\n", res);
        }
        else
        {
            if (co2_ppm == 0 && co2_raw != 0)
            {
                co2_ppm = co2_raw;
            }

            long t_int  = (long)t_degC;
            long t_frac = (long)((t_degC - t_int) * 100.0f);
            long rh_int  = (long)rh_pct;
            long rh_frac = (long)((rh_pct - rh_int) * 100.0f);

            sen5x_interface_debug_print(
                "sen5x_read (single shot RHT only): OK\r\n"
                "RHT-only: CO2=%u ppm (raw=%u), "
                "T=%ld.%02ld C (raw=%u), "
                "RH=%ld.%02ld %% (raw=%u)\r\n",
                (unsigned)co2_ppm,
                (unsigned)co2_raw,
                t_int, t_frac, t_raw,
                rh_int, rh_frac, rh_raw
            );
        }
    }

    /* self test */
    res = sen5x_perform_self_test(&g_sen5x_handle, &malfunction);
    sen5x_interface_debug_print("sen5x_perform_self_test: %s, malfunction=%u\r\n",
                                (res == 0) ? "OK" : "ERROR",
                                (unsigned)malfunction);

    /* power down / wake up / reinit – non-fatal if they fail */
    res = sen5x_power_down(&g_sen5x_handle);
    sen5x_interface_debug_print("sen5x_power_down: %s\r\n",
                                (res == 0) ? "OK" : "ERROR");
    sen5x_interface_delay_ms(10);

    res = sen5x_wake_up(&g_sen5x_handle);
    sen5x_interface_debug_print("sen5x_wake_up: %s\r\n",
                                (res == 0) ? "OK" : "ERROR");
    sen5x_interface_delay_ms(10);

    res = sen5x_reinit(&g_sen5x_handle);
    sen5x_interface_debug_print("sen5x_reinit: %s\r\n",
                                (res == 0) ? "OK" : "ERROR");
    sen5x_interface_delay_ms(20);

    /* raw get_reg for sensor variant while in idle */
    res = sen5x_get_reg(&g_sen5x_handle, 0x202F, raw_buf, 3, 1);
    sen5x_interface_debug_print("sen5x_get_reg(0x202F): %s (res=%d, bytes=%02X %02X %02X)\r\n",
                                (res == 0) ? "OK" : "ERROR",
                                res,
                                raw_buf[0], raw_buf[1], raw_buf[2]);

    /* DO NOT call sen5x_set_reg(0x202F): it's a read-only command */

    /* deinit */
    res = sen5x_deinit(&g_sen5x_handle);
    sen5x_interface_debug_print("sen5x_deinit: %s\r\n",
                                (res == 0) ? "OK" : "ERROR");

    sen5x_interface_debug_print("===== SCD4x FULL SELF-TEST DONE =====\r\n");
}
