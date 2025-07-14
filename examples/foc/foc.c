/**
 * Copyright (c) 2025 Bosch Sensortec GmbH. All rights reserved.
 *
 * BSD-3-Clause
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its
 *    contributors may be used to endorse or promote products derived from
 *    this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
 * STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING
 * IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 * @file    foc.c
 * @brief   Example for Fast Offset Compensation
 *
 */

#include <stdlib.h>
#include <stdio.h>
#include <stdbool.h>

#include "bhi385.h"
#include "bhi385_parse.h"
#include "common.h"
#include "bhi385_virtual_sensor_conf_param.h"
#include "bhi385_event_data.h"

#include "bhi385/Bosch_Shuttle3_BHI385_bsxsam_lite.fw.h"

#define WORK_BUFFER_SIZE        UINT16_C(2048)
#define PARSE_DATA_WINDOW_SIZE  UINT16_C(3000)

/*! Comment this macro to perform gyro foc  */
#define ACCEL

#define ACCEL_G_VALUE           1.0f
#define GYRO_DPS_VALUE          2000.0f
#define ACCEL_MAX_SAMPLE_VALUE  4096.0f
#define GYRO_MAX_SAMPLE_VALUE   32768.0f

static uint8_t sensor_id_back;

/*! @brief Prints API error code.
 *
 *  @param[in] rslt      : API Error code.
 *  @param[in] dev       : Device reference.
 */
static void print_api_error(int8_t rslt, struct bhi385_dev *dev);

/*! @brief Loads firmware image to ram.
 *
 *  @param[in] boot_stat : Boot status.
 *  @param[in] dev       : Device reference.
 */
static void upload_firmware(uint8_t boot_stat, struct bhi385_dev *dev);

/*! @brief Parse accelerometer data.
 *
 *  @param[in] callback_info : Sensor data info.
 *  @param[in] callback_ref  : Parse reference.
 */
static void parse_accel(const struct bhi385_fifo_parse_data_info *callback_info, void *callback_ref);

/*! @brief Parse gyro data.
 *
 *  @param[in] callback_info : Sensor data info.
 *  @param[in] callback_ref  : Parse reference.
 */
static void parse_gyro(const struct bhi385_fifo_parse_data_info *callback_info, void *callback_ref);

/*! @brief Parse meta event.
 *
 *  @param[in] callback_info : Sensor data info.
 *  @param[in] callback_ref  : Parse reference.
 */
static void parse_meta_event(const struct bhi385_fifo_parse_data_info *callback_info, void *callback_ref);

/**
 * @brief Reads sensor data from the specified sensor.
 *
 * @param[in] sensor_id : The ID of the sensor to read data from.
 * @param[in] count     : The number of data samples to read.
 * @param[in] bhy      : Pointer to the bhi385_dev structure representing the BHY device.
 */
static void read_sensor_data(uint8_t sensor_id, uint32_t count, struct bhi385_dev *bhy);

/**
 * @brief Perform FOC for the specified sensor.
 *
 * @param[in] sensor_id : The ID of the sensor to perform FOC for.
 * @param[in] bhy      : Pointer to the bhi385_dev structure representing the BHY device.
 */
static void perform_foc(uint8_t sensor_id, struct bhi385_dev *bhy);

enum bhi385_intf intf;

int main(void)
{
    uint8_t chip_id = 0;
    uint16_t version = 0;
    int8_t rslt;
    struct bhi385_dev bhy;
    uint8_t hif_ctrl, boot_status;
    uint8_t accuracy; /* Accuracy is reported as a meta event. It is being printed alongside the data */
    uint8_t work_buffer[WORK_BUFFER_SIZE];

#ifdef BHI385_USE_I2C
    intf = BHI385_I2C_INTERFACE;
#else
    intf = BHI385_SPI_INTERFACE;
#endif

    setup_interfaces(true, intf); /* Perform a power on reset */

#ifdef BHI385_USE_I2C
    rslt = bhi385_init(BHI385_I2C_INTERFACE,
                       bhi385_i2c_read,
                       bhi385_i2c_write,
                       bhi385_delay_us,
                       BHI385_RD_WR_LEN,
                       NULL,
                       &bhy);
#else
    rslt = bhi385_init(BHI385_SPI_INTERFACE,
                       bhi385_spi_read,
                       bhi385_spi_write,
                       bhi385_delay_us,
                       BHI385_RD_WR_LEN,
                       NULL,
                       &bhy);
#endif
    print_api_error(rslt, &bhy);

    rslt = bhi385_soft_reset(&bhy);
    print_api_error(rslt, &bhy);

    rslt = bhi385_get_chip_id(&chip_id, &bhy);
    print_api_error(rslt, &bhy);

    /* Check for a valid Chip ID */
    if (chip_id == BHI385_CHIP_ID)
    {
        printf("Chip ID read 0x%X\r\n", chip_id);
    }
    else
    {
        printf("Device not found. Chip ID read 0x%X\r\n", chip_id);
    }

    /* Configure the host interface */
    hif_ctrl = BHI385_HIF_CTRL_ASYNC_STATUS_CHANNEL;
    rslt = bhi385_set_host_intf_ctrl(hif_ctrl, &bhy);
    print_api_error(rslt, &bhy);
    rslt = bhi385_get_host_interrupt_ctrl(&hif_ctrl, &bhy);
    print_api_error(rslt, &bhy);

    /* Check if the sensor is ready to load firmware */
    rslt = bhi385_get_boot_status(&boot_status, &bhy);
    print_api_error(rslt, &bhy);

    if (boot_status & BHI385_BST_HOST_INTERFACE_READY)
    {
        upload_firmware(boot_status, &bhy);

        rslt = bhi385_get_kernel_version(&version, &bhy);
        print_api_error(rslt, &bhy);
        if ((rslt == BHI385_OK) && (version != 0))
        {
            printf("Boot successful. Kernel version %u.\r\n", version);
        }

        rslt = bhi385_register_fifo_parse_callback(BHI385_SYS_ID_META_EVENT, parse_meta_event, (void*)&accuracy, &bhy);
        print_api_error(rslt, &bhy);
        rslt =
            bhi385_register_fifo_parse_callback(BHI385_SYS_ID_META_EVENT_WU, parse_meta_event, (void*)&accuracy, &bhy);
        print_api_error(rslt, &bhy);
        rslt = bhi385_register_fifo_parse_callback(BHI385_SENSOR_ID_ACC_PASS, parse_accel, NULL, &bhy);
        print_api_error(rslt, &bhy);
        rslt = bhi385_register_fifo_parse_callback(BHI385_SENSOR_ID_GYRO_PASS, parse_gyro, NULL, &bhy);
        print_api_error(rslt, &bhy);

        rslt = bhi385_get_and_process_fifo(work_buffer, WORK_BUFFER_SIZE, &bhy);
        print_api_error(rslt, &bhy);
    }
    else
    {
        printf("Host interface not ready. Exiting\r\n");

        close_interfaces(intf);

        return 0;
    }

    /* Update the callback table to enable parsing of sensor data */
    rslt = bhi385_update_virtual_sensor_list(&bhy);
    print_api_error(rslt, &bhy);

    #ifdef ACCEL

    /* Read out data measured at 25Hz */
    sensor_id_back = BHI385_PHYS_SENSOR_ID_ACCELEROMETER;
    struct bhi385_virtual_sensor_conf_param_conf sensor_conf = { 0 };

    sensor_conf.sample_rate = 25.0f; /* Read out data measured at 25Hz */
    sensor_conf.latency = 0; /* Report immediately */
    rslt = bhi385_virtual_sensor_conf_param_set_cfg(BHI385_SENSOR_ID_ACC_PASS, &sensor_conf, &bhy);
    print_api_error(rslt, &bhy);
    printf("Enable %s at %.2fHz.\r\n", get_sensor_name(BHI385_SENSOR_ID_ACC_PASS), sensor_conf.sample_rate);
    coines_delay_msec(10);

    printf("Read out limited count accel data before setting FOC offset\r\n");
    read_sensor_data(BHI385_SENSOR_ID_ACC_PASS, PARSE_DATA_WINDOW_SIZE, &bhy);

    /* Disable sensor */
    sensor_conf.sample_rate = 0.0f;
    rslt = bhi385_virtual_sensor_conf_param_set_cfg(BHI385_SENSOR_ID_ACC_PASS, &sensor_conf, &bhy);
    print_api_error(rslt, &bhy);

    perform_foc(BHI385_PHYS_SENSOR_ID_ACCELEROMETER, &bhy);

    /* Flush sensor data from the FIFO */
    rslt = bhi385_flush_fifo(BHI385_SENSOR_ID_ACC_PASS, &bhy);
    print_api_error(rslt, &bhy);

    /* Read out data measured at 25Hz */
    sensor_conf.sample_rate = 25.0f;
    rslt = bhi385_virtual_sensor_conf_param_set_cfg(BHI385_SENSOR_ID_ACC_PASS, &sensor_conf, &bhy);
    print_api_error(rslt, &bhy);

    printf("Read out limited count accel data after setting FOC offset\r\n");
    read_sensor_data(BHI385_SENSOR_ID_ACC_PASS, PARSE_DATA_WINDOW_SIZE, &bhy);

    coines_delay_msec(10000);

    #else

    /* Read out data measured at 25Hz */
    sensor_id_back = BHI385_PHYS_SENSOR_ID_GYROSCOPE;
    sensor_conf.sample_rate = 25.0;
    sensor_conf.latency = 0; /* Report immediately */
    rslt = bhi385_virtual_sensor_conf_param_set_cfg(BHI385_SENSOR_ID_GYRO_PASS, &sensor_conf, &bhy);
    print_api_error(rslt, &bhy);
    printf("Enable %s at %.2fHz.\r\n", get_sensor_name(BHI385_SENSOR_ID_GYRO_PASS), sensor_conf.sample_rate);
    coines_delay_msec(10);

    printf("Read out limited count gyro data before setting FOC offset\r\n");
    read_sensor_data(BHI385_SENSOR_ID_GYRO_PASS, PARSE_DATA_WINDOW_SIZE, &bhy);

    /* Disable sensor */
    sensor_conf.sample_rate = 0.0f;
    rslt = bhi385_virtual_sensor_conf_param_set_cfg(BHI385_SENSOR_ID_GYRO_PASS, &sensor_conf, &bhy);
    print_api_error(rslt, &bhy);

    perform_foc(BHI385_PHYS_SENSOR_ID_GYROSCOPE, &bhy);

    /* Flush sensor data from the FIFO */
    rslt = bhi385_flush_fifo(BHI385_SENSOR_ID_GYRO_PASS, &bhy);
    print_api_error(rslt, &bhy);

    /* Read out data measured at 25Hz */
    sensor_conf.sample_rate = 25.0f;
    rslt = bhi385_virtual_sensor_conf_param_set_cfg(BHI385_SENSOR_ID_GYRO_PASS, &sensor_conf, &bhy);
    print_api_error(rslt, &bhy);

    printf("Read out limited count gyro data after setting FOC offset\r\n");
    read_sensor_data(BHI385_SENSOR_ID_GYRO_PASS, PARSE_DATA_WINDOW_SIZE, &bhy);
    #endif

    close_interfaces(intf);

    return rslt;
}

static void parse_accel(const struct bhi385_fifo_parse_data_info *callback_info, void *callback_ref)
{
    struct bhi385_event_data_xyz data;

    if (!callback_info)
    {
        printf("Null reference\r\n");

        return;
    }

    bhi385_event_data_parse_xyz(callback_info->data_ptr, &data);

    if (sensor_id_back == BHI385_SENSOR_ID_ACC_PASS)
    {
        printf(" x: %f, y: %f, z: %f; unit: LSB/g\r\n", data.x * (ACCEL_G_VALUE / ACCEL_MAX_SAMPLE_VALUE),
               data.y * (ACCEL_G_VALUE / ACCEL_MAX_SAMPLE_VALUE), data.z * (ACCEL_G_VALUE / ACCEL_MAX_SAMPLE_VALUE));
    }
}

static void parse_gyro(const struct bhi385_fifo_parse_data_info *callback_info, void *callback_ref)
{
    struct bhi385_event_data_xyz data;

    if (!callback_info)
    {
        printf("Null reference\r\n");

        return;
    }

    bhi385_event_data_parse_xyz(callback_info->data_ptr, &data);

    if (sensor_id_back == BHI385_PHYS_SENSOR_ID_GYROSCOPE)
    {
        printf(" x: %f, y: %f, z: %f; unit: LSB/dps\r\n", data.x * (GYRO_DPS_VALUE / GYRO_MAX_SAMPLE_VALUE),
               data.y * (GYRO_DPS_VALUE / GYRO_MAX_SAMPLE_VALUE), data.z * (GYRO_DPS_VALUE / GYRO_MAX_SAMPLE_VALUE));
    }
}

static void parse_meta_event(const struct bhi385_fifo_parse_data_info *callback_info, void *callback_ref)
{
    (void)callback_ref;
    uint8_t meta_event_type = callback_info->data_ptr[0];
    uint8_t byte1 = callback_info->data_ptr[1];
    uint8_t byte2 = callback_info->data_ptr[2];
    uint8_t *accuracy = (uint8_t*)callback_ref;
    char *event_text;

    if (callback_info->sensor_id == BHI385_SYS_ID_META_EVENT)
    {
        event_text = "[META EVENT]";
    }
    else if (callback_info->sensor_id == BHI385_SYS_ID_META_EVENT_WU)
    {
        event_text = "[META EVENT WAKE UP]";
    }
    else
    {
        return;
    }

    switch (meta_event_type)
    {
        case BHI385_META_EVENT_FLUSH_COMPLETE:
            printf("%s Flush complete for sensor id %u\r\n", event_text, byte1);
            break;
        case BHI385_META_EVENT_SAMPLE_RATE_CHANGED:
            printf("%s Sample rate changed for sensor id %u\r\n", event_text, byte1);
            break;
        case BHI385_META_EVENT_POWER_MODE_CHANGED:
            printf("%s Power mode changed for sensor id %u\r\n", event_text, byte1);
            break;
        case BHI385_META_EVENT_ALGORITHM_EVENTS:
            printf("%s Algorithm event\r\n", event_text);
            break;
        case BHI385_META_EVENT_SENSOR_STATUS:
            printf("%s Accuracy for sensor id %u changed to %u\r\n", event_text, byte1, byte2);
            if (accuracy)
            {
                *accuracy = byte2;
            }

            break;
        case BHI385_META_EVENT_BSX_DO_STEPS_MAIN:
            printf("%s BSX event (do steps main)\r\n", event_text);
            break;
        case BHI385_META_EVENT_BSX_DO_STEPS_CALIB:
            printf("%s BSX event (do steps calib)\r\n", event_text);
            break;
        case BHI385_META_EVENT_BSX_GET_OUTPUT_SIGNAL:
            printf("%s BSX event (get output signal)\r\n", event_text);
            break;
        case BHI385_META_EVENT_SENSOR_ERROR:
            printf("%s Sensor id %u reported error 0x%02X\r\n", event_text, byte1, byte2);
            break;
        case BHI385_META_EVENT_FIFO_OVERFLOW:
            printf("%s FIFO overflow\r\n", event_text);
            break;
        case BHI385_META_EVENT_DYNAMIC_RANGE_CHANGED:
            printf("%s Dynamic range changed for sensor id %u\r\n", event_text, byte1);
            break;
        case BHI385_META_EVENT_FIFO_WATERMARK:
            printf("%s FIFO watermark reached\r\n", event_text);
            break;
        case BHI385_META_EVENT_INITIALIZED:
            printf("%s Firmware initialized. Firmware version %u\r\n", event_text, ((uint16_t)byte2 << 8) | byte1);
            break;
        case BHI385_META_TRANSFER_CAUSE:
            printf("%s Transfer cause for sensor id %u\r\n", event_text, byte1);
            break;
        case BHI385_META_EVENT_SENSOR_FRAMEWORK:
            printf("%s Sensor framework event for sensor id %u\r\n", event_text, byte1);
            break;
        case BHI385_META_EVENT_RESET:
            printf("%s Reset event\r\n", event_text);
            break;
        case BHI385_META_EVENT_SPACER:
            break;
        default:
            printf("%s Unknown meta event with id: %u\r\n", event_text, meta_event_type);
            break;
    }
}

static void print_api_error(int8_t rslt, struct bhi385_dev *dev)
{
    if (rslt != BHI385_OK)
    {
        printf("%s\r\n", get_api_error(rslt));
        if ((rslt == BHI385_E_IO) && (dev != NULL))
        {
            printf("%s\r\n", get_coines_error(dev->hif.intf_rslt));
            dev->hif.intf_rslt = BHI385_INTF_RET_SUCCESS;
        }

        exit(0);
    }
}

static void upload_firmware(uint8_t boot_stat, struct bhi385_dev *dev)
{
    uint8_t sensor_error;
    int8_t temp_rslt;
    int8_t rslt = BHI385_OK;

    printf("Loading firmware into RAM.\r\n");
    rslt = bhi385_upload_firmware_to_ram(bhi385_firmware_image, sizeof(bhi385_firmware_image), dev);
    temp_rslt = bhi385_get_error_value(&sensor_error, dev);
    if (sensor_error)
    {
        printf("%s\r\n", get_sensor_error_text(sensor_error));
    }

    print_api_error(rslt, dev);
    print_api_error(temp_rslt, dev);

    printf("Booting from RAM.\r\n");
    rslt = bhi385_boot_from_ram(dev);

    temp_rslt = bhi385_get_error_value(&sensor_error, dev);
    if (sensor_error)
    {
        printf("%s\r\n", get_sensor_error_text(sensor_error));
    }

    print_api_error(rslt, dev);
    print_api_error(temp_rslt, dev);
}

static void read_sensor_data(uint8_t sensor_id, uint32_t count, struct bhi385_dev *bhy)
{
    int8_t rslt;
    uint32_t curr_ts;
    uint8_t work_buffer[WORK_BUFFER_SIZE];
    uint32_t start_ts = coines_get_millis();

    do
    {
        curr_ts = coines_get_millis();
        if (get_interrupt_status())
        {
            /* Data from the FIFO is read and the relevant callbacks if registered are called */
            rslt = bhi385_get_and_process_fifo(work_buffer, WORK_BUFFER_SIZE, bhy);
            print_api_error(rslt, bhy);
        }
    } while ((curr_ts - start_ts) < count);
}

static void perform_foc(uint8_t sensor_id, struct bhi385_dev *bhy)
{
    int8_t rslt;
    struct bhi385_foc_resp foc_resp;

    rslt = bhi385_perform_foc(sensor_id, &foc_resp, bhy);
    print_api_error(rslt, bhy);

    switch (foc_resp.foc_status)
    {
        case BHI385_FOC_SUCCESS:
            printf("FOC Success\r\n");
            break;
        case BHI385_FOC_FAILURE:
            printf("FOC failed\r\n");
            break;
        case BHI385_FOC_UNKNOWN_ERROR:
            printf("Unknown FOC failure\r\n");
            break;
        default:
            printf("Undefined FOC status %u\r\n", foc_resp.foc_status);
            break;
    }

    printf("FOC Offset, X: %d, Y: %d, Z %d\r\n", foc_resp.x_offset, foc_resp.y_offset, foc_resp.z_offset);
}
