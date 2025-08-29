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
 * @file    cal_seq.c
 * @brief   Example to do calibration process for the BHI
 *
 */

#include <stdlib.h>
#include <stdio.h>
#include <stdbool.h>

#include "bhi385.h"
#include "bhi385_parse.h"
#include "common.h"
#include "bhi385_phy_sensor_ctrl_param_defs.h"

#include "bhi385/Bosch_Shuttle3_BHI385_bsxsam_lite.fw.h"

#define SENSOR_FOC_SUCCESS       (0)
#define SENSOR_FOC_FAIL          (101)
#define SENSOR_FOC_UNKNOWN       (36)
#define SENSOR_CRT_SUCCESS       (0)
#define SENSOR_CRT_FAIL          (2)

#define WORK_BUFFER_SIZE         2048

#define BHI385_PHY_CRT_CTRL_LEN  (3)
#define CALIB_SEQ_FILE_NAME      "cal_seq.txt"

static void print_api_error(int8_t rslt, struct bhi385_dev *dev);
static int8_t upload_firmware(struct bhi385_dev *dev);
static void parse_meta_event(const struct bhi385_fifo_parse_data_info *callback_info, void *callback_ref);
static void parse_3axis_s16(const struct bhi385_fifo_parse_data_info *callback_info, void *callback_ref);
static int8_t bhi3_perform_foc_crt(struct bhi385_foc_resp *acc_foc_status,
                                   struct bhi385_foc_resp *gyro_foc_status,
                                   struct bhi385_dev *dev,
                                   uint8_t flag);

static int16_t convert_char_int(char *line, char *pattern1, char *pattern2);

static bhi385_phy_sensor_ctrl_param_gyro_fast_offset_calib foc_resp_gyro_backup;
static bhi385_phy_sensor_ctrl_param_accel_fast_offset_calib foc_resp_acc_backup;
static uint8_t crt_backup[3] = { 0 };

static bhi385_phy_sensor_ctrl_param_gyro_crt_data data_crt = { 0 };

enum bhi385_intf intf;

int main(int argc, char *argv[])
{
    int8_t rslt;
    uint8_t product_id = 0;
    uint16_t version = 0;
    struct bhi385_dev bhy;
    uint8_t hintr_ctrl, hif_ctrl, boot_status;
    struct bhi385_foc_resp foc_resp_gyro, foc_resp_acc;
    uint8_t accuracy;
    uint8_t work_buffer[WORK_BUFFER_SIZE];
    bool flag = true;
    struct bhi385_virtual_sensor_conf_param_conf sensor_conf = { 0 };
    char line[256] = { 0 };
    uint8_t loop = 0;
    uint8_t limit = 50;

    uint8_t counter = 0;

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

    rslt = bhi385_get_product_id(&product_id, &bhy);
    print_api_error(rslt, &bhy);

    /* Check for a valid product ID */
    if (product_id != BHI385_PRODUCT_ID)
    {
        printf("Product ID read %X. Expected %X\r\n", product_id, BHI385_PRODUCT_ID);
    }
    else
    {
        printf("BHI385 Product ID read %X\r\n", product_id);
    }

    /* Check the interrupt pin and FIFO configurations. Disable status and debug */
    hintr_ctrl = BHI385_ICTL_DISABLE_STATUS_FIFO | BHI385_ICTL_DISABLE_DEBUG;

    rslt = bhi385_set_host_interrupt_ctrl(hintr_ctrl, &bhy);
    print_api_error(rslt, &bhy);

    /* Configure the host interface */
    hif_ctrl = 0;
    rslt = bhi385_set_host_intf_ctrl(hif_ctrl, &bhy);
    print_api_error(rslt, &bhy);

    /* Check if the sensor is ready to load firmware */
    rslt = bhi385_get_boot_status(&boot_status, &bhy);
    print_api_error(rslt, &bhy);

    if (boot_status & BHI385_BST_HOST_INTERFACE_READY)
    {
        uint8_t sensor_error;
        int8_t temp_rslt;
        printf("Loading firmware.\r\n");
        counter++;

        rslt = upload_firmware(&bhy);
        temp_rslt = bhi385_get_error_value(&sensor_error, &bhy);
        if (sensor_error)
        {
            printf("%s\r\n", get_sensor_error_text(sensor_error));
        }

        print_api_error(rslt, &bhy);
        print_api_error(temp_rslt, &bhy);

        printf("Booting from RAM.\r\n");
        rslt = bhi385_boot_from_ram(&bhy);

        temp_rslt = bhi385_get_error_value(&sensor_error, &bhy);
        if (sensor_error)
        {
            printf("%s\r\n", get_sensor_error_text(sensor_error));
        }

        print_api_error(rslt, &bhy);
        print_api_error(temp_rslt, &bhy);

        rslt = bhi385_get_kernel_version(&version, &bhy);
        print_api_error(rslt, &bhy);
        if ((rslt == BHI385_OK) && (version != 0))
        {
            printf("Boot successful. Kernel version %u.\r\n", version);
        }
    }
    else
    {
        printf("Host interface not ready. Exiting\r\n");
        close_interfaces(intf);

        return 0;
    }

    while (true)
    {
        if (flag)
        {
            rslt = bhi3_perform_foc_crt(&foc_resp_acc, &foc_resp_gyro, &bhy, atoi(argv[1]));
            if (rslt != BHI385_OK)
            {
                close_interfaces(intf);

                return 0;
            }
        }
        else
        {
            uint8_t sensor_error;
            int8_t temp_rslt;
            counter++;

            printf("Loading firmware again.\r\n");

            rslt = upload_firmware(&bhy);
            temp_rslt = bhi385_get_error_value(&sensor_error, &bhy);
            print_api_error(rslt, &bhy);
            print_api_error(temp_rslt, &bhy);

            printf("Booting from RAM again.\r\n");
            rslt = bhi385_boot_from_ram(&bhy);

            temp_rslt = bhi385_get_error_value(&sensor_error, &bhy);
            print_api_error(rslt, &bhy);
            print_api_error(temp_rslt, &bhy);

            rslt = bhi385_get_kernel_version(&version, &bhy);
            print_api_error(rslt, &bhy);

            if ((rslt == BHI385_OK) && (version != 0))
            {
                printf("Boot successful. Kernel version %u.\r\n", version);
            }

            char *pattern1 = "x ";
            char *pattern2 = ", y ";
            char *pattern3 = ", z ";
            char *pattern4 = "\0";

            if (atoi(argv[1]) & 0x01)
            {
                FILE *fp = fopen(CALIB_SEQ_FILE_NAME, "r");
                if (fp == NULL)
                {
                    printf("Error while opening file\r\n");

                    return -1;
                }

                while (fgets(line, sizeof(line), fp))
                {
                    if (strstr(line, "Latest Accel FOC offsets"))
                    {
                        foc_resp_acc_backup.x_offset = convert_char_int(line, pattern1, pattern2);
                        foc_resp_acc_backup.y_offset = convert_char_int(line, pattern2, pattern3);
                        foc_resp_acc_backup.z_offset = convert_char_int(line, pattern3, pattern4);
                    }
                }

                rslt = bhi385_phy_sensor_ctrl_param_accel_set_foc_calibration(&foc_resp_acc_backup, &bhy);
                if (rslt != BHI385_OK)
                {
                    printf("Set accel foc failed!\r\n");
                    close_interfaces(intf);

                    return 0;
                }

                printf("Perform Accel FOC successfully\r\n");
                fclose(fp);
                printf("Acc  offsets back up: x %d, y %d, z %d\r\n",
                       foc_resp_acc_backup.x_offset,
                       foc_resp_acc_backup.y_offset,
                       foc_resp_acc_backup.z_offset);

                /* wait for load acc foc ready*/
                coines_delay_msec(10);
            }

            if (atoi(argv[1]) & 0x02)
            {
                FILE *fp = fopen(CALIB_SEQ_FILE_NAME, "r");
                if (fp == NULL)
                {
                    printf("Error while opening file\r\n");

                    return -1;
                }

                while (fgets(line, sizeof(line), fp))
                {
                    if (strstr(line, "Latest GYRO CRT status"))
                    {
                        crt_backup[0] = convert_char_int(line, pattern1, pattern2);
                        crt_backup[1] = convert_char_int(line, pattern2, pattern3);
                        crt_backup[2] = convert_char_int(line, pattern3, pattern4);
                    }
                }

                data_crt.x = crt_backup[0];
                data_crt.y = crt_backup[1];
                data_crt.z = crt_backup[2];
                rslt = bhi385_phy_sensor_ctrl_param_set_gyro_data(&data_crt, &bhy);
                if (rslt != BHI385_OK)
                {
                    printf("load gyro crt failed!\r\n");
                    printf("rslt = %d\r\n", rslt);
                    close_interfaces(intf);

                    return 0;
                }

                fclose(fp);
                printf("crt  offsets back up: x %d, y %d, z %d status %d\r\n", crt_backup[0], crt_backup[1],
                       crt_backup[2], rslt);

                /* wait for load crt ready*/
                coines_delay_msec(200);
            }

            if (atoi(argv[1]) & 0x04)
            {
                FILE *fp = fopen(CALIB_SEQ_FILE_NAME, "r");
                if (fp == NULL)
                {
                    printf("Error while opening file\r\n");

                    return -1;
                }

                while (fgets(line, sizeof(line), fp))
                {
                    if (strstr(line, "Latest GYRO FOC offsets"))
                    {
                        foc_resp_gyro_backup.x_offset = convert_char_int(line, pattern1, pattern2);
                        foc_resp_gyro_backup.y_offset = convert_char_int(line, pattern2, pattern3);
                        foc_resp_gyro_backup.z_offset = convert_char_int(line, pattern3, pattern4);
                    }
                }

                rslt = bhi385_phy_sensor_ctrl_param_gyro_set_foc_calibration(&foc_resp_gyro_backup, &bhy);
                if (rslt != BHI385_OK)
                {
                    printf("load gyro foc failed!\r\n");
                    close_interfaces(intf);

                    return 0;
                }

                fclose(fp);
                printf("gyro offsets back up: x %d, y %d, z %d\r\n",
                       foc_resp_gyro_backup.x_offset,
                       foc_resp_gyro_backup.y_offset,
                       foc_resp_gyro_backup.z_offset);

                /* wait for load gyro foc ready*/
                coines_delay_msec(10);
            }

            if (counter == 2)
            {
                printf("\r\nThe program will be exiting now ...\r\n");
                break;
            }
        }

        /* register meta event */
        rslt = bhi385_register_fifo_parse_callback(BHI385_SYS_ID_META_EVENT, parse_meta_event, (void*)&accuracy, &bhy);
        print_api_error(rslt, &bhy);
        rslt =
            bhi385_register_fifo_parse_callback(BHI385_SYS_ID_META_EVENT_WU, parse_meta_event, (void*)&accuracy, &bhy);
        print_api_error(rslt, &bhy);

        /* register sensor callback */
        rslt = bhi385_register_fifo_parse_callback(BHI385_SENSOR_ID_ACC_PASS, parse_3axis_s16, (void*)&accuracy, &bhy);
        print_api_error(rslt, &bhy);
        rslt = bhi385_register_fifo_parse_callback(BHI385_SENSOR_ID_GYRO_PASS, parse_3axis_s16, (void*)&accuracy, &bhy);
        print_api_error(rslt, &bhy);

        /* process fifo */
        rslt = bhi385_get_and_process_fifo(work_buffer, WORK_BUFFER_SIZE, &bhy);
        print_api_error(rslt, &bhy);

        /* Update the callback table to enable parsing of sensor data */
        rslt = bhi385_update_virtual_sensor_list(&bhy);
        print_api_error(rslt, &bhy);

        sensor_conf.sample_rate = 25.0f;
        sensor_conf.latency = 0;

        rslt = bhi385_virtual_sensor_conf_param_set_cfg(BHI385_SENSOR_ID_ACC_PASS, &sensor_conf, &bhy);
        print_api_error(rslt, &bhy);
        rslt = bhi385_virtual_sensor_conf_param_set_cfg(BHI385_SENSOR_ID_GYRO_PASS, &sensor_conf, &bhy);
        print_api_error(rslt, &bhy);

        while (rslt == BHI385_OK)
        {
            if (loop >= limit)
            {
                flag = false;
                break;
            }

            if (get_interrupt_status())
            {
                /* Data from the FIFO is read and the relevant callbacks if registered are called */
                rslt = bhi385_get_and_process_fifo(work_buffer, WORK_BUFFER_SIZE, &bhy);
                loop++;
                print_api_error(rslt, &bhy);
            }
        }
    }

    close_interfaces(intf);

    return rslt;
}

/*!
 * @brief To perform foc and crt
 */
static int8_t bhi3_perform_foc_crt(struct bhi385_foc_resp *acc_foc_status,
                                   struct bhi385_foc_resp *gyro_foc_status,
                                   struct bhi385_dev *dev,
                                   uint8_t flag)
{
    int8_t rslt = BHI385_OK;

    FILE *fp = fopen(CALIB_SEQ_FILE_NAME, "w");

    if (fp == NULL)
    {
        printf("Error to open file\r\n");

        return -1;
    }

    /* start acc foc process */
    if ((flag & 0x01) || flag == 0)
    {
        rslt = bhi385_perform_foc(BHI385_PHYS_SENSOR_ID_ACCELEROMETER, acc_foc_status, dev);
        if (acc_foc_status->foc_status == SENSOR_FOC_FAIL)
        {
            printf("Acc foc process error, please make sure your device is available\r\n");
            rslt = SENSOR_FOC_FAIL;

            return rslt;
        }
        else if (acc_foc_status->foc_status == SENSOR_FOC_SUCCESS)
        {
            foc_resp_acc_backup.x_offset = acc_foc_status->x_offset;
            foc_resp_acc_backup.y_offset = acc_foc_status->y_offset;
            foc_resp_acc_backup.z_offset = acc_foc_status->z_offset;
            printf("Accel Status: %u\n", acc_foc_status->foc_status);
            printf("Accel FOC offsets: x %d, y %d, z %d\n",
                   acc_foc_status->x_offset,
                   acc_foc_status->y_offset,
                   acc_foc_status->z_offset);
            fprintf(fp,
                    "Latest Accel FOC offsets: x %d, y %d, z %d\r\n",
                    acc_foc_status->x_offset,
                    acc_foc_status->y_offset,
                    acc_foc_status->z_offset);
            fflush(stdout);
        }
    }

    /* start  gyro crt process */
    if ((flag & 0x02) || flag == 0)
    {

        rslt = bhi385_phy_sensor_ctrl_param_gyro_start_comp_retrim(dev);
        rslt = bhi385_phy_sensor_ctrl_param_gyro_get_crt_data(&data_crt, dev);
        if (rslt != BHI385_OK)
        {
            printf("CRT failed!\r\n");
            close_interfaces(intf);

            return 0;
        }

        if (data_crt.status == SENSOR_CRT_FAIL)
        {
            printf("CRT process error, please make sure your device is available\r\n");
            rslt = SENSOR_CRT_FAIL;

            return rslt;
        }
        else if (data_crt.status == SENSOR_CRT_SUCCESS)
        {
            printf("GYRO CRT status %d,  value x %d y %d z %d\r\n", data_crt.status, data_crt.x, data_crt.y,
                   data_crt.z);
            fprintf(fp,
                    "Latest GYRO CRT status %d, value: x %d, y %d, z %d\r\n",
                    data_crt.status,
                    data_crt.x,
                    data_crt.y,
                    data_crt.z);
            fflush(stdout);
        }
    }

    /* start  gyro foc process */
    if ((flag & 0x04) || flag == 0)
    {
        rslt = bhi385_perform_foc(BHI385_PHYS_SENSOR_ID_GYROSCOPE, gyro_foc_status, dev);
        if (gyro_foc_status->foc_status == SENSOR_FOC_FAIL)
        {
            printf("Gyro foc process error, please make sure your device is available\r\n");
            rslt = SENSOR_FOC_FAIL;

            return rslt;
        }
        else if (gyro_foc_status->foc_status == SENSOR_FOC_SUCCESS)
        {
            printf("GYRO Status: %u\n", gyro_foc_status->foc_status);
            printf("GYRO FOC offsets: x %d, y %d, z %d\n",
                   gyro_foc_status->x_offset,
                   gyro_foc_status->y_offset,
                   gyro_foc_status->z_offset);
            fprintf(fp,
                    "Latest GYRO FOC offsets: x %d, y %d, z %d",
                    gyro_foc_status->x_offset,
                    gyro_foc_status->y_offset,
                    gyro_foc_status->z_offset);
            fflush(stdout);
        }
    }

    fclose(fp);

    return rslt;
}

static void time_to_s_ns(uint64_t time_ticks, uint32_t *s, uint32_t *ns, uint64_t *tns)
{
    *tns = time_ticks * 15625; /* timestamp is now in nanoseconds */
    *s = (uint32_t)(*tns / UINT64_C(1000000000));
    *ns = (uint32_t)(*tns - ((*s) * UINT64_C(1000000000)));
}

static void parse_3axis_s16(const struct bhi385_fifo_parse_data_info *callback_info, void *callback_ref)
{
    struct bhi385_event_data_xyz data;
    uint32_t s, ns;
    uint64_t tns;
    float scaling_factor = 0.0f;

    if (callback_info->sensor_id >= BHI385_SENSOR_ID_ACC_PASS &&
        callback_info->sensor_id <= BHI385_SENSOR_ID_ACC_RAW_WU)
    {
        scaling_factor = 1.0f / 4096.0f;
    }
    else if (callback_info->sensor_id >= BHI385_SENSOR_ID_GYRO_PASS &&
             callback_info->sensor_id <= BHI385_SENSOR_ID_GYRO_RAW_WU)
    {
        scaling_factor = 2000.0f / 32768.0f;
    }

    uint8_t *accuracy = (uint8_t*)callback_ref;

    bhi385_event_data_parse_xyz(callback_info->data_ptr, &data);

    time_to_s_ns(*callback_info->time_stamp, &s, &ns, &tns);

    printf("SID: %u; T: %u.%09u; x: %f, y: %f, z: %f; acc: %u\r\n",
           callback_info->sensor_id,
           s,
           ns,
           data.x * scaling_factor,
           data.y * scaling_factor,
           data.z * scaling_factor,
           *accuracy);
}

static void parse_meta_event(const struct bhi385_fifo_parse_data_info *callback_info, void *callback_ref)
{
    (void)callback_ref;
    uint8_t meta_event_type = callback_info->data_ptr[0];
    uint8_t byte1 = callback_info->data_ptr[1];
    uint8_t byte2 = callback_info->data_ptr[2];
    char *event_text;
    uint8_t *accuracy = (uint8_t*)callback_ref;

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
            close_interfaces(BHI385_SPI_INTERFACE);
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

static int8_t upload_firmware(struct bhi385_dev *dev)
{
    uint32_t incr = 256; /* Max command packet size */
    uint32_t len = sizeof(bhi385_firmware_image);
    int8_t rslt = BHI385_OK;

    if ((incr % 4) != 0) /* Round off to higher 4 bytes */
    {
        incr = ((incr >> 2) + 1) << 2;
    }

    for (uint32_t i = 0; (i < len) && (rslt == BHI385_OK); i += incr)
    {
        if (incr > (len - i)) /* If last payload */
        {
            incr = len - i;
            if ((incr % 4) != 0) /* Round off to higher 4 bytes */
            {
                incr = ((incr >> 2) + 1) << 2;
            }
        }

        rslt = bhi385_upload_firmware_to_ram_partly(&bhi385_firmware_image[i], len, i, incr, dev);

        printf("%.2f%% complete\r", (float)(i + incr) / (float)len * 100.0f);
    }

    printf("\n");

    return rslt;
}

static int16_t convert_char_int(char *line, char *pattern1, char *pattern2)
{
    char target[16] = { '\0' };
    char *start, *end;

    start = strstr(line, pattern1);
    if (start)
    {
        start += strlen(pattern1);
        if ((end = strstr(start, pattern2)) || pattern2 == NULL)
        {
            if (strcmp(pattern2, "") == 0)
            {
                memcpy(target, start, strlen(start));
                target[strlen(start)] = '\0';
            }
            else
            {
                memcpy(target, start, end - start);
                target[end - start] = '\0';
            }
        }
    }

    if (strcmp(target, "") != 0)
    {
        return (int16_t)atoi(target);
    }

    return INT16_MAX;
}