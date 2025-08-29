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
 * @file    rw_calib_profile.c
 * @brief   rw calib profile example for the BHI385
 *
 */

#include <stdlib.h>
#include <stdio.h>
#include <stdbool.h>
#include <inttypes.h>

#include "bhi385.h"
#include "bhi385_parse.h"
#include "common.h"
#include "bhi385_virtual_sensor_conf_param.h"
#include "bhi385_event_data.h"
#include "bhi385_bsx_algo_param.h"

#include "bhi385/Bosch_Shuttle3_BHI385_bsxsam_lite.fw.h"

#define WORK_BUFFER_SIZE         2048

#define CALIB_ACC_FILE_NAME      "calib_acc.bin"
#define CALIB_GYRO_FILE_NAME     "calib_gyro.bin"

#define ACC_ACCURACY_INDEX       UINT8_C(0)
#define GYRO_ACCURACY_INDEX      UINT8_C(1)

#define OPEN_FILE_ERR            INT8_C(-1)
#define READ_FILE_ERR            INT8_C(-2)
#define WRITE_FILE_ERR           INT8_C(-2)
#define FILE_TOO_LARGE_ERR       INT8_C(-3)
#define READ_WRITE_SUCCESS       UINT8_C(0)

#define BASE_ADDR                UINT16_C(0x200)
#define MAX_LENGTH               UINT32_C(2048)

#define NEED_UPDATE_CALIBRATION  UINT8_C(1)
#define UPDATE_CALIBRATION_DONE  UINT8_C(0)

#define CALIBRATION_COMPLETED    UINT8_C(3)
#define MAX_SENSOR_EXECUTE       UINT8_C(1)

static void parse_accuracy(const struct bhi385_fifo_parse_data_info *callback_info, void *callback_ref);
static void parse_meta_event_calib(const struct bhi385_fifo_parse_data_info *callback_info, void *callback_ref);
static void read_calib_from_file_and_set_to_bhi(uint8_t phy_sensor_id);
static void print_api_error(int8_t rslt, struct bhi385_dev *dev);
static void upload_firmware(uint8_t boot_stat, struct bhi385_dev *dev);
static int8_t write_calib_file(const char* filename, const uint8_t * calib_prof, uint32_t actual_len);
static int8_t read_calib_file(const char* filename, uint8_t* calib_prof, uint32_t max_len, uint32_t* actual_len);

enum bhi385_intf intf;
static struct bhi385_dev bhy;
uint8_t offset_update_flag[2] = { 0 };
uint8_t loop;

int main(void)
{
    uint8_t chip_id = 0;
    uint16_t version = 0;
    int8_t rslt;
    uint8_t work_buffer[WORK_BUFFER_SIZE];
    uint8_t hintr_ctrl, hif_ctrl, boot_status;
    struct bhi385_virtual_sensor_conf_param_conf sensor_conf = { 0 };
    uint8_t accuracy[2];

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

    /* Check the interrupt pin and FIFO configurations. Disable status and debug */
    hintr_ctrl = BHI385_ICTL_DISABLE_STATUS_FIFO | BHI385_ICTL_DISABLE_DEBUG;

    rslt = bhi385_set_host_interrupt_ctrl(hintr_ctrl, &bhy);
    print_api_error(rslt, &bhy);
    rslt = bhi385_get_host_interrupt_ctrl(&hintr_ctrl, &bhy);
    print_api_error(rslt, &bhy);

    printf("Host interrupt control\r\n");
    printf("    Wake up FIFO %s.\r\n", (hintr_ctrl & BHI385_ICTL_DISABLE_FIFO_W) ? "disabled" : "enabled");
    printf("    Non wake up FIFO %s.\r\n", (hintr_ctrl & BHI385_ICTL_DISABLE_FIFO_NW) ? "disabled" : "enabled");
    printf("    Status FIFO %s.\r\n", (hintr_ctrl & BHI385_ICTL_DISABLE_STATUS_FIFO) ? "disabled" : "enabled");
    printf("    Debugging %s.\r\n", (hintr_ctrl & BHI385_ICTL_DISABLE_DEBUG) ? "disabled" : "enabled");
    printf("    Fault %s.\r\n", (hintr_ctrl & BHI385_ICTL_DISABLE_FAULT) ? "disabled" : "enabled");
    printf("    Interrupt is %s.\r\n", (hintr_ctrl & BHI385_ICTL_ACTIVE_LOW) ? "active low" : "active high");
    printf("    Interrupt is %s triggered.\r\n", (hintr_ctrl & BHI385_ICTL_EDGE) ? "pulse" : "level");
    printf("    Interrupt pin drive is %s.\r\n", (hintr_ctrl & BHI385_ICTL_OPEN_DRAIN) ? "open drain" : "push-pull");

    /* Configure the host interface */
    hif_ctrl = 0;
    rslt = bhi385_set_host_intf_ctrl(hif_ctrl, &bhy);
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

        rslt = bhi385_register_fifo_parse_callback(BHI385_SYS_ID_META_EVENT,
                                                   parse_meta_event_calib,
                                                   (void*)accuracy,
                                                   &bhy);
        print_api_error(rslt, &bhy);
        rslt = bhi385_register_fifo_parse_callback(BHI385_SYS_ID_META_EVENT_WU,
                                                   parse_meta_event_calib,
                                                   (void*)accuracy,
                                                   &bhy);
        print_api_error(rslt, &bhy);
        rslt = bhi385_register_fifo_parse_callback(BHI385_SENSOR_ID_ACC, parse_accuracy, (void*)accuracy, &bhy);
        print_api_error(rslt, &bhy);
        rslt = bhi385_register_fifo_parse_callback(BHI385_SENSOR_ID_GYRO, parse_accuracy, (void*)accuracy, &bhy);
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

    read_calib_from_file_and_set_to_bhi(BHI385_PHYS_SENSOR_ID_ACCELEROMETER);
    read_calib_from_file_and_set_to_bhi(BHI385_PHYS_SENSOR_ID_GYROSCOPE);

    /* Read out data measured at 100Hz */
    sensor_conf.sample_rate = 100.0f; /* Read out data measured at 100Hz */
    sensor_conf.latency = 0; /* Report immediately */
    rslt = bhi385_virtual_sensor_conf_param_set_cfg(BHI385_SENSOR_ID_ACC, &sensor_conf, &bhy);
    print_api_error(rslt, &bhy);
    printf("Enable %s at %.2fHz.\r\n", get_sensor_name(BHI385_SENSOR_ID_ACC), sensor_conf.sample_rate);

    rslt = bhi385_virtual_sensor_conf_param_set_cfg(BHI385_SENSOR_ID_GYRO, &sensor_conf, &bhy);
    print_api_error(rslt, &bhy);
    printf("Enable %s at %.2fHz.\r\n", get_sensor_name(BHI385_SENSOR_ID_GYRO), sensor_conf.sample_rate);

    while (rslt == BHI385_OK)
    {
        if (get_interrupt_status())
        {
            /* Data from the FIFO is read and the relevant callbacks if registered are called */
            rslt = bhi385_get_and_process_fifo(work_buffer, WORK_BUFFER_SIZE, &bhy);
            print_api_error(rslt, &bhy);
            if (loop == MAX_SENSOR_EXECUTE)
            {
                printf("The program will be exiting now ...\r\n");
                break;
            }
        }
    }

    close_interfaces(intf);

    return rslt;
}

/**
 * @brief Helper functions to write calibration profiles to files.
 *
 * @param filename File to write to.
 * @param calib_prof Calibration profile data for write.
 * @param actual_len Length of the data for write
 *
 * @return int Returns an integer indicating success or type of failure.
 *
 */
static int8_t write_calib_file(const char* filename, const uint8_t * calib_prof, uint32_t actual_len)
{
    FILE* file = fopen(filename, "wb");

    if (file == NULL)
    {
        printf("no file\r\n");

        return OPEN_FILE_ERR; /* Error opening file */
    }

    /* Write the calibration profile data to the file */
    size_t written = fwrite(calib_prof, sizeof(uint8_t), actual_len, file);
    fclose(file);

    if (written != actual_len)
    {
        return WRITE_FILE_ERR; /* Error writing to file */
    }

    return READ_WRITE_SUCCESS; /* Success */
}

/**
 * @brief Helper functions toread calibration profiles from files.
 *
 * @param filename File to read from.
 * @param calib_prof Calibration profile buffer to store data for read.
 * @param actual_len maximum length to read , pointer to store the actual length read.
 *
 * @return int Returns an integer indicating success or type of failure.
 *
 * @note For reading, the `calib_prof` and `actual_len` are updated with the file contents and its length, respectively.
 */
static int8_t read_calib_file(const char* filename, uint8_t* calib_prof, uint32_t max_len, uint32_t* actual_len)
{
    FILE* file = fopen(filename, "rb");

    if (file == NULL)
    {
        return OPEN_FILE_ERR; /* Error opening file */
    }

    fseek(file, 0, SEEK_END);
    uint32_t file_size = ftell(file);
    fseek(file, 0, SEEK_SET);

    if (file_size > max_len)
    {
        fclose(file);

        return FILE_TOO_LARGE_ERR; /* File contents too large */
    }

    /* Read the calibration profile data from the file */
    size_t read = fread(calib_prof, sizeof(uint8_t), file_size, file);
    fclose(file);

    if (read != file_size)
    {
        return READ_FILE_ERR; /* Error reading from file */
    }

    *actual_len = file_size; /* Set the actual length read */
    return READ_WRITE_SUCCESS; /* Success */
}

/**
 * @brief Reads calibration data from a file for a specified physical sensor.
 *
 * @param phy_sensor_id ID of the physical sensor (accelerometer, gyroscope).
 *
 * @note This function reads calibration data into a global buffer and updates the sensor calibration profile in the BHI2/3 device.
 */
static void read_calib_from_file_and_set_to_bhi(uint8_t phy_sensor_id)
{
    int8_t rslt = 0;
    uint16_t param_id = BASE_ADDR | phy_sensor_id;
    uint8_t calib_prof[WORK_BUFFER_SIZE];
    uint32_t actual_len = 0;
    bhi385_bsx_algo_param_state_exg bsx_state_exg[BHI385_BSX_STATE_MAX_BLOCKS] = { { 0 } };

    printf("%s for sensor id %d\r\n", __func__, phy_sensor_id);
    switch (phy_sensor_id)
    {
        case BHI385_PHYS_SENSOR_ID_ACCELEROMETER:

            /* Read accelerometer calibration profile from file */
            rslt = read_calib_file(CALIB_ACC_FILE_NAME, calib_prof, MAX_LENGTH, &actual_len);
            break;
        case BHI385_PHYS_SENSOR_ID_GYROSCOPE:

            /* Read gyroscope calibration profile from file */
            rslt = read_calib_file(CALIB_GYRO_FILE_NAME, calib_prof, MAX_LENGTH, &actual_len);
            break;
        default:
            break;
    }

    if (rslt == BHI385_OK)
    {
        uint16_t num = 0;
        uint32_t state_buf_size;

        /* Initialize the BSX state structure */
        memset(bsx_state_exg, 0, sizeof(bsx_state_exg));

        /* Traverse each block in the BSX states until finished */
        for (uint32_t pos = 0; pos < actual_len; pos += BHI385_BSX_STATE_BLOCK_LEN, num++)
        {
            bsx_state_exg[num].block_info = num & BHI385_BSX_STATE_BLOCK_NUMBER;
            state_buf_size = actual_len - pos;
            if (state_buf_size <= BHI385_BSX_STATE_BLOCK_LEN)
            {
                bsx_state_exg[num].block_info |= BHI385_BSX_STATE_TRANSFER_COMPLETE;
                bsx_state_exg[num].block_len = (uint8_t)state_buf_size;
            }
            else
            {
                bsx_state_exg[num].block_len = BHI385_BSX_STATE_BLOCK_LEN;
            }

            bsx_state_exg[num].struct_len = actual_len;

            /* Copy the calibration profile data into the BSX state structure */
            memcpy(&(bsx_state_exg[num].state_data[0]), &calib_prof[pos], bsx_state_exg[num].block_len);
        }

        /* Set bsx states for physical sensors*/
        rslt = bhi385_bsx_algo_param_set_bsx_states(param_id, bsx_state_exg, &bhy);

        if (rslt == BHI385_OK)
        {
            printf("Resume calibration to BSX Done with sensor id %d.\r\n", phy_sensor_id);
        }
        else
        {
            print_api_error(rslt, &bhy);
        }
    }
}

/**
 * @brief Writes calibration data to a file for a specified physical sensor if certain conditions are met.
 *
 * @param accuracy Accuracy level of the sensor data.
 * @param phy_sensor_id ID of the physical sensor.
 *
 * @note This function writes the calibration profile to a file if the accuracy is 3 and an update is required.
 */
static void get_calib_from_bhi_and_write_to_file(uint8_t accuracy, uint8_t phy_sensor_id)
{
    int8_t rslt = BHI385_OK;
    uint8_t *p_wr_acc = NULL;
    uint16_t param_id = BASE_ADDR | phy_sensor_id;
    uint32_t actual_len = 0;
    bhi385_bsx_algo_param_state_exg bsx_state_exg[BHI385_BSX_STATE_MAX_BLOCKS] = { { 0 } };
    uint8_t calib_prof[WORK_BUFFER_SIZE];

    /* the following switch code block indicates which physical sensor need to update the profile */
    switch (phy_sensor_id)
    {
        case BHI385_PHYS_SENSOR_ID_ACCELEROMETER:
            p_wr_acc = &offset_update_flag[ACC_ACCURACY_INDEX];
            break;
        case BHI385_PHYS_SENSOR_ID_GYROSCOPE:
            p_wr_acc = &offset_update_flag[GYRO_ACCURACY_INDEX];
            break;

        default:
            break;
    }

    if (accuracy == CALIBRATION_COMPLETED && *p_wr_acc == NEED_UPDATE_CALIBRATION)
    {
        printf("Calibration done for sensor with id %d.\r\n", phy_sensor_id);

        /* Get bsx states for BSX fusion parameters*/
        rslt = bhi385_bsx_algo_param_get_bsx_states(param_id,
                                                    bsx_state_exg,
                                                    sizeof(bsx_state_exg) * 20,
                                                    &actual_len,
                                                    &bhy);
        if (rslt == BHI385_OK)
        {
            printf("Get BSX Calibration profile\r\n");
            for (uint8_t block = 0; block < BHI385_BSX_STATE_MAX_BLOCKS; block++)
            {
                bhi385_bsx_algo_param_state_exg *current_block = &bsx_state_exg[block];

                printf("Block number: %u\r\n", current_block->block_info & BHI385_BSX_STATE_BLOCK_NUMBER);
                current_block->block_info & BHI385_BSX_STATE_TRANSFER_COMPLETE ? printf("Transfer complete\r\n") :
                printf("Transfer not complete\r\n");
                printf("Block length: %u\r\n", current_block->block_len);
                printf("Struct length: %u\r\n", current_block->struct_len);

                printf("-------------------------------------------\r\n");
                printf("Block data \r\n");
                printf("Byte hex      dec | Data\r\n");
                printf("-------------------------------------------\r\n");

                for (uint8_t i = 0; i < current_block->block_len; i++)
                {
                    if (i % 8 == 0)
                    {
                        printf("0x%06x %8d |", i, i);
                    }

                    printf("%02x ", current_block->state_data[i]);

                    if ((i + 1) % 8 == 0 || i == current_block->block_len - 1)
                    {
                        printf("\r\n");
                    }
                }

                printf("\r\n\r\n");

                /* Copy the calibration profile data into the buffer */
                memcpy(calib_prof + block * 64, &(bsx_state_exg[block].state_data[0]), 64);

                if ((current_block->block_len == 0) ||
                    (current_block->block_info & BHI385_BSX_STATE_TRANSFER_COMPLETE) != 0)
                {
                    break;
                }
            }

            switch (phy_sensor_id)
            {
                case BHI385_PHYS_SENSOR_ID_ACCELEROMETER:

                    /* Write accelerometer calibration profile to file*/
                    rslt = write_calib_file(CALIB_ACC_FILE_NAME, calib_prof, actual_len);
                    break;
                case BHI385_PHYS_SENSOR_ID_GYROSCOPE:

                    /* Write gyroscope calibration profile to file*/
                    rslt = write_calib_file(CALIB_GYRO_FILE_NAME, calib_prof, actual_len);
                    break;
                default:
                    break;
            }

            printf("%s status:  %d\r\n", __func__, rslt);
            if (rslt == BHI385_OK)
            {
                *p_wr_acc = UPDATE_CALIBRATION_DONE;
                loop++;
            }
        }
    }
}

/**
 * @brief Parses and processes sensor data for accuracy, and triggers calibration file update if necessary.
 *
 * @param callback_info Information about the FIFO parse data.
 * @param callback_ref Reference data passed to the callback (in this case, accuracy data).
 *
 * @note This function prints sensor data and updates calibration files if needed.
 */
static void parse_accuracy(const struct bhi385_fifo_parse_data_info *callback_info, void *callback_ref)
{
    (void)callback_ref;
    struct bhi385_event_data_xyz data;
    uint8_t *accuracy = (uint8_t*)callback_ref;
    if (callback_info->data_size != 7) /* Check for a valid payload size. Includes sensor ID */
    {
        return;
    }

    bhi385_event_data_parse_xyz(callback_info->data_ptr, &data);

    if (accuracy)
    {
        switch (callback_info->sensor_id)
        {
            case BHI385_SENSOR_ID_ACC:
            case BHI385_SENSOR_ID_ACC_BIAS:
                if (accuracy[ACC_ACCURACY_INDEX] == CALIBRATION_COMPLETED)
                {
                    get_calib_from_bhi_and_write_to_file(accuracy[ACC_ACCURACY_INDEX],
                                                         BHI385_PHYS_SENSOR_ID_ACCELEROMETER);
                }

                break;
            case BHI385_SENSOR_ID_GYRO:
            case BHI385_SENSOR_ID_GYRO_BIAS:
                if (accuracy[GYRO_ACCURACY_INDEX] == CALIBRATION_COMPLETED)
                {
                    get_calib_from_bhi_and_write_to_file(accuracy[GYRO_ACCURACY_INDEX],
                                                         BHI385_PHYS_SENSOR_ID_GYROSCOPE);
                }

                break;
            default:
                break;
        }

    }
}

static void parse_meta_event_calib(const struct bhi385_fifo_parse_data_info *callback_info, void *callback_ref)
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
                uint8_t sensor_id = byte1;
                switch (sensor_id)
                {
                    case BHI385_SENSOR_ID_ACC_BIAS:
                    case BHI385_SENSOR_ID_ACC:
                        accuracy[ACC_ACCURACY_INDEX] = byte2;
                        if (accuracy[ACC_ACCURACY_INDEX] == 3)
                        {
                            offset_update_flag[ACC_ACCURACY_INDEX] = 1;
                        }

                        break;
                    case BHI385_SENSOR_ID_GYRO_BIAS:
                    case BHI385_SENSOR_ID_GYRO:
                        accuracy[GYRO_ACCURACY_INDEX] = byte2;
                        if (accuracy[GYRO_ACCURACY_INDEX] == 3)
                        {
                            offset_update_flag[GYRO_ACCURACY_INDEX] = 1;
                        }

                        break;
                    default:
                        break;
                }
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