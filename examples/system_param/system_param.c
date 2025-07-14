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
 * @file    system_param.c
 * @brief   System parameters example
 *
 */

#include <stdlib.h>
#include <stdio.h>
#include <stdbool.h>
#include <inttypes.h>

#include "bhi385.h"
#include "common.h"
#include "bhi385_system_param.h"

#include "bhi385/Bosch_Shuttle3_BHI385_bsxsam_lite.fw.h"

/*!
 * @brief printfing the error codes that generated from the API
 * @param[in] callback_info fifo data available here
 * @param[in] callback_ref
 *
 * @return  void
 *
 */
static void print_api_error(int8_t rlst, struct bhi385_dev *dev);

static void get_api_name(unsigned int line, const char *func, int8_t val);

/*! File pointer to download firmware to BHI3 */
static void upload_firmware(uint8_t boot_stat, struct bhi385_dev *dev);

/*! Device structure */
struct bhi385_dev bhy;

enum bhi385_intf intf;

struct bhi385_system_param_phys_sensor_info psi;

struct bhi385_system_param_timestamp ts;

struct bhi385_system_param_firmware_version fw_ver;

struct bhi385_system_param_fifo_control fifo_ctrl;

bhi385_system_param_multi_meta_event_ctrl_t meta_event;

static int8_t assert_rslt;

#define BHI385_ASSERT(x)  assert_rslt = x; if (assert_rslt) { get_api_name(__LINE__, __FUNCTION__, assert_rslt); }

int main(void)
{
    uint8_t chip_id = 0;
    uint16_t version = 0;
    int8_t rslt;

    uint8_t hintr_ctrl, hif_ctrl, boot_status;

    /*! Selecting the SPI interface for sensor communication */
#ifdef BHI385_USE_I2C
    intf = BHI385_I2C_INTERFACE;
#else
    intf = BHI385_SPI_INTERFACE;
#endif

    setup_interfaces(true, intf); /* Perform a power on reset */

#ifdef BHI385_USE_I2C
    BHI385_ASSERT(bhi385_init(BHI385_I2C_INTERFACE, bhi385_i2c_read, bhi385_i2c_write, bhi385_delay_us,
                              BHI385_RD_WR_LEN, NULL, &bhy));
#else
    BHI385_ASSERT(bhi385_init(BHI385_SPI_INTERFACE, bhi385_spi_read, bhi385_spi_write, bhi385_delay_us,
                              BHI385_RD_WR_LEN, NULL, &bhy));
#endif
    print_api_error(assert_rslt, &bhy);

    BHI385_ASSERT(bhi385_soft_reset(&bhy));
    print_api_error(assert_rslt, &bhy);

    BHI385_ASSERT(bhi385_get_chip_id(&chip_id, &bhy));
    print_api_error(assert_rslt, &bhy);

    /* Check for a valid Chip ID */
    if (chip_id == BHI385_CHIP_ID)
    {
        printf("Chip ID read 0x%X\r\n", chip_id);
    }
    else
    {
        printf("Device not found. Chip ID read 0x%X\r\n", chip_id);
    }

    /*! Check the interrupt pin and FIFO configurations. Disable status and debug */
    hintr_ctrl = BHI385_ICTL_DISABLE_STATUS_FIFO | BHI385_ICTL_DISABLE_DEBUG;

    BHI385_ASSERT(bhi385_set_host_interrupt_ctrl(hintr_ctrl, &bhy));
    print_api_error(assert_rslt, &bhy);
    BHI385_ASSERT(bhi385_get_host_interrupt_ctrl(&hintr_ctrl, &bhy));
    print_api_error(assert_rslt, &bhy);

    printf("Host interrupt control\r\n");
    printf("    Wake up FIFO %s.\r\n", (hintr_ctrl & BHI385_ICTL_DISABLE_FIFO_W) ? "disabled" : "enabled");
    printf("    Non wake up FIFO %s.\r\n", (hintr_ctrl & BHI385_ICTL_DISABLE_FIFO_NW) ? "disabled" : "enabled");
    printf("    Status FIFO %s.\r\n", (hintr_ctrl & BHI385_ICTL_DISABLE_STATUS_FIFO) ? "disabled" : "enabled");
    printf("    Debugging %s.\r\n", (hintr_ctrl & BHI385_ICTL_DISABLE_DEBUG) ? "disabled" : "enabled");
    printf("    Fault %s.\r\n", (hintr_ctrl & BHI385_ICTL_DISABLE_FAULT) ? "disabled" : "enabled");
    printf("    Interrupt is %s.\r\n", (hintr_ctrl & BHI385_ICTL_ACTIVE_LOW) ? "active low" : "active high");
    printf("    Interrupt is %s triggered.\r\n", (hintr_ctrl & BHI385_ICTL_EDGE) ? "pulse" : "level");
    printf("    Interrupt pin drive is %s.\r\n", (hintr_ctrl & BHI385_ICTL_OPEN_DRAIN) ? "open drain" : "push-pull");

    /*! Configure the host interface */
    hif_ctrl = 0;
    BHI385_ASSERT(bhi385_set_host_intf_ctrl(hif_ctrl, &bhy));
    print_api_error(assert_rslt, &bhy);

    /*! Check if the sensor is ready to load firmware */
    BHI385_ASSERT(bhi385_get_boot_status(&boot_status, &bhy));
    print_api_error(assert_rslt, &bhy);

    if (boot_status & BHI385_BST_HOST_INTERFACE_READY)
    {
        upload_firmware(boot_status, &bhy);

        BHI385_ASSERT(bhi385_get_kernel_version(&version, &bhy));
        print_api_error(assert_rslt, &bhy);
        if ((assert_rslt == BHI385_OK) && (version != 0))
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

    struct bhi385_system_param_orient_matrix ort_mtx = { { 0 } };
    BHI385_ASSERT(bhi385_system_param_get_physical_sensor_info(BHI385_PHYS_SENSOR_ID_ACCELEROMETER, &psi, &bhy));
    print_api_error(assert_rslt, &bhy);

    if (assert_rslt == BHI385_OK)
    {
        printf("\r\n");
        printf("Field Name            hex                    | Value (dec)\r\n");
        printf("----------------------------------------------------------\r\n");
        printf("Physical Sensor ID    %02X                     | %d\r\n", psi.sensor_type, psi.sensor_type);
        printf("Driver ID             %02X                     | %d\r\n", psi.driver_id, psi.driver_id);
        printf("Driver Version        %02X                     | %d\r\n", psi.driver_version, psi.driver_version);
        printf("Current Consumption   %02X                     | %0.3fmA\r\n",
               psi.power_current,
               psi.power_current / 10.f);
        printf("Dynamic Range         %04X                   | %d\r\n", psi.curr_range.u16_val, psi.curr_range.u16_val);

        const char *irq_status[2] = { "Disabled", "Enabled" };
        const char *master_intf[5] = { "None", "SPI0", "I2C0", "SPI1", "I2C1" };
        const char *power_mode[8] =
        { "Sensor Not Present", "Power Down", "Suspend", "Self-Test", "Interrupt Motion", "One Shot",
          "Low Power Active", "Active" };

        printf("Flags                 %02X                     | IRQ status       : %s\r\n", psi.flags,
               irq_status[psi.flags & 0x01]);
        printf("                                             | Master interface : %s\r\n",
               master_intf[(psi.flags >> 1) & 0x0F]);
        printf("                                             | Power mode       : %s\r\n",
               power_mode[(psi.flags >> 5) & 0x07]);
        printf("Slave Address         %02X                     | %d\r\n", psi.slave_address, psi.slave_address);
        printf("GPIO Assignment       %02X                     | %d\r\n", psi.gpio_assignment, psi.gpio_assignment);

        /*lint -e10 Error 10: Lint does not understand PRIxxx */
        printf("Current Rate          %08" PRIX32 "               | %.3fHz\r\n",
               psi.curr_rate.u32_val,
               psi.curr_rate.f_val);

        /*lint +e10 */
        printf("Number of axes        %02X                     | %d\r\n", psi.num_axis, psi.num_axis);

        #define INT4_TO_INT8(INT4)  ((int8_t)(((INT4) > 1) ? -1 : (INT4)))

        ort_mtx.c[0] = INT4_TO_INT8(psi.orientation_matrix[0] & 0x0F);
        ort_mtx.c[1] = INT4_TO_INT8(psi.orientation_matrix[0] >> 8);
        ort_mtx.c[2] = INT4_TO_INT8(psi.orientation_matrix[1] & 0x0F);
        ort_mtx.c[3] = INT4_TO_INT8(psi.orientation_matrix[1] >> 8);
        ort_mtx.c[4] = INT4_TO_INT8(psi.orientation_matrix[2] & 0x0F);
        ort_mtx.c[5] = INT4_TO_INT8(psi.orientation_matrix[2] >> 8);
        ort_mtx.c[6] = INT4_TO_INT8(psi.orientation_matrix[3] & 0x0F);
        ort_mtx.c[7] = INT4_TO_INT8(psi.orientation_matrix[3] >> 8);
        ort_mtx.c[8] = INT4_TO_INT8(psi.orientation_matrix[4] & 0x0F);

        printf("Orientation Matrix    %02X%02X%02X%02X%02X             | %+02d %+02d %+02d |\r\n",
               psi.orientation_matrix[0],
               psi.orientation_matrix[1],
               psi.orientation_matrix[2],
               psi.orientation_matrix[3],
               psi.orientation_matrix[4],
               ort_mtx.c[0],
               ort_mtx.c[1],
               ort_mtx.c[2]);
        printf("                                             | %+02d %+02d %+02d |\r\n",
               ort_mtx.c[3],
               ort_mtx.c[4],
               ort_mtx.c[5]);
        printf("                                             | %+02d %+02d %+02d |\r\n",
               ort_mtx.c[6],
               ort_mtx.c[7],
               ort_mtx.c[8]);
        printf("Reserved              %02X                     | %d\r\n", psi.reserved, psi.reserved);
        printf("\r\n");
    }

    ort_mtx.c[0] = 0;

    BHI385_ASSERT(bhi385_system_param_set_physical_sensor_info(BHI385_PHYS_SENSOR_ID_ACCELEROMETER, &ort_mtx, &bhy));
    print_api_error(assert_rslt, &bhy);

    BHI385_ASSERT(bhi385_system_param_get_physical_sensor_info(BHI385_PHYS_SENSOR_ID_ACCELEROMETER, &psi, &bhy));
    print_api_error(assert_rslt, &bhy);

    if (assert_rslt == BHI385_OK)
    {
        ort_mtx.c[0] = INT4_TO_INT8(psi.orientation_matrix[0] & 0x0F);
        printf("ort_mtx.c[0] after changed = %+02d\r\n", ort_mtx.c[0]);
    }

    BHI385_ASSERT(bhi385_system_param_get_virtual_sensor_present(&bhy));
    print_api_error(assert_rslt, &bhy);

    if (assert_rslt == BHI385_OK)
    {
        printf("\r\n");
        printf("Virtual sensor list.\r\n");
        printf("Sensor ID |                          Sensor Name\r\n");
        printf("----------+--------------------------------------|\r\n");

        for (uint8_t i = 0; i < BHI385_SENSOR_ID_MAX; i++)
        {
            if (bhi385_is_sensor_available(i, &bhy))
            {
                if (i < BHI385_SENSOR_ID_CUSTOM_START)
                {
                    printf(" %8u | %36s \r\n", i, get_sensor_name(i));
                }
                else
                {
                    printf(" %8u | Undefined custom sensor\n", i);
                }
            }
        }
    }

    BHI385_ASSERT(bhi385_system_param_get_physical_sensor_present(&bhy));
    print_api_error(assert_rslt, &bhy);

    if (assert_rslt == BHI385_OK)
    {
        printf("\r\n");
        printf("Physical sensor list.\r\n");
        printf("Sensor ID |                          Sensor Name\r\n");
        printf("----------+--------------------------------------|\r\n");
        for (uint8_t i = 0; i < BHI385_PHYSICAL_SENSOR_ID_MAX; i++)
        {
            if (bhi385_is_physical_sensor_available(i, &bhy))
            {
                printf(" %8u | %36s \r\n", i, get_physical_sensor_name(i));
            }
        }
    }

    BHI385_ASSERT(bhi385_system_param_get_timestamps(&ts, &bhy));
    print_api_error(assert_rslt, &bhy);

    if (assert_rslt == BHI385_OK)
    {
        printf("\r\n");
        uint32_t s, ns;
        ts.host_int_ts *= 15625; /* Timestamp is now in nanoseconds */
        s = (uint32_t)(ts.host_int_ts / UINT64_C(1000000000));
        ns = (uint32_t)(ts.host_int_ts - (s * UINT64_C(1000000000)));

        /*lint -e10 Error 10: Lint does not understand PRIxxx */
        printf("Host interrupt timestamp: %" PRIu32 ".%09" PRIu32 "\r\n", s, ns);

        /*lint +e10 */

        ts.cur_ts *= 15625; /* Timestamp is now in nanoseconds */
        s = (uint32_t)(ts.cur_ts / UINT64_C(1000000000));
        ns = (uint32_t)(ts.cur_ts - (s * UINT64_C(1000000000)));

        /*lint -e10 Error 10: Lint does not understand PRIxxx */
        printf("Current timestamp: %" PRIu32 ".%09" PRIu32 "\r\n", s, ns);

        /*lint +e10 */

        ts.event_ts *= 15625; /* Timestamp is now in nanoseconds */
        s = (uint32_t)(ts.event_ts / UINT64_C(1000000000));
        ns = (uint32_t)(ts.event_ts - (s * UINT64_C(1000000000)));

        /*lint -e10 Error 10: Lint does not understand PRIxxx */
        printf("Timestamp event: %" PRIu32 ".%09" PRIu32 "\r\n", s, ns);

        /*lint +e10 */
    }

    BHI385_ASSERT(bhi385_system_param_get_firmware_version(&fw_ver, &bhy));
    print_api_error(assert_rslt, &bhy);

    if (assert_rslt == BHI385_OK)
    {
        printf("\r\n");
        printf("Custom version number: %u\r\n", fw_ver.custom_ver_num);

    #ifdef PC

        /*lint -e10 Error 10: Lint does not understand PRIxxx */
        printf("EM Hash: %" PRIx64 "\r\n", fw_ver.em_hash);
        printf("BST Hash: %" PRIx64 "\r\n", fw_ver.bst_hash);
        printf("User Hash: %" PRIx64 "\r\n", fw_ver.user_hash);

        /*lint +e10 */
    #else
        char temp_em_hash[12 + 1] = { '\0' }, temp1_em_hash[8 + 1] = { '\0' }, temp2_em_hash[4 + 1] = { '\0' };
        sprintf(temp1_em_hash, "%08lx", (uint32_t)((fw_ver.em_hash >> 16) & 0xFFFFFFFF));
        sprintf(temp2_em_hash, "%04x", (uint16_t)(fw_ver.em_hash & 0xFFFF));

        strcat(temp_em_hash, temp1_em_hash);
        strcat(temp_em_hash, temp2_em_hash);
        printf("EM Hash: %s\r\n", temp_em_hash);

        char temp_bst_hash[12 + 1] = { '\0' }, temp1_bst_hash[8 + 1] = { '\0' }, temp2_bst_hash[4 + 1] = { '\0' };
        sprintf(temp1_bst_hash, "%08lx", (uint32_t)((fw_ver.bst_hash >> 16) & 0xFFFFFFFF));
        sprintf(temp2_bst_hash, "%04x", (uint16_t)(fw_ver.bst_hash & 0xFFFF));

        strcat(temp_bst_hash, temp1_bst_hash);
        strcat(temp_bst_hash, temp2_bst_hash);
        printf("BST Hash: %s\r\n", temp_bst_hash);

        char temp_user_hash[12 + 1] = { '\0' }, temp1_user_hash[8 + 1] = { '\0' }, temp2_user_hash[4 + 1] = { '\0' };
        sprintf(temp1_user_hash, "%08lx", (uint32_t)((fw_ver.user_hash >> 16) & 0xFFFFFFFF));
        sprintf(temp2_user_hash, "%04x", (uint16_t)(fw_ver.user_hash & 0xFFFF));

        strcat(temp_user_hash, temp1_user_hash);
        strcat(temp_user_hash, temp2_user_hash);
        printf("User Hash: %s\r\n", temp_user_hash);
    #endif
    }

    BHI385_ASSERT(bhi385_system_param_get_fifo_control(&fifo_ctrl, &bhy));
    print_api_error(assert_rslt, &bhy);

    if (assert_rslt == BHI385_OK)
    {
        printf("\r\n");

        /*lint -e10 Error 10: Lint does not understand PRIxxx */
        printf("Wakeup FIFO Watermark = %" PRIu32 "\r\n", fifo_ctrl.wakeup_fifo_watermark);
        printf("Wakeup FIFO size =  %" PRIu32 "\r\n", fifo_ctrl.wakeup_fifo_size);
        printf("Non Wakeup FIFO Watermark = %" PRIu32 "\r\n", fifo_ctrl.non_wakeup_fifo_watermark);
        printf("Non Wakeup FIFO size = %" PRIu32 "\r\n", fifo_ctrl.non_wakeup_fifo_size);

        /*lint +e10 */
    }

    fifo_ctrl.wakeup_fifo_watermark = 500;
    BHI385_ASSERT(bhi385_system_param_set_wakeup_fifo_control(&fifo_ctrl, &bhy));
    print_api_error(assert_rslt, &bhy);

    BHI385_ASSERT(bhi385_system_param_get_fifo_control(&fifo_ctrl, &bhy));
    print_api_error(assert_rslt, &bhy);

    if (assert_rslt == BHI385_OK)
    {
        /*lint -e10 Error 10: Lint does not understand PRIxxx */
        printf("Wakeup FIFO Watermark after changed =  %" PRIu32 "\r\n", fifo_ctrl.wakeup_fifo_watermark);

        /*lint +e10 */
    }

    fifo_ctrl.non_wakeup_fifo_watermark = 500;
    BHI385_ASSERT(bhi385_system_param_set_nonwakeup_fifo_control(&fifo_ctrl, &bhy));
    print_api_error(assert_rslt, &bhy);

    BHI385_ASSERT(bhi385_system_param_get_fifo_control(&fifo_ctrl, &bhy));
    print_api_error(assert_rslt, &bhy);

    if (assert_rslt == BHI385_OK)
    {
        /*lint -e10 Error 10: Lint does not understand PRIxxx */
        printf("Non Wakeup FIFO Watermark after changed = %" PRIu32 "\r\n", fifo_ctrl.non_wakeup_fifo_watermark);

        /*lint +e10 */
    }

    BHI385_ASSERT(bhi385_system_param_get_meta_event_control(BHI385_SYSTEM_PARAM_META_EVENT_CONTROL_WAKE_UP_FIFO,
                                                             &meta_event, &bhy));
    print_api_error(assert_rslt, &bhy);

    if (assert_rslt == BHI385_OK)
    {
        printf("\r\n");
        printf("Meta event information:\r\n");
        for (uint8_t index = 0; index < 8; index++)
        {
            printf("%d %d %d %d %d %d %d %d\r\n",
                   meta_event.group[index].as_s.meta_event4_enable_state,
                   meta_event.group[index].as_s.meta_event4_int_enable_state,
                   meta_event.group[index].as_s.meta_event3_enable_state,
                   meta_event.group[index].as_s.meta_event3_int_enable_state,
                   meta_event.group[index].as_s.meta_event2_enable_state,
                   meta_event.group[index].as_s.meta_event2_int_enable_state,
                   meta_event.group[index].as_s.meta_event1_enable_state,
                   meta_event.group[index].as_s.meta_event1_int_enable_state);
        }
    }

    meta_event.group[0].as_uint8 = 128;
    BHI385_ASSERT(bhi385_system_param_set_meta_event_control(BHI385_SYSTEM_PARAM_META_EVENT_CONTROL_WAKE_UP_FIFO,
                                                             &meta_event, &bhy));
    print_api_error(assert_rslt, &bhy);

    if (assert_rslt == BHI385_OK)
    {
        printf("\r\n");
        printf("Meta event information after changed:\r\n");

        for (uint8_t index = 0; index < 8; index++)
        {
            printf("%d %d %d %d %d %d %d %d\r\n",
                   meta_event.group[index].as_s.meta_event4_enable_state,
                   meta_event.group[index].as_s.meta_event4_int_enable_state,
                   meta_event.group[index].as_s.meta_event3_enable_state,
                   meta_event.group[index].as_s.meta_event3_int_enable_state,
                   meta_event.group[index].as_s.meta_event2_enable_state,
                   meta_event.group[index].as_s.meta_event2_int_enable_state,
                   meta_event.group[index].as_s.meta_event1_enable_state,
                   meta_event.group[index].as_s.meta_event1_int_enable_state);
        }
    }

    /*! Close all the active communication */
    close_interfaces(intf);

    return rslt;
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
    }
}

/*!
 * @brief Uploading the firmware to the sensor RAM
 * @param[in] dev Firmware data available here
 *
 * @return  rslt execution result
 *
 */
static void upload_firmware(uint8_t boot_stat, struct bhi385_dev *dev)
{
    uint8_t sensor_error;

    printf("Loading firmware into RAM.\r\n");
    BHI385_ASSERT(bhi385_upload_firmware_to_ram(bhi385_firmware_image, sizeof(bhi385_firmware_image), dev));
    print_api_error(assert_rslt, dev);
    BHI385_ASSERT(bhi385_get_error_value(&sensor_error, dev));
    print_api_error(assert_rslt, dev);
    if (sensor_error)
    {
        printf("%s\r\n", get_sensor_error_text(sensor_error));
    }

    printf("Booting from RAM.\r\n");
    BHI385_ASSERT(bhi385_boot_from_ram(dev));
    print_api_error(assert_rslt, dev);

    BHI385_ASSERT(bhi385_get_error_value(&sensor_error, dev));

    if (sensor_error)
    {
        printf("%s\r\n", get_sensor_error_text(sensor_error));
    }

    print_api_error(assert_rslt, dev);
}

static void get_api_name(unsigned int line, const char *func, int8_t val)
{
    printf("system_param.c failed at line %u. The function %s returned error code %d.\r\n", line, func, val);
}
