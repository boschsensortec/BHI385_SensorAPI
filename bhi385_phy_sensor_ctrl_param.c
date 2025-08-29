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
* @file       bhi385_phy_sensor_ctrl_param.c
* @date       2025-08-20
* @version    v2.0.0
*
*/

#include "bhi385_phy_sensor_ctrl_param.h"
#include "bhi385_hif.h"
#include "bhi385.h"

#define BHI385_PHY_SENSOR_CTRL_PARAM_READ_LENGTH          UINT8_C(4)

#define BHI385_PHY_SENSOR_CTRL_PARAM_CODE_LENGTH          UINT8_C(1)

#define BHI385_PHY_PARAM_ANY_MOTION_DURATION_UPPER_MASK   UINT16_C(0x1F)
#define BHI385_PHY_PARAM_ANY_MOTION_AXIS_SEL_SHIFT        UINT16_C(0x05)
#define BHI385_PHY_PARAM_ANY_MOTION_AXIS_SEL_MASK         UINT16_C(0x07)
#define BHI385_PHY_PARAM_ANY_MOTION_THRESHOLD_UPPER_MASK  UINT16_C(0x07)

#define BHI385_PHY_PARAM_NO_MOTION_DURATION_UPPER_MASK    UINT16_C(0x1F)
#define BHI385_PHY_PARAM_NO_MOTION_AXIS_SEL_SHIFT         UINT16_C(0x05)
#define BHI385_PHY_PARAM_NO_MOTION_AXIS_SEL_MASK          UINT16_C(0x07)
#define BHI385_PHY_PARAM_NO_MOTION_THRESHOLD_UPPER_MASK   UINT16_C(0x07)

#define BHI385_ROUND_UP_4_MUL(x)                          (((x) % 4) ? (uint16_t)((((x) / 4) + 1) * \
                                                                                  4) : (uint16_t)(x))

/*lint -e506, -e778*/

static int8_t bhi385_get_phy_sensor_ctrl_info(uint8_t sensor_id,
                                              uint8_t ctrl_code,
                                              uint8_t *payload,
                                              uint32_t payload_len,
                                              uint32_t *actual_len,
                                              struct bhi385_hif_dev *hif);

static int8_t bhi385_get_phy_sensor_ctrl_info(uint8_t sensor_id,
                                              uint8_t ctrl_code,
                                              uint8_t *payload,
                                              uint32_t payload_len,
                                              uint32_t *actual_len,
                                              struct bhi385_hif_dev *hif)
{
    uint16_t code = 0;
    uint8_t old_status;
    uint8_t tmp_buf[BHI385_PHY_SENSOR_CTRL_PARAM_READ_LENGTH] = { 0 };
    uint8_t buf = 0;
    uint32_t length;
    int8_t rslt;
    uint16_t param_id = BHI385_PARAM_PHY_SENSOR_CTRL_PAGE_BASE | sensor_id;

    /* Change Status FIFO to Sync mode */
    rslt = bhi385_hif_get_regs(BHI385_REG_HOST_INTERFACE_CTRL, &buf, 1, hif);
    if (rslt != BHI385_OK)
    {
        return rslt;
    }

    old_status = buf;
    buf &= (uint8_t)(~(BHI385_HIF_CTRL_ASYNC_STATUS_CHANNEL));
    rslt = bhi385_hif_set_regs(BHI385_REG_HOST_INTERFACE_CTRL, &buf, 1, hif);
    if (rslt != BHI385_OK)
    {
        return rslt;
    }

    /* Execute 'Set parameter' command to request data */
    tmp_buf[0] = ctrl_code | BHI385_PARAM_PHY_SENSOR_CTRL_READ;
    length = BHI385_PHY_SENSOR_CTRL_PARAM_READ_LENGTH;
    rslt = bhi385_hif_set_parameter(param_id, tmp_buf, length, hif);
    if (rslt != BHI385_OK)
    {
        return rslt;
    }

    /* Execute 'Get parameter' command to read data */
    length = 0U;
    rslt = bhi385_hif_set_parameter(param_id | BHI385_PARAM_READ_MASK, NULL, length, hif);
    if (rslt != BHI385_OK)
    {
        return rslt;
    }

    /* Wait and process the return data */
    rslt = bhi385_hif_wait_status_ready(hif);
    if (rslt != BHI385_OK)
    {
        return rslt;
    }

    rslt = bhi385_hif_get_status_fifo(&code, payload, payload_len, actual_len, hif);
    if (rslt != BHI385_OK)
    {
        return rslt;
    }

    /* Revert previous Status FIFO mode */
    buf = old_status;
    rslt = bhi385_hif_set_regs(BHI385_REG_HOST_INTERFACE_CTRL, &buf, 1, hif);
    if (rslt == BHI385_OK)
    {
        if (code != param_id)
        {
            rslt = BHI385_E_INVALID_PARAM;
        }
        else if (payload[0] != ctrl_code)
        {
            rslt = BHI385_E_INVALID_PARAM;
        }
    }

    return rslt;
}

/**
 * @brief Function to set accelerometer FOC calibration
 * @param[in] calib  : Reference to FOC calibration
 * @param[in] dev    : Device instance
 * @return API error codes
 */
int8_t bhi385_phy_sensor_ctrl_param_accel_set_foc_calibration(
    const bhi385_phy_sensor_ctrl_param_accel_fast_offset_calib * calib,
    struct bhi385_dev *dev)
{
    int8_t rslt;
    uint16_t param_id;
    uint8_t command;

    if ((calib == NULL) || (dev == NULL))
    {
        rslt = BHI385_E_NULL_PTR;
    }
    else
    {
        param_id = (uint16_t)BHI385_PARAM_PHY_SENSOR_CTRL_PAGE_BASE | BHI385_PHYS_SENSOR_ID_ACCELEROMETER;
        command = (uint8_t)BHI385_PHY_PARAM_ACCEL_FAST_OFFSET_CALIB_COD | BHI385_PARAM_PHY_SENSOR_CTRL_WRITE;
        rslt = bhi385_hif_exec_cmd_generic(param_id,
                                           (const uint8_t *)calib,
                                           BHI385_PHY_PARAM_ACCEL_FAST_OFFSET_CALIB_CTRL_LENGTH,
                                           &command,
                                           1,
                                           0,
                                           &dev->hif);
    }

    return rslt;
}

/**
 * @brief Function to get accelerometer FOC calibration
 * @param[out] calib  : Reference to FOC calibration
 * @param[in]  dev    : Device instance
 * @return API error codes
 */
int8_t bhi385_phy_sensor_ctrl_param_accel_get_foc_calibration(
    bhi385_phy_sensor_ctrl_param_accel_fast_offset_calib * calib,
    struct bhi385_dev *dev)
{
    int8_t rslt;
    uint8_t command;
    uint32_t act_len;
    uint8_t payload[BHI385_ROUND_UP_4_MUL(BHI385_PHY_PARAM_ACCEL_FAST_OFFSET_CALIB_CTRL_LENGTH)] = { 0U };

    if ((calib == NULL) || (dev == NULL))
    {
        rslt = BHI385_E_NULL_PTR;
    }
    else
    {
        command = (uint8_t)BHI385_PHY_PARAM_ACCEL_FAST_OFFSET_CALIB_COD;
        rslt = bhi385_get_phy_sensor_ctrl_info(BHI385_PHYS_SENSOR_ID_ACCELEROMETER,
                                               command,
                                               payload,
                                               BHI385_ROUND_UP_4_MUL(
                                                   BHI385_PHY_PARAM_ACCEL_FAST_OFFSET_CALIB_CTRL_LENGTH),
                                               &act_len,
                                               &dev->hif);
        if (rslt == BHI385_OK)
        {
            calib->x_offset = BHI385_LE2S16(&payload[0 + BHI385_PHY_SENSOR_CTRL_PARAM_CODE_LENGTH]);
            calib->y_offset = BHI385_LE2S16(&payload[2 + BHI385_PHY_SENSOR_CTRL_PARAM_CODE_LENGTH]);
            calib->z_offset = BHI385_LE2S16(&payload[4 + BHI385_PHY_SENSOR_CTRL_PARAM_CODE_LENGTH]);
        }
    }

    return rslt;
}

/**
 * @brief Function to set accelerometer power mode
 * @param[in] mode  : Accelerometer power mode
 * @param[in] dev   : Device instance
 * @return API error codes
 */
int8_t bhi385_phy_sensor_ctrl_param_accel_set_power_mode(uint8_t mode, struct bhi385_dev *dev)
{
    int8_t rslt;
    uint16_t param_id;
    uint8_t command;
    uint8_t payload[BHI385_ROUND_UP_4_MUL(BHI385_PHY_PARAM_ACCEL_LOW_POWER_MODE_CTRL_LENGTH)] = { 0U };

    if (dev == NULL)
    {
        rslt = BHI385_E_NULL_PTR;
    }
    else
    {
        param_id = (uint16_t)BHI385_PARAM_PHY_SENSOR_CTRL_PAGE_BASE | BHI385_PHYS_SENSOR_ID_ACCELEROMETER;
        command = (uint8_t)BHI385_PHY_PARAM_ACCEL_LOW_POWER_MODE_COD | BHI385_PARAM_PHY_SENSOR_CTRL_WRITE;
        payload[0] = mode;
        rslt = bhi385_hif_exec_cmd_generic(param_id,
                                           (const uint8_t *)payload,
                                           BHI385_PHY_PARAM_ACCEL_LOW_POWER_MODE_CTRL_LENGTH,
                                           &command,
                                           1,
                                           0,
                                           &dev->hif);
    }

    return rslt;
}

/**
 * @brief Function to get accelerometer mode
 * @param[out] mode  : Reference to accelerometer mode
 * @param[in]  dev   : Device instance
 * @return API error codes
 */
int8_t bhi385_phy_sensor_ctrl_param_accel_get_power_mode(uint8_t* mode, struct bhi385_dev *dev)
{
    int8_t rslt;
    uint8_t command;
    uint32_t act_len;
    uint8_t payload[BHI385_ROUND_UP_4_MUL(BHI385_PHY_PARAM_ACCEL_LOW_POWER_MODE_CTRL_LENGTH)] = { 0U };

    if ((mode == NULL) || (dev == NULL))
    {
        rslt = BHI385_E_NULL_PTR;
    }
    else
    {
        command = (uint8_t)BHI385_PHY_PARAM_ACCEL_LOW_POWER_MODE_COD;
        rslt = bhi385_get_phy_sensor_ctrl_info(BHI385_PHYS_SENSOR_ID_ACCELEROMETER,
                                               command,
                                               payload,
                                               BHI385_ROUND_UP_4_MUL(BHI385_PHY_PARAM_ACCEL_LOW_POWER_MODE_CTRL_LENGTH),
                                               &act_len,
                                               &dev->hif);
        if (rslt == BHI385_OK)
        {
            *mode = payload[0 + BHI385_PHY_SENSOR_CTRL_PARAM_CODE_LENGTH];
        }
    }

    return rslt;
}

/**
 * @brief Function to set axis remapping for internal imu features
 * @param[in] remap  : Reference to axis remapping
 * @param[in] dev    : Device instance
 * @return API error codes
 */
int8_t bhi385_phy_sensor_ctrl_param_accel_set_axis_remapping(
    const bhi385_phy_sensor_ctrl_param_accel_axis_remap * remap,
    struct bhi385_dev *dev)
{
    int8_t rslt;
    uint16_t param_id;
    uint8_t command;

    if ((remap == NULL) || (dev == NULL))
    {
        rslt = BHI385_E_NULL_PTR;
    }
    else
    {
        param_id = (uint16_t)BHI385_PARAM_PHY_SENSOR_CTRL_PAGE_BASE | BHI385_PHYS_SENSOR_ID_ACCELEROMETER;
        command = (uint8_t)BHI385_PHY_PARAM_ACCEL_AXIS_REMAP_COD | BHI385_PARAM_PHY_SENSOR_CTRL_WRITE;
        rslt = bhi385_hif_exec_cmd_generic(param_id,
                                           (const uint8_t *)remap,
                                           BHI385_PHY_PARAM_ACCEL_AXIS_REMAP_CTRL_LENGTH,
                                           &command,
                                           1,
                                           0,
                                           &dev->hif);
    }

    return rslt;
}

/**
 * @brief Function to get axis remapping for internal imu features
 * @param[in] remap  : Reference to axis remapping
 * @param[in] dev    : Device instance
 * @return API error codes
 */
int8_t bhi385_phy_sensor_ctrl_param_accel_get_axis_remapping(bhi385_phy_sensor_ctrl_param_accel_axis_remap * remap,
                                                             struct bhi385_dev *dev)
{
    int8_t rslt;
    uint8_t command;
    uint32_t act_len;
    uint8_t payload[BHI385_ROUND_UP_4_MUL(BHI385_PHY_PARAM_ACCEL_AXIS_REMAP_CTRL_LENGTH)] = { 0U };

    if ((remap == NULL) || (dev == NULL))
    {
        rslt = BHI385_E_NULL_PTR;
    }
    else
    {
        command = (uint8_t)BHI385_PHY_PARAM_ACCEL_AXIS_REMAP_COD;
        rslt = bhi385_get_phy_sensor_ctrl_info(BHI385_PHYS_SENSOR_ID_ACCELEROMETER,
                                               command,
                                               payload,
                                               BHI385_ROUND_UP_4_MUL(BHI385_PHY_PARAM_ACCEL_AXIS_REMAP_CTRL_LENGTH),
                                               &act_len,
                                               &dev->hif);
        if (rslt == BHI385_OK)
        {
            remap->map_x_axis = payload[0 + BHI385_PHY_SENSOR_CTRL_PARAM_CODE_LENGTH];
            remap->map_x_axis_sign = payload[1 + BHI385_PHY_SENSOR_CTRL_PARAM_CODE_LENGTH];
            remap->map_y_axis = payload[2 + BHI385_PHY_SENSOR_CTRL_PARAM_CODE_LENGTH];
            remap->map_y_axis_sign = payload[3 + BHI385_PHY_SENSOR_CTRL_PARAM_CODE_LENGTH];
            remap->map_z_axis = payload[4 + BHI385_PHY_SENSOR_CTRL_PARAM_CODE_LENGTH];
            remap->map_z_axis_sign = payload[5 + BHI385_PHY_SENSOR_CTRL_PARAM_CODE_LENGTH];
        }
    }

    return rslt;
}

/**
 * @brief Function to trigger a NVM writing for accelerometer
 * @param[in] dev   : Device instance
 * @return API error codes
 */
int8_t bhi385_phy_sensor_ctrl_param_accel_trigger_nvm_writing(struct bhi385_dev *dev)
{
    int8_t rslt;
    uint16_t param_id;
    uint8_t command;
    uint8_t payload[BHI385_ROUND_UP_4_MUL(BHI385_PHY_PARAM_ACCEL_NVM_WRITE_TRIGGER_CTRL_LENGTH)] = { 0U };

    if (dev == NULL)
    {
        rslt = BHI385_E_NULL_PTR;
    }
    else
    {
        param_id = (uint16_t)BHI385_PARAM_PHY_SENSOR_CTRL_PAGE_BASE | BHI385_PHYS_SENSOR_ID_ACCELEROMETER;
        command = (uint8_t)BHI385_PHY_PARAM_ACCEL_NVM_WRITE_TRIGGER_COD | BHI385_PARAM_PHY_SENSOR_CTRL_WRITE;
        payload[0] = BHI385_PHY_PARAM_ACCEL_NVM_WRITE_TRIGGER_ENABLE;
        rslt = bhi385_hif_exec_cmd_generic(param_id,
                                           (const uint8_t *)payload,
                                           BHI385_PHY_PARAM_ACCEL_NVM_WRITE_TRIGGER_CTRL_LENGTH,
                                           &command,
                                           1,
                                           0,
                                           &dev->hif);
    }

    return rslt;
}

/**
 * @brief Function to get NVM writing status for accelerometer
 * @param[out] status  : Reference to NVM writing status
 * @param[in] dev      : Device instance
 * @return API error codes
 */
int8_t bhi385_phy_sensor_ctrl_param_accel_get_nvm_status(uint8_t* status, struct bhi385_dev *dev)
{
    int8_t rslt;
    uint8_t command;
    uint32_t act_len;
    uint8_t payload[BHI385_ROUND_UP_4_MUL(BHI385_PHY_PARAM_ACCEL_NVM_WRITE_TRIGGER_CTRL_LENGTH)] = { 0U };

    if ((status == NULL) || (dev == NULL))
    {
        rslt = BHI385_E_NULL_PTR;
    }
    else
    {
        command = (uint8_t)BHI385_PHY_PARAM_ACCEL_NVM_WRITE_TRIGGER_COD;
        rslt = bhi385_get_phy_sensor_ctrl_info(BHI385_PHYS_SENSOR_ID_ACCELEROMETER,
                                               command,
                                               payload,
                                               BHI385_ROUND_UP_4_MUL(
                                                   BHI385_PHY_PARAM_ACCEL_NVM_WRITE_TRIGGER_CTRL_LENGTH),
                                               &act_len,
                                               &dev->hif);
        if (rslt == BHI385_OK)
        {
            *status = payload[0 + BHI385_PHY_SENSOR_CTRL_PARAM_CODE_LENGTH];
        }
    }

    return rslt;
}

/**
 * @brief Function to set gyroscope FOC calibration
 * @param[in] calib  : Reference to FOC calibration
 * @param[in] dev    : Device instance
 * @return API error codes
 */
int8_t bhi385_phy_sensor_ctrl_param_gyro_set_foc_calibration(
    const bhi385_phy_sensor_ctrl_param_gyro_fast_offset_calib * calib,
    struct bhi385_dev *dev)
{
    int8_t rslt;
    uint16_t param_id;
    uint8_t command;

    if ((calib == NULL) || (dev == NULL))
    {
        rslt = BHI385_E_NULL_PTR;
    }
    else
    {
        param_id = (uint16_t)BHI385_PARAM_PHY_SENSOR_CTRL_PAGE_BASE | BHI385_PHYS_SENSOR_ID_GYROSCOPE;
        command = (uint8_t)BHI385_PHY_PARAM_GYRO_FAST_OFFSET_CALIB_COD | BHI385_PARAM_PHY_SENSOR_CTRL_WRITE;
        rslt = bhi385_hif_exec_cmd_generic(param_id,
                                           (const uint8_t *)calib,
                                           BHI385_PHY_PARAM_GYRO_FAST_OFFSET_CALIB_CTRL_LENGTH,
                                           &command,
                                           1,
                                           0,
                                           &dev->hif);
    }

    return rslt;
}

/**
 * @brief Function to get gyroscope FOC calibration
 * @param[out] calib  : Reference to FOC calibration
 * @param[in]  dev    : Device instance
 * @return API error codes
 */
int8_t bhi385_phy_sensor_ctrl_param_gyro_get_foc_calibration(
    bhi385_phy_sensor_ctrl_param_gyro_fast_offset_calib * calib,
    struct bhi385_dev *dev)
{
    int8_t rslt;
    uint8_t command;
    uint32_t act_len;
    uint8_t payload[BHI385_ROUND_UP_4_MUL(BHI385_PHY_PARAM_GYRO_FAST_OFFSET_CALIB_CTRL_LENGTH)] = { 0U };

    if ((calib == NULL) || (dev == NULL))
    {
        rslt = BHI385_E_NULL_PTR;
    }
    else
    {
        command = (uint8_t)BHI385_PHY_PARAM_GYRO_FAST_OFFSET_CALIB_COD;
        rslt =
            bhi385_get_phy_sensor_ctrl_info(BHI385_PHYS_SENSOR_ID_GYROSCOPE,
                                            command,
                                            payload,
                                            BHI385_ROUND_UP_4_MUL(BHI385_PHY_PARAM_GYRO_FAST_OFFSET_CALIB_CTRL_LENGTH),
                                            &act_len,
                                            &dev->hif);
        if (rslt == BHI385_OK)
        {
            calib->x_offset = BHI385_LE2S16(&payload[0 + BHI385_PHY_SENSOR_CTRL_PARAM_CODE_LENGTH]);
            calib->y_offset = BHI385_LE2S16(&payload[2 + BHI385_PHY_SENSOR_CTRL_PARAM_CODE_LENGTH]);
            calib->z_offset = BHI385_LE2S16(&payload[4 + BHI385_PHY_SENSOR_CTRL_PARAM_CODE_LENGTH]);
        }
    }

    return rslt;
}

/**
 * @brief Function to set gyroscope OIS configuration
 * @param[in] config  : OIS configuration
 * @param[in] dev     : Device instance
 * @return API error codes
 */
int8_t bhi385_phy_sensor_ctrl_param_gyro_set_ois_config(uint8_t config, struct bhi385_dev *dev)
{
    int8_t rslt;
    uint16_t param_id;
    uint8_t command;
    uint8_t payload[BHI385_ROUND_UP_4_MUL(BHI385_PHY_PARAM_GYRO_OIS_ENABLE_CTRL_LENGTH)] = { 0U };

    if (dev == NULL)
    {
        rslt = BHI385_E_NULL_PTR;
    }
    else
    {
        param_id = (uint16_t)BHI385_PARAM_PHY_SENSOR_CTRL_PAGE_BASE | BHI385_PHYS_SENSOR_ID_GYROSCOPE;
        command = (uint8_t)BHI385_PHY_PARAM_GYRO_OIS_ENABLE_COD | BHI385_PARAM_PHY_SENSOR_CTRL_WRITE;
        payload[0] = config;
        rslt = bhi385_hif_exec_cmd_generic(param_id,
                                           (const uint8_t *)payload,
                                           BHI385_PHY_PARAM_GYRO_OIS_ENABLE_CTRL_LENGTH,
                                           &command,
                                           1,
                                           0,
                                           &dev->hif);
    }

    return rslt;
}

/**
 * @brief Function to get gyroscope OIS configuration
 * @param[out] config  : Reference to OIS configuration
 * @param[in]  dev     : Device instance
 * @return API error codes
 */
int8_t bhi385_phy_sensor_ctrl_param_gyro_get_ois_config(uint8_t* config, struct bhi385_dev *dev)
{
    int8_t rslt;
    uint8_t command;
    uint32_t act_len;
    uint8_t payload[BHI385_ROUND_UP_4_MUL(BHI385_PHY_PARAM_GYRO_OIS_ENABLE_CTRL_LENGTH)] = { 0U };

    if ((config == NULL) || (dev == NULL))
    {
        rslt = BHI385_E_NULL_PTR;
    }
    else
    {
        command = (uint8_t)BHI385_PHY_PARAM_GYRO_OIS_ENABLE_COD;
        rslt =
            bhi385_get_phy_sensor_ctrl_info(BHI385_PHYS_SENSOR_ID_GYROSCOPE,
                                            command,
                                            payload,
                                            BHI385_ROUND_UP_4_MUL(BHI385_PHY_PARAM_GYRO_OIS_ENABLE_CTRL_LENGTH),
                                            &act_len,
                                            &dev->hif);
        if (rslt == BHI385_OK)
        {
            *config = payload[0 + BHI385_PHY_SENSOR_CTRL_PARAM_CODE_LENGTH];
        }
    }

    return rslt;
}

/**
 * @brief Function to set gyroscope Fast start up configuration
 * @param[in] config  : Fast start up configuration
 * @param[in] dev     : Device instance
 * @return API error codes
 */
int8_t bhi385_phy_sensor_ctrl_param_gyro_set_fast_startup_cfg(uint8_t config, struct bhi385_dev *dev)
{
    int8_t rslt;
    uint16_t param_id;
    uint8_t command;
    uint8_t payload[BHI385_ROUND_UP_4_MUL(BHI385_PHY_PARAM_GYRO_FAST_STARTUP_ENABLE_CTRL_LENGTH)] = { 0U };

    if (dev == NULL)
    {
        rslt = BHI385_E_NULL_PTR;
    }
    else
    {
        param_id = (uint16_t)BHI385_PARAM_PHY_SENSOR_CTRL_PAGE_BASE | BHI385_PHYS_SENSOR_ID_GYROSCOPE;
        command = (uint8_t)BHI385_PHY_PARAM_GYRO_FAST_STARTUP_ENABLE_COD | BHI385_PARAM_PHY_SENSOR_CTRL_WRITE;
        payload[0] = config;
        rslt = bhi385_hif_exec_cmd_generic(param_id,
                                           (const uint8_t *)payload,
                                           BHI385_PHY_PARAM_GYRO_FAST_STARTUP_ENABLE_CTRL_LENGTH,
                                           &command,
                                           1,
                                           0,
                                           &dev->hif);
    }

    return rslt;
}

/**
 * @brief Function to get gyroscope Fast start up configuration
 * @param[out] config  : Reference to Fast start up configuration
 * @param[in]  dev     : Device instance
 * @return API error codes
 */
int8_t bhi385_phy_sensor_ctrl_param_gyro_get_fast_startup_cfg(uint8_t* config, struct bhi385_dev *dev)
{
    int8_t rslt;
    uint8_t command;
    uint32_t act_len;
    uint8_t payload[BHI385_ROUND_UP_4_MUL(BHI385_PHY_PARAM_GYRO_FAST_STARTUP_ENABLE_CTRL_LENGTH)] = { 0U };

    if ((config == NULL) || (dev == NULL))
    {
        rslt = BHI385_E_NULL_PTR;
    }
    else
    {
        command = (uint8_t)BHI385_PHY_PARAM_GYRO_FAST_STARTUP_ENABLE_COD;
        rslt =
            bhi385_get_phy_sensor_ctrl_info(BHI385_PHYS_SENSOR_ID_GYROSCOPE,
                                            command,
                                            payload,
                                            BHI385_ROUND_UP_4_MUL(
                                                BHI385_PHY_PARAM_GYRO_FAST_STARTUP_ENABLE_CTRL_LENGTH),
                                            &act_len,
                                            &dev->hif);
        if (rslt == BHI385_OK)
        {
            *config = payload[0 + BHI385_PHY_SENSOR_CTRL_PARAM_CODE_LENGTH];
        }
    }

    return rslt;
}

/**
 * @brief Function to start gyroscope Component ReTrim (CRT)
 * @param[in] dev     : Device instance
 * @return API error codes
 */
int8_t bhi385_phy_sensor_ctrl_param_gyro_start_comp_retrim(struct bhi385_dev *dev)
{
    int8_t rslt;
    uint16_t param_id;
    uint8_t command;
    uint8_t payload[BHI385_ROUND_UP_4_MUL(BHI385_PHY_PARAM_GYRO_COMP_RETRIM_CTRL_LENGTH)] = { 0U };

    if (dev == NULL)
    {
        rslt = BHI385_E_NULL_PTR;
    }
    else
    {
        param_id = (uint16_t)BHI385_PARAM_PHY_SENSOR_CTRL_PAGE_BASE | BHI385_PHYS_SENSOR_ID_GYROSCOPE;
        command = (uint8_t)BHI385_PHY_PARAM_GYRO_COMP_RETRIM_COD | BHI385_PARAM_PHY_SENSOR_CTRL_WRITE;
        payload[0] = BHI385_PHY_PARAM_GYRO_FAST_STARTUP_ENABLE;
        rslt = bhi385_hif_exec_cmd_generic(param_id,
                                           (const uint8_t *)payload,
                                           BHI385_PHY_PARAM_GYRO_COMP_RETRIM_CTRL_LENGTH,
                                           &command,
                                           1,
                                           0,
                                           &dev->hif);
    }

    return rslt;
}

/**
 * @brief Function to get gyroscope Component ReTrim (CRT) status
 * @param[out] crt     : Reference to CRT status
 * @param[in]  dev     : Device instance
 * @return API error codes
 */
int8_t bhi385_phy_sensor_ctrl_param_gyro_get_crt_data(bhi385_phy_sensor_ctrl_param_gyro_crt_data* crt,
                                                      struct bhi385_dev *dev)
{
    int8_t rslt;
    uint8_t command;
    uint32_t act_len;
    uint8_t payload[BHI385_ROUND_UP_4_MUL(BHI385_PHY_PARAM_GYRO_COMP_RETRIM_CTRL_LENGTH)] = { 0U };

    if ((crt == NULL) || (dev == NULL))
    {
        rslt = BHI385_E_NULL_PTR;
    }
    else
    {
        command = (uint8_t)BHI385_PHY_PARAM_GYRO_COMP_RETRIM_COD;
        rslt =
            bhi385_get_phy_sensor_ctrl_info(BHI385_PHYS_SENSOR_ID_GYROSCOPE,
                                            command,
                                            payload,
                                            BHI385_ROUND_UP_4_MUL(BHI385_PHY_PARAM_GYRO_COMP_RETRIM_CTRL_LENGTH),
                                            &act_len,
                                            &dev->hif);
        if (rslt == BHI385_OK)
        {
            crt->status = payload[0 + BHI385_PHY_SENSOR_CTRL_PARAM_CODE_LENGTH];
            crt->x = payload[1 + BHI385_PHY_SENSOR_CTRL_PARAM_CODE_LENGTH];
            crt->y = payload[2 + BHI385_PHY_SENSOR_CTRL_PARAM_CODE_LENGTH];
            crt->z = payload[3 + BHI385_PHY_SENSOR_CTRL_PARAM_CODE_LENGTH];
        }
    }

    return rslt;
}

/**
 * @brief Function to set gyroscope Component ReTrim (CRT) offset values
 * @param[in] crt     : Reference to Gyro CRT offset values
 * @param[in]  dev     : Device instance
 * @return API error codes
 */
int8_t bhi385_phy_sensor_ctrl_param_set_gyro_data(bhi385_phy_sensor_ctrl_param_gyro_crt_data* gyro_crt,
                                                  struct bhi385_dev *dev)
{
    int8_t rslt = BHI385_OK;
    uint16_t cmnd_len = BHI385_LE24MUL(BHI385_PHY_GYRO_CRT_CTRL_LEN + 1); /*1 byte added for control code */
    uint8_t cmnd[cmnd_len];

    memset(cmnd, 0, cmnd_len);

    cmnd[0] = BHI385_PHY_SENSOR_CTRL_CODE(BHI385_PHY_SENSOR_CTRL_WRITE, BHI385_PHY_GYRO_CRT_CTRL_CODE);

    if (dev == NULL || gyro_crt == NULL)
    {
        rslt = BHI385_E_NULL_PTR;
    }
    else
    {
        cmnd[1] = gyro_crt->status;
        cmnd[2] = gyro_crt->x;
        cmnd[3] = gyro_crt->y;
        cmnd[4] = gyro_crt->z;

        rslt = bhi385_set_parameter(BHI385_PHY_SENSOR_CTRL_PARAM(BHI385_PHYS_SENSOR_ID_GYROSCOPE), cmnd, cmnd_len, dev);

    }

    return rslt;
}

/**
 * @brief Function to set gyroscope power mode
 * @param[in] mode  : Gyroscope power mode
 * @param[in] dev   : Device instance
 * @return API error codes
 */
int8_t bhi385_phy_sensor_ctrl_param_gyro_set_power_mode(uint8_t mode, struct bhi385_dev *dev)
{
    int8_t rslt;
    uint16_t param_id;
    uint8_t command;
    uint8_t payload[BHI385_ROUND_UP_4_MUL(BHI385_PHY_PARAM_GYRO_PERF_MODE_CTRL_LENGTH)] = { 0U };

    if (dev == NULL)
    {
        rslt = BHI385_E_NULL_PTR;
    }
    else
    {
        param_id = (uint16_t)BHI385_PARAM_PHY_SENSOR_CTRL_PAGE_BASE | BHI385_PHYS_SENSOR_ID_GYROSCOPE;
        command = (uint8_t)BHI385_PHY_PARAM_GYRO_PERF_MODE_COD | BHI385_PARAM_PHY_SENSOR_CTRL_WRITE;
        payload[0] = mode;
        rslt = bhi385_hif_exec_cmd_generic(param_id,
                                           (const uint8_t *)payload,
                                           BHI385_PHY_PARAM_GYRO_PERF_MODE_CTRL_LENGTH,
                                           &command,
                                           1,
                                           0,
                                           &dev->hif);
    }

    return rslt;
}

/**
 * @brief Function to set gyroscope mode
 * @param[out] mode  : Reference to gyroscope mode
 * @param[in]  dev   : Device instance
 * @return API error codes
 */
int8_t bhi385_phy_sensor_ctrl_param_gyro_get_power_mode(uint8_t* mode, struct bhi385_dev *dev)
{
    int8_t rslt;
    uint8_t command;
    uint32_t act_len;
    uint8_t payload[BHI385_ROUND_UP_4_MUL(BHI385_PHY_PARAM_GYRO_PERF_MODE_CTRL_LENGTH)] = { 0U };

    if ((mode == NULL) || (dev == NULL))
    {
        rslt = BHI385_E_NULL_PTR;
    }
    else
    {
        command = (uint8_t)BHI385_PHY_PARAM_GYRO_PERF_MODE_COD;
        rslt =
            bhi385_get_phy_sensor_ctrl_info(BHI385_PHYS_SENSOR_ID_GYROSCOPE,
                                            command,
                                            payload,
                                            BHI385_ROUND_UP_4_MUL(BHI385_PHY_PARAM_GYRO_PERF_MODE_CTRL_LENGTH),
                                            &act_len,
                                            &dev->hif);
        if (rslt == BHI385_OK)
        {
            *mode = payload[0 + BHI385_PHY_SENSOR_CTRL_PARAM_CODE_LENGTH];
        }
    }

    return rslt;
}

/**
 * @brief Function to set gyroscope timer auto trim configuration
 * @param[in] config  : Timer auto trim configuration
 * @param[in] dev     : Device instance
 * @return API error codes
 */
int8_t bhi385_phy_sensor_ctrl_param_gyro_set_auto_trim_cfg(uint8_t config, struct bhi385_dev *dev)
{
    int8_t rslt;
    uint16_t param_id;
    uint8_t command;
    uint8_t payload[BHI385_ROUND_UP_4_MUL(BHI385_PHY_PARAM_GYRO_TIMER_AUTO_TRIM_CTRL_LENGTH)] = { 0U };

    if (dev == NULL)
    {
        rslt = BHI385_E_NULL_PTR;
    }
    else
    {
        param_id = (uint16_t)BHI385_PARAM_PHY_SENSOR_CTRL_PAGE_BASE | BHI385_PHYS_SENSOR_ID_GYROSCOPE;
        command = (uint8_t)BHI385_PHY_PARAM_GYRO_TIMER_AUTO_TRIM_COD | BHI385_PARAM_PHY_SENSOR_CTRL_WRITE;
        payload[0] = config;
        rslt = bhi385_hif_exec_cmd_generic(param_id,
                                           (const uint8_t *)payload,
                                           BHI385_PHY_PARAM_GYRO_TIMER_AUTO_TRIM_CTRL_LENGTH,
                                           &command,
                                           1,
                                           0,
                                           &dev->hif);
    }

    return rslt;
}

/**
 * @brief Function to get gyroscope timer auto trim configuration
 * @param[out] config  : Reference to Timer auto trim configuration
 * @param[in]  dev     : Device instance
 * @return API error codes
 */
int8_t bhi385_phy_sensor_ctrl_param_gyro_get_auto_trim_cfg(uint8_t* config, struct bhi385_dev *dev)
{
    int8_t rslt;
    uint8_t command;
    uint32_t act_len;
    uint8_t payload[BHI385_ROUND_UP_4_MUL(BHI385_PHY_PARAM_GYRO_TIMER_AUTO_TRIM_CTRL_LENGTH)] = { 0U };

    if ((config == NULL) || (dev == NULL))
    {
        rslt = BHI385_E_NULL_PTR;
    }
    else
    {
        command = (uint8_t)BHI385_PHY_PARAM_GYRO_TIMER_AUTO_TRIM_COD;
        rslt =
            bhi385_get_phy_sensor_ctrl_info(BHI385_PHYS_SENSOR_ID_GYROSCOPE,
                                            command,
                                            payload,
                                            BHI385_ROUND_UP_4_MUL(BHI385_PHY_PARAM_GYRO_TIMER_AUTO_TRIM_CTRL_LENGTH),
                                            &act_len,
                                            &dev->hif);
        if (rslt == BHI385_OK)
        {
            *config = payload[0 + BHI385_PHY_SENSOR_CTRL_PARAM_CODE_LENGTH];
        }
    }

    return rslt;
}

/**
 * @brief Function to trigger a NVM writing for gyroscope
 * @param[in] dev   : Device instance
 * @return API error codes
 */
int8_t bhi385_phy_sensor_ctrl_param_gyro_trigger_nvm_writing(struct bhi385_dev *dev)
{
    int8_t rslt;
    uint16_t param_id;
    uint8_t command;
    uint8_t payload[BHI385_ROUND_UP_4_MUL(BHI385_PHY_PARAM_GYRO_NVM_WRITE_TRIGGER_CTRL_LENGTH)] = { 0U };

    if (dev == NULL)
    {
        rslt = BHI385_E_NULL_PTR;
    }
    else
    {
        param_id = (uint16_t)BHI385_PARAM_PHY_SENSOR_CTRL_PAGE_BASE | BHI385_PHYS_SENSOR_ID_GYROSCOPE;
        command = (uint8_t)BHI385_PHY_PARAM_GYRO_NVM_WRITE_TRIGGER_COD | BHI385_PARAM_PHY_SENSOR_CTRL_WRITE;
        payload[0] = BHI385_PHY_PARAM_GYRO_NVM_WRITE_TRIGGER_ENABLE;
        rslt = bhi385_hif_exec_cmd_generic(param_id,
                                           (const uint8_t *)payload,
                                           BHI385_PHY_PARAM_GYRO_NVM_WRITE_TRIGGER_CTRL_LENGTH,
                                           &command,
                                           1,
                                           0,
                                           &dev->hif);
    }

    return rslt;
}

/**
 * @brief Function to get NVM writing status for gyroscope
 * @param[out] status  : Reference to NVM writing status
 * @param[in] dev      : Device instance
 * @return API error codes
 */
int8_t bhi385_phy_sensor_ctrl_param_gyro_get_nvm_status(uint8_t* status, struct bhi385_dev *dev)
{
    int8_t rslt;
    uint8_t command;
    uint32_t act_len;
    uint8_t payload[BHI385_ROUND_UP_4_MUL(BHI385_PHY_PARAM_GYRO_NVM_WRITE_TRIGGER_CTRL_LENGTH)] = { 0U };

    if ((status == NULL) || (dev == NULL))
    {
        rslt = BHI385_E_NULL_PTR;
    }
    else
    {
        command = (uint8_t)BHI385_PHY_PARAM_GYRO_NVM_WRITE_TRIGGER_COD;
        rslt =
            bhi385_get_phy_sensor_ctrl_info(BHI385_PHYS_SENSOR_ID_GYROSCOPE,
                                            command,
                                            payload,
                                            BHI385_ROUND_UP_4_MUL(BHI385_PHY_PARAM_GYRO_NVM_WRITE_TRIGGER_CTRL_LENGTH),
                                            &act_len,
                                            &dev->hif);
        if (rslt == BHI385_OK)
        {
            *status = payload[0 + BHI385_PHY_SENSOR_CTRL_PARAM_CODE_LENGTH];
        }
    }

    return rslt;
}

/**
 * @brief Function to set Wrist Wear Wakeup configuration
 * @param[in] config  : Reference to Wrist Wear Wakeup configuration
 * @param[in] dev     : Device instance
 * @return API error codes
 */
int8_t bhi385_phy_sensor_ctrl_param_set_wrist_wear_wakeup_cfg(
    const bhi385_phy_sensor_ctrl_param_wrist_wear_wakeup* config,
    struct bhi385_dev *dev)
{
    int8_t rslt;
    uint16_t param_id;
    uint8_t command;

    if ((config == NULL) || (dev == NULL))
    {
        rslt = BHI385_E_NULL_PTR;
    }
    else
    {
        param_id = (uint16_t)BHI385_PARAM_PHY_SENSOR_CTRL_PAGE_BASE | BHI385_PHYS_SENSOR_ID_WRIST_WEAR_WAKEUP;
        command = (uint8_t)BHI385_PHY_PARAM_WRIST_WEAR_WAKEUP_COD | BHI385_PARAM_PHY_SENSOR_CTRL_WRITE;
        rslt = bhi385_hif_exec_cmd_generic(param_id,
                                           (const uint8_t *)config,
                                           BHI385_PHY_PARAM_WRIST_WEAR_WAKEUP_CTRL_LENGTH,
                                           &command,
                                           1,
                                           0,
                                           &dev->hif);
    }

    return rslt;
}

/**
 * @brief Function to get Wrist Wear Wakeup configuration
 * @param[out] config  : Reference to Wrist Wear Wakeup configuration
 * @param[in]  dev     : Device instance
 * @return API error codes
 */
int8_t bhi385_phy_sensor_ctrl_param_get_wrist_wear_wakeup_cfg(bhi385_phy_sensor_ctrl_param_wrist_wear_wakeup* config,
                                                              struct bhi385_dev *dev)
{
    int8_t rslt;
    uint8_t command;
    uint32_t act_len;
    uint8_t payload[BHI385_ROUND_UP_4_MUL(BHI385_PHY_PARAM_WRIST_WEAR_WAKEUP_CTRL_LENGTH)] = { 0U };

    if ((config == NULL) || (dev == NULL))
    {
        rslt = BHI385_E_NULL_PTR;
    }
    else
    {
        command = (uint8_t)BHI385_PHY_PARAM_WRIST_WEAR_WAKEUP_COD;
        rslt = bhi385_get_phy_sensor_ctrl_info(BHI385_PHYS_SENSOR_ID_WRIST_WEAR_WAKEUP,
                                               command,
                                               payload,
                                               BHI385_ROUND_UP_4_MUL(BHI385_PHY_PARAM_WRIST_WEAR_WAKEUP_CTRL_LENGTH),
                                               &act_len,
                                               &dev->hif);
        if (rslt == BHI385_OK)
        {
            config->min_angle_focus = BHI385_LE2U16(&payload[0 + BHI385_PHY_SENSOR_CTRL_PARAM_CODE_LENGTH]);
            config->min_angle_non_focus = BHI385_LE2U16(&payload[2 + BHI385_PHY_SENSOR_CTRL_PARAM_CODE_LENGTH]);
            config->angle_landscape_right = payload[4 + BHI385_PHY_SENSOR_CTRL_PARAM_CODE_LENGTH];
            config->angle_landscape_left = payload[5 + BHI385_PHY_SENSOR_CTRL_PARAM_CODE_LENGTH];
            config->angle_portrait_down = payload[6 + BHI385_PHY_SENSOR_CTRL_PARAM_CODE_LENGTH];
            config->angle_portrait_up = payload[7 + BHI385_PHY_SENSOR_CTRL_PARAM_CODE_LENGTH];
            config->min_dur_moved = payload[8 + BHI385_PHY_SENSOR_CTRL_PARAM_CODE_LENGTH];
            config->min_dur_quite = payload[9 + BHI385_PHY_SENSOR_CTRL_PARAM_CODE_LENGTH];
        }
    }

    return rslt;
}

/**
 * @brief Function to set Any Motion configuration
 * @param[in] config  : Reference to Any Motion configuration
 * @param[in] dev     : Device instance
 * @return API error codes
 */
int8_t bhi385_phy_sensor_ctrl_param_set_any_motion_config(const bhi385_phy_sensor_ctrl_param_any_motion* config,
                                                          struct bhi385_dev *dev)
{
    int8_t rslt;
    uint16_t param_id;
    uint8_t command;

    if ((config == NULL) || (dev == NULL))
    {
        rslt = BHI385_E_NULL_PTR;
    }
    else
    {
        param_id = (uint16_t)BHI385_PARAM_PHY_SENSOR_CTRL_PAGE_BASE | BHI385_PHYS_SENSOR_ID_PHYS_ANY_MOTION;
        command = (uint8_t)BHI385_PHY_PARAM_ANY_MOTION_COD | BHI385_PARAM_PHY_SENSOR_CTRL_WRITE;
        rslt = bhi385_hif_exec_cmd_generic(param_id,
                                           (const uint8_t *)config,
                                           BHI385_PHY_PARAM_ANY_MOTION_CTRL_LENGTH,
                                           &command,
                                           1,
                                           0,
                                           &dev->hif);
    }

    return rslt;
}

/**
 * @brief Function to get Any Motion configuration
 * @param[out] config  : Reference to Any Motion configuration
 * @param[in]  dev     : Device instance
 * @return API error codes
 */
int8_t bhi385_phy_sensor_ctrl_param_get_any_motion_config(bhi385_phy_sensor_ctrl_param_any_motion* config,
                                                          struct bhi385_dev *dev)
{
    int8_t rslt;
    uint8_t command;
    uint32_t act_len;
    uint8_t payload[BHI385_ROUND_UP_4_MUL(BHI385_PHY_PARAM_ANY_MOTION_CTRL_LENGTH)] = { 0U };

    if ((config == NULL) || (dev == NULL))
    {
        rslt = BHI385_E_NULL_PTR;
    }
    else
    {
        command = (uint8_t)BHI385_PHY_PARAM_ANY_MOTION_COD;
        rslt = bhi385_get_phy_sensor_ctrl_info(BHI385_PHYS_SENSOR_ID_PHYS_ANY_MOTION,
                                               command,
                                               payload,
                                               BHI385_ROUND_UP_4_MUL(BHI385_PHY_PARAM_ANY_MOTION_CTRL_LENGTH),
                                               &act_len,
                                               &dev->hif);
        if (rslt == BHI385_OK)
        {
            config->duration = (uint16_t)payload[0 + BHI385_PHY_SENSOR_CTRL_PARAM_CODE_LENGTH];
            config->duration |=
                (uint16_t)(((uint16_t)payload[1 + BHI385_PHY_SENSOR_CTRL_PARAM_CODE_LENGTH] &
                            BHI385_PHY_PARAM_ANY_MOTION_DURATION_UPPER_MASK) << 8U);
            config->axis_sel =
                (uint16_t)(((uint16_t)payload[1 + BHI385_PHY_SENSOR_CTRL_PARAM_CODE_LENGTH] >>
                            BHI385_PHY_PARAM_ANY_MOTION_AXIS_SEL_SHIFT) & BHI385_PHY_PARAM_ANY_MOTION_AXIS_SEL_MASK);
            config->threshold = (uint16_t)payload[2 + BHI385_PHY_SENSOR_CTRL_PARAM_CODE_LENGTH];
            config->threshold |=
                (uint16_t)(((uint16_t)payload[3 + BHI385_PHY_SENSOR_CTRL_PARAM_CODE_LENGTH] &
                            BHI385_PHY_PARAM_ANY_MOTION_THRESHOLD_UPPER_MASK) << 8U);
        }
    }

    return rslt;
}

/**
 * @brief Function to set No Motion configuration
 * @param[in] config  : Reference to No Motion configuration
 * @param[in] dev     : Device instance
 * @return API error codes
 */
int8_t bhi385_phy_sensor_ctrl_param_set_no_motion_config(const bhi385_phy_sensor_ctrl_param_no_motion* config,
                                                         struct bhi385_dev *dev)
{
    int8_t rslt;
    uint16_t param_id;
    uint8_t command;

    if ((config == NULL) || (dev == NULL))
    {
        rslt = BHI385_E_NULL_PTR;
    }
    else
    {
        param_id = (uint16_t)BHI385_PARAM_PHY_SENSOR_CTRL_PAGE_BASE | BHI385_PHYS_SENSOR_ID_PHYS_NO_MOTION;
        command = (uint8_t)BHI385_PHY_PARAM_NO_MOTION_COD | BHI385_PARAM_PHY_SENSOR_CTRL_WRITE;
        rslt = bhi385_hif_exec_cmd_generic(param_id,
                                           (const uint8_t *)config,
                                           BHI385_PHY_PARAM_NO_MOTION_CTRL_LENGTH,
                                           &command,
                                           1,
                                           0,
                                           &dev->hif);
    }

    return rslt;
}

/**
 * @brief Function to get No Motion configuration
 * @param[out] config  : Reference to No Motion configuration
 * @param[in]  dev     : Device instance
 * @return API error codes
 */
int8_t bhi385_phy_sensor_ctrl_param_get_no_motion_config(bhi385_phy_sensor_ctrl_param_no_motion* config,
                                                         struct bhi385_dev *dev)
{
    int8_t rslt;
    uint8_t command;
    uint32_t act_len;
    uint8_t payload[BHI385_ROUND_UP_4_MUL(BHI385_PHY_PARAM_NO_MOTION_CTRL_LENGTH)] = { 0U };

    if ((config == NULL) || (dev == NULL))
    {
        rslt = BHI385_E_NULL_PTR;
    }
    else
    {
        command = (uint8_t)BHI385_PHY_PARAM_NO_MOTION_COD;
        rslt = bhi385_get_phy_sensor_ctrl_info(BHI385_PHYS_SENSOR_ID_PHYS_NO_MOTION,
                                               command,
                                               payload,
                                               BHI385_ROUND_UP_4_MUL(BHI385_PHY_PARAM_NO_MOTION_CTRL_LENGTH),
                                               &act_len,
                                               &dev->hif);
        if (rslt == BHI385_OK)
        {
            config->duration = (uint16_t)payload[0 + BHI385_PHY_SENSOR_CTRL_PARAM_CODE_LENGTH];
            config->duration |=
                (uint16_t)(((uint16_t)payload[1 + BHI385_PHY_SENSOR_CTRL_PARAM_CODE_LENGTH] &
                            BHI385_PHY_PARAM_NO_MOTION_DURATION_UPPER_MASK) << 8U);
            config->axis_sel =
                (uint16_t)(((uint16_t)payload[1 + BHI385_PHY_SENSOR_CTRL_PARAM_CODE_LENGTH] >>
                            BHI385_PHY_PARAM_NO_MOTION_AXIS_SEL_SHIFT) & BHI385_PHY_PARAM_NO_MOTION_AXIS_SEL_MASK);
            config->threshold = (uint16_t)payload[2 + BHI385_PHY_SENSOR_CTRL_PARAM_CODE_LENGTH];
            config->threshold |=
                (uint16_t)(((uint16_t)payload[3 + BHI385_PHY_SENSOR_CTRL_PARAM_CODE_LENGTH] &
                            BHI385_PHY_PARAM_NO_MOTION_THRESHOLD_UPPER_MASK) << 8U);
        }
    }

    return rslt;
}

/**
 * @brief Function to set Wrist Gesture Detector configuration
 * @param[in] config  : Reference to Wrist Gesture Detector configuration
 * @param[in] dev     : Device instance
 * @return API error codes
 */
int8_t bhi385_phy_sensor_ctrl_param_set_wrist_gesture_cfg(
    const bhi385_phy_sensor_ctrl_param_wrist_gesture_detector* config,
    struct bhi385_dev *dev)
{
    int8_t rslt;
    uint16_t param_id;
    uint8_t command;

    if ((config == NULL) || (dev == NULL))
    {
        rslt = BHI385_E_NULL_PTR;
    }
    else
    {
        param_id = (uint16_t)BHI385_PARAM_PHY_SENSOR_CTRL_PAGE_BASE | BHI385_PHYS_SENSOR_ID_WRIST_GESTURE_DETECT;
        command = (uint8_t)BHI385_PHY_PARAM_WRIST_GESTURE_DETECTOR_COD | BHI385_PARAM_PHY_SENSOR_CTRL_WRITE;
        rslt = bhi385_hif_exec_cmd_generic(param_id,
                                           (const uint8_t *)config,
                                           BHI385_PHY_PARAM_WRIST_GESTURE_DETECTOR_CTRL_LENGTH,
                                           &command,
                                           1,
                                           0,
                                           &dev->hif);
    }

    return rslt;
}

/**
 * @brief Function to get Wrist Gesture Detector configuration
 * @param[out] config  : Reference to Wrist Gesture Detector configuration
 * @param[in]  dev     : Device instance
 * @return API error codes
 */
int8_t bhi385_phy_sensor_ctrl_param_get_wrist_gesture_cfg(bhi385_phy_sensor_ctrl_param_wrist_gesture_detector* config,
                                                          struct bhi385_dev *dev)
{
    int8_t rslt;
    uint8_t command;
    uint32_t act_len;
    uint8_t payload[BHI385_ROUND_UP_4_MUL(BHI385_PHY_PARAM_WRIST_GESTURE_DETECTOR_CTRL_LENGTH)] = { 0U };

    if ((config == NULL) || (dev == NULL))
    {
        rslt = BHI385_E_NULL_PTR;
    }
    else
    {
        command = (uint8_t)BHI385_PHY_PARAM_WRIST_GESTURE_DETECTOR_COD;
        rslt = bhi385_get_phy_sensor_ctrl_info(BHI385_PHYS_SENSOR_ID_WRIST_GESTURE_DETECT,
                                               command,
                                               payload,
                                               BHI385_ROUND_UP_4_MUL(
                                                   BHI385_PHY_PARAM_WRIST_GESTURE_DETECTOR_CTRL_LENGTH),
                                               &act_len,
                                               &dev->hif);
        if (rslt == BHI385_OK)
        {
            config->min_flick_peak_y_thres = BHI385_LE2U16(&payload[0 + BHI385_PHY_SENSOR_CTRL_PARAM_CODE_LENGTH]);
            config->min_flick_peak_z_thres = BHI385_LE2U16(&payload[2 + BHI385_PHY_SENSOR_CTRL_PARAM_CODE_LENGTH]);
            config->gravity_bounds_x_pos = BHI385_LE2U16(&payload[4 + BHI385_PHY_SENSOR_CTRL_PARAM_CODE_LENGTH]);
            config->gravity_bounds_x_neg = BHI385_LE2U16(&payload[6 + BHI385_PHY_SENSOR_CTRL_PARAM_CODE_LENGTH]);
            config->gravity_bounds_y_neg = BHI385_LE2U16(&payload[8 + BHI385_PHY_SENSOR_CTRL_PARAM_CODE_LENGTH]);
            config->gravity_bounds_z_neg = BHI385_LE2U16(&payload[10 + BHI385_PHY_SENSOR_CTRL_PARAM_CODE_LENGTH]);
            config->flick_peak_decay_coeff = BHI385_LE2U16(&payload[12 + BHI385_PHY_SENSOR_CTRL_PARAM_CODE_LENGTH]);
            config->lp_mean_filter_coeff = BHI385_LE2U16(&payload[14 + BHI385_PHY_SENSOR_CTRL_PARAM_CODE_LENGTH]);
            config->max_duration_jiggle_peaks = BHI385_LE2U16(&payload[16 + BHI385_PHY_SENSOR_CTRL_PARAM_CODE_LENGTH]);
            config->device_pos = payload[18 + BHI385_PHY_SENSOR_CTRL_PARAM_CODE_LENGTH];
        }
    }

    return rslt;
}

/**
 * @brief Function to set Step Counter configuration
 * @param[in] config  : Reference to Step Counter configuration
 * @param[in] dev     : Device instance
 * @return API error codes
 */
int8_t bhi385_phy_sensor_ctrl_param_set_step_counter_config(const bhi385_phy_sensor_ctrl_param_step_counter* config,
                                                            struct bhi385_dev *dev)
{
    int8_t rslt;
    uint16_t param_id;
    uint8_t command;

    if ((config == NULL) || (dev == NULL))
    {
        rslt = BHI385_E_NULL_PTR;
    }
    else
    {
        param_id = (uint16_t)BHI385_PARAM_PHY_SENSOR_CTRL_PAGE_BASE | BHI385_PHYS_SENSOR_ID_PHYS_STEP_COUNTER;
        command = (uint8_t)BHI385_PHY_PARAM_STEP_COUNTER_COD | BHI385_PARAM_PHY_SENSOR_CTRL_WRITE;
        rslt = bhi385_hif_exec_cmd_generic(param_id,
                                           (const uint8_t *)config,
                                           BHI385_PHY_PARAM_STEP_COUNTER_CTRL_LENGTH,
                                           &command,
                                           1,
                                           0,
                                           &dev->hif);
    }

    return rslt;
}

/**
 * @brief Function to get Step Counter configuration
 * @param[out] config  : Reference to Step Counter configuration
 * @param[in]  dev     : Device instance
 * @return API error codes
 */
int8_t bhi385_phy_sensor_ctrl_param_get_step_counter_config(bhi385_phy_sensor_ctrl_param_step_counter* config,
                                                            struct bhi385_dev *dev)
{
    int8_t rslt;
    uint8_t command;
    uint32_t act_len;
    uint8_t payload[BHI385_ROUND_UP_4_MUL(BHI385_PHY_PARAM_STEP_COUNTER_CTRL_LENGTH)] = { 0U };

    if ((config == NULL) || (dev == NULL))
    {
        rslt = BHI385_E_NULL_PTR;
    }
    else
    {
        command = (uint8_t)BHI385_PHY_PARAM_STEP_COUNTER_COD;
        rslt = bhi385_get_phy_sensor_ctrl_info(BHI385_PHYS_SENSOR_ID_PHYS_STEP_COUNTER,
                                               command,
                                               payload,
                                               BHI385_ROUND_UP_4_MUL(BHI385_PHY_PARAM_STEP_COUNTER_CTRL_LENGTH),
                                               &act_len,
                                               &dev->hif);
        if (rslt == BHI385_OK)
        {
            config->env_min_dist_up = BHI385_LE2U16(&payload[0 + BHI385_PHY_SENSOR_CTRL_PARAM_CODE_LENGTH]);
            config->env_coef_up = BHI385_LE2U16(&payload[2 + BHI385_PHY_SENSOR_CTRL_PARAM_CODE_LENGTH]);
            config->env_min_dist_down = BHI385_LE2U16(&payload[4 + BHI385_PHY_SENSOR_CTRL_PARAM_CODE_LENGTH]);
            config->env_coef_down = BHI385_LE2U16(&payload[6 + BHI385_PHY_SENSOR_CTRL_PARAM_CODE_LENGTH]);
            config->step_buffer_size = BHI385_LE2U16(&payload[8 + BHI385_PHY_SENSOR_CTRL_PARAM_CODE_LENGTH]);
            config->mean_val_decay = BHI385_LE2U16(&payload[10 + BHI385_PHY_SENSOR_CTRL_PARAM_CODE_LENGTH]);
            config->mean_step_dur = BHI385_LE2U16(&payload[12 + BHI385_PHY_SENSOR_CTRL_PARAM_CODE_LENGTH]);
            config->filter_coeff_b2 = BHI385_LE2U16(&payload[14 + BHI385_PHY_SENSOR_CTRL_PARAM_CODE_LENGTH]);
            config->filter_coeff_b1 = BHI385_LE2U16(&payload[16 + BHI385_PHY_SENSOR_CTRL_PARAM_CODE_LENGTH]);
            config->filter_coeff_b0 = BHI385_LE2U16(&payload[18 + BHI385_PHY_SENSOR_CTRL_PARAM_CODE_LENGTH]);
            config->filter_coeff_a2 = BHI385_LE2U16(&payload[20 + BHI385_PHY_SENSOR_CTRL_PARAM_CODE_LENGTH]);
            config->filter_coeff_a1 = BHI385_LE2U16(&payload[22 + BHI385_PHY_SENSOR_CTRL_PARAM_CODE_LENGTH]);
            config->filter_cascade_enabled = BHI385_LE2U16(&payload[24 + BHI385_PHY_SENSOR_CTRL_PARAM_CODE_LENGTH]);
            config->peak_duration_min_walking = BHI385_LE2U16(&payload[26 + BHI385_PHY_SENSOR_CTRL_PARAM_CODE_LENGTH]);
            config->peak_duration_min_running = BHI385_LE2U16(&payload[28 + BHI385_PHY_SENSOR_CTRL_PARAM_CODE_LENGTH]);
            config->step_duration_max = BHI385_LE2U16(&payload[30 + BHI385_PHY_SENSOR_CTRL_PARAM_CODE_LENGTH]);
            config->step_duration_window = BHI385_LE2U16(&payload[32 + BHI385_PHY_SENSOR_CTRL_PARAM_CODE_LENGTH]);
            config->half_step_enabled = BHI385_LE2U16(&payload[34 + BHI385_PHY_SENSOR_CTRL_PARAM_CODE_LENGTH]);
            config->activity_detection_factor = BHI385_LE2U16(&payload[36 + BHI385_PHY_SENSOR_CTRL_PARAM_CODE_LENGTH]);
            config->activity_detection_thres = BHI385_LE2U16(&payload[38 + BHI385_PHY_SENSOR_CTRL_PARAM_CODE_LENGTH]);
            config->step_counter_increment = BHI385_LE2U16(&payload[40 + BHI385_PHY_SENSOR_CTRL_PARAM_CODE_LENGTH]);
            config->step_duration_pp_enabled = BHI385_LE2U16(&payload[42 + BHI385_PHY_SENSOR_CTRL_PARAM_CODE_LENGTH]);
            config->step_dur_thres = BHI385_LE2U16(&payload[44 + BHI385_PHY_SENSOR_CTRL_PARAM_CODE_LENGTH]);
            config->en_mcr_pp = BHI385_LE2U16(&payload[46 + BHI385_PHY_SENSOR_CTRL_PARAM_CODE_LENGTH]);
            config->mcr_thres = BHI385_LE2U16(&payload[48 + BHI385_PHY_SENSOR_CTRL_PARAM_CODE_LENGTH]);
            config->sc_26 = BHI385_LE2U16(&payload[50 + BHI385_PHY_SENSOR_CTRL_PARAM_CODE_LENGTH]);
            config->sc_27 = BHI385_LE2U16(&payload[52 + BHI385_PHY_SENSOR_CTRL_PARAM_CODE_LENGTH]);
        }
    }

    return rslt;
}
