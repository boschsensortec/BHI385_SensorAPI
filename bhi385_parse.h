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
* @file       bhi385_parse.h
* @date       2025-08-20
* @version    v2.0.0
*
*/
#ifndef __BHI385_PARSE_H__
#define __BHI385_PARSE_H__

/* Start of CPP Guard */
#ifdef __cplusplus
extern "C" {
#endif /*__cplusplus */

#include <stdbool.h>

#include "bhi385.h"
#include "bhi385_logbin.h"
#include "bhi385_klio_param.h"

#define PARSE_FLAG_NONE              UINT8_C(0x00)
#define PARSE_FLAG_STREAM            UINT8_C(0x01)
#define PARSE_FLAG_LOG               UINT8_C(0x02)
#define PARSE_FLAG_HEXSTREAM         UINT8_C(0x04)

#define PARSE_SET_FLAG(var, flag)    (var | flag)
#define PARSE_CLEAR_FLAG(var, flag)  (var & ~flag)

struct bhi385_parse_sensor_details
{
    uint8_t id;
    uint8_t accuracy;
    float scaling_factor;
    uint8_t parse_flag;
};

struct bhi385_parse_ref
{
    struct bhi385_parse_sensor_details sensor[BHI385_MAX_SIMUL_SENSORS];
    struct bhi385_dev *bhy;
    struct bhi385_logbin_dev logdev;
};

/* *INDENT-OFF* */
/* Contains information about klio capabilities and current state, as well as runtime configuration */
typedef struct
{
    uint8_t max_patterns;
    uint8_t max_pattern_blob_size;
    uint8_t auto_load_pattern_write_index;
    uint8_t auto_load_pattern;
} klio_info;
/* *INDENT-ON* */

/**
* @brief Function to get sensor details
* @param[in] id  : Sensor ID
* @param[in] ref : Parse reference
* @return Sensor details on success, or NULL on failure
*/
struct bhi385_parse_sensor_details *bhi385_parse_get_sensor_details(uint8_t id, struct bhi385_parse_ref *ref);

/**
* @brief Function to add sensor details
* @param[in] id  : Sensor ID
* @param[in] ref : Parse reference
* @return Sensor details on success, or NULL on failure
*/
struct bhi385_parse_sensor_details *bhi385_parse_add_sensor_details(uint8_t id, struct bhi385_parse_ref *ref);

/**
* @brief Function to parse meta event (wake-up and non-wake-up)
* @param[in] callback_info : Pointer to callback information
* @param[in] callback_ref  : Pointer to callback reference
*/
void bhi385_parse_meta_event(const struct bhi385_fifo_parse_data_info *callback_info, void *callback_ref);

/**
* @brief Function to parse 3-axis format
* @param[in] callback_info : Pointer to callback information
* @param[in] callback_ref  : Pointer to callback reference
*/
void bhi385_parse_3axis_s16(const struct bhi385_fifo_parse_data_info *callback_info, void *callback_ref);

/**
* @brief Function to parse quaternion format
* @param[in] callback_info : Pointer to callback information
* @param[in] callback_ref  : Pointer to callback reference
*/
void bhi385_parse_quaternion(const struct bhi385_fifo_parse_data_info *callback_info, void *callback_ref);

/**
* @brief Function to parse scalar event format
* @param[in] callback_info : Pointer to callback information
* @param[in] callback_ref  : Pointer to callback reference
*/
void bhi385_parse_scalar_event(const struct bhi385_fifo_parse_data_info *callback_info, void *callback_ref);

/**
* @brief Function to parse activity format
* @param[in] callback_info : Pointer to callback information
* @param[in] callback_ref  : Pointer to callback reference
*/
void bhi385_parse_activity(const struct bhi385_fifo_parse_data_info *callback_info, void *callback_ref);

/**
* @brief Function to parse generic format
* @param[in] callback_info : Pointer to callback information
* @param[in] callback_ref  : Pointer to callback reference
*/
void bhi385_parse_generic(const struct bhi385_fifo_parse_data_info *callback_info, void *callback_ref);

/**
* @brief Function to parse debug message
* @param[in] callback_info : Pointer to callback information
* @param[in] callback_ref  : Pointer to callback reference
*/
void bhi385_parse_debug_message(const struct bhi385_fifo_parse_data_info *callback_info, void *callback_ref);

/**
* @brief Function to parse Klio
* @param[in] callback_info : Pointer to callback information
* @param[in] callback_ref  : Pointer to callback reference
*/
void bhi385_parse_klio(const struct bhi385_fifo_parse_data_info *callback_info, void *callback_ref);

void bhi385_parse_klio_handle_learnt_pattern(const struct bhi385_fifo_parse_data_info *callback_info,
                                             uint32_t s,
                                             uint32_t ns,
                                             struct bhi385_dev *bhy);

/**
* @brief Function to parse log Klio
* @param[in] callback_info : Pointer to callback information
* @param[in] callback_ref  : Pointer to callback reference
*/
void bhi385_parse_klio_log(const struct bhi385_fifo_parse_data_info *callback_info, void *callback_ref);

/**
* @brief Function to parse for Multi-tap
* @param[in] callback_info : Pointer to callback information
* @param[in] callback_ref  : Pointer to callback reference
*/
void bhi385_parse_multitap(const struct bhi385_fifo_parse_data_info *callback_info, void *callback_ref);

/**
* @brief Function to parse for Wrist Gesture Detector
* @param[in] callback_info : Pointer to callback information
* @param[in] callback_ref  : Pointer to callback reference
*/
void bhi385_parse_wrist_gesture_detect(const struct bhi385_fifo_parse_data_info *callback_info, void *callback_ref);

/**
* @brief Function to set down sampling flag
* @param[in] sen_id : Virtual sensor ID
* @param[in] enable : Down sampling value
*/
void bhi385_set_downsampling_flag(uint8_t sen_id, bool enable);

/**
* @brief Function to get down sampling flag
* @param[in] sen_id  : Virtual sensor ID
* @return Down sampling value
*/
bool bhi385_get_downsampling_flag(uint8_t sen_id);

/**
* @brief Function to set down sampling ratio
* @param[in] sen_id : Virtual sensor ID
* @param[in] enable : Down sampling ratio
*/
void bhi385_set_downsampling_odr(uint8_t sen_id, int16_t odr);

/**
* @brief Function to set Klio information (capabilities, state and runtime configuration)
* @param[in] klio_info : Klio information
*/
void bhi385_set_klio_info(const klio_info* info);

/**
* @brief Function to get Klio information (capabilities, state and runtime configuration)
* @return Klio information
*/
klio_info* bhi385_get_klio_info(void);

/**
* @brief Function to parse log Step counter
* @param[in] callback_info : Pointer to callback information
* @param[in] callback_ref  : Pointer to callback reference
*/
void bhi385_parse_step_counter_data(const struct bhi385_fifo_parse_data_info *callback_info, void *callback_ref);

/**
* @brief Function to parse log Wrist Wear Wakeup
* @param[in] callback_info : Pointer to callback information
* @param[in] callback_ref  : Pointer to callback reference
*/
void bhi385_parse_wrist_wear_wakeup_data(const struct bhi385_fifo_parse_data_info *callback_info, void *callback_ref);

/* End of CPP Guard */
#ifdef __cplusplus
}
#endif /*__cplusplus */

#endif /* __BHI385_PARSE_H__ */
