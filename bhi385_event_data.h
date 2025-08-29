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
* @file       bhi385_event_data.h
* @date       2025-08-20
* @version    v2.0.0
*
*/

#ifndef __BHI385_EVENT_DATA_H__
#define __BHI385_EVENT_DATA_H__

/* Start of CPP Guard */
#ifdef __cplusplus
extern "C" {
#endif /*__cplusplus */

#include "bhi385_event_data_defs.h"

/**
 * @brief Function to parse FIFO frame data into quaternion
 * @param[in] data          : Reference to the data buffer storing data from the FIFO
 * @param[out] quaternion   : Reference to the data buffer to store quaternion
 */
void bhi385_event_data_parse_quaternion(const uint8_t *data, struct bhi385_event_data_quaternion *quaternion);

/**
 * @brief Function to parse FIFO frame data into 3 axes vector
 * @param[in] data      : Reference to the data buffer storing data from the FIFO
 * @param[out] vector   : Reference to the data buffer to store vector
 */
void bhi385_event_data_parse_xyz(const uint8_t *data, struct bhi385_event_data_xyz *vector);

/*!
 * @brief Parsing the fifo data to MULTI_TAP output structure format
 *
 * @param[in]  data    Multi Tap data
 * @param[out] output  Reference to store parameter data
 *
 * @return  API error codes
 *
 */
int8_t bhi385_event_data_multi_tap_parsing(const uint8_t *data, uint8_t *output);

/*!
 * @brief To parse Wrist Gesture Detection Data
 *
 * @param[in] data  : Gesture Detection Data
 * @param[out] Wrist : Gesture Detection Output Structure
 *
 * @return API error codes
 *
 */
int8_t bhi385_event_data_wrist_gesture_detect_parsing(const uint8_t *data,
                                                      bhi385_event_data_wrist_gesture_detect_t *output);

/* End of CPP Guard */
#ifdef __cplusplus
}
#endif /*__cplusplus */

#endif /* __BHI385_EVENT_DATA_H__ */
