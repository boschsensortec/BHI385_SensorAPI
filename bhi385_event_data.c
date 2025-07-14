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
* @file       bhi385_event_data.c
* @date       2025-03-28
* @version    v1.0.0
*
*/

#include "bhi385_event_data.h"
#include "bhi385_hif.h"

/**
 * @brief Function to parse FIFO frame data into 3 axes vector
 * @param[in] data      : Reference to the data buffer storing data from the FIFO
 * @param[out] vector   : Reference to the data buffer to store vector
 */
void bhi385_event_data_parse_xyz(const uint8_t *data, struct bhi385_event_data_xyz *vector)
{
    vector->x = BHI385_LE2S16(data);
    vector->y = BHI385_LE2S16(data + 2);
    vector->z = BHI385_LE2S16(data + 4);
}

/**
 * @brief Function to parse FIFO frame data into quaternion
 * @param[in] data          : Reference to the data buffer storing data from the FIFO
 * @param[out] quaternion   : Reference to the data buffer to store quaternion
 */
void bhi385_event_data_parse_quaternion(const uint8_t *data, struct bhi385_event_data_quaternion *quaternion)
{
    quaternion->x = BHI385_LE2S16(data);
    quaternion->y = BHI385_LE2S16(data + 2);
    quaternion->z = BHI385_LE2S16(data + 4);
    quaternion->w = BHI385_LE2S16(data + 6);
    quaternion->accuracy = BHI385_LE2U16(data + 8);
}

/*!
 * @brief Parsing the fifo data to Multi Tap output format
 *
 * @param[in]  data    Multi Tap data
 * @param[out] output  Reference to store parameter data
 *
 * @return  API error codes
 *
 */
int8_t bhi385_event_data_multi_tap_parsing(const uint8_t *data, uint8_t* output)
{
    int8_t rslt = BHI385_OK;

    if ((data == NULL) || (output == NULL))
    {
        rslt = BHI385_E_NULL_PTR;
    }
    else
    {
        *output = *data;
    }

    return rslt;
}

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
                                                      bhi385_event_data_wrist_gesture_detect_t *output)
{
    int8_t rslt = BHI385_OK;

    if ((data == NULL) || (output == NULL))
    {
        rslt = BHI385_E_NULL_PTR;
    }
    else
    {
        output->wrist_gesture = (enum bhi385_event_data_wrist_gesture_activity)*(data);
    }

    return rslt;
}
