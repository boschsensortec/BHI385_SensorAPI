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
* @file       bhi385_event_data_defs.h
* @date       2025-03-28
* @version    v1.0.0
*
*/

#ifndef __BHI385_EVENT_DATA_DEFS_H__
#define __BHI385_EVENT_DATA_DEFS_H__

/* Start of CPP Guard */
#ifdef __cplusplus
extern "C" {
#endif /*__cplusplus */

#include <stdio.h>
#include <stdint.h>

#ifndef BHI385_PACKED
#define BHI385_PACKED  __attribute__ ((__packed__))
#endif

struct bhi385_event_data_xyz
{
    int16_t x;
    int16_t y;
    int16_t z;
};

struct bhi385_event_data_quaternion
{
    int16_t x;
    int16_t y;
    int16_t z;
    int16_t w;
    uint16_t accuracy;
};

/*!
 * Multi Tap Setting.
 */
typedef enum {
    BHI385_NO_TAP,
    BHI385_SINGLE_TAP,
    BHI385_DOUBLE_TAP,
    BHI385_DOUBLE_SINGLE_TAP,
    BHI385_TRIPLE_TAP,
    BHI385_TRIPLE_SINGLE_TAP,
    BHI385_TRIPLE_DOUBLE_TAP,
    BHI385_TRIPLE_DOUBLE_SINGLE_TAP
} bhi385_event_data_multi_tap;

/*!
 * Multi Tap Output.
 */
static const char * const bhi385_event_data_multi_tap_string_out[] = {
    [BHI385_NO_TAP] = "NO_TAP", [BHI385_SINGLE_TAP] = "SINGLE_TAP", [BHI385_DOUBLE_TAP] = "DOUBLE_TAP",
    [BHI385_DOUBLE_SINGLE_TAP] = "DOUBLE_SINGLE_TAP", [BHI385_TRIPLE_TAP] = "TRIPLE_TAP",
    [BHI385_TRIPLE_SINGLE_TAP] = "TRIPLE_SINGLE_TAP", [BHI385_TRIPLE_DOUBLE_TAP] = "TRIPLE_DOUBLE_TAP",
    [BHI385_TRIPLE_DOUBLE_SINGLE_TAP] = "TRIPLE_DOUBLE_SINGLE_TAP" };                                                   /*lint -e528
                                                                                                           * */

enum bhi385_event_data_wrist_gesture_activity {
    BHI385_NO_GESTURE,
    BHI385_WRIST_SHAKE_JIGGLE = 0x03,
    BHI385_FLICK_IN,
    BHI385_FLICK_OUT
};

typedef struct bhi385_event_data_wrist_gesture_detect
{
    enum bhi385_event_data_wrist_gesture_activity wrist_gesture;
} __attribute__ ((packed)) bhi385_event_data_wrist_gesture_detect_t;

static const char * const bhi385_event_data_wrist_gesture_detect_output[] = {
    [BHI385_NO_GESTURE] = "NO_GESTURE", [BHI385_WRIST_SHAKE_JIGGLE] = "WRIST_SHAKE_JIGGLE",
    [BHI385_FLICK_IN] = "FLICK_IN", [BHI385_FLICK_OUT] = "FLICK_OUT" }; /*lint -e528 */

/* End of CPP Guard */
#ifdef __cplusplus
}
#endif /*__cplusplus */

#endif /* __BHI385_EVENT_DATA_DEFS_H__ */
