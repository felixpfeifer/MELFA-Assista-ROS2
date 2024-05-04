// Copyright 2018 Tokyo Opensource Robotics Kyokai Association (TORK)
// Licensed under the Apache License, Version 2.0 (the "License"); // Copyright 2018 Tokyo Opensource Robotics Kyokai Association (TORK)
// Licensed under the Apache License, Version 2.0 (the "License"); // you may not use this file except in compliance with the License.
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
// http://www.apache.org/licenses/LICENSE-2.0
// // Unless required by applicable law or agreed upon.
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS, // WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either expressed or implied.
// See the License for the specific language governing permissions and
//limitations under the License.

//***************************************************************
// Real-time control sample program
// Communication packet data structure definition header file
//***************************************************************
// strdef.h

#include <cstdint>

#define VER_H7
/********************************************************/
/* Joint coordinate system (unused axes are set to 0) */
/* For details of each component, refer to the instruction manuals that come with each robot */
/* Refer to the instruction manual that comes with each robot */
/********************************************************/
typedef struct {
    float j1; // J1 axis angle (radians)
    float j2; // J2-axis angle (radians)
    float j3; // J3-axis angle (radians)
    float j4; // J4-axis angle (radians)
    float j5; // J5-axis angle (radians)
    float j6; // J6 axis angle (radians)
    float j7; // additional axis 1 (J7 axis angle) (radians)
    float j8; // additional axis 2 (J8 axis angle) (radians)
} JOINT;
/********************************************************/
/* Cartesian coordinate system (unused axes are set to 0) */
/* For details of each component, please refer to the instruction manuals that come with each robot */ /*
/* Refer to the instruction manual supplied with each robot */ /*
/********************************************************/
typedef struct {
    float x; // X-axis coordinate value (mm)
    float y; // Y-axis coordinate value (mm)
    float z; // Z-axis coordinate value (mm)
    float a; // A-axis coordinate value (radians)
    float b; // B-axis coordinate value (radians)
    float c; // C-axis coordinate value (radians)
    float l1; // additional axis 1 (mm or radians)
    float l2; // additional axis 2 (mm or radians)
} WORLD;
typedef struct {
    WORLD w;
    uint32_t sflg1; // structure flag 1
    uint32_t sflg2; // structure flag 2
} POSE;

/********************************************************/
/* Pulse coordinate system (unused axes should be 0) */ /*
/* Coordinates of each joint in terms of motor pulse values */
/********************************************************/
typedef struct {
    int32_t p1; // Motor 1 axis
    int32_t p2; // motor 2 axis
    int32_t p3; // motor axis 3
    int32_t p4; // motor axis 4
    int32_t p5; // motor 5-axis
    int32_t p6; // motor 6-axis
    int32_t p7; // additional axis 1 (motor axis 7)
    int32_t p8; // additional axis 2 (motor axis 8)
} PULSE;
/****************************************/
/* real-time function communication data packet */
/****************************************/
typedef struct enet_rtcmd_str {
    uint16_t Command; // Command.
#define MXT_CMD_NULL 0 // no real-time external command
#define MXT_CMD_MOVE 1 // with real-time external command
#define MXT_CMD_END 255 // End of real-time external command
    uint16_t SendType; // data type specification of the command data
    uint16_t RecvType; // monitor data type specified

    //////////// Command or monitor data type //

#define MXT_TYP_NULL 0 // No data

    // for command and monitor ////////////////////

#define MXT_TYP_POSE 1 // Quadrature data (command value)
#define MXT_TYP_JOINT 2 // Joint data (command value)
#define MXT_TYP_PULSE 3 // Pulse data (command value)

    // For position-related monitoring ////////////////////////
#define MXT_TYP_FPOSE 4 // Quadrature data (after command value filtering)
#define MXT_TYP_FJOINT 5 // Joint data (after command value filtering)
#define MXT_TYP_FPULSE 6 // Pulse data (after command value filtering)
#define MXT_TYP_FB_POSE 7 // Quadrature data (encoder feedback value)
#define MXT_TYP_FB_JOINT 8 // Joint data (encoder feedback value)
#define MXT_TYP_FB_PULSE 9 // Pulse data (encoder feedback value)
    // For current-related monitoring ///////////////////////

#define MXT_TYP_CMDCUR 10 // Current command
#define MXT_TYP_FBKCUR 11 // current feedback
    union rtdata { // command data
        POSE pos; // Cartesian type [mm/rad].
        JOINT jnt; // Joint type [rad].
        PULSE pls; // pulse type [pls].
        int32_t lng[8]; // integer type [%/no units, etc.]
    } dat;
    uint16_t SendIOType; // send I/O signal data designation
    uint16_t RecvIOType; // specify reply input/output signal data
#define MXT_IO_NULL 0 // no data
#define MXT_IO_OUT 1 // output signal
#define MXT_IO_IN 2 // input signal
    uint16_t BitTop; // Leading bit number
    uint16_t BitMask; // Bit mask pattern specification for transmission (0x0001-0xffff)
    uint16_t IoData; // Input/output signal data (0x0000-0xffff)
    uint16_t TCount; // Counter value for timeout time
    uint32_t CCount; // Counter value for communication data
    uint16_t RecvType1; // data type specification for reply data type 1
    union rtdata1 { // monitor data1
        POSE pos1; // orthogonal type [mm/rad].
        JOINT jnt1; // joint type [rad].
        PULSE pls1; // Pulse type [pls].
        int32_t lng1[8]; // integer type [%/no units, etc.]
    } dat1;
    uint16_t RecvType2; // Reply data type designation 2
    union rtdata2 { // monitor data2
        POSE pos2; // orthogonal type [mm/rad].
        JOINT jnt2; // joint type [rad].
        PULSE pls2; // pulse type [pls] or integer type [%/no units, etc.]
        int32_t lng2[8]; // integer type [%/no unit, etc.]
    } dat2;
    uint16_t RecvType3; // reply data type specification 3
    union rtdata3 { // monitor data3
        POSE pos3; // orthogonal type [mm/rad].
        JOINT jnt3; // joint type [rad].
        PULSE pls3; // pulse type [pls] or integer type [%/no units, etc.]
        int32_t lng3[8]; // integer type [%/no unit, etc.]
    } dat3;
} MXTCMD;