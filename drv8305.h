/* Microchip Technology Inc. and its subsidiaries.  You may use this software 
 * and any derivatives exclusively with Microchip products. 
 * 
 * THIS SOFTWARE IS SUPPLIED BY MICROCHIP "AS IS".  NO WARRANTIES, WHETHER 
 * EXPRESS, IMPLIED OR STATUTORY, APPLY TO THIS SOFTWARE, INCLUDING ANY IMPLIED 
 * WARRANTIES OF NON-INFRINGEMENT, MERCHANTABILITY, AND FITNESS FOR A 
 * PARTICULAR PURPOSE, OR ITS INTERACTION WITH MICROCHIP PRODUCTS, COMBINATION 
 * WITH ANY OTHER PRODUCTS, OR USE IN ANY APPLICATION. 
 *
 * IN NO EVENT WILL MICROCHIP BE LIABLE FOR ANY INDIRECT, SPECIAL, PUNITIVE, 
 * INCIDENTAL OR CONSEQUENTIAL LOSS, DAMAGE, COST OR EXPENSE OF ANY KIND 
 * WHATSOEVER RELATED TO THE SOFTWARE, HOWEVER CAUSED, EVEN IF MICROCHIP HAS 
 * BEEN ADVISED OF THE POSSIBILITY OR THE DAMAGES ARE FORESEEABLE.  TO THE 
 * FULLEST EXTENT ALLOWED BY LAW, MICROCHIP'S TOTAL LIABILITY ON ALL CLAIMS 
 * IN ANY WAY RELATED TO THIS SOFTWARE WILL NOT EXCEED THE AMOUNT OF FEES, IF 
 * ANY, THAT YOU HAVE PAID DIRECTLY TO MICROCHIP FOR THIS SOFTWARE.
 *
 * MICROCHIP PROVIDES THIS SOFTWARE CONDITIONALLY UPON YOUR ACCEPTANCE OF THESE 
 * TERMS. 
 */

/* 
 * File:   
 * Author: 
 * Comments:
 * Revision history: 
 */

// This is a guard condition so that contents of this file are not included
// more than once.  
#ifndef _DRV8305_H
#define	_DRV8305_H

//#define NO_MOSFETS (1) // COMMENT OUT IF MOSFETS ARE ON THE PCB

#include <xc.h> // include processor files - each processor file is guarded.  
#include "general.h"
#include <xc.h>
#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include "spi.h"

// TODO Insert appropriate #include <>

#define CHECK_BIT(var,pos) ( ( (var) & (1<<(pos)) ) >> pos  )
#define EXTRACT_BITS(var,high,low) ( ( ( var & ( ( 1<<(high+1) )-1) ) >> low) )

// TODO Insert C++ class definitions if appropriate
#define WARNINGS_AND_WATCHDOG_RESET_ADR     0x1
#define OV_VDS_FAULTS_ADR                   0x2
#define IC_FAULTS_ADR                       0x3
#define VGS_FAULTS_ADR                      0x4
#define HS_GATE_DRIVE_CONTROL_ADR           0x5
#define LS_GATE_DRIVE_CONTROL_ADR           0x6
#define GATE_DRIVE_CONTROL_ADR              0x7
#define RESERVED_ADR                        0x8
#define IC_OPERATION_ADR                    0x9
#define SHUNT_AMPLIFIER_CONTROL_ADR         0xA
#define VOLTAGE_REGULATOR_CONTROL_ADR       0xB
#define VDS_SENSE_CONTROL_ADR               0xC


static uint8_t volatile DRV_SET_DISABLE = 0;
static uint8_t volatile DRV_SET_ENABLE = 1;

typedef enum {
    WARNINGS_AND_WATCHDOG_RESET_ENUM = 0,
    OV_VDS_FAULTS_ENUM,
    IC_FAULTS_ENUM,
    VGS_FAULTS_ENUM,
    HS_GATE_DRIVE_CONTROL_ENUM,
    LS_GATE_DRIVE_CONTROL_ENUM,
    GATE_DRIVE_CONTROL_ENUM,
    IC_OPERATION_ENUM,
    SHUNT_AMPLIFIER_CONTROL_ENUM,
    VOLTAGE_REGULATOR_CONTROL_ENUM,
    VDS_SENSE_CONTROL_ENUM
} DRV8305_READ_REGISTER;

//--------------------------------//
// BITMAP OF
// WARNINGS_AND_WATCHDOG_RESET_ADR
//--------------------------------//
typedef struct
{
    uint8_t FAULT;
    uint8_t RSVD;
    uint8_t TEMP_FLAG4;
    uint8_t PVDD_UVFL;
    uint8_t PVDD_OVFL; 
    uint8_t VDS_STATUS; 
    uint8_t VCPH_UVFL; 
    uint8_t TEMP_FLAG1;
    uint8_t TEMP_FLAG2;
    uint8_t TEMP_FLAG3;
    uint8_t OTW;
} WARNINGS_AND_WATCHDOG_RESET_MAP;

extern volatile WARNINGS_AND_WATCHDOG_RESET_MAP mWARNINGS_AND_WATCHDOG_RESET_MAP;

//--------------------------------//
// BITMAP OF
// OV_VDS_FAULTS_ADR
//--------------------------------//

typedef struct
{
    uint8_t VDS_HA;
    uint8_t VDS_LA;
    uint8_t VDS_HB;
    uint8_t VDS_LB;
    uint8_t VDS_HC;
    uint8_t VDS_LC;
    const uint8_t RSVD;
    uint8_t SNS_C_OCP;
    uint8_t SNS_B_OCP;
    uint8_t SNS_A_OCP;
} OV_VDS_FAULTS_MAP;

extern volatile OV_VDS_FAULTS_MAP mOV_VDS_FAULTS_MAP;

//--------------------------------//
// BITMAP OF
// IC_FAULTS_ADR
//--------------------------------//

typedef struct
{
    uint8_t PVDD_UVLO2;
    uint8_t WD_FAULT;
    uint8_t OTSD;
    const uint8_t RSVD1;
    uint8_t VREG_UV; 
    uint8_t AVDD_UVLO;
    uint8_t VCP_LSD_UVLO2;
    const uint8_t RSVD2;
    uint8_t VCPH_UVLO2;
    uint8_t VCPH_OVLO;
    uint8_t VCPH_OVLO_ABS;
} IC_FAULTS_MAP;

extern volatile IC_FAULTS_MAP mIC_FAULTS_MAP;

//--------------------------------//
// BITMAP OF
// VGS_FAULTS_ADR
//--------------------------------//

typedef struct
{
    uint8_t VGS_HA; 
    uint8_t VGS_LA;
    uint8_t VGS_HB;
    uint8_t VGS_LB;
    uint8_t VGS_HC;
    uint8_t VGS_LC;
    const uint8_t RSVD;
} VGS_FAULTS_MAP;

extern volatile VGS_FAULTS_MAP mVGS_FAULTS_MAP;

//--------------------------------//
// BITMAP OF
// HS_GATE_DRIVE_CONTROL_ADR
//--------------------------------//

typedef struct
{
    const uint8_t RSVD;
    uint8_t TDRIVEN;
    uint8_t IDRIVEN_HS;
    uint8_t IDRIVEP_HS;
} HS_GATE_DRIVE_CONTROL_MAP;

extern volatile HS_GATE_DRIVE_CONTROL_MAP mHS_GATE_DRIVE_CONTROL_MAP;

//--------------------------------//
// BITMAP OF
// LS_GATE_DRIVE_CONTROL_ADR
//--------------------------------//

typedef struct
{
    const uint8_t RSVD;
    uint8_t TDRIVEN;
    uint8_t IDRIVEN_LS;
    uint8_t IDRIVEP_LS;
} LS_GATE_DRIVE_CONTROL_MAP;

extern volatile LS_GATE_DRIVE_CONTROL_MAP mLS_GATE_DRIVE_CONTROL_MAP;

//--------------------------------//
// BITMAP OF
// GATE_DRIVE_CONTROL_ADR
//--------------------------------//

typedef struct
{
    const uint8_t RSVD;
    uint8_t COMM_OPTION;
    uint8_t PWM_MODE;
    uint8_t DEAD_TIME;
    uint8_t TBLANK; 
    uint8_t TVDS;
} GATE_DRIVE_CONTROL_MAP;

extern volatile GATE_DRIVE_CONTROL_MAP mGATE_DRIVE_CONTROL_MAP;

//--------------------------------//
// BITMAP OF
// IC_OPERATION_ADR
//--------------------------------//

typedef struct
{
    uint8_t FLIP_OTSD;
    uint8_t DIS_PVDD_UVLO2;
    uint8_t DIS_GDRV_FAULT;
    uint8_t EN_SNS_CLAMP;
    uint8_t WD_DLY;
    uint8_t DIS_SNS_OCP;
    uint8_t WD_EN; 
    uint8_t SLEEP;
    uint8_t CLR_FLTS;
    uint8_t SET_VCPH_UV;

} IC_OPERATION_MAP;

extern volatile IC_OPERATION_MAP mIC_OPERATION_MAP;

//--------------------------------//
// BITMAP OF
// SHUNT_AMPLIFIER_CONTROL_ADR
//--------------------------------//

typedef struct
{
    uint8_t DC_CAL_CH3;
    uint8_t DC_CAL_CH2;
    uint8_t DC_CAL_CH1;
    uint8_t CS_BLANK;
    uint8_t GAIN_CS3;
    uint8_t GAIN_CS2;
    uint8_t GAIN_CS1;
} SHUNT_AMPLIFIER_CONTROL_MAP;

extern volatile SHUNT_AMPLIFIER_CONTROL_MAP mSHUNT_AMPLIFIER_CONTROL_MAP;


//--------------------------------//
// BITMAP OF
// VOLTAGE_REGULATOR_CONTROL_ADR
//--------------------------------//

typedef struct
{
    const uint8_t RSVD1;
    uint8_t VREF_SCALE;
    const uint8_t RSVD2;
    uint8_t SLEEP_DLY;
    uint8_t DIS_VREG_PWRGD;
    uint8_t VREG_UV_LEVEL;

} VOLTAGE_REGULATOR_CONTROL_MAP;

extern volatile VOLTAGE_REGULATOR_CONTROL_MAP mVOLTAGE_REGULATOR_CONTROL_MAP;

//--------------------------------//
// BITMAP OF
// VDS_SENSE_CONTROL_ADR
//--------------------------------//

typedef struct
{
    const uint8_t RSVD;
    uint8_t VDS_LEVEL;
    uint8_t VDS_MODE;
} VDS_SENSE_CONTROL_MAP;

extern volatile VDS_SENSE_CONTROL_MAP mVDS_SENSE_CONTROL_MAP;



// TODO Insert declarations


//////////////////////////////////////////////////////////////////////////
// CHIP SPECIFIC FUNCTIONS READING / SENDING VIA CHIP SPI MODULE  ////////
//////////////////////////////////////////////////////////////////////////

uint16_t sendCommand_DRV8305(uint16_t cmd);

uint16_t readRegister(DRV8305_READ_REGISTER);

void readAllRegisters(void);

void readAllStatusRegisters(void);

void readAllControlRegisters(void);

// This function should be changed to fit the settings the user want on the
// Gate Driver
void setUp_DRV8305(void);

//////////////////////////////////////////////////////////////////////////
// FUNCTIONS TO SET OBJECTS SETTINGS THAT WAS RECEVIEVD FROM SPI  ////////
//////////////////////////////////////////////////////////////////////////

void setSettingsObject_DRV8305_WARNINGS_AND_WATCHDOG_RESET(uint16_t data);

void setSettingsObject_DRV8305_OV_VDS_FAULTS(uint16_t data);

void setSettingsObject_DRV8305_IC_FAULTS(uint16_t data);

void setSettingsObject_DRV8305_VGS_FAULTS(uint16_t data);

void setSettingsObject_DRV8305_HS_GATE_DRIVE_CONTROL(uint16_t data);

void setSettingsObject_DRV8305_LS_GATE_DRIVE_CONTROL(uint16_t data);

void setSettingsObject_DRV8305_GATE_DRIVE_CONTROL(uint16_t data );

void setSettingsObject_DRV8305_IC_OPERATION(uint16_t data );

void setSettingsObject_DRV8305_SHUNT_AMPLIFIER_CONTROL(uint16_t data);

void setSettingsObject_DRV8305_VOLTAGE_REGULATOR_CONTROL(uint16_t data);

void setSettingsObject_DRV8305_VDS_SENSE_CONTROL(uint16_t data);

////////////////////////////////////////////////////////////
// FUNCTIONS TO GET DATA THAT IS TO BE SEND VIA SPI ////////
////////////////////////////////////////////////////////////

uint16_t getDataString_DRV8305_WARNINGS_AND_WATCHDOG_RESET(bool isWrite, WARNINGS_AND_WATCHDOG_RESET_MAP bitmap);

uint16_t getDataString_DRV8305_OV_VDS_FAULTS(bool isWrite, OV_VDS_FAULTS_MAP bitmap);

uint16_t getDataString_DRV8305_IC_FAULTS(bool isWrite, IC_FAULTS_MAP bitmap);

uint16_t getDataString_DRV8305_VGS_FAULTS(bool isWrite, VGS_FAULTS_MAP bitmap);

uint16_t getDataString_DRV8305_HS_GATE_DRIVE_CONTROL(bool isWrite, HS_GATE_DRIVE_CONTROL_MAP bitmap);

uint16_t getDataString_DRV8305_LS_GATE_DRIVE_CONTROL(bool isWrite, LS_GATE_DRIVE_CONTROL_MAP bitmap);

uint16_t getDataString_DRV8305_GATE_DRIVE_CONTROL(bool isWrite, GATE_DRIVE_CONTROL_MAP bitmap);

uint16_t getDataString_DRV8305_IC_OPERATION(bool isWrite, IC_OPERATION_MAP bitmap);

uint16_t getDataString_DRV8305_SHUNT_AMPLIFIER_CONTROL(bool isWrite, SHUNT_AMPLIFIER_CONTROL_MAP bitmap);

uint16_t getDataString_DRV8305_VOLTAGE_REGULATOR_CONTROL(bool isWrite, VOLTAGE_REGULATOR_CONTROL_MAP bitmap);

uint16_t getDataString_DRV8305_VDS_SENSE_CONTROL(bool isWrite, VDS_SENSE_CONTROL_MAP bitmap);


// TODO Insert declarations or function prototypes (right here) to leverage 
// live documentation

#ifdef	__cplusplus
extern "C" {
#endif /* __cplusplus */

    // TODO If C++ is being used, regular C code needs function names to have C 
    // linkage so the functions can be used by the c code. 

#ifdef	__cplusplus
}
#endif /* __cplusplus */

#endif	/* _DRV8305_H */

