/*
 * File:   drv8305.c
 * Author: rasmus
 *
 * Created on November 1, 2016, 3:14 PM
 */


#include <xc.h> // include processor files - each processor file is guarded.  
#include "general.h"
#include "drv8305.h"

volatile WARNINGS_AND_WATCHDOG_RESET_MAP mWARNINGS_AND_WATCHDOG_RESET_MAP;
volatile OV_VDS_FAULTS_MAP mOV_VDS_FAULTS_MAP;
volatile IC_FAULTS_MAP mIC_FAULTS_MAP;
volatile VGS_FAULTS_MAP mVGS_FAULTS_MAP;
volatile HS_GATE_DRIVE_CONTROL_MAP mHS_GATE_DRIVE_CONTROL_MAP;
volatile LS_GATE_DRIVE_CONTROL_MAP mLS_GATE_DRIVE_CONTROL_MAP;
volatile GATE_DRIVE_CONTROL_MAP mGATE_DRIVE_CONTROL_MAP;
volatile IC_OPERATION_MAP mIC_OPERATION_MAP;
volatile SHUNT_AMPLIFIER_CONTROL_MAP mSHUNT_AMPLIFIER_CONTROL_MAP;
volatile VOLTAGE_REGULATOR_CONTROL_MAP mVOLTAGE_REGULATOR_CONTROL_MAP;
volatile VDS_SENSE_CONTROL_MAP mVDS_SENSE_CONTROL_MAP;


uint16_t sendCommand_DRV8305(uint16_t cmd){

    uint16_t receive = SPI1_Exchange16bit(cmd);

    return receive;
}

uint16_t readRegister(DRV8305_READ_REGISTER read_reg){
    uint16_t cmd = 0;
    uint16_t returnVal = 0;
    uint16_t receive = 0;
    switch(read_reg){
        case WARNINGS_AND_WATCHDOG_RESET_ENUM :
            cmd = getDataString_DRV8305_WARNINGS_AND_WATCHDOG_RESET(false,mWARNINGS_AND_WATCHDOG_RESET_MAP);
            receive = sendCommand_DRV8305(cmd);
            setSettingsObject_DRV8305_WARNINGS_AND_WATCHDOG_RESET(receive);
            returnVal = receive;
            break;
        case OV_VDS_FAULTS_ENUM :
            cmd = getDataString_DRV8305_OV_VDS_FAULTS(false,mOV_VDS_FAULTS_MAP);
            receive = sendCommand_DRV8305(cmd);
            setSettingsObject_DRV8305_OV_VDS_FAULTS(receive);
            returnVal = receive;
            break;
        case IC_FAULTS_ENUM :
            cmd = getDataString_DRV8305_IC_FAULTS(false,mIC_FAULTS_MAP);
            receive = sendCommand_DRV8305(cmd);
            setSettingsObject_DRV8305_IC_FAULTS(receive);
            returnVal = receive;
            break;
        case VGS_FAULTS_ENUM :
            cmd = getDataString_DRV8305_VGS_FAULTS(false,mVGS_FAULTS_MAP);
            receive = sendCommand_DRV8305(cmd);
            setSettingsObject_DRV8305_VGS_FAULTS(receive);
            returnVal = receive;
            break;
        case HS_GATE_DRIVE_CONTROL_ENUM :
            cmd = getDataString_DRV8305_HS_GATE_DRIVE_CONTROL(false,mHS_GATE_DRIVE_CONTROL_MAP);
            receive = sendCommand_DRV8305(cmd);
            setSettingsObject_DRV8305_HS_GATE_DRIVE_CONTROL(receive);
            returnVal = receive;
            break;
        case LS_GATE_DRIVE_CONTROL_ENUM :
            cmd = getDataString_DRV8305_LS_GATE_DRIVE_CONTROL(false,mLS_GATE_DRIVE_CONTROL_MAP);
            receive = sendCommand_DRV8305(cmd);
            setSettingsObject_DRV8305_LS_GATE_DRIVE_CONTROL(receive);
            returnVal = receive;
            break;
        case GATE_DRIVE_CONTROL_ENUM :
            cmd = getDataString_DRV8305_GATE_DRIVE_CONTROL(false,mGATE_DRIVE_CONTROL_MAP);
            receive = sendCommand_DRV8305(cmd);
            setSettingsObject_DRV8305_GATE_DRIVE_CONTROL(receive);
            returnVal = receive;
            break;
        case IC_OPERATION_ENUM :
            cmd = getDataString_DRV8305_IC_OPERATION(false,mIC_OPERATION_MAP);
            receive = sendCommand_DRV8305(cmd);
            setSettingsObject_DRV8305_IC_OPERATION(receive);
            returnVal = receive;
            break;
        case SHUNT_AMPLIFIER_CONTROL_ENUM :
            cmd = getDataString_DRV8305_SHUNT_AMPLIFIER_CONTROL(false,mSHUNT_AMPLIFIER_CONTROL_MAP);
            receive = sendCommand_DRV8305(cmd);
            setSettingsObject_DRV8305_SHUNT_AMPLIFIER_CONTROL(receive);
            returnVal = receive;
            break;
        case VOLTAGE_REGULATOR_CONTROL_ENUM :
            cmd = getDataString_DRV8305_VOLTAGE_REGULATOR_CONTROL(false,mVOLTAGE_REGULATOR_CONTROL_MAP);
            receive = sendCommand_DRV8305(cmd);
            setSettingsObject_DRV8305_VOLTAGE_REGULATOR_CONTROL(receive);
            returnVal = receive;
            break;
        case VDS_SENSE_CONTROL_ENUM :
            cmd = getDataString_DRV8305_VDS_SENSE_CONTROL(false,mVDS_SENSE_CONTROL_MAP);
            receive = sendCommand_DRV8305(cmd);
            setSettingsObject_DRV8305_VDS_SENSE_CONTROL(receive);
            returnVal = receive;
            break;
        default:
            return 0;
    }
    return returnVal;
}

void readAllRegisters(){
    readAllStatusRegisters();
    readAllControlRegisters();
}

void readAllStatusRegisters(){
    readRegister(WARNINGS_AND_WATCHDOG_RESET_ENUM);
    readRegister(OV_VDS_FAULTS_ENUM);
    readRegister(IC_FAULTS_ENUM);
    readRegister(VGS_FAULTS_ENUM);
}

void readAllControlRegisters(){
    readRegister(HS_GATE_DRIVE_CONTROL_ENUM);
    readRegister(LS_GATE_DRIVE_CONTROL_ENUM);
    readRegister(GATE_DRIVE_CONTROL_ENUM);
    readRegister(IC_OPERATION_ENUM);
    readRegister(SHUNT_AMPLIFIER_CONTROL_ENUM);
    readRegister(VOLTAGE_REGULATOR_CONTROL_ENUM);
    readRegister(VDS_SENSE_CONTROL_ENUM);
}

void setUp_DRV8305(void){
    uint16_t testport = PORTB;
    readAllStatusRegisters();
    int fault = 1;
    uint16_t getcmd = 0;
    
    while(fault){
        fault = 0;
    //// SETUP OF HS_GATE_DRIVE_CONTROL
        HS_GATE_DRIVE_CONTROL_MAP Setup_HS_GATE_DRIVE_CONTROL_MAP;
        Setup_HS_GATE_DRIVE_CONTROL_MAP.TDRIVEN             = 0b10;   // High-side gate driver peak source time 880ns
        Setup_HS_GATE_DRIVE_CONTROL_MAP.IDRIVEN_HS          = 0b1011; // High-side gate driver peak sink current 1A
        Setup_HS_GATE_DRIVE_CONTROL_MAP.IDRIVEP_HS          = 0b1011; // High-side gate driver peak source current 1A

        uint16_t cmd_HS_GATE_DRIVE_CONTROL_MAP = getDataString_DRV8305_HS_GATE_DRIVE_CONTROL(true,Setup_HS_GATE_DRIVE_CONTROL_MAP);
        sendCommand_DRV8305(cmd_HS_GATE_DRIVE_CONTROL_MAP);
        getcmd = readRegister(HS_GATE_DRIVE_CONTROL_ENUM);
        if(EXTRACT_BITS(cmd_HS_GATE_DRIVE_CONTROL_MAP,10,0) != EXTRACT_BITS(getcmd,10,0))
            fault = 1;
    ////-----------------------------------------------------
    
    //// SETUP OF LS_GATE_DRIVE_CONTROL
        LS_GATE_DRIVE_CONTROL_MAP Setup_LS_GATE_DRIVE_CONTROL_MAP;
        Setup_LS_GATE_DRIVE_CONTROL_MAP.TDRIVEN             = 0b10;   // Low-side gate driver peak source time 880ns
        Setup_LS_GATE_DRIVE_CONTROL_MAP.IDRIVEN_LS          = 0b1011; // Low-side gate driver peak sink current 1A
        Setup_LS_GATE_DRIVE_CONTROL_MAP.IDRIVEP_LS          = 0b1011; // Low-side gate driver peak source current 1A

        uint16_t cmd_LS_GATE_DRIVE_CONTROL_MAP = getDataString_DRV8305_LS_GATE_DRIVE_CONTROL(true,Setup_LS_GATE_DRIVE_CONTROL_MAP);
        sendCommand_DRV8305(cmd_LS_GATE_DRIVE_CONTROL_MAP);
        getcmd = readRegister(LS_GATE_DRIVE_CONTROL_ENUM);
        if(EXTRACT_BITS(cmd_LS_GATE_DRIVE_CONTROL_MAP,10,0) != EXTRACT_BITS(getcmd,10,0))
            fault = 1;
    ////-----------------------------------------------------

    //// SETUP OF GATE_DRIVE_CONTROL
        GATE_DRIVE_CONTROL_MAP Setup_GATE_DRIVE_CONTROL_MAP;
        Setup_GATE_DRIVE_CONTROL_MAP.COMM_OPTION            = 0b1;  // Rectification control - b'0 = diode freewheeling , b'1 = active freewheeling
        Setup_GATE_DRIVE_CONTROL_MAP.PWM_MODE               = 0b00; // PWM Mode
                                                                    // b'00 - PWM with 6 independent inputs
                                                                    // b'01 - PWM with 3 independent inputs
                                                                    // b'10 - PWM with one input
                                                                    // b'11 - PWM with 6 independent inputs
        Setup_GATE_DRIVE_CONTROL_MAP.DEAD_TIME              = 0b011;// Dead time b'001 - 52 ns
        Setup_GATE_DRIVE_CONTROL_MAP.TBLANK                 = 0b01; // VDS sense blanking //b'01 - 1.75 µs
        Setup_GATE_DRIVE_CONTROL_MAP.TVDS                   = 0b10; //VDS sense deglitch  //b'10 - 3.5 µs

        uint16_t cmd_GATE_DRIVE_CONTROL_MAP = getDataString_DRV8305_GATE_DRIVE_CONTROL(true,Setup_GATE_DRIVE_CONTROL_MAP);
        sendCommand_DRV8305(cmd_GATE_DRIVE_CONTROL_MAP);
        getcmd = readRegister(GATE_DRIVE_CONTROL_ENUM);
        if(EXTRACT_BITS(cmd_GATE_DRIVE_CONTROL_MAP,10,0) != EXTRACT_BITS(getcmd,10,0))
            fault = 1;
    ////-----------------------------------------------------
        testport = PORTB;
        readAllStatusRegisters();
    //// SETUP OF IC_OPERATION
        IC_OPERATION_MAP Setup_IC_OPERATION_MAP;
        Setup_IC_OPERATION_MAP.FLIP_OTSD                    = 0b1;  // Over Temperature Shot Down
        Setup_IC_OPERATION_MAP.DIS_PVDD_UVLO2               = 0b0;  // Enable Under Voltage Fault and reporting
        Setup_IC_OPERATION_MAP.DIS_GDRV_FAULT               = 0b0;  // Enable gate driver fault
        Setup_IC_OPERATION_MAP.EN_SNS_CLAMP                 = 0b1;  // Sense amplifier clamp is enabled ~ 3.3V
        Setup_IC_OPERATION_MAP.WD_DLY                       = 0b01; // Watchdog delay 20ms
        Setup_IC_OPERATION_MAP.DIS_SNS_OCP                  = 0b0;  // Low side Over Current Protection Enable
        #ifdef NO_MOSFETS
            Setup_IC_OPERATION_MAP.DIS_SNS_OCP              = 0b1;  // Low side Over Current Protection Disable for testing
        #endif
        Setup_IC_OPERATION_MAP.WD_EN                        = 0b0;  // Watchdog disabled
        Setup_IC_OPERATION_MAP.SLEEP                        = 0b0;  // Device Awake
        Setup_IC_OPERATION_MAP.CLR_FLTS                     = 0b0;  // Normal operation. If fault is received the fault can be cleard in 
                                                                // this bit if the fault-condition is removed
        Setup_IC_OPERATION_MAP.SET_VCPH_UV                  = 0b0;  // Under voltage threshold b'0 = 4,9V , b'1 = 4,6V

        uint16_t cmd_IC_OPERATION_MAP = getDataString_DRV8305_IC_OPERATION(true,Setup_IC_OPERATION_MAP);
        sendCommand_DRV8305(cmd_IC_OPERATION_MAP);
        getcmd = readRegister(IC_OPERATION_ENUM);
        if(EXTRACT_BITS(cmd_IC_OPERATION_MAP,10,0) != EXTRACT_BITS(getcmd,10,0))
            fault = 1;
    ////-----------------------------------------------------

    //// SETUP OF SHUNT_AMPLIFIER_CONTROL        
        SHUNT_AMPLIFIER_CONTROL_MAP Setup_SHUNT_AMPLIFIER_CONTROL_MAP;
        Setup_SHUNT_AMPLIFIER_CONTROL_MAP.DC_CAL_CH3        = 0b0;  // b'0 = Normal operation, b'1 = DC calibration mode
        Setup_SHUNT_AMPLIFIER_CONTROL_MAP.DC_CAL_CH2        = 0b0;        
        Setup_SHUNT_AMPLIFIER_CONTROL_MAP.DC_CAL_CH1        = 0b0;
        Setup_SHUNT_AMPLIFIER_CONTROL_MAP.CS_BLANK          = 0b00; // blanking time = 0ns;
        Setup_SHUNT_AMPLIFIER_CONTROL_MAP.GAIN_CS1          = 0b01; // b'00 = 10 V/V,   b'01 = 20 V/V,  b'10 = 40 V/V,  b'11 - 80 V/V
                                                                    // Rsense = 0,002 ohm max 40 A <=> Vsense = 0,002*40 = 0,080V
                                                                    // Vref = 3,3V <=> Output = Vref/(Vrefscale=2) - Gain * (0-Vsense) 
                                                                    //             <=>  3,3V  = 3,3V/2             - Gain * ( -0,080V)
                                                                    //             <=>  Gain = ( 1,65V - 3,3V ) / ( - 0,080) =  20,625V/V
                                                                    // Actual range ->  Vsense  = - ( 1,65V - 3,3V ) / ( 20[V/V]) =  20,625V/V <=> 0,0825V / 0,002ohm = 41,25A
                                                                    // Wattage = 0,002 * 40^2 = 3,2W
                                                                    // Wattage max 3W <=> Max current = sqrt(3W/0,002ohm) = 38,8A  // OBS wattage is typically divided between phases
                                                                    // Should be able to use 40A when motor is rotating fast. Turn down to 38A when Rotating slow
        Setup_SHUNT_AMPLIFIER_CONTROL_MAP.GAIN_CS2          = 0b01;
        Setup_SHUNT_AMPLIFIER_CONTROL_MAP.GAIN_CS3          = 0b01;

        uint16_t cmd_SHUNT_AMPLIFIER_CONTROL_MAP = getDataString_DRV8305_SHUNT_AMPLIFIER_CONTROL(true,Setup_SHUNT_AMPLIFIER_CONTROL_MAP);
        sendCommand_DRV8305(cmd_SHUNT_AMPLIFIER_CONTROL_MAP);
        getcmd = readRegister(SHUNT_AMPLIFIER_CONTROL_ENUM);
        if(EXTRACT_BITS(cmd_SHUNT_AMPLIFIER_CONTROL_MAP,10,0) != EXTRACT_BITS(getcmd,10,0))
            fault = 1;
    ////-----------------------------------------------------

    //// SETUP OF VOLTAGE_REGULATOR_CONTROL        
        VOLTAGE_REGULATOR_CONTROL_MAP Setup_VOLTAGE_REGULATOR_CONTROL_MAP;
        Setup_VOLTAGE_REGULATOR_CONTROL_MAP.VREF_SCALE      = 0b01; // See Gain calculations R = 2
        Setup_VOLTAGE_REGULATOR_CONTROL_MAP.SLEEP_DLY       = 0b1;  // Delay to power down VREG after SLEEP 10us
        Setup_VOLTAGE_REGULATOR_CONTROL_MAP.DIS_VREG_PWRGD  = 0b0;  // VREG undervoltage Enabled    
        Setup_VOLTAGE_REGULATOR_CONTROL_MAP.VREG_UV_LEVEL   = 0b01; // VREF Undervoltage Point = VREG*0,8
        
        uint16_t cmd_VOLTAGE_REGULATOR_CONTROL_MAP = getDataString_DRV8305_VOLTAGE_REGULATOR_CONTROL(true,Setup_VOLTAGE_REGULATOR_CONTROL_MAP);                                                                
        sendCommand_DRV8305(cmd_VOLTAGE_REGULATOR_CONTROL_MAP);
        getcmd = readRegister(VOLTAGE_REGULATOR_CONTROL_ENUM);
        if(EXTRACT_BITS(cmd_VOLTAGE_REGULATOR_CONTROL_MAP,10,0) != EXTRACT_BITS(getcmd,10,0))
            fault = 1;
            
    ////-----------------------------------------------------

    //// SETUP OF VOLTAGE_REGULATOR_CONTROL     

        VDS_SENSE_CONTROL_MAP Setup_VDS_SENSE_CONTROL_MAP;
        Setup_VDS_SENSE_CONTROL_MAP.VDS_LEVEL               = 0b01001; // b'01001 - 0.175 V
                                                                    // Over Current = 0.175 V / 0,002ohm = 87,5A
        Setup_VDS_SENSE_CONTROL_MAP.VDS_MODE                = 0b001;// VDS mode
                                                                    // b'000 - Latched shut down when over-current detected
                                                                    // b'001 - Report only when over current detected
                                                                    // b'010 - VDS protection disabled (no overcurrent sensing or reporting)
#ifdef NO_MOSFETS
            Setup_VDS_SENSE_CONTROL_MAP.VDS_MODE                = 0b010;  // For testing
#endif

        uint16_t cmd_VDS_SENSE_CONTROL_MAP = getDataString_DRV8305_VDS_SENSE_CONTROL(true,Setup_VDS_SENSE_CONTROL_MAP);                                                                
        sendCommand_DRV8305(cmd_VDS_SENSE_CONTROL_MAP);
        getcmd = readRegister(VDS_SENSE_CONTROL_ENUM);
        if(EXTRACT_BITS(cmd_VDS_SENSE_CONTROL_MAP,8,0) != EXTRACT_BITS(getcmd,8,0))
            fault = 1;
    }

}


//////////////////////////////////////////////////////////////////////////
// FUNCTIONS TO SET OBJECTS SETTINGS THAT WAS RECEVIEVD FROM SPI  ////////
//////////////////////////////////////////////////////////////////////////

void setSettingsObject_DRV8305_WARNINGS_AND_WATCHDOG_RESET(uint16_t data){
    mWARNINGS_AND_WATCHDOG_RESET_MAP.FAULT          = CHECK_BIT(data, 10);  
                                //  << 9 is reserved
    mWARNINGS_AND_WATCHDOG_RESET_MAP.TEMP_FLAG4     = CHECK_BIT(data,8);    
    mWARNINGS_AND_WATCHDOG_RESET_MAP.PVDD_UVFL      = CHECK_BIT(data,7);
    mWARNINGS_AND_WATCHDOG_RESET_MAP.PVDD_OVFL      = CHECK_BIT(data,6);
    mWARNINGS_AND_WATCHDOG_RESET_MAP.VDS_STATUS     = CHECK_BIT(data,5);
    mWARNINGS_AND_WATCHDOG_RESET_MAP.VCPH_UVFL      = CHECK_BIT(data,4);
    mWARNINGS_AND_WATCHDOG_RESET_MAP.TEMP_FLAG1     = CHECK_BIT(data,3);
    mWARNINGS_AND_WATCHDOG_RESET_MAP.TEMP_FLAG2     = CHECK_BIT(data,2);
    mWARNINGS_AND_WATCHDOG_RESET_MAP.TEMP_FLAG3     = CHECK_BIT(data,1);
    mWARNINGS_AND_WATCHDOG_RESET_MAP.OTW            = CHECK_BIT(data,0);
        
}

void setSettingsObject_DRV8305_OV_VDS_FAULTS(uint16_t data){

    mOV_VDS_FAULTS_MAP.VDS_HA                       = CHECK_BIT(data,10);
    mOV_VDS_FAULTS_MAP.VDS_LA                       = CHECK_BIT(data,9);
    mOV_VDS_FAULTS_MAP.VDS_HB                       = CHECK_BIT(data,8);
    mOV_VDS_FAULTS_MAP.VDS_LB                       = CHECK_BIT(data,7);
    mOV_VDS_FAULTS_MAP.VDS_HC                       = CHECK_BIT(data,6);
    mOV_VDS_FAULTS_MAP.VDS_LC                       = CHECK_BIT(data,5);
                                //  << 4 is reserved
                                //  << 3 is reserved
    mOV_VDS_FAULTS_MAP.SNS_C_OCP                    = CHECK_BIT(data,2);
    mOV_VDS_FAULTS_MAP.SNS_B_OCP                    = CHECK_BIT(data,1);
    mOV_VDS_FAULTS_MAP.SNS_A_OCP                    = CHECK_BIT(data,0);
}

void setSettingsObject_DRV8305_IC_FAULTS(uint16_t data){
    mIC_FAULTS_MAP.PVDD_UVLO2                       = CHECK_BIT(data,10);
    mIC_FAULTS_MAP.WD_FAULT                         = CHECK_BIT(data,9);
    mIC_FAULTS_MAP.OTSD                             = CHECK_BIT(data,8);
                                //  << 7 is reserved
    mIC_FAULTS_MAP.VREG_UV                          = CHECK_BIT(data,6);
    mIC_FAULTS_MAP.AVDD_UVLO                        = CHECK_BIT(data,5);
    mIC_FAULTS_MAP.VCP_LSD_UVLO2                    = CHECK_BIT(data,4);
                                //  << 3 is reserved
    mIC_FAULTS_MAP.VCPH_UVLO2                       = CHECK_BIT(data,2);
    mIC_FAULTS_MAP.VCPH_OVLO                        = CHECK_BIT(data,1);
    mIC_FAULTS_MAP.VCPH_OVLO_ABS                    = CHECK_BIT(data,0);
}

void setSettingsObject_DRV8305_VGS_FAULTS(uint16_t data){

    mVGS_FAULTS_MAP.VGS_HA                          = CHECK_BIT(data,10);
    mVGS_FAULTS_MAP.VGS_LA                          = CHECK_BIT(data,9);
    mVGS_FAULTS_MAP.VGS_HB                          = CHECK_BIT(data,8);
    mVGS_FAULTS_MAP.VGS_LB                          = CHECK_BIT(data,7);
    mVGS_FAULTS_MAP.VGS_HC                          = CHECK_BIT(data,6);
    mVGS_FAULTS_MAP.VGS_LC                          = CHECK_BIT(data,5);
                                                    //  << 4~0 is reserved
}

void setSettingsObject_DRV8305_HS_GATE_DRIVE_CONTROL(uint16_t data){
                                    //  << 10 is reserved
    mHS_GATE_DRIVE_CONTROL_MAP.TDRIVEN              = EXTRACT_BITS(data,9,8); // 8~9 TDRIVEN
    mHS_GATE_DRIVE_CONTROL_MAP.IDRIVEN_HS           = EXTRACT_BITS(data,7,4); // 4~7 IDRIVEN_HS
    mHS_GATE_DRIVE_CONTROL_MAP.IDRIVEP_HS           = EXTRACT_BITS(data,3,0); // 0~3 IDRIVEP_HS       
}

void setSettingsObject_DRV8305_LS_GATE_DRIVE_CONTROL(uint16_t data){
                                        //  << 10 is reserved
    mLS_GATE_DRIVE_CONTROL_MAP.TDRIVEN              = EXTRACT_BITS(data,9,8); // 8~9 TDRIVEN
    mLS_GATE_DRIVE_CONTROL_MAP.IDRIVEN_LS           = EXTRACT_BITS(data,7,4); // 4~7 IDRIVEN_HSS
    mLS_GATE_DRIVE_CONTROL_MAP.IDRIVEP_LS           = EXTRACT_BITS(data,3,0); // 0~3 IDRIVEP_HS      
}

void setSettingsObject_DRV8305_GATE_DRIVE_CONTROL(uint16_t data ){
                                        //  << 10 is reserved
    mGATE_DRIVE_CONTROL_MAP.COMM_OPTION             = CHECK_BIT(data,9);
    mGATE_DRIVE_CONTROL_MAP.PWM_MODE                = EXTRACT_BITS(data,8,7);  // 7~8
    mGATE_DRIVE_CONTROL_MAP.DEAD_TIME               = EXTRACT_BITS(data,6,4);  // 4~6
    mGATE_DRIVE_CONTROL_MAP.TBLANK                  = EXTRACT_BITS(data,3,2);  // 2~3
    mGATE_DRIVE_CONTROL_MAP.TVDS                    = EXTRACT_BITS(data,1,0); // 0~1     
}

void  setSettingsObject_DRV8305_IC_OPERATION(uint16_t data ){
    mIC_OPERATION_MAP.FLIP_OTSD                     = CHECK_BIT(data,10);
    mIC_OPERATION_MAP.DIS_PVDD_UVLO2                = CHECK_BIT(data,9);
    mIC_OPERATION_MAP.DIS_GDRV_FAULT                = CHECK_BIT(data,8);
    mIC_OPERATION_MAP.EN_SNS_CLAMP                  = CHECK_BIT(data,7);

    mIC_OPERATION_MAP.WD_DLY                        = EXTRACT_BITS(data,6,5); // 5~6
    mIC_OPERATION_MAP.DIS_SNS_OCP                   = CHECK_BIT(data,4);
    mIC_OPERATION_MAP.WD_EN                         = CHECK_BIT(data,3);
    mIC_OPERATION_MAP.SLEEP                         = CHECK_BIT(data,2);
    mIC_OPERATION_MAP.CLR_FLTS                      = CHECK_BIT(data,1);
    mIC_OPERATION_MAP.SET_VCPH_UV                   = CHECK_BIT(data,0);
   
}

void setSettingsObject_DRV8305_SHUNT_AMPLIFIER_CONTROL(uint16_t data){
    mSHUNT_AMPLIFIER_CONTROL_MAP.DC_CAL_CH3         = CHECK_BIT(data,10);
    mSHUNT_AMPLIFIER_CONTROL_MAP.DC_CAL_CH2         = CHECK_BIT(data,9);
    mSHUNT_AMPLIFIER_CONTROL_MAP.DC_CAL_CH1         = CHECK_BIT(data,8);
    mSHUNT_AMPLIFIER_CONTROL_MAP.CS_BLANK           = EXTRACT_BITS(data,7,6);  // 6~7
    mSHUNT_AMPLIFIER_CONTROL_MAP.GAIN_CS3           = EXTRACT_BITS(data,5,4);  // 4~5
    mSHUNT_AMPLIFIER_CONTROL_MAP.GAIN_CS2           = EXTRACT_BITS(data,3,2);  // 2~3
    mSHUNT_AMPLIFIER_CONTROL_MAP.GAIN_CS1           = EXTRACT_BITS(data,1,0);  // 0~1
}

void setSettingsObject_DRV8305_VOLTAGE_REGULATOR_CONTROL(uint16_t data){
                                                    //  << 10 is reserved
    mVOLTAGE_REGULATOR_CONTROL_MAP.VREF_SCALE       = EXTRACT_BITS(data,9,8);  // 8~9
                                                //  << 5~7 is reserved
    mVOLTAGE_REGULATOR_CONTROL_MAP.SLEEP_DLY        = EXTRACT_BITS(data,4,3);  // 3~4
    mVOLTAGE_REGULATOR_CONTROL_MAP.DIS_VREG_PWRGD   = CHECK_BIT(data,2);
    mVOLTAGE_REGULATOR_CONTROL_MAP.VREG_UV_LEVEL    = EXTRACT_BITS(data,1,0);// 0~1
}

void setSettingsObject_DRV8305_VDS_SENSE_CONTROL(uint16_t data){
                                                    //  << 8~10 is reserved
    mVDS_SENSE_CONTROL_MAP.VDS_LEVEL                = EXTRACT_BITS(data,7,3);  // 3~7
    mVDS_SENSE_CONTROL_MAP.VDS_MODE                 = EXTRACT_BITS(data,2,0);  // 0~2    
}

////////////////////////////////////////////////////////////
// FUNCTIONS TO GET DATA THAT IS TO BE SEND VIA SPI ////////
////////////////////////////////////////////////////////////

uint16_t getDataString_DRV8305_WARNINGS_AND_WATCHDOG_RESET(bool isWrite, WARNINGS_AND_WATCHDOG_RESET_MAP bitmap){
    uint16_t returnVal = 0x0000;
    isWrite = 0;
    if(isWrite){
        returnVal |= (0 << 15); // Write or Read - 0 if write
        returnVal |= (WARNINGS_AND_WATCHDOG_RESET_ADR << 11) ;
        returnVal |= (bitmap.FAULT      << 10);
                                    //  << 9 is reserved
        returnVal |= (bitmap.TEMP_FLAG4 << 8);
        returnVal |= (bitmap.PVDD_UVFL  << 7);
        returnVal |= (bitmap.PVDD_OVFL  << 6);
        returnVal |= (bitmap.VDS_STATUS << 5);
        returnVal |= (bitmap.VCPH_UVFL  << 4);
        returnVal |= (bitmap.TEMP_FLAG1 << 3);
        returnVal |= (bitmap.TEMP_FLAG2 << 2);
        returnVal |= (bitmap.TEMP_FLAG3 << 1);
        returnVal |= (bitmap.OTW        << 0);
    }
    else{
        returnVal |= (1 << 15); // Write or Read - 1 if read
        returnVal |= (WARNINGS_AND_WATCHDOG_RESET_ADR << 11) ;
    }
    return returnVal;
}

uint16_t getDataString_DRV8305_OV_VDS_FAULTS(bool isWrite, OV_VDS_FAULTS_MAP bitmap){
        uint16_t returnVal = 0x0000;
        isWrite = 0;
    if(isWrite){
        returnVal |= (0 << 15); // Write or Read - 0 if write
        returnVal |= (OV_VDS_FAULTS_ADR << 11) ;
        returnVal |= (bitmap.VDS_HA     << 10);
        returnVal |= (bitmap.VDS_LA     << 9);
        returnVal |= (bitmap.VDS_HB     << 8);
        returnVal |= (bitmap.VDS_LB     << 7);
        returnVal |= (bitmap.VDS_HC     << 6);
        returnVal |= (bitmap.VDS_LC     << 5);
                                    //  << 4 is reserved
                                    //  << 3 is reserved
        returnVal |= (bitmap.SNS_C_OCP  << 2);
        returnVal |= (bitmap.SNS_B_OCP  << 1);
        returnVal |= (bitmap.SNS_A_OCP  << 0);
    }
    else{
        returnVal |= (1 << 15); // Write or Read - 1 if read
        returnVal |= (OV_VDS_FAULTS_ADR << 11) ;
    }
    return returnVal;
}

uint16_t getDataString_DRV8305_IC_FAULTS(bool isWrite, IC_FAULTS_MAP bitmap){
        uint16_t returnVal = 0x0000;
        isWrite = 0;
    if(isWrite){
        returnVal |= (0 << 15); // Write or Read - 0 if write
        returnVal |= (IC_FAULTS_ADR << 11) ;
        returnVal |= (bitmap.PVDD_UVLO2 << 10);
        returnVal |= (bitmap.WD_FAULT   << 9);
        returnVal |= (bitmap.OTSD       << 8);
                                    //  << 7 is reserved
        returnVal |= (bitmap.VREG_UV    << 6);
        returnVal |= (bitmap.AVDD_UVLO  << 5);
        returnVal |= (bitmap.VCP_LSD_UVLO2  << 4);
                                    //  << 3 is reserved
        returnVal |= (bitmap.VCPH_UVLO2 << 2);
        returnVal |= (bitmap.VCPH_OVLO  << 1);
        returnVal |= (bitmap.VCPH_OVLO_ABS  << 0);
    }
    else{
        returnVal |= (1 << 15); // Write or Read - 1 if read
        returnVal |= (IC_FAULTS_ADR << 11) ;
    }
    return returnVal;    
}

uint16_t getDataString_DRV8305_VGS_FAULTS(bool isWrite, VGS_FAULTS_MAP bitmap){
            uint16_t returnVal = 0x0000;
            isWrite = 0;
    if(isWrite){
        returnVal |= (0 << 15); // Write or Read - 0 if write
        returnVal |= (VGS_FAULTS_ADR    << 11);
        returnVal |= (bitmap.VGS_HA     << 10);
        returnVal |= (bitmap.VGS_LA     << 9);
        returnVal |= (bitmap.VGS_HB     << 8);
        returnVal |= (bitmap.VGS_LB     << 7);
        returnVal |= (bitmap.VGS_HC     << 6);
        returnVal |= (bitmap.VGS_LC     << 5);
                                    //  << 4~0 is reserved
    }
    else{
        returnVal |= (1 << 15); // Write or Read - 1 if read
        returnVal |= (VGS_FAULTS_ADR << 11);
    }
    return returnVal;    
}

uint16_t getDataString_DRV8305_HS_GATE_DRIVE_CONTROL(bool isWrite, HS_GATE_DRIVE_CONTROL_MAP bitmap){
            uint16_t returnVal = 0x0000;
    if(isWrite){
        returnVal |= (0 << 15); // Write or Read - 0 if write
        returnVal |= (HS_GATE_DRIVE_CONTROL_ADR << 11); 
                                        //  << 10 is reserved
        returnVal |= (bitmap.TDRIVEN    << 8); // 8~9 TDRIVEN
        returnVal |= (bitmap.IDRIVEN_HS << 4); // 4~7 IDRIVEN_HS
        returnVal |= (bitmap.IDRIVEP_HS);      // 0~3 IDRIVEP_HS       
    }
    else{
        returnVal |= (1 << 15); // Write or Read - 1 if read
        returnVal |= (HS_GATE_DRIVE_CONTROL_ADR << 11);
    }
    return returnVal;    
}

uint16_t getDataString_DRV8305_LS_GATE_DRIVE_CONTROL(bool isWrite, LS_GATE_DRIVE_CONTROL_MAP bitmap){
    uint16_t returnVal = 0x0000;
    if(isWrite){
        returnVal |= (0 << 15); // Write or Read - 0 if write
        returnVal |= (LS_GATE_DRIVE_CONTROL_ADR << 11);
                                        //  << 10 is reserved
        returnVal |= (bitmap.TDRIVEN    << 8); // 8~9 TDRIVEN
        returnVal |= (bitmap.IDRIVEN_LS << 4); // 4~7 IDRIVEN_HS
        returnVal |= (bitmap.IDRIVEP_LS);      // 0~3 IDRIVEP_HS       
    }
    else{
        returnVal |= (1 << 15); // Write or Read - 1 if read
        returnVal |= (LS_GATE_DRIVE_CONTROL_ADR << 11) ;
    }
    return returnVal; 
}

uint16_t getDataString_DRV8305_GATE_DRIVE_CONTROL(bool isWrite, GATE_DRIVE_CONTROL_MAP bitmap){
     uint16_t returnVal = 0x0000;
    if(isWrite){
        returnVal |= (0 << 15); // Write or Read - 0 if write
        returnVal |= (GATE_DRIVE_CONTROL_ADR        << 11) ;
                                        //  << 10 is reserved
        returnVal |= (bitmap.COMM_OPTION    << 9);
        returnVal |= (bitmap.PWM_MODE       << 7);  // 7~8
        returnVal |= (bitmap.DEAD_TIME      << 4);  // 4~6
        returnVal |= (bitmap.TBLANK         << 2);  // 2~3
        returnVal |= (bitmap.TVDS);                 // 0~1
    }
    else{
        returnVal |= (1 << 15); // Write or Read - 1 if read
        returnVal |= (GATE_DRIVE_CONTROL_ADR << 11);
    }
    return returnVal;     
}

uint16_t getDataString_DRV8305_IC_OPERATION(bool isWrite, IC_OPERATION_MAP bitmap){
    uint16_t returnVal = 0x0000;
    if(isWrite){
        returnVal |= (0 << 15); // Write or Read - 0 if write
        returnVal |= (IC_OPERATION_ADR      << 11) ;
        returnVal |= (bitmap.FLIP_OTSD      << 10);
        returnVal |= (bitmap.DIS_PVDD_UVLO2 << 9);
        returnVal |= (bitmap.DIS_GDRV_FAULT << 8);
        returnVal |= (bitmap.EN_SNS_CLAMP   << 7);
        
        returnVal |= (bitmap.WD_DLY         << 5); // 5~6
        returnVal |= (bitmap.DIS_SNS_OCP    << 4);
        returnVal |= (bitmap.WD_EN          << 3);
        returnVal |= (bitmap.SLEEP          << 2);
        returnVal |= (bitmap.CLR_FLTS       << 1);
        returnVal |= (bitmap.SET_VCPH_UV    << 0);
    }
    else{
        returnVal |= (1 << 15); // Write or Read - 1 if read
        returnVal |= (IC_OPERATION_ADR << 11) ;
    }
    return returnVal;    
}

uint16_t getDataString_DRV8305_SHUNT_AMPLIFIER_CONTROL(bool isWrite, SHUNT_AMPLIFIER_CONTROL_MAP bitmap){
    int16_t returnVal = 0x0000;
    if(isWrite){
        returnVal |= (0 << 15); // Write or Read - 0 if write
        returnVal |= (SHUNT_AMPLIFIER_CONTROL_ADR   << 11);
        returnVal |= (bitmap.DC_CAL_CH3             << 10);
        returnVal |= (bitmap.DC_CAL_CH2             << 9);
        returnVal |= (bitmap.DC_CAL_CH1             << 8);
        returnVal |= (bitmap.CS_BLANK               << 6);  // 6~7
        returnVal |= (bitmap.GAIN_CS3               << 4);  // 4~5
        returnVal |= (bitmap.GAIN_CS2               << 2);  // 2~3
        returnVal |= (bitmap.GAIN_CS1);                     // 0~1
    }
    else{
        returnVal |= (1 << 15); // Write or Read - 1 if read
        returnVal |= (SHUNT_AMPLIFIER_CONTROL_ADR << 11);
    }
    return returnVal;  
}

uint16_t getDataString_DRV8305_VOLTAGE_REGULATOR_CONTROL(bool isWrite, VOLTAGE_REGULATOR_CONTROL_MAP bitmap){
    int16_t returnVal = 0x0000;
    if(isWrite){
        returnVal |= (0 << 15); // Write or Read - 0 if write
        returnVal |= (VOLTAGE_REGULATOR_CONTROL_ADR     << 11) ;
                                                    //  << 10 is reserved
        returnVal |= (bitmap.VREF_SCALE                 << 8);  // 8~9
                                                    //  << 5~7 is reserved
        returnVal |= (bitmap.SLEEP_DLY                  << 3);  // 3~4
        returnVal |= (bitmap.DIS_VREG_PWRGD             << 2);  // 6~7
        returnVal |= (bitmap.VREG_UV_LEVEL);                    // 0~1
    }
    else{
        returnVal |= (1 << 15); // Write or Read - 1 if read
        returnVal |= (VOLTAGE_REGULATOR_CONTROL_ADR << 11) ;
    }
    return returnVal;  
}

uint16_t getDataString_DRV8305_VDS_SENSE_CONTROL(bool isWrite, VDS_SENSE_CONTROL_MAP bitmap){
    int16_t returnVal = 0x0000;
    if(isWrite){
        returnVal |= (0 << 15); // Write or Read - 0 if write
        returnVal |= (VDS_SENSE_CONTROL_ADR             << 11);
                                                    //  << 8~10 is reserved
        returnVal |= (bitmap.VDS_LEVEL                  << 3);  // 3~7
        returnVal |= (bitmap.VDS_MODE);                         // 0~2
    }
    else{
        returnVal |= (1 << 15); // Write or Read - 1 if read
        returnVal |= (VDS_SENSE_CONTROL_ADR << 11) ;
    }
    return returnVal;      
}