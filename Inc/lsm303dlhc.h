/*
 * lsm303dlhc.h
 *
 *  Created on: 14 kwi 2018
 *      Author: User
 */

#ifndef LSM303DLHC_H_
#define LSM303DLHC_H_


// ----------------------------------------------------------------------------
// STUB TODO:
// List all possible device I2C addresses here, if they are predefined. Some
// devices only have one possible address, in which case that one should be
// defined as both [LSM303DLHC]_ADDRESS and [LSM303DLHC]_DEFAULT_ADDRESS for
// consistency. The *_DEFAULT address will be used if a device object is
// created without an address provided in the constructor. Note that I2C
// devices conventionally use 7-bit addresses, so these will generally be
// between 0x00 and 0x7F.
// The LSM303DLHC uses two device addresses, one for the accerometer, and on
// for the megnetometer.
// ----------------------------------------------------------------------------
#define LSM303DLHC_ADDRESS_A            (0x19 << 1)
#define LSM303DLHC_ADDRESS_M            (0x1E << 1)
#define LSM303DLHC_DEFAULT_ADDRESS_A    0x19
#define LSM303DLHC_DEFAULT_ADDRESS_M    0x1E

#define LSM303DLHC_ADDRESS_A_WRITE		0x32
#define LSM303DLHC_ADDRESS_A_READ		0x33

#define LSM303DLHC_ADDRESS_M_WRITE		0x3C
#define LSM303DLHC_ADDRESS_M_READ		0x3D

// ----------------------------------------------------------------------------
// STUB TODO:
// List all registers that this device has. The goal for all device libraries
// is to have complete coverage of all possible functions, so be sure to add
// everything in the datasheet. Register address constants should use the extra
// prefix "RA_", followed by the datasheet's given name for each register.
// ----------------------------------------------------------------------------
#define LSM303DLHC_RA_CTRL_REG1_A       0x20    //  rw  0000 0111
#define LSM303DLHC_RA_CTRL_REG2_A       0x21    //  rw  0000 0000
#define LSM303DLHC_RA_CTRL_REG3_A       0x22    //  rw  0000 0000
#define LSM303DLHC_RA_CTRL_REG4_A       0x23    //  rw  0000 0000
#define LSM303DLHC_RA_CTRL_REG5_A       0x24    //  rw  0000 0000
#define LSM303DLHC_RA_CTRL_REG6_A       0x25    //  rw  0000 0000
#define LSM303DLHC_RA_REFERENCE_A       0x06    //  rw  0000 0000
#define LSM303DLHC_RA_STATUS_REG_A      0x27    //  r   0000 0000
#define LSM303DLHC_RA_OUT_X_L_A         0x28    //  r
#define LSM303DLHC_RA_OUT_X_H_A         0x29    //  r
#define LSM303DLHC_RA_OUT_Y_L_A         0x2A    //  r
#define LSM303DLHC_RA_OUT_Y_H_A         0x2B    //  r
#define LSM303DLHC_RA_OUT_Z_L_A         0x2C    //  r
#define LSM303DLHC_RA_OUT_Z_H_A         0x2D    //  r
#define LSM303DLHC_RA_FIFO_CTRL_REG_A   0x2E    //  rw  0000 0000
#define LSM303DLHC_RA_FIFO_SRC_REG_A    0x2F    //  r
#define LSM303DLHC_RA_INT1_CFG_A        0x30    //  rw  0000 0000
#define LSM303DLHC_RA_INT1_SRC_A        0x31    //  r   0000 0000
#define LSM303DLHC_RA_INT1_THS_A        0x32    //  rw  0000 0000
#define LSM303DLHC_RA_INT1_DURATION_A   0x33    //  rw  0000 0000
#define LSM303DLHC_RA_INT2_CFG_A        0x34    //  rw  0000 0000
#define LSM303DLHC_RA_INT2_SRC_A        0x35    //  r   0000 0000
#define LSM303DLHC_RA_INT2_THS_A        0x36    //  rw  0000 0000
#define LSM303DLHC_RA_INT2_DURATION_A   0x37    //  rw  0000 0000
#define LSM303DLHC_RA_CLICK_CFG_A       0x38    //  rw  0000 0000
#define LSM303DLHC_RA_CLICK_SRC_A       0x39    //  rw  0000 0000
#define LSM303DLHC_RA_CLICK_THS_A       0x3A    //  rw  0000 0000
#define LSM303DLHC_RA_TIME_LIMIT_A      0x3B    //  rw  0000 0000
#define LSM303DLHC_RA_TIME_LATENCY_A    0x3C    //  rw  0000 0000
#define LSM303DLHC_RA_TIME_WINDOW_A     0x3D    //  rw  0000 0000

#define LSM303DLHC_RA_CRA_REG_M         0x00    //  rw  0001 0000
#define LSM303DLHC_RA_CRB_REG_M         0x01    //  rw  0010 0000
#define LSM303DLHC_RA_MR_REG_M          0x02    //  rw  0000 0011
#define LSM303DLHC_RA_OUT_X_H_M         0x03    //  r
#define LSM303DLHC_RA_OUT_X_L_M         0x04    //  r
#define LSM303DLHC_RA_OUT_Z_H_M         0x05    //  r
#define LSM303DLHC_RA_OUT_Z_L_M         0x06    //  r
#define LSM303DLHC_RA_OUT_Y_H_M         0x07    //  r
#define LSM303DLHC_RA_OUT_Y_L_M         0x08    //  r
#define LSM303DLHC_RA_SR_REG_M          0x09    //  r   0000 0000
#define LSM303DLHC_RA_IRA_REG_M         0x0A    //  r   0100 1000
#define LSM303DLHC_RA_IRB_REG_M         0x0B    //  r   0011 0100
#define LSM303DLHC_RA_IRC_REG_M         0x0C    //  r   0011 0011
#define LSM303DLHC_RA_TEMP_OUT_H_M      0x31    //  r
#define LSM303DLHC_RA_TEMP_OUT_L_M      0x32    //  r

// ----------------------------------------------------------------------------
// STUB TODO:
// List register structures wherever necessary. Bit position constants should
// end in "_BIT", and are defined from 7 to 0, with 7 being the left/MSB and
// 0 being the right/LSB. If the device uses 16-bit registers instead of the
// more common 8-bit registers, the range is 15 to 0. Field length constants
// should end in "_LENGTH", but otherwise match the name of bit position
// constant that it complements. Fields that are a single bit in length don't
// need a separate field length constant.
// ----------------------------------------------------------------------------
// ----------------------------------------------------------------------------
// STUB TODO:
// List any special predefined values for each register according to the
// datasheet. For example, MEMS devices often provide different options for
// measurement rates, say 25Hz, 50Hz, 100Hz, and 200Hz. These are typically
// represented by arbitrary bit values, say 0b00, 0b01, 0b10, and 0b11 (or 0x0,
// 0x1, 0x2, and 0x3). Defining them here makes it easy to know which options
// are available.
// ----------------------------------------------------------------------------

//CTRL_REG1_A
#define LSM303DLHC_ODR_BIT              7
#define LSM303DLHC_ODR_LENGTH           4
#define LSM303DLHC_LPEN_BIT             3
#define LSM303DLHC_ZEN_BIT              2
#define LSM303DLHC_YEN_BIT              1
#define LSM303DLHC_XEN_BIT              0

#define LSM303DLHC_ODR_RATE_POWERDOWN       0x00
#define LSM303DLHC_ODR_RATE_1               0x10
#define LSM303DLHC_ODR_RATE_10              0x20
#define LSM303DLHC_ODR_RATE_25              0x30
#define LSM303DLHC_ODR_RATE_50              0x40
#define LSM303DLHC_ODR_RATE_100             0x50
#define LSM303DLHC_ODR_RATE_200             0x60
#define LSM303DLHC_ODR_RATE_400             0x70
#define LSM303DLHC_ODR_RATE_1620_LP         0x80
#define LSM303DLHC_ODR_RATE_1344_N_5376_LP  0x90

#define LSM303DLHC_LPEN_NORMAL_MODE			0x00
#define LSM303DLHC_LPEN_LOW_POWER_MODE		0x08

#define LSM303DLHC_3AXIS_ENABLE				0x07

//CTRL_REG2_A
#define LSM303DLHC_HPM_BIT              7
#define LSM303DLHC_HPM_LENGTH           2
#define LSM303DLHC_HPCF_BIT             5
#define LSM303DLHC_HPCF_LENGTH          2
#define LSM303DLHC_FDS_BIT              3
#define LSM303DLHC_HPCLICK_BIT          2
#define LSM303DLHC_HPIS2_BIT            1
#define LSM303DLHC_HPIS1_BIT            0

#define LSM303DLHC_HPM_HRF              0b00
#define LSM303DLHC_HPM_REFERANCE        0b01
#define LSM303DLHC_HPM_NORMAL           0b10
#define LSM303DLHC_HPM_AUTORESET        0b11

#define LSM303DLHC_HPCF_1               0b00
#define LSM303DLHC_HPCF_2               0b01
#define LSM303DLHC_HPCF_3               0b10
#define LSM303DLHC_HPCF_4               0b11

//CTRL_REG3_A
#define LSM303DLHC_I1_CLICK_BIT         7
#define LSM303DLHC_I1_AOI1_BIT          6
#define LSM303DLHC_I1_AOI2_BIT          5
#define LSM303DLHC_I1_DRDY1_BIT         4
#define LSM303DLHC_I1_DRDY2_BIT         3
#define LSM303DLHC_I1_WTM_BIT           2
#define LSM303DLHC_I1_OVERRUN_BIT       1

#define LSM303DLHC_I1_DRDY2_ENABLE		0x08
#define LSM303DLHC_I1_DRDY1_ENABLE		0x10

//CTRL_REG4_A
#define LSM303DLHC_BDU_BIT              7
#define LSM303DLHC_BLE_BIT              6
#define LSM303DLHC_FS_BIT               5
#define LSM303DLHC_FS_LENGTH            2
#define LSM303DLHC_HR_BIT               3
#define LSM303DLHC_SIM_BIT              0

#define LSM303DLHC_CONTINUOS_UPDATE		0x00
#define LSM303DLHC_LITTLE_ENDIAN        0x00
#define LSM303DLHC_BIG_ENDIAN           0x40
#define LSM303DLHC_FS_2                 0x00
#define LSM303DLHC_FS_4                 0x10
#define LSM303DLHC_FS_8                 0x20
#define LSM303DLHC_FS_16                0x30
#define LSM303DLHC_HIGH_RES				0x08


//CTRL_REG5_A
#define LSM303DLHC_BOOT_BIT             7
#define LSM303DLHC_FIFO_EN_BIT          6
#define LSM303DLHC_LIR_INT1_BIT         3
#define LSM303DLHC_D4D_INT1_BIT         2
#define LSM303DLHC_LIR_INT2_BIT         1
#define LSM303DLHC_D4D_INT2_BIT         0

//CTRL_REG6_A
#define LSM303DLHC_I2_CLICK_BIT         7
#define LSM303DLHC_I2_INT1_BIT          6
#define LSM303DLHC_I2_INT2_BIT          5
#define LSM303DLHC_BOOT_I1_BIT          4
#define LSM303DLHC_P2_ACT_BIT           3
#define LSM303DLHC_H_ACTIVE_BIT         1

//REFERENCE_A

//STATUS_REG_A
#define LSM303DLHC_ZYXOR_BIT            7
#define LSM303DLHC_ZOR_BIT              6
#define LSM303DLHC_YOR_BIT              5
#define LSM303DLHC_XOR_BIT              4
#define LSM303DLHC_ZYXDA_BIT            3
#define LSM303DLHC_ZDA_BIT              2
#define LSM303DLHC_YDA_BIT              1
#define LSM303DLHC_XDA_BIT              0

//FIFO_CTRL_REG_A
#define LSM303DLHC_FM_BIT               7
#define LSM303DLHC_FM_LENGTH            2
#define LSM303DLHC_TR_BIT               5
#define LSM303DLHC_FTH_BIT              4
#define LSM303DLHC_FTH_LENGTH           5

#define LSM303DLHC_FM_BYBASS            0b00
#define LSM303DLHC_FM_FIFO              0b01
#define LSM303DLHC_FM_STREAM            0b10
#define LSM303DLHC_FM_TRIGGER           0b11
#define LSM303DLHC_TR_INT1              0
#define LSM303DLHC_TR_INT2              1

//FIFO_SRC_REG_A
#define LSM303DLHC_WTM_BIT              7
#define LSM303DLHC_OVRN_FIFO_BIT        6
#define LSM303DLHC_EMPTY_BIT            5
#define LSM303DLHC_FSS_BIT              4
#define LSM303DLHC_FSS_LENGTH           5

//INT1_CFG_A
#define LSM303DLHC_INT1_AOI_BIT              7
#define LSM303DLHC_INT1_6D_BIT               6
#define LSM303DLHC_INT1_ZHIE_ZUPE_BIT        5
#define LSM303DLHC_INT1_ZLIE_ZDOWNE_BIT      4
#define LSM303DLHC_INT1_YHIE_YUPE_BIT        3
#define LSM303DLHC_INT1_YLIE_YDOWNE_BIT      2
#define LSM303DLHC_INT1_XHIE_XUPE_BIT        1
#define LSM303DLHC_INT1_XLIE_XDOWNE_BIT      0

//INT1_SRC_A
#define LSM303DLHC_INT1_IA_BIT                   6
#define LSM303DLHC_INT1_ZH_BIT                   5
#define LSM303DLHC_INT1_ZL_BIT                   4
#define LSM303DLHC_INT1_YH_BIT                   3
#define LSM303DLHC_INT1_YL_BIT                   2
#define LSM303DLHC_INT1_XH_BIT                   1
#define LSM303DLHC_INT1_XL_BIT                   0

//INT1_THS_A
#define LSM303DLHC_INT1_THS_BIT                 6
#define LSM303DLHC_INT1_THS_LENGTH              7

//INT1_DURATION_A
#define LSM303DLHC_INT1_DURATION_BIT            6
#define LSM303DLHC_INT1_DURATION_LENGTH         7

//INT2_CFG_A
#define LSM303DLHC_INT2_AOI_BIT                  7
#define LSM303DLHC_INT2_6D_BIT                   6
#define LSM303DLHC_INT2_ZHIE_BIT                 5
#define LSM303DLHC_INT2_ZLIE_BIT                 4
#define LSM303DLHC_INT2_YHIE_BIT                 3
#define LSM303DLHC_INT2_YLIE_BIT                 2
#define LSM303DLHC_INT2_XHIE_BIT                 1
#define LSM303DLHC_INT2_XLIE_BIT                 0

//INT2_SRC_A
#define LSM303DLHC_INT2_IA_BIT                   6
#define LSM303DLHC_INT2_ZH_BIT                   5
#define LSM303DLHC_INT2_ZL_BIT                   4
#define LSM303DLHC_INT2_YH_BIT                   3
#define LSM303DLHC_INT2_YL_BIT                   2
#define LSM303DLHC_INT2_XH_BIT                   1
#define LSM303DLHC_INT2_XL_BIT                   0

//INT2_THS_A
#define LSM303DLHC_INT2_THS_BIT                 6
#define LSM303DLHC_INT2_THS_LENGTH              7

//INT2_DURATION_A
#define LSM303DLHC_INT2_DURATION_BIT            6
#define LSM303DLHC_INT2_DURATION_LENGTH         7

//CLICK_CFG_A
#define LSM303DLHC_CLICK_ZD_BIT               5
#define LSM303DLHC_CLICK_ZS_BIT               4
#define LSM303DLHC_CLICK_YD_BIT               3
#define LSM303DLHC_CLICK_YS_BIT               2
#define LSM303DLHC_CLICK_XD_BIT               1
#define LSM303DLHC_CLICK_XS_BIT               0

//CLICK_SRC_A
#define LSM303DLHC_CLICK_IA_BIT               6
#define LSM303DLHC_CLICK_DCLICK_BIT           5
#define LSM303DLHC_CLICK_SCLICK_BIT           4
#define LSM303DLHC_CLICK_SIGN_BIT             3
#define LSM303DLHC_CLICK_Z_BIT                2
#define LSM303DLHC_CLICK_Y_BIT                1
#define LSM303DLHC_CLICK_X_BIT                0

//CLICK_THS_A
#define LSM303DLHC_CLICK_THS_BIT                 6
#define LSM303DLHC_CLICK_THS_LENGTH              7

//TIME_LIMIT_A
#define LSM303DLHC_TLI_BIT                      6
#define LSM303DLHC_TLI_LENGTH                   7

//TIME_LATENCY_A

//TIME_WINDOW_A

//CRA_REG_M
#define LSM303DLHC_TEMP_EN_BIT              7
#define LSM303DLHC_DO_BIT                   4
#define LSM303DLHC_DO_LENGTH                3

#define LSM303DLHC_DO_RATE_0                0x00
#define LSM303DLHC_DO_RATE_1                0x04
#define LSM303DLHC_DO_RATE_3                0x08
#define LSM303DLHC_DO_RATE_7                0x0C
#define LSM303DLHC_DO_RATE_15               0x10
#define LSM303DLHC_DO_RATE_30               0x14
#define LSM303DLHC_DO_RATE_75               0x18
#define LSM303DLHC_DO_RATE_220              0x1C

#define LSM303DLHC_TEMP_ENABLE				0x80
#define LSM303DLHC_TEMP_DISABLE				0x00

//CRB_REG_M
#define LSM303DLHC_GN_BIT                   7
#define LSM303DLHC_GN_LENGTH                3

#define LSM303DLHC_GN_1100                  0x20
#define LSM303DLHC_GN_855                   0x40
#define LSM303DLHC_GN_670                   0x60
#define LSM303DLHC_GN_450                   0x80
#define LSM303DLHC_GN_400                   0xA0
#define LSM303DLHC_GN_330                   0xC0
#define LSM303DLHC_GN_230                   0xE0

//MR_REG_M
#define LSM303DLHC_MD_BIT                   1
#define LSM303DLHC_MD_LENGTH                2

#define LSM303DLHC_MD_CONTINUOUS            0x00
#define LSM303DLHC_MD_SINGLE                0x01
#define LSM303DLHC_MD_SLEEP                 0x02

//OUT_X_H_M
//OUT_X_L_M
//OUT_Y_H_M
//OUT_Y_L_M
//OUT_Z_H_M
//OUT_Z_L_M

#define LSM303DLHC_OUT_MULTI_READ			0x80

//SR_REG_M
#define LSM303DLHC_M_DRDY_BIT                 0
#define LSM303DLHC_M_LOCK_BIT                 1

//IRA_REG_M
//IRB_REG_M
//IRC_REG_M

//TEMP_OUT_H_M
//TEMP_OUT_L_M

#endif /* LSM303DLHC_H_ */

void LSM_Error(void);
void LSM_Success(void);
static uint8_t LSM_I2C_Read(uint16_t Addr, uint8_t Reg);
static void LSM_I2C_Write(uint16_t Addr, uint8_t Reg, uint8_t Value);
uint8_t LSM_I2C_ReadID(uint16_t Addr);
void LSM_Accel_Ini(void);
void LSM_Accel_GetXYZ(int16_t* pData);
void LSM_Mag_Ini(void);
void LSM_Mag_GetXYZ(int16_t* pData);

