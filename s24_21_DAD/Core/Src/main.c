/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "string.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

#define CC120X_IOCFG3                   0x0000
#define CC120X_IOCFG2                   0x0001
#define CC120X_IOCFG1                   0x0002
#define CC120X_IOCFG0                   0x0003
#define CC120X_SYNC3                    0x0004
#define CC120X_SYNC2                    0x0005
#define CC120X_SYNC1                    0x0006
#define CC120X_SYNC0                    0x0007
#define CC120X_SYNC_CFG1                0x0008
#define CC120X_SYNC_CFG0                0x0009
#define CC120X_DEVIATION_M              0x000A
#define CC120X_MODCFG_DEV_E             0x000B
#define CC120X_DCFILT_CFG               0x000C
#define CC120X_PREAMBLE_CFG1            0x000D
#define CC120X_PREAMBLE_CFG0            0x000E
#define CC120X_IQIC                     0x000F
#define CC120X_CHAN_BW                  0x0010
#define CC120X_MDMCFG1                  0x0011
#define CC120X_MDMCFG0                  0x0012
#define CC120X_SYMBOL_RATE2             0x0013
#define CC120X_SYMBOL_RATE1             0x0014
#define CC120X_SYMBOL_RATE0             0x0015
#define CC120X_AGC_REF                  0x0016
#define CC120X_AGC_CS_THR               0x0017
#define CC120X_AGC_GAIN_ADJUST          0x0018
#define CC120X_AGC_CFG3                 0x0019
#define CC120X_AGC_CFG2                 0x001A
#define CC120X_AGC_CFG1                 0x001B
#define CC120X_AGC_CFG0                 0x001C
#define CC120X_FIFO_CFG                 0x001D
#define CC120X_DEV_ADDR                 0x001E
#define CC120X_SETTLING_CFG             0x001F
#define CC120X_FS_CFG                   0x0020
#define CC120X_WOR_CFG1                 0x0021
#define CC120X_WOR_CFG0                 0x0022
#define CC120X_WOR_EVENT0_MSB           0x0023
#define CC120X_WOR_EVENT0_LSB           0x0024
#define CC120X_RXDCM_TIME               0x0025
#define CC120X_PKT_CFG2                 0x0026
#define CC120X_PKT_CFG1                 0x0027
#define CC120X_PKT_CFG0                 0x0028
#define CC120X_RFEND_CFG1               0x0029
#define CC120X_RFEND_CFG0               0x002A
#define CC120X_PA_CFG1                  0x002B
#define CC120X_PA_CFG0                  0x002C
#define CC120X_ASK_CFG                  0x002D
#define CC120X_PKT_LEN                  0x002E

/* Extended Configuration Registers */
#define CC120X_IF_MIX_CFG               0x2F00
#define CC120X_FREQOFF_CFG              0x2F01
#define CC120X_TOC_CFG                  0x2F02
#define CC120X_MARC_SPARE               0x2F03
#define CC120X_ECG_CFG                  0x2F04
#define CC120X_MDMCFG2                  0x2F05
#define CC120X_EXT_CTRL                 0x2F06
#define CC120X_RCCAL_FINE               0x2F07
#define CC120X_RCCAL_COARSE             0x2F08
#define CC120X_RCCAL_OFFSET             0x2F09
#define CC120X_FREQOFF1                 0x2F0A
#define CC120X_FREQOFF0                 0x2F0B
#define CC120X_FREQ2                    0x2F0C
#define CC120X_FREQ1                    0x2F0D
#define CC120X_FREQ0                    0x2F0E
#define CC120X_IF_ADC2                  0x2F0F
#define CC120X_IF_ADC1                  0x2F10
#define CC120X_IF_ADC0                  0x2F11
#define CC120X_FS_DIG1                  0x2F12
#define CC120X_FS_DIG0                  0x2F13
#define CC120X_FS_CAL3                  0x2F14
#define CC120X_FS_CAL2                  0x2F15
#define CC120X_FS_CAL1                  0x2F16
#define CC120X_FS_CAL0                  0x2F17
#define CC120X_FS_CHP                   0x2F18
#define CC120X_FS_DIVTWO                0x2F19
#define CC120X_FS_DSM1                  0x2F1A
#define CC120X_FS_DSM0                  0x2F1B
#define CC120X_FS_DVC1                  0x2F1C
#define CC120X_FS_DVC0                  0x2F1D
#define CC120X_FS_LBI                   0x2F1E
#define CC120X_FS_PFD                   0x2F1F
#define CC120X_FS_PRE                   0x2F20
#define CC120X_FS_REG_DIV_CML           0x2F21
#define CC120X_FS_SPARE                 0x2F22
#define CC120X_FS_VCO4                  0x2F23
#define CC120X_FS_VCO3                  0x2F24
#define CC120X_FS_VCO2                  0x2F25
#define CC120X_FS_VCO1                  0x2F26
#define CC120X_FS_VCO0                  0x2F27
#define CC120X_GBIAS6                   0x2F28
#define CC120X_GBIAS5                   0x2F29
#define CC120X_GBIAS4                   0x2F2A
#define CC120X_GBIAS3                   0x2F2B
#define CC120X_GBIAS2                   0x2F2C
#define CC120X_GBIAS1                   0x2F2D
#define CC120X_GBIAS0                   0x2F2E
#define CC120X_IFAMP                    0x2F2F
#define CC120X_LNA                      0x2F30
#define CC120X_RXMIX                    0x2F31
#define CC120X_XOSC5                    0x2F32
#define CC120X_XOSC4                    0x2F33
#define CC120X_XOSC3                    0x2F34
#define CC120X_XOSC2                    0x2F35
#define CC120X_XOSC1                    0x2F36
#define CC120X_XOSC0                    0x2F37
#define CC120X_ANALOG_SPARE             0x2F38
#define CC120X_PA_CFG3                  0x2F39
#define CC120X_IRQ0M                    0x2F3F
#define CC120X_IRQ0F                    0x2F40

/* Status Registers */
#define CC120X_WOR_TIME1                0x2F64
#define CC120X_WOR_TIME0                0x2F65
#define CC120X_WOR_CAPTURE1             0x2F66
#define CC120X_WOR_CAPTURE0             0x2F67
#define CC120X_BIST                     0x2F68
#define CC120X_DCFILTOFFSET_I1          0x2F69
#define CC120X_DCFILTOFFSET_I0          0x2F6A
#define CC120X_DCFILTOFFSET_Q1          0x2F6B
#define CC120X_DCFILTOFFSET_Q0          0x2F6C
#define CC120X_IQIE_I1                  0x2F6D
#define CC120X_IQIE_I0                  0x2F6E
#define CC120X_IQIE_Q1                  0x2F6F
#define CC120X_IQIE_Q0                  0x2F70
#define CC120X_RSSI1                    0x2F71
#define CC120X_RSSI0                    0x2F72
#define CC120X_MARCSTATE                0x2F73
#define CC120X_LQI_VAL                  0x2F74
#define CC120X_PQT_SYNC_ERR             0x2F75
#define CC120X_DEM_STATUS               0x2F76
#define CC120X_FREQOFF_EST1             0x2F77
#define CC120X_FREQOFF_EST0             0x2F78
#define CC120X_AGC_GAIN3                0x2F79
#define CC120X_AGC_GAIN2                0x2F7A
#define CC120X_AGC_GAIN1                0x2F7B
#define CC120X_AGC_GAIN0                0x2F7C
#define CC120X_CFM_RX_DATA_OUT         0x2F7D
#define CC120X_CFM_TX_DATA_IN          0x2F7E
#define CC120X_ASK_SOFT_RX_DATA         0x2F7F
#define CC120X_RNDGEN                   0x2F80
#define CC120X_MAGN2                    0x2F81
#define CC120X_MAGN1                    0x2F82
#define CC120X_MAGN0                    0x2F83
#define CC120X_ANG1                     0x2F84
#define CC120X_ANG0                     0x2F85
#define CC120X_CHFILT_I2                0x2F86
#define CC120X_CHFILT_I1                0x2F87
#define CC120X_CHFILT_I0                0x2F88
#define CC120X_CHFILT_Q2                0x2F89
#define CC120X_CHFILT_Q1                0x2F8A
#define CC120X_CHFILT_Q0                0x2F8B
#define CC120X_GPIO_STATUS              0x2F8C
#define CC120X_FSCAL_CTRL               0x2F8D
#define CC120X_PHASE_ADJUST             0x2F8E
#define CC120X_PARTNUMBER               0x2F8F
#define CC120X_PARTVERSION              0x2F90
#define CC120X_SERIAL_STATUS            0x2F91
#define CC120X_MODEM_STATUS1            0x2F92
#define CC120X_MODEM_STATUS0            0x2F93
#define CC120X_MARC_STATUS1             0x2F94
#define CC120X_MARC_STATUS0             0x2F95
#define CC120X_PA_IFAMP_TEST            0x2F96
#define CC120X_FSRF_TEST                0x2F97
#define CC120X_PRE_TEST                 0x2F98
#define CC120X_PRE_OVR                  0x2F99
#define CC120X_ADC_TEST                 0x2F9A
#define CC120X_DVC_TEST                 0x2F9B
#define CC120X_ATEST                    0x2F9C
#define CC120X_ATEST_LVDS               0x2F9D
#define CC120X_ATEST_MODE               0x2F9E
#define CC120X_XOSC_TEST1               0x2F9F
#define CC120X_XOSC_TEST0               0x2FA0
#define CC120X_AES                      0x2FA1
#define CC120X_MDM_TEST                 0x2FA2

#define CC120X_RXFIRST                  0x2FD2
#define CC120X_TXFIRST                  0x2FD3
#define CC120X_RXLAST                   0x2FD4
#define CC120X_TXLAST                   0x2FD5
#define CC120X_NUM_TXBYTES              0x2FD6  /* Number of bytes in TXFIFO */
#define CC120X_NUM_RXBYTES              0x2FD7  /* Number of bytes in RXFIFO */
#define CC120X_FIFO_NUM_TXBYTES         0x2FD8
#define CC120X_FIFO_NUM_RXBYTES         0x2FD9
#define CC120X_RXFIFO_PRE_BUF           0x2FDA

/* DATA FIFO Access */
#define CC120X_SINGLE_TXFIFO            0x003F     /*  TXFIFO  - Single accecss to Transmit FIFO */
#define CC120X_BURST_TXFIFO             0x007F     /*  TXFIFO  - Burst accecss to Transmit FIFO  */
#define CC120X_SINGLE_RXFIFO            0x00BF     /*  RXFIFO  - Single accecss to Receive FIFO  */
#define CC120X_BURST_RXFIFO             0x00FF     /*  RXFIFO  - Busrrst ccecss to Receive FIFO  */

/* AES Workspace */
/* AES Key */
#define CC120X_AES_KEY                  0x2FE0     /*  AES_KEY    - Address for AES key input  */
#define CC120X_AES_KEY15	        0x2FE0
#define CC120X_AES_KEY14	        0x2FE1
#define CC120X_AES_KEY13	        0x2FE2
#define CC120X_AES_KEY12	        0x2FE3
#define CC120X_AES_KEY11	        0x2FE4
#define CC120X_AES_KEY10	        0x2FE5
#define CC120X_AES_KEY9	                0x2FE6
#define CC120X_AES_KEY8	                0x2FE7
#define CC120X_AES_KEY7	                0x2FE8
#define CC120X_AES_KEY6	                0x2FE9
#define CC120X_AES_KEY5	                0x2FE10
#define CC120X_AES_KEY4	                0x2FE11
#define CC120X_AES_KEY3	                0x2FE12
#define CC120X_AES_KEY2	                0x2FE13
#define CC120X_AES_KEY1	                0x2FE14
#define CC120X_AES_KEY0	                0x2FE15

/* AES Buffer */
#define CC120X_AES_BUFFER               0x2FF0     /*  AES_BUFFER - Address for AES Buffer     */
#define CC120X_AES_BUFFER15		0x2FF0
#define CC120X_AES_BUFFER14		0x2FF1
#define CC120X_AES_BUFFER13		0x2FF2
#define CC120X_AES_BUFFER12		0x2FF3
#define CC120X_AES_BUFFER11		0x2FF4
#define CC120X_AES_BUFFER10		0x2FF5
#define CC120X_AES_BUFFER9		0x2FF6
#define CC120X_AES_BUFFER8		0x2FF7
#define CC120X_AES_BUFFER7		0x2FF8
#define CC120X_AES_BUFFER6		0x2FF9
#define CC120X_AES_BUFFER5		0x2FF10
#define CC120X_AES_BUFFER4		0x2FF11
#define CC120X_AES_BUFFER3		0x2FF12
#define CC120X_AES_BUFFER2		0x2FF13
#define CC120X_AES_BUFFER1		0x2FF14
#define CC120X_AES_BUFFER0		0x2FF15

#define CC120X_LQI_CRC_OK_BM            0x80
#define CC120X_LQI_EST_BM               0x7F

/* Command strobe registers */
#define CC120X_SRES                     0x30      /*  SRES    - Reset chip. */
#define CC120X_SFSTXON                  0x31      /*  SFSTXON - Enable and calibrate frequency synthesizer. */
#define CC120X_SXOFF                    0x32      /*  SXOFF   - Turn off crystal oscillator. */
#define CC120X_SCAL                     0x33      /*  SCAL    - Calibrate frequency synthesizer and turn it off. */
#define CC120X_SRX                      0x34      /*  SRX     - Enable RX. Perform calibration if enabled. */
#define CC120X_STX                      0x35      /*  STX     - Enable TX. If in RX state, only enable TX if CCA passes. */
#define CC120X_SIDLE                    0x36      /*  SIDLE   - Exit RX / TX, turn off frequency synthesizer. */
#define CC120X_SAFC                     0x37      /*  AFC     - Automatic Frequency Correction */
#define CC120X_SWOR                     0x38      /*  SWOR    - Start automatic RX polling sequence (Wake-on-Radio) */
#define CC120X_SPWD                     0x39      /*  SPWD    - Enter power down mode when CSn goes high. */
#define CC120X_SFRX                     0x3A      /*  SFRX    - Flush the RX FIFO buffer. */
#define CC120X_SFTX                     0x3B      /*  SFTX    - Flush the TX FIFO buffer. */
#define CC120X_SWORRST                  0x3C      /*  SWORRST - Reset real time clock. */
#define CC120X_SNOP                     0x3D      /*  SNOP    - No operation. Returns status byte. */

/* Chip states returned in status byte */
#define CC120X_STATE_IDLE               0x00
#define CC120X_STATE_RX                 0x10
#define CC120X_STATE_TX                 0x20
#define CC120X_STATE_FSTXON             0x30
#define CC120X_STATE_CALIBRATE          0x40
#define CC120X_STATE_SETTLING           0x50
#define CC120X_STATE_RXFIFO_ERROR       0x60
#define CC120X_STATE_TXFIFO_ERROR       0x70

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

typedef struct {
    uint16_t registerAddress;
    uint8_t registerValue;
} registerSetting_t;

static const registerSetting_t preferredSettings[]= {{0x0000 , 0x06}, //iocfg3
{0x0001 , 0x06}, //iocfg2
{0x0002 , 0x30}, //iocfg1
{0x0003 , 0x3c}, //iocfg0
{0x0004 , 0x93}, //sync3
{0x0005 , 0x0b}, //sync2
{0x0006 , 0x51}, //sync1
{0x0007 , 0xde}, //sync0
{0x0008 , 0xa8}, //sync_cfg1
{0x0009 , 0x03}, //sync_cfg0
{0x000a , 0x47}, //deviation_m
{0x000b , 0x2f}, //modcfg_dev_e
{0x000c , 0x1e}, //dcfilt_cfg
{0x000d , 0x14}, //preamble_cfg1
{0x000e , 0x8a}, //preamble_cfg0
{0x000f , 0x00}, //iqic
{0x0010 , 0x01}, //chan_bw
{0x0011 , 0x42}, //mdmcfg1
{0x0012 , 0x05}, //mdmcfg0
{0x0013 , 0xc9}, //symbol_rate2
{0x0014 , 0x99}, //symbol_rate1
{0x0015 , 0x99}, //symbol_rate0
{0x0016 , 0x2f}, //agc_ref
{0x0017 , 0xf7}, //agc_cs_thr
{0x0018 , 0x00}, //agc_gain_adjust
{0x0019 , 0xb1}, //agc_cfg3
{0x001a , 0x60}, //agc_cfg2
{0x001b , 0x00}, //agc_cfg1
{0x001c , 0x80}, //agc_cfg0
{0x001d , 0x00}, //fifo_cfg
{0x001e , 0x00}, //dev_addr
{0x001f , 0x03}, //settling_cfg
{0x0020 , 0x12}, //fs_cfg
{0x0021 , 0x08}, //wor_cfg1
{0x0022 , 0x20}, //wor_cfg0
{0x0023 , 0x00}, //wor_event0_msb
{0x0024 , 0x14}, //wor_event0_lsb
{0x0025 , 0x00}, //rxdcm_time
{0x0026 , 0x00}, //pkt_cfg2
{0x0027 , 0x03}, //pkt_cfg1
{0x0028 , 0x20}, //pkt_cfg0
{0x0029 , 0x0f}, //rfend_cfg1
{0x002a , 0x09}, //rfend_cfg0
{0x002b , 0x7f}, //pa_cfg1
{0x002c , 0x56}, //pa_cfg0
{0x002d , 0x0f}, //ask_cfg
{0x002e , 0xff}, //pkt_len
{0x2f00 , 0x00}, //if_mix_cfg
{0x2f01 , 0x23}, //freqoff_cfg
{0x2f02 , 0x0b}, //toc_cfg
{0x2f03 , 0x00}, //marc_spare
{0x2f04 , 0x00}, //ecg_cfg
{0x2f05 , 0x00}, //mdmcfg2
{0x2f06 , 0x01}, //ext_ctrl
{0x2f07 , 0x00}, //rccal_fine
{0x2f08 , 0x00}, //rccal_coarse
{0x2f09 , 0x00}, //rccal_offset
{0x2f0a , 0x00}, //freqoff1
{0x2f0b , 0x00}, //freqoff0
{0x2f0c , 0x5c}, //freq2
{0x2f0d , 0x0f}, //freq1
{0x2f0e , 0x5c}, //freq0
{0x2f0f , 0x02}, //if_adc2
{0x2f10 , 0xee}, //if_adc1
{0x2f11 , 0x10}, //if_adc0
{0x2f12 , 0x07}, //fs_dig1
{0x2f13 , 0xa0}, //fs_dig0
{0x2f14 , 0x00}, //fs_cal3
{0x2f15 , 0x20}, //fs_cal2
{0x2f16 , 0x40}, //fs_cal1
{0x2f17 , 0x0e}, //fs_cal0
{0x2f18 , 0x28}, //fs_chp
{0x2f19 , 0x03}, //fs_divtwo
{0x2f1a , 0x00}, //fs_dsm1
{0x2f1b , 0x33}, //fs_dsm0
{0x2f1c , 0xff}, //fs_dvc1
{0x2f1d , 0x17}, //fs_dvc0
{0x2f1e , 0x00}, //fs_lbi
{0x2f1f , 0x00}, //fs_pfd
{0x2f20 , 0x6e}, //fs_pre
{0x2f21 , 0x1c}, //fs_reg_div_cml
{0x2f22 , 0xac}, //fs_spare
{0x2f23 , 0x14}, //fs_vco4
{0x2f24 , 0x00}, //fs_vco3
{0x2f25 , 0x00}, //fs_vco2
{0x2f26 , 0x00}, //fs_vco1
{0x2f27 , 0xb5}, //fs_vco0
{0x2f28 , 0x00}, //gbias6
{0x2f29 , 0x02}, //gbias5
{0x2f2a , 0x00}, //gbias4
{0x2f2b , 0x00}, //gbias3
{0x2f2c , 0x10}, //gbias2
{0x2f2d , 0x00}, //gbias1
{0x2f2e , 0x00}, //gbias0
{0x2f2f , 0x0d}, //ifamp
{0x2f30 , 0x01}, //lna
{0x2f31 , 0x01}, //rxmix
{0x2f32 , 0x0e}, //xosc5
{0x2f33 , 0xa0}, //xosc4
{0x2f34 , 0x03}, //xosc3
{0x2f35 , 0x04}, //xosc2
{0x2f36 , 0x03}, //xosc1
{0x2f37 , 0x00}, //xosc0
{0x2f38 , 0x00}, //analog_spare
{0x2f39 , 0x00}, //pa_cfg3
{0x2f64 , 0x00}, //wor_time1
{0x2f65 , 0x00}, //wor_time0
{0x2f66 , 0x00}, //wor_capture1
{0x2f67 , 0x00}, //wor_capture0
{0x2f68 , 0x00}, //bist
{0x2f69 , 0x00}, //dcfiltoffset_i1
{0x2f6a , 0x00}, //dcfiltoffset_i0
{0x2f6b , 0x00}, //dcfiltoffset_q1
{0x2f6c , 0x00}, //dcfiltoffset_q0
{0x2f6d , 0x00}, //iqie_i1
{0x2f6e , 0x00}, //iqie_i0
{0x2f6f , 0x00}, //iqie_q1
{0x2f70 , 0x00}, //iqie_q0
{0x2f71 , 0x80}, //rssi1
{0x2f72 , 0x00}, //rssi0
{0x2f73 , 0x41}, //marcstate
{0x2f74 , 0x00}, //lqi_val
{0x2f75 , 0xff}, //pqt_sync_err
{0x2f76 , 0x00}, //dem_status
{0x2f77 , 0x00}, //freqoff_est1
{0x2f78 , 0x00}, //freqoff_est0
{0x2f79 , 0x00}, //agc_gain3
{0x2f7a , 0xd1}, //agc_gain2
{0x2f7b , 0x00}, //agc_gain1
{0x2f7c , 0x3f}, //agc_gain0
{0x2f7d , 0x00}, //cfm_rx_data_out
{0x2f7e , 0x00}, //cfm_tx_data_in
{0x2f7f , 0x30}, //ask_soft_rx_data
{0x2f80 , 0x7f}, //rndgen
{0x2f81 , 0x00}, //magn2
{0x2f82 , 0x00}, //magn1
{0x2f83 , 0x00}, //magn0
{0x2f84 , 0x00}, //ang1
{0x2f85 , 0x00}, //ang0
{0x2f86 , 0x02}, //chfilt_i2
{0x2f87 , 0x00}, //chfilt_i1
{0x2f88 , 0x00}, //chfilt_i0
{0x2f89 , 0x00}, //chfilt_q2
{0x2f8a , 0x00}, //chfilt_q1
{0x2f8b , 0x00}, //chfilt_q0
{0x2f8c , 0x00}, //gpio_status
{0x2f8d , 0x01}, //fscal_ctrl
{0x2f8e , 0x00}, //phase_adjust
{0x2f8f , 0x00}, //partnumber
{0x2f90 , 0x00}, //partversion
{0x2f91 , 0x00}, //serial_status
{0x2f92 , 0x01}, //modem_status1
{0x2f93 , 0x00}, //modem_status0
{0x2f94 , 0x00}, //marc_status1
{0x2f95 , 0x00}, //marc_status0
{0x2f96 , 0x00}, //pa_ifamp_test
{0x2f97 , 0x00}, //fsrf_test
{0x2f98 , 0x00}, //pre_test
{0x2f99 , 0x00}, //pre_ovr
{0x2f9a , 0x00}, //adc_test
{0x2f9b , 0x0b}, //dvc_test
{0x2f9c , 0x40}, //atest
{0x2f9d , 0x00}, //atest_lvds
{0x2f9e , 0x00}, //atest_mode
{0x2f9f , 0x3c}, //xosc_test1
{0x2fa0 , 0x00}, //xosc_test0
{0x2fa1 , 0x00}, //aes
{0x2fa2 , 0x00}, //mdm_test
{0x2fd2 , 0x00}, //rxfirst
{0x2fd3 , 0x00}, //txfirst
{0x2fd4 , 0x00}, //rxlast
{0x2fd5 , 0x00}, //txlast
{0x2fd6 , 0x00}, //num_txbytes
{0x2fd7 , 0x00}, //num_rxbytes
{0x2fd8 , 0x0f}, //fifo_num_txbytes
{0x2fd9 , 0x00}, //fifo_num_rxbytes
{0x2fda , 0x00}, //rxfifo_pre_buf
{0x2fe0 , 0x00}, //aes_key15
{0x2fe1 , 0x00}, //aes_key14
{0x2fe2 , 0x00}, //aes_key13
{0x2fe3 , 0x00}, //aes_key12
{0x2fe4 , 0x00}, //aes_key11
{0x2fe5 , 0x00}, //aes_key10
{0x2fe6 , 0x00}, //aes_key9
{0x2fe7 , 0x00}, //aes_key8
{0x2fe8 , 0x00}, //aes_key7
{0x2fe9 , 0x00}, //aes_key6
{0x2fea , 0x00}, //aes_key5
{0x2feb , 0x00}, //aes_key4
{0x2fec , 0x00}, //aes_key3
{0x2fed , 0x00}, //aes_key2
{0x2fee , 0x00}, //aes_key1
{0x2fef , 0x00}, //aes_key0
{0x2ff0 , 0x00}, //aes_buffer15
{0x2ff1 , 0x00}, //aes_buffer14
{0x2ff2 , 0x00}, //aes_buffer13
{0x2ff3 , 0x00}, //aes_buffer12
{0x2ff4 , 0x00}, //aes_buffer11
{0x2ff5 , 0x00}, //aes_buffer10
{0x2ff6 , 0x00}, //aes_buffer9
{0x2ff7 , 0x00}, //aes_buffer8
{0x2ff8 , 0x00}, //aes_buffer7
{0x2ff9 , 0x00}, //aes_buffer6
{0x2ffa , 0x00}, //aes_buffer5
{0x2ffb , 0x00}, //aes_buffer4
{0x2ffc , 0x00}, //aes_buffer3
{0x2ffd , 0x00}, //aes_buffer2
{0x2ffe , 0x00}, //aes_buffer1
{0x2fff , 0x00}, //aes_buffer0
};

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
#define TEMPERATURE 0b00
#define HUMIDITY 0b01
#define SOUND 0b10
#define VIBRATION 0b11

#define FIFO_SIZE 4//2300


typedef struct {
  uint8_t sop;        // Start of packet
  uint8_t datatype;   // Data type
  uint8_t sensorId;   // Sensor ID
  uint32_t timestamp; // Timestamp
//  uint64_t data; 	  // Sensor data
  int64_t data;        // Sensor data
  uint8_t crc;        // CRC for error checking
  uint8_t eop;
} SensorDataPacket;

typedef enum { // FSM States
    UART_WAIT_FOR_SOP,
    UART_DATATYPE,
    UART_SENSOR_ID,
    UART_TIMESTAMP,
    UART_DATA,
    UART_CRC,
	UART_EOP,
    UART_DONE
} UART_State_t;

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
#if defined ( __ICCARM__ ) /*!< IAR Compiler */
#pragma location=0x30000000
ETH_DMADescTypeDef  DMARxDscrTab[ETH_RX_DESC_CNT]; /* Ethernet Rx DMA Descriptors */
#pragma location=0x30000200
ETH_DMADescTypeDef  DMATxDscrTab[ETH_TX_DESC_CNT]; /* Ethernet Tx DMA Descriptors */

#elif defined ( __CC_ARM )  /* MDK ARM Compiler */

__attribute__((at(0x30000000))) ETH_DMADescTypeDef  DMARxDscrTab[ETH_RX_DESC_CNT]; /* Ethernet Rx DMA Descriptors */
__attribute__((at(0x30000200))) ETH_DMADescTypeDef  DMATxDscrTab[ETH_TX_DESC_CNT]; /* Ethernet Tx DMA Descriptors */

#elif defined ( __GNUC__ ) /* GNU Compiler */
ETH_DMADescTypeDef DMARxDscrTab[ETH_RX_DESC_CNT] __attribute__((section(".RxDecripSection"))); /* Ethernet Rx DMA Descriptors */
ETH_DMADescTypeDef DMATxDscrTab[ETH_TX_DESC_CNT] __attribute__((section(".TxDecripSection")));   /* Ethernet Tx DMA Descriptors */

#endif

ETH_TxPacketConfig TxConfig;

ETH_HandleTypeDef heth;

SPI_HandleTypeDef hspi1;
SPI_HandleTypeDef hspi5;
DMA_HandleTypeDef hdma_spi1_tx;

TIM_HandleTypeDef htim16;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
UART_HandleTypeDef huart3;
UART_HandleTypeDef huart6;

/* USER CODE BEGIN PV */

uint8_t rx_data1[1];
SensorDataPacket sensorData1;
UART_State_t uartState1 = UART_WAIT_FOR_SOP;
uint32_t timestampBuffer1;
uint64_t dataBuffer1;
uint32_t dataIndex1 = 0; // Used for buffer indexing

// Declaration for USART2
uint8_t rx_data2[1];
SensorDataPacket sensorData2;
UART_State_t uartState2 = UART_WAIT_FOR_SOP;
uint32_t timestampBuffer2;
uint64_t dataBuffer2;
uint32_t dataIndex2 = 0; // Used for buffer indexing

// Declaration for USART3
uint8_t rx_data3[1];
SensorDataPacket sensorData3;
UART_State_t uartState3 = UART_WAIT_FOR_SOP;
uint32_t timestampBuffer3;
uint64_t dataBuffer3;
uint32_t dataIndex3 = 0; // Used for buffer indexing


// Declaration for USART6
uint8_t rx_data6[1];
SensorDataPacket sensorData6;
UART_State_t uartState6 = UART_WAIT_FOR_SOP;
uint32_t timestampBuffer6;
uint64_t dataBuffer6;
uint32_t dataIndex6 = 0; // Used for buffer indexing

uint8_t crc_calculated = 0; // Placeholder for the calculated CRC



SensorDataPacket *PutPt1;
SensorDataPacket *GetPt1;
SensorDataPacket Fifo1[FIFO_SIZE];


SensorDataPacket *PutPt2;
SensorDataPacket *GetPt2;
SensorDataPacket Fifo2[FIFO_SIZE];

SensorDataPacket *PutPt3;
SensorDataPacket *GetPt3;
SensorDataPacket Fifo3[FIFO_SIZE];

SensorDataPacket *PutPt6;
SensorDataPacket *GetPt6;
SensorDataPacket Fifo6[FIFO_SIZE];


uint8_t timer_counter = 0;
uint8_t test_timer = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_USB_OTG_HS_USB_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_USART6_UART_Init(void);
static void MX_SPI1_Init(void);
static void MX_SPI5_Init(void);
static void MX_ETH_Init(void);
static void MX_TIM16_Init(void);
/* USER CODE BEGIN PFP */

void resetState(void);
int validateCRC(const SensorDataPacket *packet);
void printData(const SensorDataPacket *packet);
void processUartData(UART_HandleTypeDef *huart, SensorDataPacket *sensorData, uint8_t *rxData,
                     UART_State_t *uartState, uint32_t *timestampBuffer, uint64_t *dataBuffer, uint32_t *dataIndex);
void resetUartState(UART_State_t *uartState, uint32_t *timestampBuffer, uint64_t *dataBuffer);
void unpackData(uint64_t packedData, int16_t* x, int16_t* y, int16_t* z);


int GetFifo1(SensorDataPacket* data);
int PutFifo1(SensorDataPacket data);
int GetFifo2(SensorDataPacket* data);
int PutFifo2(SensorDataPacket data);
void InitFifo(void);


uint8_t tx_data[8];
uint8_t rx_data[8]={0,0,0,0,0,0,0,0};
uint8_t rx_data_sender[8]={0,0,0,0,0,0,0,0};


/////// reset pin for TX is B7   GPIOB GPIO_PIN_7
/////// reset pin for RX is E4   GPIOE, GPIO_PIN_4
void CC1200_Reset(void);
void CC1200_Init(registerSetting_t *init_seq);
void CC1200_Offset(int16_t offset);
void CC1200_BurstModeIncr(uint8_t enable);
void CC1200_RXMode(void);
void CC1200_TXMode(void);
void CC1200_TXMode(void);
void CC1200_TXStart(void);
void CC1200_TXRXEnd(void);
void CC1200_SetFreq(uint32_t freq);
void CC1200_SetPwr(uint8_t pwr);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */


/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_USART3_UART_Init();
  MX_USB_OTG_HS_USB_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  MX_USART6_UART_Init();
  MX_SPI1_Init();
  MX_SPI5_Init();
  MX_ETH_Init();
  MX_TIM16_Init();
  /* USER CODE BEGIN 2 */

  HAL_UART_Receive_IT(&huart2, rx_data2, 1);
  //chip reset
  //CC1200_Reset();
  //HAL_Delay(100);

  //chip config
  //CC1200_Init(preferredSettings);
  //CC1200_Init(preferredSettings);

  //mode - TX/RX
  //CC1200_TXMode();
  //CC1200_RXMode();


  //dont increment address in burst mode
  //CC1200_BurstModeIncr(0);




  //HAL_UART_Receive_IT(&huart1, rx_data1, 1);
  //HAL_UART_Receive_IT(&huart2, rx_data2, 1);
  //HAL_UART_Receive_IT(&huart3, rx_data3, 1);
  HAL_UART_Receive_IT(&huart6, rx_data6, 1);

  //HAL_TIM_Base_Start_IT(&htim16);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Supply configuration update enable
  */
  HAL_PWREx_ConfigSupply(PWR_LDO_SUPPLY);

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE0);

  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI48|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
  RCC_OscInitStruct.HSI48State = RCC_HSI48_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 275;
  RCC_OscInitStruct.PLL.PLLP = 1;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  RCC_OscInitStruct.PLL.PLLR = 2;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_1;
  RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOWIDE;
  RCC_OscInitStruct.PLL.PLLFRACN = 0;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                              |RCC_CLOCKTYPE_D3PCLK1|RCC_CLOCKTYPE_D1PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV2;
  RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ETH Initialization Function
  * @param None
  * @retval None
  */
static void MX_ETH_Init(void)
{

  /* USER CODE BEGIN ETH_Init 0 */

  /* USER CODE END ETH_Init 0 */

   static uint8_t MACAddr[6];

  /* USER CODE BEGIN ETH_Init 1 */

  /* USER CODE END ETH_Init 1 */
  heth.Instance = ETH;
  MACAddr[0] = 0x00;
  MACAddr[1] = 0x80;
  MACAddr[2] = 0xE1;
  MACAddr[3] = 0x00;
  MACAddr[4] = 0x00;
  MACAddr[5] = 0x00;
  heth.Init.MACAddr = &MACAddr[0];
  heth.Init.MediaInterface = HAL_ETH_RMII_MODE;
  heth.Init.TxDesc = DMATxDscrTab;
  heth.Init.RxDesc = DMARxDscrTab;
  heth.Init.RxBuffLen = 1524;

  /* USER CODE BEGIN MACADDRESS */

  /* USER CODE END MACADDRESS */

  if (HAL_ETH_Init(&heth) != HAL_OK)
  {
    Error_Handler();
  }

  memset(&TxConfig, 0 , sizeof(ETH_TxPacketConfig));
  TxConfig.Attributes = ETH_TX_PACKETS_FEATURES_CSUM | ETH_TX_PACKETS_FEATURES_CRCPAD;
  TxConfig.ChecksumCtrl = ETH_CHECKSUM_IPHDR_PAYLOAD_INSERT_PHDR_CALC;
  TxConfig.CRCPadCtrl = ETH_CRC_PAD_INSERT;
  /* USER CODE BEGIN ETH_Init 2 */

  /* USER CODE END ETH_Init 2 */

}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_128;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 0x0;
  hspi1.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  hspi1.Init.NSSPolarity = SPI_NSS_POLARITY_LOW;
  hspi1.Init.FifoThreshold = SPI_FIFO_THRESHOLD_01DATA;
  hspi1.Init.TxCRCInitializationPattern = SPI_CRC_INITIALIZATION_ALL_ZERO_PATTERN;
  hspi1.Init.RxCRCInitializationPattern = SPI_CRC_INITIALIZATION_ALL_ZERO_PATTERN;
  hspi1.Init.MasterSSIdleness = SPI_MASTER_SS_IDLENESS_00CYCLE;
  hspi1.Init.MasterInterDataIdleness = SPI_MASTER_INTERDATA_IDLENESS_00CYCLE;
  hspi1.Init.MasterReceiverAutoSusp = SPI_MASTER_RX_AUTOSUSP_DISABLE;
  hspi1.Init.MasterKeepIOState = SPI_MASTER_KEEP_IO_STATE_DISABLE;
  hspi1.Init.IOSwap = SPI_IO_SWAP_DISABLE;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief SPI5 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI5_Init(void)
{

  /* USER CODE BEGIN SPI5_Init 0 */

  /* USER CODE END SPI5_Init 0 */

  /* USER CODE BEGIN SPI5_Init 1 */

  /* USER CODE END SPI5_Init 1 */
  /* SPI5 parameter configuration*/
  hspi5.Instance = SPI5;
  hspi5.Init.Mode = SPI_MODE_MASTER;
  hspi5.Init.Direction = SPI_DIRECTION_2LINES;
  hspi5.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi5.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi5.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi5.Init.NSS = SPI_NSS_SOFT;
  hspi5.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_128;
  hspi5.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi5.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi5.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi5.Init.CRCPolynomial = 0x0;
  hspi5.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  hspi5.Init.NSSPolarity = SPI_NSS_POLARITY_LOW;
  hspi5.Init.FifoThreshold = SPI_FIFO_THRESHOLD_01DATA;
  hspi5.Init.TxCRCInitializationPattern = SPI_CRC_INITIALIZATION_ALL_ZERO_PATTERN;
  hspi5.Init.RxCRCInitializationPattern = SPI_CRC_INITIALIZATION_ALL_ZERO_PATTERN;
  hspi5.Init.MasterSSIdleness = SPI_MASTER_SS_IDLENESS_00CYCLE;
  hspi5.Init.MasterInterDataIdleness = SPI_MASTER_INTERDATA_IDLENESS_00CYCLE;
  hspi5.Init.MasterReceiverAutoSusp = SPI_MASTER_RX_AUTOSUSP_DISABLE;
  hspi5.Init.MasterKeepIOState = SPI_MASTER_KEEP_IO_STATE_DISABLE;
  hspi5.Init.IOSwap = SPI_IO_SWAP_DISABLE;
  if (HAL_SPI_Init(&hspi5) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI5_Init 2 */

  /* USER CODE END SPI5_Init 2 */

}

/**
  * @brief TIM16 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM16_Init(void)
{

  /* USER CODE BEGIN TIM16_Init 0 */

  /* USER CODE END TIM16_Init 0 */

  /* USER CODE BEGIN TIM16_Init 1 */

  /* USER CODE END TIM16_Init 1 */
  htim16.Instance = TIM16;
  htim16.Init.Prescaler = 550;
  htim16.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim16.Init.Period = 10000;
  htim16.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim16.Init.RepetitionCounter = 0;
  htim16.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim16) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM16_Init 2 */

  /* USER CODE END TIM16_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart1, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart1, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart2, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart2, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  huart3.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart3.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart3.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart3, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart3, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

/**
  * @brief USART6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART6_UART_Init(void)
{

  /* USER CODE BEGIN USART6_Init 0 */

  /* USER CODE END USART6_Init 0 */

  /* USER CODE BEGIN USART6_Init 1 */

  /* USER CODE END USART6_Init 1 */
  huart6.Instance = USART6;
  huart6.Init.BaudRate = 115200;
  huart6.Init.WordLength = UART_WORDLENGTH_8B;
  huart6.Init.StopBits = UART_STOPBITS_1;
  huart6.Init.Parity = UART_PARITY_NONE;
  huart6.Init.Mode = UART_MODE_TX_RX;
  huart6.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart6.Init.OverSampling = UART_OVERSAMPLING_16;
  huart6.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart6.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart6.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart6) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart6, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart6, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart6) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART6_Init 2 */

  /* USER CODE END USART6_Init 2 */

}

/**
  * @brief USB_OTG_HS Initialization Function
  * @param None
  * @retval None
  */
static void MX_USB_OTG_HS_USB_Init(void)
{

  /* USER CODE BEGIN USB_OTG_HS_Init 0 */

  /* USER CODE END USB_OTG_HS_Init 0 */

  /* USER CODE BEGIN USB_OTG_HS_Init 1 */

  /* USER CODE END USB_OTG_HS_Init 1 */
  /* USER CODE BEGIN USB_OTG_HS_Init 2 */

  /* USER CODE END USB_OTG_HS_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream0_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_4|LED_YELLOW_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, LED_GREEN_Pin|LED_RED_Pin|GPIO_PIN_7, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(USB_FS_PWR_EN_GPIO_Port, USB_FS_PWR_EN_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : PE4 LED_YELLOW_Pin */
  GPIO_InitStruct.Pin = GPIO_PIN_4|LED_YELLOW_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LED_GREEN_Pin LED_RED_Pin PB7 */
  GPIO_InitStruct.Pin = LED_GREEN_Pin|LED_RED_Pin|GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : USB_FS_PWR_EN_Pin */
  GPIO_InitStruct.Pin = USB_FS_PWR_EN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(USB_FS_PWR_EN_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : USB_FS_OVCR_Pin */
  GPIO_InitStruct.Pin = USB_FS_OVCR_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(USB_FS_OVCR_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : USB_FS_VBUS_Pin */
  GPIO_InitStruct.Pin = USB_FS_VBUS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(USB_FS_VBUS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : USB_FS_ID_Pin */
  GPIO_InitStruct.Pin = USB_FS_ID_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF10_OTG1_HS;
  HAL_GPIO_Init(USB_FS_ID_GPIO_Port, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

void InitFifo(void) {

    PutPt1 = GetPt1 = &Fifo1[0];
    PutPt2 = GetPt2 = &Fifo2[0];
    PutPt3 = GetPt3 = &Fifo3[0];
    PutPt6 = GetPt6 = &Fifo6[0];
}

int PutFifo1(SensorDataPacket data) {

	SensorDataPacket* Ppt;

    Ppt = PutPt1;

    *(Ppt++) = data;

    if (Ppt == &Fifo1[FIFO_SIZE]) {

        Ppt = &Fifo1[0]; // wrap around

    }

    if (Ppt == GetPt1) {
        return 0; // buffer full so fail
    }

    PutPt1 = Ppt;

    return 1;

}


int GetFifo1(SensorDataPacket* data) {

    if (GetPt1 == PutPt1) {

        return 0; // buffer is empty
    }
    //data pointer gets pointed to the next GetPointer
    *data = *(GetPt1++);
    if (GetPt1 == &Fifo1[FIFO_SIZE]) {

        GetPt1 = &Fifo1[0];
    }
    return 1;
}


int PutFifo2(SensorDataPacket data) {

	SensorDataPacket* Ppt;

    Ppt = PutPt2;

    *(Ppt++) = data;

    if (Ppt == &Fifo2[FIFO_SIZE]) {

        Ppt = &Fifo2[0]; // wrap around

    }

    if (Ppt == GetPt2) {
        return 0; // buffer full so fail
    }

    PutPt2 = Ppt;

    return 1;

}


int GetFifo2(SensorDataPacket* data) {

    if (GetPt2 == PutPt2) {

        return 0; // buffer is empty
    }
    //data pointer gets pointed to the next GetPointer
    *data = *(GetPt2++);
    if (GetPt2 == &Fifo2[FIFO_SIZE]) {

        GetPt2 = &Fifo2[0];
    }
    return 1;
}


int PutFifo3(SensorDataPacket data) {

	SensorDataPacket* Ppt;

    Ppt = PutPt3;

    *(Ppt++) = data;

    if (Ppt == &Fifo3[FIFO_SIZE]) {

        Ppt = &Fifo3[0]; // wrap around

    }

    if (Ppt == GetPt3) {
        return 0; // buffer full so fail
    }

    PutPt3 = Ppt;

    return 1;

}


int GetFifo3(SensorDataPacket* data) {

    if (GetPt3 == PutPt3) {

        return 0; // buffer is empty
    }
    //data pointer gets pointed to the next GetPointer
    *data = *(GetPt3++);
    if (GetPt3 == &Fifo3[FIFO_SIZE]) {

        GetPt3 = &Fifo3[0];
    }
    return 1;
}


int PutFifo6(SensorDataPacket data) {

	SensorDataPacket* Ppt;

    Ppt = PutPt6;

    *(Ppt++) = data;

    if (Ppt == &Fifo6[FIFO_SIZE]) {

        Ppt = &Fifo6[0]; // wrap around

    }

    if (Ppt == GetPt6) {
        return 0; // buffer full so fail
    }

    PutPt6 = Ppt;

    return 1;

}


int GetFifo6(SensorDataPacket* data) {

    if (GetPt6 == PutPt6) {

        return 0; // buffer is empty
    }
    //data pointer gets pointed to the next GetPointer
    *data = *(GetPt6++);
    if (GetPt6 == &Fifo6[FIFO_SIZE]) {

        GetPt6 = &Fifo6[0];
    }
    return 1;
}



void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	__NOP();

	if (htim == &htim16)
	{

		timer_counter++;
		SensorDataPacket data1[4];
		/*for(int i = 0; i < 220; i++){
			GetFifo1(&data1[i]);

		}
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, 0);
		HAL_SPI_Transmit_DMA(&hspi1, (uint8_t*)&data1, sizeof(data1));
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, 1);


		//SensorDataPacket data2[220];
		for(int i = 0; i < 220; i++){
			GetFifo2(&data1[i]);

		}
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, 0);
		HAL_SPI_Transmit_DMA(&hspi1, (uint8_t*)&data1, sizeof(data1));
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, 1);
		*/
		//SensorDataPacket data3[220];
		for(int i = 0; i < 4; i++){
			GetFifo6(&data1[i]);

		}
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, 0);
		HAL_SPI_Transmit_DMA(&hspi1, (uint8_t*)&data1, sizeof(data1));
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, 1);



		//SensorDataPacket data6[220];
		for(int i = 0; i < 220; i++){
			GetFifo6(&data1[i]);

		}
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, 0);
		HAL_SPI_Transmit_DMA(&hspi1, (uint8_t*)&data1, sizeof(data1));
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, 1);
		if(timer_counter == 25){
			HAL_GPIO_TogglePin(GPIOE, GPIO_PIN_1); //yellow
			timer_counter = 0;
		}

	}




}

void HAL_SPI_TxCpltCallback(SPI_HandleTypeDef * hspi)
{
	__NOP();

	if(hspi->Instance == SPI1){
		test_timer++;
		if(test_timer == 10)
		{
			test_timer = 0;
			HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_0); //green
		}

		//HAL_Delay(500);
	}
	else if (hspi->Instance == SPI5){
		HAL_GPIO_TogglePin(GPIOE, GPIO_PIN_1); //yellow
	}



}


void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
    if(huart->Instance == USART1) {
        // Process data from USART1
    	processUartData(huart, &sensorData1, rx_data1, &uartState1, &timestampBuffer1, &dataBuffer1, &dataIndex1);
    }
    else if(huart->Instance == USART2) {
        // Process data from USART2
    	processUartData(huart, &sensorData2, rx_data2, &uartState2, &timestampBuffer2, &dataBuffer2, &dataIndex2);
    }
    else if(huart->Instance == USART3) {
        // Process data from USART2
    	processUartData(huart, &sensorData3, rx_data3, &uartState3, &timestampBuffer3, &dataBuffer3, &dataIndex3);
    }

    else if(huart->Instance == USART6) {
        // Process data from USART2
    	processUartData(huart, &sensorData6, rx_data6, &uartState6, &timestampBuffer6, &dataBuffer6, &dataIndex6);
    }

    // Re-enable UART reception interrupt correctly for each port
    if (huart->Instance == USART1) {
        HAL_UART_Receive_IT(&huart1, rx_data1, 1);
    }
    else if (huart->Instance == USART2) {
        HAL_UART_Receive_IT(&huart2, rx_data2, 1);
    }
    else if (huart->Instance == USART3) {
    	HAL_UART_Receive_IT(&huart3, rx_data3, 1);
    }
    else if (huart->Instance == USART6) {
    	HAL_UART_Receive_IT(&huart6, rx_data6, 1);
    }
}



void processUartData(UART_HandleTypeDef *huart, SensorDataPacket *sensorData, uint8_t *rxData,
                     UART_State_t *uartState, uint32_t *timestampBuffer, uint64_t *dataBuffer, uint32_t *dataIndex) {    // Your existing switch case logic here, adapted for the specific sensorData and rx_data
    // This function needs to be adapted from your existing HAL_UART_RxCpltCallback logic
	uint8_t rxByte = *rxData; // The received byte
//    	sprintf(buffer, "RxByte: 0x%08lX\r\n", rxByte);
//    	HAL_UART_Transmit(&huart3, (uint8_t*)buffer, strlen(buffer), 100); // Print debug info
    switch (*uartState) {
        case UART_WAIT_FOR_SOP: // SOP Case
            if (rxByte == 0x53) { // SOP byte = 0x53 ('S')
            	sensorData->sop = rxByte; // Set the sop
            	*uartState = UART_DATATYPE;
//                    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_13, GPIO_PIN_SET); // Orange LED set when packet is complete
            }
            break;
        case UART_DATATYPE: // Data type Case
        	sensorData->datatype = rxByte; // Set th		e data type (Temp = 00, Humidity = 01, Sound = 10, Vibration = 11)
            *uartState = UART_SENSOR_ID; // Next parameter
            break;

        case UART_SENSOR_ID: // Sensor ID Case
        	sensorData->sensorId = rxByte; // Set the sensor ID (000, 001, 010, 011, 100, 101, 110, 111 (i.e. Sensor 1-8)
        	*dataIndex = 0; // Reset dataIndex for the next field
            *uartState = UART_TIMESTAMP; // Next parameter
//                HAL_GPIO_WritePin(GPIOD, GPIO_PIN_13, GPIO_PIN_SET); // Orange LED set when packet is complete
            break;

        case UART_TIMESTAMP: // Timestamp Case
            // Combine byte into timestamp assuming little endian - least significant byte first
//            	timestampBuffer |= ((uint32_t)rxByte << (24 - (dataIndex * 8)));
        	*timestampBuffer |= ((uint32_t)rxByte << ((*dataIndex-1) * 8));
//            	sprintf(buffer, "RxByte: 0x%08lX\r\n", rxByte);
//            	HAL_UART_Transmit(&huart3, (uint8_t*)buffer, strlen(buffer), 100); // Print debug info
//            	sprintf(buffer, "Timestamp partial: 0x%08lX\r\n", timestampBuffer);
//            	HAL_UART_Transmit(&huart3, (uint8_t*)buffer, strlen(buffer), 100); // Print debug info
            (*dataIndex)++;
            if (*dataIndex >= sizeof(sensorData->timestamp)) {
                sensorData->timestamp = *timestampBuffer; // Assign the complete timestamp
                *dataIndex = 0; // Reset dataIndex for the data field
                *timestampBuffer = 0; // Clear the buffer for the next use
                *uartState = UART_DATA; // Move to the next state
            }
            break;

        case UART_DATA: // Data Case
            // Combine byte into data assuming little endian - least significant byte first
        	*dataBuffer |= ((uint64_t)rxByte << ((*dataIndex-1) * 8));//            	sprintf(buffer, "Data partial: 0x%016llx\r\n", dataBuffer);
//            	HAL_UART_Transmit(&huart3, (uint8_t*)buffer, strlen(buffer), 100); // Print debug info
            (*dataIndex)++;
            if (*dataIndex >= sizeof(sensorData->data)) {
                sensorData->data = *dataBuffer; // Assign the complete data
                *dataIndex = 0; // Reset dataIndex for the CRC field
                *dataBuffer = 0; // Clear the buffer for the next use
                *uartState = UART_CRC; // Move to the next state
            }
            break;

        case UART_CRC: // CRC Case
        	if(rxByte != 0){
                sensorData->crc = rxByte; // Set the CRC value based on algorithm
                *uartState = UART_EOP; // Next parameter
        	}
//                HAL_GPIO_WritePin(GPIOD, GPIO_PIN_13, GPIO_PIN_SET); // Orange LED set when packet is complete
            break;

        case UART_EOP:
            if (rxByte == 0x45) { // EOP byte = 0x45 ('E')
                *uartState = UART_DONE; // Packet reception is complete
                sensorData->eop = rxByte; // Set the eop
                if(huart->Instance == USART1){
                	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, GPIO_PIN_SET); // Red LED set when packet is complete
                }
                else if(huart->Instance == USART2){
                	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_13, GPIO_PIN_SET); // Orange LED set when packet is complete
                }
                else if(huart->Instance == UART4){
                	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14, GPIO_PIN_SET); // Blue LED set when packet is complete
                }
//                HAL_GPIO_WritePin(GPIOD, GPIO_PIN_13, GPIO_PIN_SET); // Orange LED set when packet is complete
            } else {
//                    uartState = UART_DONE; // Packet reception is complete
//                    sensorData.eop = rxByte; // Set the eop
//                    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_13, GPIO_PIN_SET); // Orange LED set when packet is complete
                uartState = UART_WAIT_FOR_SOP; // Invalid EOP, reset FSM
            }
            break;

        case UART_DONE:
            // Packet is complete, validate CRC and take appropriate action
//                if (validateCRC(&sensorData)) {
//                    processData(&sensorData); // Process the data
//                }
        	//printData(sensorData); // Process the data
        	//
        	/*uint8_t stupid;
        	for(uint8_t i = 0; i < 17; i++){
        		stupid = sensorData[i];
        		HAL_SPI_Transmit(&hspi5, (uint8_t*)&stupid, 1, 100);

        	}
        	*/
        	for(size_t i = 0; i < sizeof(SensorDataPacket); i++) {
        	    uint8_t* bytePtr = ((uint8_t*)sensorData) + i; // Point to each byte in sequence
        	    HAL_SPI_Transmit(&hspi5, bytePtr, 1, 100); // Transmit one byte at a time
        	}

        	//HAL_SPI_Transmit(&hspi5, (uint8_t*)sensorData, sizeof(*sensorData), 100);
        	//uint8_t test_val[17] = {'Z','Z','Z','Z','Z','Z','Z','Z','Z','Z','Z','Z','Z','Z','Z','Z','Z'};
        	//uint8_t test_val = 'Z';
        	//HAL_SPI_Transmit(&hspi5, (uint8_t*)&test_val, 1, 100);
        	//HAL_SPI_Transmit(&hspi5, (uint8_t*)&test_val, 17, 100);
        	/*if (huart->Instance == USART1) {
        	        PutFifo1(*sensorData);
        	    }
        	else if (huart->Instance == USART2) {
        		 PutFifo2(*sensorData);
        	    }
        	else if (huart->Instance == USART3) {
        		 PutFifo3(*sensorData);
        	    }
        	else if (huart->Instance == USART6) {
        		 PutFifo6(*sensorData);
        	    }
        	    */
        	resetUartState(uartState, timestampBuffer, dataBuffer);
            break;
    }
    // Ready to receive the next byte
   // HAL_UART_Receive_IT(huart, rxData, 1);
}



void CC1200_Reset(void)
{

	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_4, 0);
	HAL_Delay(500);
	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_4, 1);
	HAL_Delay(500);





	tx_data[0]=0x30;
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, 0);
	HAL_Delay(10);
	HAL_SPI_Transmit_IT(&hspi1, tx_data, 1);
	HAL_Delay(10);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, 1);
	HAL_Delay(10);
	HAL_GPIO_WritePin(GPIOE, GPIO_PIN_4, 0);
	HAL_Delay(10);
	HAL_SPI_Transmit_IT(&hspi5, tx_data, 1);
	HAL_Delay(10);
	HAL_GPIO_WritePin(GPIOE, GPIO_PIN_4, 1);
	HAL_Delay(10);
}


void CC1200_Init(registerSetting_t *init_seq)
{
	uint8_t spi_buf[3];
	//int test = sizeof(init_seq);
	for(uint8_t i=0; i < 207 ; i++)
	{
		spi_buf[0] = (uint8_t)((init_seq[i].registerAddress >> 8) & 0xFF); // High byte of the register address
        spi_buf[1] = (uint8_t)(init_seq[i].registerAddress & 0xFF);        // Low byte of the register address
        spi_buf[2] = init_seq[i].registerValue;
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, 0);
		HAL_Delay(10);
		//if(init_seq[i*3])
			HAL_SPI_Transmit_IT(&hspi1, spi_buf, 3);
		//else
		//	HAL_SPI_Transmit_IT(&hspi1, spi_buf, 2);
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, 1);
		HAL_Delay(10);
	}

	for(uint8_t i=0; i < 207; i++)
	{
		spi_buf[0] = (uint8_t)((init_seq[i].registerAddress >> 8) & 0xFF); // High byte of the register address
        spi_buf[1] = (uint8_t)(init_seq[i].registerAddress & 0xFF);        // Low byte of the register address
        spi_buf[2] = init_seq[i].registerValue;
		  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_4, 0);
		  //if(init_seq[i*3])
 			  HAL_SPI_Transmit_IT(&hspi5, spi_buf, 3);
		 // else
		//	  HAL_SPI_Transmit_IT(&hspi5, spi_buf, 2);
		 HAL_GPIO_WritePin(GPIOE, GPIO_PIN_4, 1);
		 HAL_Delay(10);
	}
}

void CC1200_Offset(int16_t offset)
{
	tx_data[0]=0x2F|0x40;
	tx_data[1]=0x0A;
	tx_data[2]=*((uint8_t*)&offset+1);
	tx_data[3]=*((uint8_t*)&offset);

	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, 0);
	HAL_SPI_Transmit_IT(&hspi1, tx_data, 4);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, 1);


	HAL_GPIO_WritePin(GPIOE, GPIO_PIN_4, 0);
	HAL_SPI_Transmit_IT(&hspi5, tx_data, 4);
	HAL_GPIO_WritePin(GPIOE, GPIO_PIN_4, 1);
}

void CC1200_BurstModeIncr(uint8_t enable)
{
	tx_data[0]=0x2F;
	tx_data[1]=0x06;
	tx_data[2]=enable;

	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, 0);
	HAL_SPI_Transmit_IT(&hspi1, tx_data, 3);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, 1);


	HAL_GPIO_WritePin(GPIOE, GPIO_PIN_4, 0);
	HAL_SPI_Transmit_IT(&hspi5, tx_data, 3);
	HAL_GPIO_WritePin(GPIOE, GPIO_PIN_4, 1);
}

void CC1200_RXMode(void)
{
	tx_data[0]=0x34;
	HAL_GPIO_WritePin(GPIOE, GPIO_PIN_4, 0);
	HAL_Delay(10);
	HAL_SPI_Transmit(&hspi5, tx_data, 1, 100);
	HAL_Delay(10);
	HAL_GPIO_WritePin(GPIOE, GPIO_PIN_4, 1);
	HAL_Delay(10);
}

void CC1200_TXMode(void)
{
	tx_data[0]=0x35;
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, 0);
	HAL_Delay(10);
	HAL_SPI_Transmit(&hspi1, tx_data, 1, 100);
	HAL_Delay(10);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, 1);
	HAL_Delay(10);
}

void CC1200_RXStart(void)
{
	tx_data[0]=0x2F|0xC0;
	tx_data[1]=0x7D;
	tx_data[2]=0;

	HAL_GPIO_WritePin(GPIOE, GPIO_PIN_4, 0);
	HAL_SPI_TransmitReceive(&hspi5, tx_data, rx_data, 3, 10);
}
void CC1200_TXStart(void)
{
	tx_data[0]=0x2F|0x40;
	tx_data[1]=0x7E;
	tx_data[2]=0;

	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, 0);
	HAL_SPI_TransmitReceive(&hspi1, tx_data, rx_data, 3, 10);
}

void CC1200_TXRXEnd(void)
{
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, 1);
}

//set frequency - burst mode
void CC1200_SetFreq(uint32_t freq)
{
	uint32_t val=(float)freq/5000000*(1<<16);

	tx_data[0]=0x2F|0x40;
	tx_data[1]=0x0C;
	tx_data[2]=(val>>16)&0xFF;
	tx_data[3]=(val>>8)&0xFF;
	tx_data[4]=val&0xFF;

	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, 0);
	HAL_SPI_Transmit(&hspi1, tx_data, 5, 10);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, 1);

	HAL_GPIO_WritePin(GPIOE, GPIO_PIN_4, 0);
	HAL_SPI_Transmit(&hspi5, tx_data, 5, 10);
	HAL_GPIO_WritePin(GPIOE, GPIO_PIN_4, 1);
}

//set power (0x01..0x3F)
void CC1200_SetPwr(uint8_t pwr)
{
	tx_data[0]=0x2B;
	tx_data[1]=pwr;

	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, 0);
	HAL_SPI_Transmit(&hspi1, tx_data, 2, 10);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, 1);

	HAL_GPIO_WritePin(GPIOE, GPIO_PIN_4, 0);
	HAL_SPI_Transmit(&hspi5, tx_data, 2, 10);
	HAL_GPIO_WritePin(GPIOE, GPIO_PIN_4, 1);
}


void resetUartState(UART_State_t *uartState, uint32_t *timestampBuffer, uint64_t *dataBuffer) {
    *uartState = UART_WAIT_FOR_SOP; // Reset UART state
    *timestampBuffer = 0; // Clear the timestamp buffer
    *dataBuffer = 0; // Clear the data buffer
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_13, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_15, GPIO_PIN_RESET);
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
