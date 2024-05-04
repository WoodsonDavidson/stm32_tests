/*
 * HTU21D.h
 *
 *  Created on: 12 мар. 2024 г.
 *      Author: Dmitrii
 */

#ifndef HTU21D_H_
#define HTU21D_H_



#endif /* HTU21D_H_ */


#define HTU21D_IIC_ADDR         0x40
#define HTU21D_RESET_CMD        0xFE
#define HTU21D_CFG_READ_CMD     0xE7
#define HTU21D_CFG_WRITE_CMD    0xE6
#define HTU21D_TRIG_T_CMD       0xF3
#define HTU21D_TRIG_H_CMD       0xF5
#define HTU21D_TRIG_T_HOLD_CMD  0xE3	//
#define HTU21D_TRIG_H_HOLD_CMD  0xE5

#define HTU21D_RES_HIGH     0b00000000 // RH = 12Bit, T = 14Bit
#define HTU21D_RES_MEDIUM   0b10000001 // RH = 11Bit, T = 11Bit
#define HTU21D_RES_LOW      0b00000001 // RH = 8Bit, T = 12Bit
