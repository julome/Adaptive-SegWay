/********************************************************
 			Library for manage MPU6050		
    Copyright (C) 2014  Juan Lopez Medina

    This program is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 2 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License along
    with this program; if not, write to the Free Software Foundation, Inc.,
    51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.

* You can contact me by mail : julome21@gmail.com
*********************************************************/

#include "MPU6050.h"
#include "twi_master.h"

// Write byte to IMU through TWI
void TWI_Write(unsigned char reg, unsigned char data){  // reg= Direction de registro, data= data to write

unsigned char messageBuf[8] = {0};					// Buffer for TX through TWI

messageBuf[0] = MPU6050_DEFAULT_ADDRESS;			// TWI slave address (IMU) + Write.
messageBuf[1] = reg;								// Registry Address to write.
messageBuf[2] = data;								// Data to Write to IMU.
TWI_Start_Transceiver_With_Data( messageBuf, 3 );	// TX Reg+Data to Write IMU
return;
}

// Read measurements of IMU Through TWI
void TWI_Read(unsigned char reg, int *result, unsigned char size){
	
	unsigned char messageBuf[8] = {0};					// Buffer for TX through TWI
	
	messageBuf[0] = MPU6050_DEFAULT_ADDRESS;			// TWI slave address (IMU) + Write.
	messageBuf[1] = reg;								// Registry Address to write.
	TWI_Start_Transceiver_With_Data(messageBuf, 2);		// TX Reg to Write IMU
	while (TWI_Transceiver_Busy());						// Wait until TWI is ready for next transmission.
	if (TWI_statusReg.lastTransOK){						// Check if the last operation was successful
		// Request/collect the data from the Slave
		messageBuf[0] = (MPU6050_DEFAULT_ADDRESS | 0x01);			// TWI slave address (IMU) + Read.
		TWI_Start_Transceiver_With_Data(messageBuf, size + 1);
	} else return;													// Out of function
	while (TWI_Transceiver_Busy());									// Wait until TWI is ready for next transmission.
	if (TWI_statusReg.lastTransOK){									// Check if the last operation was successful
		TWI_Get_Data_From_Transceiver(messageBuf, size + 1);
		if (size > 1){		
			for (int i = 0; i < (size / 2); i++)						// Get reads 16 bit on array result
			{
				result[i] = (((int)messageBuf[i * 2]) << 8) | (int)messageBuf[(i * 2) + 1];
			}
		} else result[0] = messageBuf[0];
	} else return; 										// Out of function
}

// Write bit to IMU
void TWI_WriteBit(unsigned char reg, unsigned char bit_num, unsigned char data ){
	int read_aux[1];	
	unsigned char data_add = 0;
	
	TWI_Read(reg, read_aux, 1);							// Read byte of a registry of IMU
	if (data == True) data_add |= (1 << bit_num);		// Select false or true for bit to write
		else data_add &= ~(1 << bit_num);				
	read_aux[0] |= data_add;							// Compound new data with bit for write	
	TWI_Write(reg, read_aux[0]);						// Write byte actualized	
}

// Initialize IMU MPU-6050
/** Power on and prepare for general usage.
 * This will activate the device and take it out of sleep mode (which must be done
 * after start-up). This function also sets both the accelerometer and the gyroscope
 * to their scale +/- xg and +/- x degrees/sec, and sets the clock source to use the
 * X Gyro for reference, which is slightly better than the default internal clock source.
 */

void imu_init(){
	TWI_Write(MPU6050_RA_PWR_MGMT_1, MPU6050_CLOCK_PLL_XGYRO);				// Clock selection PLL X_GYRO temp disables
	TWI_Write(MPU6050_RA_GYRO_CONFIG, MPU6050_GYRO_FS_500);					// Gyro Scale +/- 500 º/s
	TWI_Write(MPU6050_RA_ACCEL_CONFIG, MPU6050_ACCEL_FS_16);				// Accel Scale at +/- 16 G
	TWI_Write(MPU6050_RA_CONFIG, MPU6050_DLPF_BW_256);						// Config DLPF Low Pass Filter IMU Off
	TWI_Write(MPU6050_RA_SMPLRT_DIV, MPU6050_CLOCK_DIV_308);				// Sample rate divider at 2 KHz if DLPF OFF
	TWI_WriteBit(MPU6050_RA_PWR_MGMT_1, MPU6050_PWR1_TEMP_DIS_BIT, True);	// Disable sleep mode for wake up IMU
	TWI_WriteBit(MPU6050_RA_PWR_MGMT_1, MPU6050_PWR1_SLEEP_BIT, False);		// Disable sleep mode for wake up IMU		
}
