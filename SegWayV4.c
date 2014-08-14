/*********************************************************************

	Robot balancer with Adaptive predictive control instead PID
	Created: 21/01/2014
	
	Improve - DCM IMU instead Complementary Filter
			- Period samples without interrupts
	
			Copyright (C) 2013  Juan Lopez Medina

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.

	You can contact me in julome21@gmail.com

    Robot balancer Adaptive  Copyright (C) 2013  Juan Lopez Medina
	
    This program comes with ABSOLUTELY NO WARRANTY; for details type `show w'.
    This is free software, and you are welcome to redistribute it
    under certain conditions; type `show c' for details.

 ****************************************************************************/  

#define F_CPU 16000000UL

#include <avr/interrupt.h>
#include <avr/io.h>
#include "usart.h"
#include "twi_master.h"
#include "MPU6050.h"
#include "DCM.h"
#include "Adaptive.h"

// Configuration
#define T_SAMPLE		8					// Time sample Calculate attitude T = T_SAMPLE * 1ms (max 16 ms). 
#define T_CONTROL		3					// CP = T_SAMPLE * T_CONTROL in ms
#define T_CNT			T_SAMPLE * 1000/64	// Number count temp2.  1 Count Temp1 64us => T_sample = Value_CNT1 = ms/64us 

// Out PWMs
#define RM			PINB1					// Out PWM control Right Motor Timer1 OC1A DPIN 9 Arduino Pro Mini (Atmega328p)
#define LM			PINB2					// Out PWM control Left Motor Timer1 OC1B DPIN 10 Arduino Pro Mini (Atmega328p)

// Variables globales
unsigned char t_sample;						// Read CNT2 for sample timer IMU / Increment time between samples
unsigned char t_control;					// Count for Period control

// Timer count for samples. 1 count = 64us. Max = 256 * 64us = 16ms
void timer2_init(){	
	TCCR2A = 0;												// Normal Mode
	TCCR2B = (1 << CS22) | (1 << CS20) | (1 << CS21);		// Prescaler = 1024 Normal Mode
}

// Initialize Timer1 PWM Fast Mode for control Right motors ESC
void timer1_init(){
	//Fast PWM Mode Fpwm = F_cpu / (N * (End + 1))
	TCCR1B = (2 << CS10);			// Configuration prescaler to 1/8 N = 8
	ICR1 = 24999;					// End count for Fpwm = 50Hz (20ms) => 39999 for Fpwm = 80Hz (12.5ms) => 24999
	TCCR1A = (2 << COM1A0);			// No inverter mode PIN OC1A
	TCCR1A |= (2 << COM1B0);		// No inverter Mode PIN OC1B
	// Enable FAST PWM Mode
	TCCR1A |= (1 << WGM11);
	TCCR1B |= (1 << WGM12) | (1 << WGM13);
	OCR1A = 3000;					// Initialize comparison registry Timer1 PIN A for PWM servo 2000 = 1ms 4000 = 2ms
	OCR1B = 3000;					// Initialize comparison registry Timer1 PIN B for PWM servo 2000 = 1ms 4000 = 2ms		
}

int main(void){
    // Initialize
    volatile int right_motor = 3000;				// Power for motor right
    volatile int left_motor = 3000;					// Power for motor left
    int out_balancer = 0;							// Out regulator AP control Motor balancer
    int out_yaw = 0;
	
	double yr[PmA+2] = {0};								// Array out incremental y(k), y(k-1), y(k-2), y(k-3)
	double ur[PmB+3] = {0};								// Array process input incremental u(k), u(k-1), u(k-2), u(k-3), u(k-4)
	double tr[5] = {0.50, 0.49, 0.032, 0.04, 0.034};	// Array parameters adaptive mechanism a1k, a2k, b1k, b2k, b3k	
	//double tr[5] = {0, 0, 0, 0, 0};
	double spr[2] = {0};								// Set point process sp(k)
	double yrp[2] = {0};								// Array process out yp(k)				
	
	double yy[PmA+2] = {0};								// Array out incremental y(k), y(k-1), y(k-2), y(k-3)
	double uy[PmB+3] = {0};								// Array process input incremental u(k), u(k-1), u(k-2), u(k-3), u(k-4)
	double ty[5] = {0.5, 0.5, 0.035, 0.042, 0.039};		// Array parameters adaptive mechanism a1k, a2k, b1k, b2k, b3k	
	//double ty[5] = {0, 0, 0, 0, 0};
	double spy[2] = {0};								// Set point process sp(k)
	double yyp[2] = {0};								// Array process out yp(k)					
	
	DDRB = (1 << RM) | (1 << LM) | (1 << LED_READY);	// Config output/input pins out motors	
	
	//Initialization
	cli();													// Disable all interrupts				
	conductor_block();										// Calculate parameters conductor block
	//usart_init();											// Initialize serial port
	TWI_Master_Initialise();								// Initialize TWI Port	
	timer2_init();											// Initialize timer2 for time system 64us				
	sei();													// Enable global interrupts	
	imu_init();												// Initialize IMU MPU-6050
	gyro_offset();											// Offset gyro	
	timer1_init();											// Initialize timer 1 for PWM OC1A Right-Motor, OC1B Left-Motor
	GTCCR |= (1 << TSM) | (1 << PSRASY) | (1 << PSRSYNC);	// Stop and reset Timers
	GTCCR = 0;												// Start Timers synchronized
		
	while(1){		
		t_sample = TCNT2;									// Catch sample time for integer angle gyro	
		if (t_sample >= T_CNT){								// Attitude calculates Read IMU (Accel and Gyro)			  	
			TCNT2 = 0;										// Restart sample time					
			t_control++;									// Increment time Period control
			sample_meters();								// Samples read IMU
			matrix_update(t_sample);
			normalize();
			drift_correction();
			//euler_angles();			
		}		
		if (t_control >= T_CONTROL){						//Control action balancer										
			t_control = 0;			
			euler_angles();
			
			//Adaptive predictive balancer process			
			spr[0] = 0;														// Set point									
			yrp[0] = a_result[1];											// Process out y(k). 											
			adaptive(spr, tr, yr, ur, yrp, MaxOut_Roll);					// Call adaptive function												
			out_balancer = ur[0] * GainT_Roll;								// Out Controller adaptive						
			if (out_balancer > UP_Roll) out_balancer = UP_Roll;				// Upper limit out
				else if (out_balancer < -UP_Roll) out_balancer = -UP_Roll;													
			//out_balancer = 0;												// Off control
			
			//Adaptive predictive Yaw process									
			spy[0] = 0;														// Set point
			yyp[0] = a_result[2];											// Process out y(k).					
			adaptive(spy, ty, yy, uy, yyp, MaxOut_Yaw);						// Call adaptive function
			out_yaw = uy[0] * GainT_Yaw;									// Out Controller adaptive						
			if (out_yaw > UP_Yaw) out_yaw = UP_Yaw;							// Upper limit out
				else if (out_yaw < -UP_Yaw) out_yaw = -UP_Yaw;						
			//out_yaw = 0;													// Off control
									
			// Assign control to motors
			right_motor = 3000 + out_balancer + out_yaw;	
			left_motor = 3000 - out_balancer + out_yaw;
					
			//Limits PWM motors
			if (right_motor > 4000) right_motor = 4000;
				else if (right_motor < 2000) right_motor = 2000;										
						
			//Assignments PWMs
			OCR1A =  right_motor;
			OCR1B =  left_motor;			
						
			//put_float(tr[0]);
			//put_string(" ");
			//put_float(tr[1]);
			//put_string(" ");
			//put_float(tr[2]);
			//put_string(" ");
			//put_int(ch[0]);
			//put_string(" ");
			//put_int(vch1);							
			//put_string("\n");																	
		}			       	    
    }
}
