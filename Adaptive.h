/*********************************************************************
			Library Adaptive predictive control
			Copyright (C) 2014  Juan Lopez Medina

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

#ifndef ADAPTIVE_H_
#define ADAPTIVE_H_

#define Ref_Meters		180.0		// Reference for meters in adaptive predictive control
#define NL				0.005		// Noise Level for Adaptive Mechanism.
#define GainA 			0.6			// Gain for Adaptive Mechanism A
#define GainB 			0.6			// Gain for Adaptive Mechanism B
#define PmA				2			// Delay Parameters a
#define PmB				2			// Delay Parameters b
#define n				9.0			// Conductor block periods control for rise to set point ts = n * CP
#define hz				5			// Prediction Horizon (Horizon max = n + 2)
#define UP_Roll			800.0		// Upper limit out
#define UP_Yaw			150.0		// Upper limit out
#define GainT_Roll		12.0		// Total Gain Roll Out Controller
#define GainT_Yaw		5.0			// Total Gain Yaw Out Controller
#define MaxOut_Roll		UP_Roll/GainT_Roll
#define MaxOut_Yaw		UP_Yaw/GainT_Yaw

void conductor_block(void);
void adaptive(double *sp, double *t, double *y, double *u, double *yp, double max_out);

#endif /* ADAPTIVE_H_ */
