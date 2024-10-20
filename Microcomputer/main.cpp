#include "stm32f4xx.h"
#include "stm32f4xx_nucleo.h"
#include "sken_library/include.h"
#include "asimawari_library/asimawari.h"
			
Asimawari asimawari;
DebugData debugdata;
Motor motor;

double motor_target[4],motor_out[4],vx,vy=100,omega,robot_diameter = 362/2,wheel_radius = 72;

int main(void)
{
	sken_system.init();
	/*
	encoder
		A0 A1 TIMER5
		B3 A5 TIMER2
		B6 B7 TIMER4
		C6 C7 TIMER3
	MD
		B14(TIMER8 CH2) B15(TIMER8 CH3)
		A8(TIMER1 CH1) A11(TIMER1 CH2)
		A6(TIMER13 CH1) A7(TIMER14 CH1)
		B8(TIMER10 CH1) B9(TIMER11 CH1)
	  */

	asimawari.mtr_pin_init(FR_Dokusute,Bpin,B8,TIMER10,CH1);
	asimawari.mtr_pin_init(FR_Dokusute,Apin,B9,TIMER11,CH1);
	asimawari.mtr_pin_init(FL_Dokusute,Bpin,A6,TIMER13,CH1);
	asimawari.mtr_pin_init(FL_Dokusute,Apin,A7,TIMER14,CH1);
	asimawari.mtr_pin_init(BR_Dokusute,Bpin,A8,TIMER1,CH1);
	asimawari.mtr_pin_init(BR_Dokusute,Apin,A11,TIMER1,CH4);
	asimawari.mtr_pin_init(BL_Dokusute,Bpin,B14,TIMER12,CH1);
	asimawari.mtr_pin_init(BL_Dokusute,Apin,B15,TIMER12,CH2);

	asimawari.enc_pin_init(FR_Dokusute,A0,A1,TIMER5,wheel_radius);
	asimawari.enc_pin_init(FL_Dokusute,B3,A5,TIMER2,wheel_radius);
	asimawari.enc_pin_init(BR_Dokusute,B6,B7,TIMER4,wheel_radius);
	asimawari.enc_pin_init(BL_Dokusute,C6,C7,TIMER3,wheel_radius);

	asimawari.pid_set(FR_Dokusute,1.5,0.01,0.008);
	asimawari.pid_set(FL_Dokusute,1.5,0.01,0.008);
	asimawari.pid_set(BR_Dokusute,1.5,0.01,0.008);
	asimawari.pid_set(BL_Dokusute,1.5,0.01,0.008);

	while(true){
		debugdata = asimawari.get_debug_data();
		asimawari.turn(dokusute,vx,vy,omega,robot_diameter,wheel_radius);
	}
}
