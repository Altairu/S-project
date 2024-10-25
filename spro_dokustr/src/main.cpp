/**
  ******************************************************************************
  * @file    main.c
  * @author  Ac6
  * @version V1.0
  * @date    01-December-2013
  * @brief   Default main function.
  ******************************************************************************
*/

#include "stm32f4xx.h"
#include "stm32f4xx_nucleo.h"
#include "sken_library/include.h"
#include "math.h"

Gpio led;

Uart serial;
uint8_t esp_data;

Can can;
uint8_t data[8];
int16_t vol_data[3];

Motor mtr;

//////////変更パラメータ//////////
float Vmax = 30;			//平行移動の最高速度，斜めに動くとルート2倍
float Omega_max = 4;		//角速度の最高速度
float V_resolution = 25;	//台形調整時の１秒間の変動量（Vmax/V_resolution）秒で最高速度になる
//float V_resolution2 = 20;	//台形調整時の１秒間の変動量（Vmax/V_resolution）秒で最高速度になる
float Omega_resolution = 3;	//台形調整時の１秒間の変動量（Omega‗max/Omega_resolution）秒で最高角速度になる
//////////////////////////////

float Vx,Vy,Omega;		//各速度の目標速度，台形調整前
float vx,vy,omega;		//各速度の速度，台形調整後

void make_velo(){
	Vx = (esp_data&0x04)? Vmax : (esp_data&0x01)? -1*Vmax :0;
	Vy = (esp_data&0x02)? Vmax : (esp_data&0x08)? -1*Vmax :0;
	Omega = (esp_data&0x40)? Omega_max : (esp_data&0x10)? -1*Omega_max :0;
	if(Omega!=0){
		Vx=0;
		Vy=0;
	}

	vx += (vx<Vx)? V_resolution/1000.0 : 0 ;
	vx -= (vx>Vx)? V_resolution/1000.0 : 0 ;
	if(abs(vx)<1 && Vx==0) vx=0;

	vy += (vy<Vy)? V_resolution/1000.0 : 0 ;
	vy -= (vy>Vy)? V_resolution/1000.0 : 0 ;
	if(abs(vy)<1 && Vy==0) vx=0;

	omega += (omega<Omega)? Omega_resolution/1000.0 : 0 ;
	omega -= (omega>Omega)? Omega_resolution/1000.0 : 0 ;
	if(abs(omega)<1 && Omega==0) omega=0;
}

void send_can(){
	// データをバイト配列に格納（上位バイトを先に）
	vol_data[0] = vx*100;
	vol_data[1] = vy*100;
	vol_data[2] = omega*100;

	data[0] = (vol_data[0] >> 8) & 0xFF;    // Vxの上位バイト
	data[1] = vol_data[0] & 0xFF;           // Vxの下位バイト
	data[2] = (vol_data[1] >> 8) & 0xFF;    // Vyの上位バイト
	data[3] = vol_data[1] & 0xFF;           // Vyの下位バイト
	data[4] = (vol_data[2] >> 8) & 0xFF; // omegaの上位バイト
	data[5] = vol_data[2] & 0xFF;        // omegaの下位バイト
}

void interrupt(){
	make_velo();
	send_can();
	if(esp_data&0x20)mtr.write(20);
}

void read_esp(){
	esp_data = serial.read(20);
}

int main(void){
	sken_system.init();

	serial.init(C10,C11,SERIAL3,115200);
	sken_system.startCanCommunicate(A12,A11,CAN_1);

	mtr.init(Apin,B9 ,TIMER11,CH1);
	mtr.init(Bpin,B8 ,TIMER10,CH1);

	sken_system.addTimerInterruptFunc(interrupt,0,1);
	sken_system.addTimerInterruptFunc(read_esp,1,100);

	while(true){
		sken_system.canTransmit(CAN_1,0x200,data,6,10);
	}
}
