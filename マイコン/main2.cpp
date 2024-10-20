#include "stm32f4xx.h"
#include "stm32f4xx_nucleo.h"
#include "sken_library/include.h"
			

// モーターとエンコーダーのインスタンス
Motor motor1, motor2, motor3, motor4;
Encoder encoder1, encoder2, encoder3, encoder4;
Encoder_data e_data1, e_data2, e_data3, e_data4;
Pid pid1, pid2, pid3, pid4;
Gpio botan;

double control_value1;
double control_value2;
double control_value3;
double control_value4;

// ターゲット角度（90度）
double target_angle = 0;

void encoder1_interrupt() {
    encoder1.interrupt(&e_data1); // encoder1のデータを更新
}

void encoder2_interrupt() {
    encoder2.interrupt(&e_data2); // encoder2のデータを更新
}

void encoder3_interrupt() {
    encoder3.interrupt(&e_data3); // encoder3のデータを更新
}

void encoder4_interrupt() {
    encoder4.interrupt(&e_data4); // encoder4のデータを更新
}

void setup() {
    sken_system.init();

    // モーターの初期化
    motor1.init(Bpin, B8, TIMER10, CH1); // ApinとB8でモーター1を初期化
    motor1.init(Apin, B9, TIMER11, CH1);

    motor2.init(Bpin, A6, TIMER13, CH1); // ApinとA6でモーター2を初期化
    motor2.init(Apin, A7, TIMER14, CH1);

    motor3.init(Bpin, A8, TIMER1, CH1);  // ApinとA8でモーター3を初期化
    motor3.init(Apin, A11, TIMER1, CH4); // CH3からCH4に修正

    motor4.init(Bpin, B14, TIMER12, CH1); // ApinとB14でモーター4を初期化
    motor4.init(Apin, B15, TIMER12, CH2); // 問題なし

    // エンコーダーの初期化
    encoder1.init(A0, A1, TIMER5, 100, 8192);  // エンコーダー1を初期化
    encoder2.init(B3, A5, TIMER2, 100, 8192);  // エンコーダー2を初期化
    encoder3.init(B6, B7, TIMER4, 100, 8192);  // エンコーダー3を初期化
    encoder4.init(C6, C7, TIMER3, 100, 8192);  // エンコーダー4を初期化

    // 割り込み関数をタイマーに登録
    sken_system.addTimerInterruptFunc(encoder1_interrupt, 0, 1); // encoder1の割り込み
    sken_system.addTimerInterruptFunc(encoder2_interrupt, 1, 1); // encoder2の割り込み
    sken_system.addTimerInterruptFunc(encoder3_interrupt, 2, 1); // encoder3の割り込み
    sken_system.addTimerInterruptFunc(encoder4_interrupt, 3, 1); // encoder4の割り込み

    // PIDのゲイン設定
    pid1.setGain(1.0, 0.05, 0.05);  // PID1のゲイン設定
    pid2.setGain(1.0, 0.05, 0.05);  // PID2のゲイン設定
    pid3.setGain(1.0, 0.05, 0.05);  // PID3のゲイン設定
    pid4.setGain(1.0, 0.05, 0.05);  // PID4のゲイン設定
}

void loop() {
	if(!botan.read()){
			target_angle=-90*3.25;
	}else{
			target_angle =0;
	}
    // エンコーダーから取得した角度を使用
    double current_angle1 = e_data1.deg;
    double current_angle2 = e_data2.deg;
    double current_angle3 = e_data3.deg;
    double current_angle4 = e_data4.deg;

    // 各モーターの制御
     control_value1 = pid1.control(target_angle, current_angle1, 1); // 1ms周期
     control_value2 = pid2.control(target_angle, current_angle2, 1);
     control_value3 = pid3.control(target_angle, current_angle3, 1);
     control_value4 = pid4.control(target_angle, current_angle4, 1);

    // モーターの回転速度設定
    motor1.write((int)control_value1);
    motor2.write((int)control_value2);
    motor3.write((int)control_value3);
    motor4.write((int)control_value4);

}


int main(void) {
    HAL_Init();
    botan.init(C13,INPUT_PULLUP);
    setup();

    while (1) {
        loop();
    }
}
