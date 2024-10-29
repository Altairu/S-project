#include "mbed.h"

// シリアル通信用とCAN通信の設定
BufferedSerial pc_serial(USBTX, USBRX, 115200); // PCと通信するシリアルポート
CAN can(PA_11, PA_12, 1000000);                 // CAN通信ピン設定、通信速度1Mbps

// CAN ID
const uint32_t ID_SWITCH = 0x100;     // スイッチのデータ送信用
const uint32_t ID_TARGET_POS = 0x200; // X_mm, Y_mm, Z_mmのデータ送信用
const uint32_t ID_SELF_POS = 0x300;   // 自己位置データの受信用

// ヘッダーとデータ
const char HEADER[2] = {0xA5, 0xA5};
const char FOOTER[2] = {0x5A, 0x5A};
int16_t X_mm = 0;                    // 目標位置X
int16_t Y_mm = 0;                    // 目標位置Y
int16_t Z_mm = 0;                    // 目標位置Z
uint8_t switch_data[8] = {0};        // スイッチのON/OFF状態（8スイッチ分）
uint8_t target_pos_data[8] = {0};    // 目標位置データバッファ
uint8_t self_position_data[8] = {0}; // 自己位置データバッファ (x, y, z)

DigitalOut led(LED1);

// スイッチのピン設定
DigitalIn switch1(PB_14);
DigitalIn switch2(PB_15);
DigitalIn switch3(PA_8);
DigitalIn switch4(PA_11);
DigitalIn switch5(PA_6);
DigitalIn switch6(PA_7);
DigitalIn switch7(PB_9);
DigitalIn switch8(PB_8);

// シリアル通信でPCからデータを受け取る関数
bool receive_serial_data()
{
    char header[2] = {0, 0};
    char serial_data[8];

    // ヘッダーが見つかるまで待機
    while (true)
    {
        char byte_in;
        if (pc_serial.read(&byte_in, 1))
        {
            header[0] = header[1];
            header[1] = byte_in;
            if (header[0] == HEADER[0] && header[1] == HEADER[1])
            {
                break; // ヘッダーが見つかった
            }
        }
    }

    // データを読み取り
    if (pc_serial.read(serial_data, 8) == 8)
    { // 8バイトのデータ
        X_mm = (serial_data[0] << 8) | serial_data[1];
        Y_mm = (serial_data[2] << 8) | serial_data[3];
        Z_mm = (serial_data[4] << 8) | serial_data[5];
        return true; // データ受信成功
    }
    else
    {
        return false; // データ受信失敗
    }
}

// スイッチの状態を読み取る関数
void read_switch_states()
{
    switch_data[0] = switch1.read();
    switch_data[1] = switch2.read();
    switch_data[2] = switch3.read();
    switch_data[3] = switch4.read();
    switch_data[4] = switch5.read();
    switch_data[5] = switch6.read();
    switch_data[6] = switch7.read();
    switch_data[7] = switch8.read();
}

// 自己位置データをPCに送信する関数
void send_data_to_pc()
{
    int16_t x_pos = (self_position_data[0] << 8) | self_position_data[1];
    int16_t y_pos = (self_position_data[2] << 8) | self_position_data[3];
    int16_t z_pos = (self_position_data[4] << 8) | self_position_data[5];

    char send_data[10];
    send_data[0] = HEADER[0];
    send_data[1] = HEADER[1];
    send_data[2] = (x_pos >> 8) & 0xFF;
    send_data[3] = x_pos & 0xFF;
    send_data[4] = (y_pos >> 8) & 0xFF;
    send_data[5] = y_pos & 0xFF;
    send_data[6] = (z_pos >> 8) & 0xFF;
    send_data[7] = z_pos & 0xFF;

    pc_serial.write(send_data, sizeof(send_data));
}

// CANメッセージの受信と処理
void check_can_message()
{
    CANMessage msg;
    if (can.read(msg) && msg.id == ID_SELF_POS)
    {
        for (int i = 0; i < msg.len; i++)
        {
            self_position_data[i] = msg.data[i];
        }
        send_data_to_pc(); // PCに自己位置データを送信
    }
}

// メインループ
int main()
{
    while (true)
    {
        // 現在の時刻を取得
        auto now = HighResClock::now();
        static auto pre = now;

        // 1msごとのタイミングで送信
        if (now - pre > 1ms)
        {
            // スイッチの状態を取得し、CANで送信
            read_switch_states();
            CANMessage msg_switch(ID_SWITCH, switch_data, sizeof(switch_data));
            can.write(msg_switch);

            // PCから目標位置データを受信し、CANで送信
            bool data_received = receive_serial_data();
            if (data_received)
            {
                led = 0;
                target_pos_data[0] = (X_mm >> 8) & 0xFF;
                target_pos_data[1] = X_mm & 0xFF;
                target_pos_data[2] = (Y_mm >> 8) & 0xFF;
                target_pos_data[3] = Y_mm & 0xFF;
                target_pos_data[4] = (Z_mm >> 8) & 0xFF;
                target_pos_data[5] = Z_mm & 0xFF;

                CANMessage msg_target(ID_TARGET_POS, target_pos_data, sizeof(target_pos_data));
                can.write(msg_target);
            }
            else
            {
                led = 1;
            }

            // MDDから自己位置データの受信を確認
            check_can_message();

            // 前回の実行時間を更新
            pre = now;
        }
        ThisThread::sleep_for(1ms); // 1msごとのループ
    }
}
