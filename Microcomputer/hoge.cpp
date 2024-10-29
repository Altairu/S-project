#include "mbed.h"

// シリアル通信およびCAN通信の設定
BufferedSerial pc_serial(USBTX, USBRX, 115200); // Pythonと通信するシリアルポート
CAN can(PA_11, PA_12, 1000000);                 // CAN通信ピン設定、通信速度1Mbps

// CAN ID
const uint32_t ID_TARGET_POS = 0x200; // X_mm, Y_mm, Z_mmのデータ送信用

// ヘッダー・フッターと受信バッファ
const char HEADER[2] = {0xA5, 0xA5};
const char FOOTER[2] = {0x5A, 0x5A};
int16_t X_mm = 0; // 目標位置X
int16_t Y_mm = 0; // 目標位置Y
int16_t Z_mm = 0; // 目標位置Z

// シリアルデータを受信する関数
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

// 位置データをCANで送信する関数
void send_position_data()
{
    uint8_t target_pos_data[6] = {
        (X_mm >> 8) & 0xFF, X_mm & 0xFF,
        (Y_mm >> 8) & 0xFF, Y_mm & 0xFF,
        (Z_mm >> 8) & 0xFF, Z_mm & 0xFF};
    CANMessage msg_target(ID_TARGET_POS, target_pos_data, 6);
    can.write(msg_target);
}

int main()
{
    while (true)
    {
        if (receive_serial_data()) // シリアルデータ受信
        {
            send_position_data(); // CANでMDDに位置データ送信
        }
        ThisThread::sleep_for(100ms); // 少し待機して安定性を保つ
    }
}
