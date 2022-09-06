#include "Wire.h"
// #include "I2Cdev.h"
// #include "MPU6050.h"
#include "Kalman.h"
#include "Kalman_Vel.h"
#include "Kalman_Pos.h"
#define SerialPort SerialUSB

#define DebugPrint_main
// #define DebugPrint_MPU6050
// #define DebugPrint_VL53L4CX
// #define DebugPrint_DS3502

#define NeoPixcel_ENABLE
struct control_data_str{
    float   yaw_raw;
    float   pitch_raw;
    float   roll_raw;
    float   yaw_data;
    float   pitch_data;
    float   roll_data;
    float   accel_x;
    float   accel_y;
    float   accel_z;
    float   accel_x_temp;
    float   accel_y_temp;
    uint32_t object_no;
    float   distance_raw;
    float   distance_data;
    float   VL53L4CX_Signallebel;
    uint8_t VL53L4CX_RangeStatus;
    uint8_t Lch_wiper_position;
    uint8_t Rch_wiper_position;
}control_data_str;

const float ROOM_H = 3.0;
const float ROOM_L = 3.0;
const float RESET_DISTANCE = 10;

double x = 3.0;
double y = 0;
uint32_t timer;
double diff_x, diff_y;
int flag_x, flag_y = 0;
float walk_x = 0.5;
float walk_y = 0.9;

// ================================================================
// ===                      INITIAL SETUP                       ===
// ================================================================

void setup() {
    // join I2C bus (I2Cdev library doesn't do this automatically)  
    Wire.begin();
    Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties

    // initialize serial communication
    SerialPort.begin(115200);

    Init_VL53L4CX();
    Init_MPU6050();
    Init_NeoPixcel();
    Init_DS3502(0);
    Init_DS3502(1);
    control_data_str.accel_x_temp = 0;
    control_data_str.accel_y_temp = 0;  
}

// ================================================================
// ===                    MAIN PROGRAM LOOP                     ===
// ================================================================

void loop() {
    double dt = (double)(micros() - timer) / 1000000; // Calculate delta time
    timer = micros();
    VL53L4CX_task();
    //SerialUSB.println("");
    MPU6050_task();
    ds3502_task();
    NeoPixcel_task();
    diff_x = control_data_str.accel_x - control_data_str.accel_x_temp;
    diff_y = control_data_str.accel_y - control_data_str.accel_y_temp;
    // if (flag_x == 0){
            
    // }else if(flag_x == 1){

    // }else if(flag_x == -1){

    // }

    
    // if (diff_x > walk_x){
    //     if(flag_x != -1){
    //     flag_x = 1;
    //     }else{
    //         flag_x = 0;
    //     }
    // }else if (diff_x < -walk_x){
    //     if(flag_x != 1){
    //         flag_x = -1;
    //     }else{
    //         flag_x = 0;
    //     }
        
    // }

    //if (fabs(control_data_str.yaw_data) < 10 && fabs(control_data_str.pitch_data) < 10 && fabs(control_data_str.roll_data) < 10){
    x = (double)control_data_str.distance_raw / 1000;
    //}
    
    if (diff_y > walk_y){
        if(flag_y != -1){
        flag_y = 1;
        }else{
            flag_y = 0;
        }
    }else if (diff_y < -walk_y){
        if(flag_y != 1){
            flag_y = -1;
        }else{
            flag_y = 0;
        }
        
    }
    
    // x += flag_x * 0.9 * dt;
    y += flag_y * 0.9 * dt;
    if(x > (double)ROOM_L) x = (double)ROOM_L;
    if(x < 0) x = 0;
    if(y > (double)ROOM_H) y = (double)ROOM_H;
    if(y < 0) y = 0;
    setGain((float)x, (float)y);
    control_data_str.accel_x_temp = control_data_str.accel_x;
    control_data_str.accel_y_temp = control_data_str.accel_y;

    if (control_data_str.distance_raw < RESET_DISTANCE){
        x = 3;
        y = 0;
        flag_x = 0;
        flag_y = 0;
    } 


    #ifdef DebugPrint_main
        SerialPort.print(" acc ");
        SerialPort.print(control_data_str.accel_x); SerialPort.print(" : ");
        SerialPort.print(control_data_str.accel_y); SerialPort.print(" : ");
        SerialPort.print(control_data_str.accel_z); SerialPort.print(" : ");
        SerialPort.print(" x y ");
        SerialPort.print(x); SerialPort.print(" : ");
        SerialPort.print(y); SerialPort.print(" : ");
        SerialPort.print("L, R ");
        SerialPort.print(control_data_str.Rch_wiper_position);SerialPort.print(" : ");
        SerialPort.print(control_data_str.Lch_wiper_position);SerialPort.print(" : ");
        SerialPort.print("distance");
        SerialPort.print(control_data_str.distance_raw);SerialPort.print(" : ");
        SerialPort.print(control_data_str.distance_data);SerialPort.print(" : ");
        SerialPort.print("yaw pitch roll");
        SerialPort.print(control_data_str.yaw_data);SerialPort.print(" : ");
        SerialPort.print(control_data_str.pitch_data);SerialPort.print(" : ");
        SerialPort.print(control_data_str.roll_data);SerialPort.print(" : ");
        SerialPort.println("");
    #endif
}
