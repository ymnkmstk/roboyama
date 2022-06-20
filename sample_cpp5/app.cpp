#include "app.h"
#include "TouchSensor.h"
#include "SonarSensor.h"
#include "ColorSensor.h"
#include "GyroSensor.h"
#include "Motor.h"
#include <stdio.h>

using namespace ev3api;

/* オブジェクトへのポインタ定義 */
TouchSensor*    touchSensor;
SonarSensor*    sonarSensor;
ColorSensor*    colorSensor;
GyroSensor*     gyroSensor;
Motor*          armMotor;
Motor*          leftMotor;
Motor*          rightMotor;

/* メインタスク(起動時にのみ関数コールされる) */
void main_task(intptr_t unused) {

    /* センサー入力ポートの設定 */
    touchSensor = new TouchSensor(PORT_1);
    sonarSensor = new SonarSensor(PORT_3);
    colorSensor = new ColorSensor(PORT_2);
    gyroSensor  = new GyroSensor(PORT_4);
    /*
    ev3_sensor_config(touch_sensor ,TOUCH_SENSOR);
    ev3_sensor_config(color_sensor ,COLOR_SENSOR);
    ev3_sensor_config(sonar_sensor ,ULTRASONIC_SENSOR);
    ev3_sensor_config(gyro_sensor  ,GYRO_SENSOR);
    */
    
    /* モーター出力ポートの設定 */
    armMotor    = new Motor(PORT_A, true, LARGE_MOTOR);
    leftMotor   = new Motor(PORT_C, true, MEDIUM_MOTOR);
    rightMotor  = new Motor(PORT_B, true, MEDIUM_MOTOR);
    /*
    ev3_motor_config(arm_motor     ,LARGE_MOTOR);
    ev3_motor_config(left_motor    ,MEDIUM_MOTOR);
    ev3_motor_config(right_motor   ,MEDIUM_MOTOR);
    */
    
    printf("Start Line Trace!!\n");
    
    /* ライントレースタスクの起動 */
    sta_cyc(LINE_TRACER_TASK_CYC);

    /* タスク終了 */
    ext_tsk();
}
