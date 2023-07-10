/*
    This file is part of Repetier-Firmware.

    Repetier-Firmware is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    Repetier-Firmware is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with Repetier-Firmware.  If not, see <http://www.gnu.org/licenses/>.

*/

#ifndef CONFIGURATION_H
#define CONFIGURATION_H

/**************** READ FIRST ************************

   This configuration file was created with the configuration tool. For that
   reason, it does not contain the same informations as the original Configuration.h file.
   It misses the comments and unused parts. Open this file file in the config tool
   to see and change the data. You can also upload it to newer/older versions. The system
   will silently add new options, so compilation continues to work.

   This file is optimized for version 0.92
   generator: http://www.repetier.com/firmware/v092/

   If you are in doubt which named functions use which pins on your board, please check the
   pins.h for the used name->pin assignments and your board documentation to verify it is
   as you expect.

*/

#define NUM_EXTRUDER 0 //定义挤出机的数量
#define MOTHERBOARD 33 //定义主板的类型 https://github.com/repetier/Repetier-Firmware/blob/development/src/ArduinoAVR/Repetier/Configuration.h
#include "pins.h"

// ################## 手动编辑这些设置 ################
// ################ 结束手动设置 ##########################

#undef FAN_BOARD_PIN //取消定义风扇引脚
#define FAN_BOARD_PIN 9 //将风扇引脚设为-1，表示没有连接
#define BOARD_FAN_SPEED 255 //将风扇速度设为255，表示最大速度
#define FAN_THERMO_PIN -1 //将风扇温度传感器引脚设为-1，表示没有连接
#define FAN_THERMO_MIN_PWM 128 //将风扇温度传感器的最小PWM值设为128，表示最小占空比
#define FAN_THERMO_MAX_PWM 255 //将风扇温度传感器的最大PWM值设为255，表示最大占空比
#define FAN_THERMO_MIN_TEMP 45 //将风扇温度传感器的最小温度设为45摄氏度，表示低于这个温度时风扇不工作
#define FAN_THERMO_MAX_TEMP 60 //将风扇温度传感器的最小温度设为45摄氏度，表示低于这个温度时风扇不工作
#define FAN_THERMO_THERMISTOR_PIN -1 //将风扇温度传感器的热敏电阻引脚设为-1，表示没有连接
#define FAN_THERMO_THERMISTOR_TYPE 1 //将风扇温度传感器的热敏电阻引脚设为-1，表示没有连接

//#define EXTERNALSERIAL  use Arduino serial library instead of build in. Requires more ram, has only 63 byte input buffer.
// Uncomment the following line if you are using Arduino compatible firmware made for Arduino version earlier then 1.0
// If it is incompatible you will get compiler errors about write functions not being compatible!
//#define COMPAT_PRE1

#define BLUETOOTH_SERIAL  -1 //定义蓝牙端口 -1为没有蓝牙
#define BLUETOOTH_BAUD  115200 //蓝牙波特率
#define MIXING_EXTRUDER 0 //混合挤出机的数量为 ·0 

#define DRIVE_SYSTEM 1 //定义驱动器系统，1表示corexy结构
#define XAXIS_STEPS_PER_MM 320 //定义X轴每毫米的步数为80(16细分)160（32细分）
#define YAXIS_STEPS_PER_MM 320 //定义Y轴每毫米的步数为80（16细分）160（32细分）
#define ZAXIS_STEPS_PER_MM 800 //定义Z轴每毫米的步数为80（16细分）160（32细分）
#define EXTRUDER_FAN_COOL_TEMP 50 // 定义挤出机风扇开启的温度为50摄氏度
#define PDM_FOR_EXTRUDER 0 // 定义挤出机是否使用PDM（脉冲密度调制）控制，0表示否，1表示是
#define PDM_FOR_COOLER 0 // 定义冷却风扇是否使用PDM控制，0表示否，1表示是
#define DECOUPLING_TEST_MAX_HOLD_VARIANCE 20 // 定义温度稳定时的最大偏差为20摄氏度
#define DECOUPLING_TEST_MIN_TEMP_RISE 1 // 定义温度上升时的最小增量为1摄氏度
#define KILL_IF_SENSOR_DEFECT 0 // 定义当传感器故障时是否停止打印，0表示否，1表示是
#define RETRACT_ON_PAUSE 2 // 定义暂停打印时的回抽长度为2毫米
#define PAUSE_START_COMMANDS "" // 定义暂停打印时执行的G代码命令，用双引号包围，用\n分隔
#define PAUSE_END_COMMANDS "" // 定义恢复打印时执行的G代码命令，用双引号包围，用\n分隔
#define SHARED_EXTRUDER_HEATER 0 // 定义是否共享挤出机加热器，0表示否，1表示是

#define FEATURE_RETRACTION 1 // 定义是否启用回抽功能，0表示否，1表示是
#define AUTORETRACT_ENABLED 0// 定义是否自动回抽，0表示否，1表示是
#define RETRACTION_LENGTH 3 // 定义回抽长度为3毫米
#define RETRACTION_LONG_LENGTH 13 // 定义长距离移动时的回抽长度为13毫米
#define RETRACTION_SPEED 100 // 定义回抽速度为40毫米/秒
#define RETRACTION_Z_LIFT 0 // 定义回抽时Z轴提升的高度为0毫米
#define RETRACTION_UNDO_EXTRA_LENGTH 0 // 定义恢复挤出时额外推进的长度为0毫米
#define RETRACTION_UNDO_EXTRA_LONG_LENGTH 0 // 定义长距离移动后恢复挤出时额外推进的长度为0毫米
#define RETRACTION_UNDO_SPEED 200 // 定义恢复挤出时的速度为20毫米/秒
#define FILAMENTCHANGE_X_POS 0 // 定义换料时X轴的位置为0毫米
#define FILAMENTCHANGE_Y_POS 0 // 定义换料时Y轴的位置为0毫米
#define FILAMENTCHANGE_Z_ADD  2 // 定义换料时Z轴提升的高度为2毫米
#define FILAMENTCHANGE_REHOME 1 // 定义换料后是否重新归零，0表示否，1表示是
#define FILAMENTCHANGE_SHORTRETRACT 5 // 定义换料前的短回抽长度为5毫米
#define FILAMENTCHANGE_LONGRETRACT 50 // 定义换料前的长回抽长度为50毫米
#define JAM_STEPS 220 // 定义检测到堵塞时需要走过的步数为220步
#define JAM_SLOWDOWN_STEPS 320 // 定义减速打印时需要走过的步数为320步
#define JAM_SLOWDOWN_TO 70 // 定义减速打印时的百分比速度为70%
#define JAM_ERROR_STEPS 500 // 定义报错停止打印时需要走过的步数为500步
#define JAM_MIN_STEPS 10 // 定义最小有效步数为10步
#define JAM_ACTION 1  // 定义堵塞后执行的动作，0表示无动作，1表示暂停打印

#define RETRACT_DURING_HEATUP true// 定义加热过程中是否回抽，true表示是，false表示否
#define PID_CONTROL_RANGE 20// 定义PID控制范围为20摄氏度以内
#define SKIP_M109_IF_WITHIN 2// 定义如果目标温度和当前温度相差小于2摄氏度，则跳过M109命令（等待目标温度）             
#define SCALE_PID_TO_MAX 0// 定义是否根据最大功率比例调整PID输出值，0表示否，非零值表示是，并且该值就是最大功率百分比（例如64表示64%）
#define TEMP_HYSTERESIS 0// 定义温度滞后值为0摄氏度（即温度波动范围）
#define EXTRUDE_MAXLENGTH 160// 定义最大挤出长度为160毫米（防止意外操作导致损坏）
#define NUM_TEMPS_USERTHERMISTOR0 0// 定义用户自定义热敏电阻表0中的温度点个数为0（即不使用）
#define USER_THERMISTORTABLE0 {}// 略过用户自定义热敏电阻表0中的数据（因为不使用）
#define NUM_TEMPS_USERTHERMISTOR1 0// 同上，对于用户自定义热敏电阻表1
#define USER_THERMISTORTABLE1 {}
#define NUM_TEMPS_USERTHERMISTOR2 0// 同上，对于用户自定义热敏电阻表2
#define USER_THERMISTORTABLE2 {}
#define GENERIC_THERM_VREF 5//热敏电阻的参考电压
#define GENERIC_THERM_NUM_ENTRIES 33//热敏电阻表中温度点个数
#define HEATER_PWM_SPEED 0//指加热器的PWM（脉冲宽度调制）速度，0表示关闭PWM，1表示15Hz，2表示7.5Hz，3表示3.75Hz，4表示1.875Hz，5表示0.94Hz。

// ############# Heated bed configuration ########################

#define HAVE_HEATED_BED 0
#define HEATED_BED_MAX_TEMP 120
#define SKIP_M190_IF_WITHIN 3
#define HEATED_BED_SENSOR_TYPE 1
#define HEATED_BED_SENSOR_PIN TEMP_1_PIN
#define HEATED_BED_HEATER_PIN HEATER_1_PIN
#define HEATED_BED_SET_INTERVAL 5000
#define HEATED_BED_HEAT_MANAGER 0
#define HEATED_BED_PID_INTEGRAL_DRIVE_MAX 255
#define HEATED_BED_PID_INTEGRAL_DRIVE_MIN 80
#define HEATED_BED_PID_PGAIN_OR_DEAD_TIME   196
#define HEATED_BED_PID_IGAIN   33
#define HEATED_BED_PID_DGAIN 290
#define HEATED_BED_PID_MAX 255
#define HEATED_BED_DECOUPLE_TEST_PERIOD 300000
#define MIN_EXTRUDER_TEMP 150

#define MAXTEMP 275
#define MIN_DEFECT_TEMPERATURE -10
#define MAX_DEFECT_TEMPERATURE 290

// ##########################################################################################
// ##                             Laser configuration                                      ##
// ##########################################################################################

/*
If the firmware is in laser mode, it can control a laser output to cut or engrave materials.
Please use this feature only if you know about safety and required protection. Lasers are
dangerous and can hurt or make you blind!!!

The default laser driver only supports laser on and off. Here you control the eíntensity with
your feedrate. For exchangeable diode lasers this is normally enough. If you need more control
you can set the intensity in a range 0-255 with a custom extension to the driver. See driver.h
and comments on how to extend the functions non invasive with our event system.

If you have a laser - powder system you will like your E override. If moves contain a
increasing extruder position it will laser that move. With this trick you can
use existing fdm slicers to laser the output. Laser width is extrusion width.

Other tools may use M3 and M5 to enable/disable laser. Here G1/G2/G3 moves have laser enabled
and G0 moves have it disables.

In any case, laser only enables while moving. At the end of a move it gets
automatically disabled.

这串注释是用来说明Repetier-Firmware中的激光模式的功能和注意事项。
激光模式可以控制一个激光输出来切割或雕刻材料，但是需要注意安全和防护，
因为激光很危险，可能会伤害或致盲。默认的激光驱动只支持激光开和关，
通过进给速度来控制强度。对于可更换的二极管激光器，这通常足够了。如果需要更多的控制，
可以用一个自定义的扩展来设置0-255范围内的强度。具体的方法可以参考driver.h文件和相关的注释，
利用事件系统来非侵入式地扩展功能。如果有一个激光-粉末系统，可以用E覆盖功能。如果移动包含一
个增加的挤出机位置，那么就会在那个移动上激光。利用这个技巧，可以用现有的fdm切片器来激光输出。
激光宽度就是挤出宽度。其他的工具可能用M3和M5来开启/关闭激光。在这里，G1/G2/G3移动有激光开启，
而G0移动有激光关闭。在任何情况下，激光只在移动时开启。在移动结束时，它会自动关闭。

*/

#define SUPPORT_LASER 1 //是否支持激光模式
#define LASER_PIN HEATER_0_PIN //控制激光输出的引脚。这里用了加热器0的引脚
#define LASER_ON_HIGH 1 //激光开启时的电平

// ##                              CNC configuration                                       ##

/*
If the firmware is in CNC mode, it can control a mill with M3/M4/M5. It works
similar to laser mode, but mill keeps enabled during G0 moves and it allows
setting rpm (only with event extension that supports this) and milling direction.
It also can add a delay to wait for spindle to run on full speed.
*/

#define SUPPORT_CNC 0
#define CNC_WAIT_ON_ENABLE 300
#define CNC_WAIT_ON_DISABLE 0
#define CNC_ENABLE_PIN -1
#define CNC_ENABLE_WITH 1
#define CNC_DIRECTION_PIN -1
#define CNC_DIRECTION_CW 1


#define DEFAULT_PRINTER_MODE 1

// ################ Endstop configuration 定义限位开关的参数#####################

#define ENDSTOP_PULLUP_X_MIN true // 定义X轴最小限位开关是否使用上拉电阻，true表示是，false表示否
#define ENDSTOP_X_MIN_INVERTING true // 定义X轴最小限位开关的逻辑电平，true表示低电平触发，false表示高电平触发
#define MIN_HARDWARE_ENDSTOP_X true // 定义X轴是否有最小限位开关，true表示是，false表示否
#define ENDSTOP_PULLUP_Y_MIN true
#define ENDSTOP_Y_MIN_INVERTING true
#define MIN_HARDWARE_ENDSTOP_Y true
#define ENDSTOP_PULLUP_Z_MIN true
#define ENDSTOP_Z_MIN_INVERTING true
#define MIN_HARDWARE_ENDSTOP_Z true
#define ENDSTOP_PULLUP_X_MAX true// 同上，对于X轴最大限位开关
#define ENDSTOP_X_MAX_INVERTING false
#define MAX_HARDWARE_ENDSTOP_X false
#define ENDSTOP_PULLUP_Y_MAX true
#define ENDSTOP_Y_MAX_INVERTING false
#define MAX_HARDWARE_ENDSTOP_Y false
#define ENDSTOP_PULLUP_Z_MAX true
#define ENDSTOP_Z_MAX_INVERTING false
#define MAX_HARDWARE_ENDSTOP_Z false
#define max_software_endstop_r true // 定义是否使用软件限位开关来防止打印头超出打印范围，true表示是，false表示否

#define min_software_endstop_x false// 定义是否使用软件限位开关来防止X轴超出最小位置，true表示是，false表示否
#define min_software_endstop_y false
#define min_software_endstop_z false
#define max_software_endstop_x true// 定义是否使用软件限位开关来防止X轴超出最大位置，true表示是，false表示否
#define max_software_endstop_y true
#define max_software_endstop_z true
#define ENDSTOP_X_BACK_MOVE 5// 定义X轴回零时，在触发限位开关后向反方向移动的距离（毫米）
#define ENDSTOP_Y_BACK_MOVE 5
#define ENDSTOP_Z_BACK_MOVE 0
#define ENDSTOP_X_RETEST_REDUCTION_FACTOR 3// 定义X轴回零时，在移动回去后重新测试限位开关时的速度减少因子（越大则速度越慢）
#define ENDSTOP_Y_RETEST_REDUCTION_FACTOR 3
#define ENDSTOP_Z_RETEST_REDUCTION_FACTOR 3
#define ENDSTOP_X_BACK_ON_HOME 1// 定义X轴回零后再向反方向移动的距离（毫米）
#define ENDSTOP_Y_BACK_ON_HOME 1
#define ENDSTOP_Z_BACK_ON_HOME 0
#define ALWAYS_CHECK_ENDSTOPS 1// 定义是否在每一步都检查限位开关的状态，1表示是，0表示否（只在回零时检查）

// ################# XYZ movements ###################

#define X_ENABLE_ON 0// 定义X轴电机使能时的电平，0表示低电平，1表示高电平
#define Y_ENABLE_ON 0
#define Z_ENABLE_ON 0
#define DISABLE_X 0// 定义是否在打印结束后关闭X轴电机，0表示否，1表示是
#define DISABLE_Y 0
#define DISABLE_Z 0
#define DISABLE_E 1
#define INVERT_X_DIR 0// 定义是否反转X轴的方向，0表示否，1表示是
#define INVERT_Y_DIR 0
#define INVERT_Z_DIR 1
#define X_HOME_DIR -1// 定义X轴回零时的方向，-1表示最小限位开关方向，1表示最大限位开关方向
#define Y_HOME_DIR -1
#define Z_HOME_DIR -1
#define X_MAX_LENGTH 150// 定义X轴的最大长度（毫米）
#define Y_MAX_LENGTH 150
#define Z_MAX_LENGTH 180
#define X_MIN_POS 0// 定义X轴的最小位置（毫米）
#define Y_MIN_POS 0
#define Z_MIN_POS 0
#define DISTORTION_CORRECTION 0// 定义是否启用扭曲校正功能，0表示否，1表示是
#define DISTORTION_CORRECTION_POINTS 5// 定义扭曲校正时使用的网格点数（每行/列）
#define DISTORTION_CORRECTION_R 100// 定义扭曲校正时使用的圆形区域半径（毫米）
#define DISTORTION_PERMANENT 1// 定义是否永久保存扭曲校正数据到EEPROM中，0表示否，1表示是
#define DISTORTION_UPDATE_FREQUENCY 15// 定义扭曲校正时每多少步更新一次Z高度（步数）
#define DISTORTION_START_DEGRADE 0.5// 定义扭曲校正开始衰减的高度（毫米）
#define DISTORTION_END_HEIGHT 1// 定义扭曲校正结束的高度（毫米）
#define DISTORTION_EXTRAPOLATE_CORNERS 0// 定义是否在网格外插值计算扭曲值，0表示否，1表示是
#define DISTORTION_XMIN 10// 定义扭曲校正网格的X最小坐标（毫米）
#define DISTORTION_YMIN 10
#define DISTORTION_XMAX 190
#define DISTORTION_YMAX 190

// ##########################################################################################
// ##                           Movement settings                                          ##
// ##########################################################################################

#define FEATURE_BABYSTEPPING 1 // 定义是否启用微调功能，0表示否，1表示是
#define BABYSTEP_MULTIPLICATOR 1 // 定义微调的倍数，越大则每次微调的距离越大

#define DELTA_SEGMENTS_PER_SECOND_PRINT 180 // Move accurate setting for print moves// 定义三角洲式打印机在打印时的分段数，越大则打印的曲线越平滑
#define DELTA_SEGMENTS_PER_SECOND_MOVE 70 // Less accurate setting for other moves// 定义三角洲式打印机在非打印移动时的分段数，越大则移动的曲线越平滑
#define EXACT_DELTA_MOVES 1// 定义是否使用精确的三角洲式打印机移动算法，0表示否，1表示是

// Delta settings
#define DELTA_HOME_ON_POWER 0// 定义是否在上电后自动回零三角洲式打印机，0表示否，1表示是

#define DELTASEGMENTS_PER_PRINTLINE 24 // 定义三角洲式打印机在打印时每条直线的分段数，越大则打印的曲线越平滑
#define STEPPER_INACTIVE_TIME 360L // 定义电机在无动作后多久关闭（秒），0表示不关闭
#define MAX_INACTIVE_TIME 0L // 定义打印机在无动作后多久关闭（秒），0表示不关闭
#define MAX_FEEDRATE_X 2000 // 定义X轴的最大进给速度（毫米/秒）
#define MAX_FEEDRATE_Y 2000 // 定义Y轴的最大进给速度（毫米/秒）
#define MAX_FEEDRATE_Z 2 // 定义Z轴的最大进给速度（毫米/秒）
#define HOMING_FEEDRATE_X 40 // 定义X轴回零时的进给速度（毫米/秒）
#define HOMING_FEEDRATE_Y 40 // 定义Y轴回零时的进给速度（毫米/秒）
#define HOMING_FEEDRATE_Z 2 // 定义Z轴回零时的进给速度（毫米/秒）
#define HOMING_ORDER HOME_ORDER_ZXY // 定义回零时的顺序，可以是HOME_ORDER_XYZ, HOME_ORDER_XZY, HOME_ORDER_YXZ, HOME_ORDER_YZX, HOME_ORDER_ZXY 或 HOME_ORDER_ZYX
#define ZHOME_MIN_TEMPERATURE 0 // 定义Z轴回零时挤出头的最低温度（摄氏度）
#define ZHOME_HEAT_ALL 1 // 定义Z轴回零时是否加热所有挤出头，0表示否，1表示是
#define ZHOME_HEAT_HEIGHT 20 // 定义Z轴回零时加热挤出头的高度（毫米）
#define ZHOME_X_POS 999999 // 定义Z轴回零时X轴的位置（毫米），999999表示不改变位置
#define ZHOME_Y_POS 999999 // 定义Z轴回零时Y轴的位置（毫米），999999表示不改变位置
#define ENABLE_BACKLASH_COMPENSATION 0 // 定义是否启用反向间隙补偿功能，0表示否，1表示是
#define X_BACKLASH 0 // 定义X轴的反向间隙值（毫米）
#define Y_BACKLASH 0 // 定义Y轴的反向间隙值（毫米）
#define Z_BACKLASH 0 // 定义Z轴的反向间隙值（毫米）
#define RAMP_ACCELERATION 1 // 定义是否启用加速度控制功能，0表示否，1表示是
#define STEPPER_HIGH_DELAY 1 // 定义步进电机高电平持续时间（微秒）
#define DIRECTION_DELAY 0 // 定义步进电机方向切换延迟时间（微秒）
#define STEP_DOUBLER_FREQUENCY 12000 // 定义步进电机双倍频率（赫兹）
#define ALLOW_QUADSTEPPING 1 // 定义是否允许步进电机四倍步进，0表示否，1表示是
#define DOUBLE_STEP_DELAY 0 // 定义步进电机双倍步进延迟时间（微秒）
#define MAX_ACCELERATION_UNITS_PER_SQ_SECOND_X 8000 // 定义X轴的最大加速度（毫米/平方秒）
#define MAX_ACCELERATION_UNITS_PER_SQ_SECOND_Y 8000 // 同上，对于Y轴
#define MAX_ACCELERATION_UNITS_PER_SQ_SECOND_Z 100 // 同上，对于Z轴
#define MAX_TRAVEL_ACCELERATION_UNITS_PER_SQ_SECOND_X 1000 // 同上，对于X轴非打印移动
#define MAX_TRAVEL_ACCELERATION_UNITS_PER_SQ_SECOND_Y 1000 // 同上，对于Y轴非打印移动
#define MAX_TRAVEL_ACCELERATION_UNITS_PER_SQ_SECOND_Z 100 // 同上，对于Z轴非打印移动
#define INTERPOLATE_ACCELERATION_WITH_Z 0 // 定义是否根据Z高度插值计算加速度，0表示否，1表示是
#define ACCELERATION_FACTOR_TOP 100 // 定义加速度插值计算时使用的百分比因子
#define MAX_JERK 20 // 定义最大冲击值（毫米/秒），决定了加速度变化时能承受多大的突变量
#define MAX_ZJERK 0.3 // 同上，对于Z轴
#define PRINTLINE_CACHE_SIZE 16 // 定义缓存队列中能存储多少条指令
#define MOVE_CACHE_LOW 10 // 定义缓存队列中剩余多少条指令时开始减慢速度
#define LOW_TICKS_PER_MOVE 250000 // 定义每条指令执行所需最少时间（微秒），防止过快执行造成错误或丢失步数
#define EXTRUDER_SWITCH_XY_SPEED 100 // 定义切换挤出头时XY轴移动的速度（毫米/分钟）
#define DUAL_X_AXIS 0 // 定义是否使用双X轴功能，0表示否，1表示是
#define FEATURE_TWO_XSTEPPER 0 // 同上，对于双X轴电机功能
#define X2_STEP_PIN ORIG_E1_STEP_PIN // 如果使用双X轴电机功能，则定义第二个X轴电机步进信号引脚为挤出机1步进信号引脚
#define X2_DIR_PIN ORIG_E1_DIR_PIN   // 如果使用双X轴电机功能，则定义第二个X轴电机方向信号引脚为挤出机1方向信号引脚
#define X2_ENABLE_PIN ORIG_E1_ENABLE_PIN   // 如果使用双X轴电机功能，则定义第二个X轴电机使能信号引脚为挤出机1使能信号引脚
#define FEATURE_TWO_YSTEPPER 0   // 同上，对于双Y轴电机功能
#define Y2_STEP_PIN ORIG_E1_STEP_PIN   // 如果使用双Y轴电机功能，则定义第二个Y轴电机步进信号引脚为挤出机1步进信号引脚
#define Y2_DIR_PIN ORIG_E1_DIR_PIN   // 如果使用双Y轴电机功能，则定义第二个Y轴电机方向信号引脚为挤出机1方向信号引脚
#define Y2_ENABLE_PIN ORIG_E1_ENABLE_PIN   // 如果使用双Y轴
#define FEATURE_TWO_ZSTEPPER 0 // 定义是否使用双Z轴电机功能，0表示否，1表示是
#define Z2_STEP_PIN ORIG_E1_STEP_PIN // 如果使用双Z轴电机功能，则定义第二个Z轴电机步进信号引脚为挤出机1步进信号引脚
#define Z2_DIR_PIN ORIG_E1_DIR_PIN // 如果使用双Z轴电机功能，则定义第二个Z轴电机方向信号引脚为挤出机1方向信号引脚
#define Z2_ENABLE_PIN ORIG_E1_ENABLE_PIN // 如果使用双Z轴电机功能，则定义第二个Z轴电机使能信号引脚为挤出机1使能信号引脚
#define FEATURE_THREE_ZSTEPPER 0 // 定义是否使用三Z轴电机功能，0表示否，1表示是
#define Z3_STEP_PIN ORIG_E2_STEP_PIN // 如果使用三Z轴电机功能，则定义第三个Z轴电机步进信号引脚为挤出机2步进信号引脚
#define Z3_DIR_PIN ORIG_E2_DIR_PIN // 如果使用三Z轴电机功能，则定义第三个Z轴电机方向信号引脚为挤出机2方向信号引脚
#define Z3_ENABLE_PIN ORIG_E2_ENABLE_PIN // 如果使用三Z轴电机功能，则定义第三个Z轴电机使能信号引脚为挤出机2使能信号引脚
#define FEATURE_DITTO_PRINTING 0 // 定义是否启用ditto打印功能，即两个挤出头同时打印相同的物体，0表示否，1表示是
#define USE_ADVANCE 0 // 定义是否启用advance算法来优化挤出速度，0表示否，1表示是
#define ENABLE_QUADRATIC_ADVANCE 0 // 定义是否启用二次advance算法来优化挤出速度，0表示否，1表示是


// ################# Misc. settings ##################

#define BAUDRATE 115200 // 定义与主机软件通信的波特率（比特/秒）
#define ENABLE_POWER_ON_STARTUP 1 // 定义是否在上电后自动打开电源，0表示否，1表示是
#define POWER_INVERTING 0 // 定义是否反转电源控制信号，0表示否，1表示是
#define KILL_METHOD 1 // 定义紧急停止时的处理方式，0表示只关闭电源，1表示重置打印机
#define ACK_WITH_LINENUMBER 1 // 定义是否在回应主机软件时附加行号，0表示否，1表示是
#define WAITING_IDENTIFIER "wait" // 定义在等待主机软件发送指令时发送的字符串
#define ECHO_ON_EXECUTE 1 // 定义是否在执行指令时回显指令内容，0表示否，1表示是
#define EEPROM_MODE 2 // 定义EEPROM的工作模式，0表示禁用EEPROM，1表示使用固定地址存储数据，2表示使用动态地址存储数据
#undef PS_ON_PIN // 取消定义PS_ON_PIN宏
#define PS_ON_PIN ORIG_PS_ON_PIN // 重新定义PS_ON_PIN宏为ORIG_PS_ON_PIN宏，即电源控制信号引脚
#define JSON_OUTPUT 0 // 定义是否使用JSON格式输出数据，0表示否，1表示是

/* ======== Servos =======
Control the servos with
M340 P<servoId> S<pulseInUS>   / ServoID = 0..3  pulseInUs = 500..2500
Servos are controlled by a pulse width normally between 500 and 2500 with 1500ms in center position. 0 turns servo off.
WARNING: Servos can draw a considerable amount of current. Make sure your system can handle this or you may risk your hardware!
*/
#define FEATURE_SERVO 0 // 定义是否启用伺服电机功能，0表示否，1表示是
#define SERVO0_PIN 11 // 定义第一个伺服电机信号引脚为11号引脚
#define SERVO1_PIN -1 // 定义第二个伺服电机信号引脚为-1，即不使用
#define SERVO2_PIN -1 // 定义第三个伺服电机信号引脚为-1，即不使用
#define SERVO3_PIN -1 // 定义第四个伺服电机信号引脚为-1，即不使用
#define SERVO0_NEUTRAL_POS -1 // 定义第一个伺服电机的中立位置（微秒），-1表示使用固件默认值
#define SERVO1_NEUTRAL_POS -1 // 定义第二个伺服电机的中立位置（微秒），-1表示使用固件默认值
#define SERVO2_NEUTRAL_POS -1 // 定义第三个伺服电机的中立位置（微秒），-1表示使用固件默认值
#define SERVO3_NEUTRAL_POS -1 // 定义第四个伺服电机的中立位置（微秒），-1表示使用固件默认值
#define UI_SERVO_CONTROL 0 // 定义是否在LCD菜单中控制伺服电机，0表示否，大于0表示控制的伺服电机编号
#define FAN_KICKSTART_TIME 200 // 定义风扇启动时的最大功率持续时间（毫秒）

        #define FEATURE_WATCHDOG 0 // 定义是否启用看门狗功能，0表示否，1表示是。看门狗功能可以在打印机出现故障时自动重置打印机，防止过热或其他危险情况。

// #################### Z-Probing #####################

#define Z_PROBE_Z_OFFSET 0 // 定义Z探针在触发时与喷嘴的高度差（毫米），正值表示探针在喷嘴下方，负值表示探针在喷嘴上方
#define Z_PROBE_Z_OFFSET_MODE 0 // 定义Z探针的高度差模式，0表示使用固定值，1表示使用EEPROM中的值，2表示使用LCD菜单中的值
#define UI_BED_COATING 1 // 定义是否在LCD菜单中显示床涂层的厚度，0表示否，1表示是
#define FEATURE_Z_PROBE 0 // 定义是否启用Z探针功能，0表示否，1表示是
#define Z_PROBE_BED_DISTANCE 10 // 定义Z探针测量前移动到的高度（毫米）
#define Z_PROBE_PIN -1 // 定义Z探针信号引脚为-1，即不使用
#define Z_PROBE_PULLUP 0 // 定义是否启用Z探针信号引脚的上拉电阻，0表示否，1表示是
#define Z_PROBE_ON_HIGH 0 // 定义Z探针触发时的信号电平，0表示低电平，1表示高电平
#define Z_PROBE_X_OFFSET 0 // 定义Z探针相对于喷嘴的X方向偏移量（毫米）
#define Z_PROBE_Y_OFFSET 0 // 定义Z探针相对于喷嘴的Y方向偏移量（毫米）
#define Z_PROBE_WAIT_BEFORE_TEST 0 // 定义是否在每次测量前等待用户确认，0表示否，1表示是
#define Z_PROBE_SPEED 2 // 定义Z探针测量时的速度（毫米/秒）
#define Z_PROBE_XY_SPEED 150 // 定义Z探针移动时的速度（毫米/秒）
#define Z_PROBE_SWITCHING_DISTANCE 1 // 定义Z探针在测量前后移动的距离（毫米）
#define Z_PROBE_REPETITIONS 1 // 定义每个点的测量次数，越多越精确，但也越慢
#define Z_PROBE_HEIGHT 40 // 定义Z探针触发时喷嘴距离床面的高度（毫米），正值表示喷嘴在床面上方，负值表示喷嘴在床面下方
#define Z_PROBE_START_SCRIPT "" // 定义在开始测量前执行的G代码脚本
#define Z_PROBE_FINISHED_SCRIPT "" // 定义在结束测量后执行的G代码脚本
#define Z_PROBE_REQUIRES_HEATING 0 // 定义是否在测量前加热挤出头和热床，0表示否，1表示是
#define Z_PROBE_MIN_TEMPERATURE 150 // 定义测量前挤出头和热床的最低温度（摄氏度）
#define FEATURE_AUTOLEVEL 1 // 定义是否启用自动调平功能，0表示否，1表示是
#define Z_PROBE_X1 20 // 定义第一个测量点的X坐标（毫米）
#define Z_PROBE_Y1 20 // 定义第一个测量点的Y坐标（毫米）
#define Z_PROBE_X2 160 // 定义第二个测量点的X坐标（毫米）
#define Z_PROBE_Y2 20 // 定义第二个测量点的Y坐标（毫米）
#define Z_PROBE_X3 100 // 定义第三个测量点的X坐标（毫米）
#define Z_PROBE_Y3 160 // 定义第三个测量点的Y坐标（毫米）
#define BED_LEVELING_METHOD 0 // 定义调平方法，0表示使用3点法求平面，1表示使用最小二乘法求平面
#define BED_CORRECTION_METHOD 0 // 定义调平校正方法，0表示使用软件旋转校正，1表示使用电机调整校正
#define BED_LEVELING_GRID_SIZE 5 // 定义调平网格大小，即每行或每列有多少个测量点
#define BED_LEVELING_REPETITIONS 5 // 定义调平重复次数，即每个点重复测量多少次取平均值
#define BED_MOTOR_1_X 0 // 定义第一个床电机的X坐标（毫米）
#define BED_MOTOR_1_Y 0 // 定义第一个床电机的Y坐标（毫米）
#define BED_MOTOR_2_X 200 // 同上，对于第二个床电机
#define BENDING_CORRECTION_A 0 // 定义床弯曲校正的A系数，用于修正床在X方向的弯曲
#define BENDING_CORRECTION_B 0 // 定义床弯曲校正的B系数，用于修正床在Y方向的弯曲
#define BENDING_CORRECTION_C 0 // 定义床弯曲校正的C系数，用于修正床在XY方向的弯曲
#define FEATURE_AXISCOMP 0 // 定义是否启用轴补偿功能，0表示否，1表示是。轴补偿功能可以修正轴之间的角度误差
#define AXISCOMP_TANXY 0 // 定义XY轴之间的正切值，用于修正XY轴的角度误差
#define AXISCOMP_TANYZ 0 // 定义YZ轴之间的正切值，用于修正YZ轴的角度误差
#define AXISCOMP_TANXZ 0 // 定义XZ轴之间的正切值，用于修正XZ轴的角度误差

#ifndef SDSUPPORT // Some boards have sd support on board. These define the values already in pins.h
#define SDSUPPORT 0 // 定义是否启用SD卡支持功能，0表示否，1表示是
#undef SDCARDDETECT // 取消定义SDCARDDETECT宏
#define SDCARDDETECT -1 // 定义SD卡检测引脚为-1，即不使用
#define SDCARDDETECTINVERTED 0 // 定义SD卡检测信号是否反转，0表示否，1表示是
#endif
#define SD_EXTENDED_DIR 1 /** Show extended directory including file length. Don't use this with Pronterface! */ // 定义是否显示扩展目录，包括文件长度。不要与Pronterface软件一起使用！0表示否，1表示是
#define SD_RUN_ON_STOP "" // 定义在停止打印时执行的G代码脚本
#define SD_STOP_HEATER_AND_MOTORS_ON_STOP 1 // 定义在停止打印时是否关闭加热器和电机，0表示否，1表示是
#define ARC_SUPPORT 1 // 定义是否支持圆弧指令（G2/G3），0表示否，1表示是
#define FEATURE_MEMORY_POSITION 1 // 定义是否启用记忆位置功能，即使用G60/G61指令保存和恢复位置，0表示否，1表示是
#define FEATURE_CHECKSUM_FORCED 0 // 定义是否强制使用校验和功能，即如果主机软件没有发送校验和，则拒绝执行指令，0表示否，1表示是
#define FEATURE_FAN_CONTROL 1 // 定义是否启用风扇控制功能，0表示否，1表示是
#define FEATURE_FAN2_CONTROL 0 // 定义是否启用第二个风扇控制功能，0表示否，1表示是
#define FEATURE_CONTROLLER 0 // 定义使用哪种LCD控制器类型，具体值参考固件文档
#define ADC_KEYPAD_PIN -1 // 定义ADC键盘引脚为-1，即不使用
#define LANGUAGE_EN_ACTIVE 1 // 定义是否启用英语语言选项，0表示否，1表示是
#define LANGUAGE_DE_ACTIVE 1 // 同上，对于德语语言选项
#define LANGUAGE_NL_ACTIVE 0
#define LANGUAGE_PT_ACTIVE 1
#define LANGUAGE_IT_ACTIVE 1
#define LANGUAGE_ES_ACTIVE 1
#define LANGUAGE_FI_ACTIVE 0
#define LANGUAGE_SE_ACTIVE 0
#define LANGUAGE_FR_ACTIVE 1
#define LANGUAGE_CZ_ACTIVE 0
#define LANGUAGE_PL_ACTIVE 1
#define LANGUAGE_TR_ACTIVE 1
#define UI_PRINTER_NAME "RepRap" // 定义打印机的名称，显示在LCD界面上
#define UI_PRINTER_COMPANY "Home made" // 定义打印机的制造商，显示在LCD界面上
#define UI_PAGES_DURATION 4000 // 定义LCD界面的页面切换时间（毫秒）
#define UI_ANIMATION 0 // 定义是否启用LCD界面的动画效果，0表示否，1表示是
#define UI_SPEEDDEPENDENT_POSITIONING 0 // 定义是否启用速度相关的位置控制，0表示否，1表示是。这个功能可以根据旋钮的旋转速度来调整移动距离
#define UI_DISABLE_AUTO_PAGESWITCH 1 // 定义是否禁用LCD界面的自动页面切换，0表示否，1表示是
#define UI_AUTORETURN_TO_MENU_AFTER 30000 // 定义在多长时间后自动返回LCD界面的主菜单（毫秒）
#define FEATURE_UI_KEYS 0 // 定义是否启用LCD界面的按键功能，0表示否，1表示是
#define UI_ENCODER_SPEED 1 // 定义LCD界面的旋钮速度，即每次旋转多少步改变一个值。可选值有1,2,4
#define UI_REVERSE_ENCODER 0 // 定义是否反转LCD界面的旋钮方向，0表示否，1表示是
#define UI_KEY_BOUNCETIME 10 // 定义LCD界面的按键消抖时间（毫秒）
#define UI_KEY_FIRST_REPEAT 500 // 定义LCD界面的按键第一次重复发送信号的延迟时间（毫秒）
#define UI_KEY_REDUCE_REPEAT 50 // 定义LCD界面的按键重复发送信号的递减时间（毫秒）
#define UI_KEY_MIN_REPEAT 50 // 定义LCD界面的按键重复发送信号的最小时间（毫秒）
#define FEATURE_BEEPER 0 // 定义是否启用蜂鸣器功能，0表示否，1表示是。蜂鸣器功能可以在按键或重要操作时发出声音提示
#define CASE_LIGHTS_PIN -1 // 定义机箱灯控制引脚为-1，即不使用
#define CASE_LIGHT_DEFAULT_ON 1 // 定义机箱灯默认状态，0表示关闭，1表示打开
#define UI_START_SCREEN_DELAY 1000 // 定义LCD界面启动画面的延迟时间（毫秒）
#define UI_DYNAMIC_ENCODER_SPEED 1 // 定义是否启用动态旋钮速度功能，0表示否，1表示是。这个功能可以根据旋钮的旋转速度来调整移动距离
 /**
Beeper sound definitions for short beeps during key actions
and longer beeps for important actions.
Parameter is delay in microseconds and the secons is the number of repetitions.
Values must be in range 1..255
*/
#define BEEPER_SHORT_SEQUENCE 2,2 // 定义短蜂鸣声序列为2微秒延迟和2次重复
#define BEEPER_LONG_SEQUENCE 8,8 // 定义长蜂鸣声序列为8微秒延迟和8次重复
#define UI_SET_PRESET_HEATED_BED_TEMP_PLA 60 // 定义PLA材料预设热床温度为60摄氏度
#define UI_SET_PRESET_EXTRUDER_TEMP_PLA 190 // 定义PLA材料预设挤出头温度为190摄氏度
#define UI_SET_PRESET_HEATED_BED_TEMP_ABS 110 // 同上，对于ABS材料预设热床温度
#define UI_SET_MIN_HEATED_BED_TEMP  30
#define UI_SET_MAX_HEATED_BED_TEMP 120
#define UI_SET_MIN_EXTRUDER_TEMP   170
#define UI_SET_MAX_EXTRUDER_TEMP   260
#define UI_SET_EXTRUDER_FEEDRATE 2
#define UI_SET_EXTRUDER_RETRACT_DISTANCE 3


#define NUM_MOTOR_DRIVERS 1 // 定义使用的电机驱动器的数量为1
<<<<<<< HEAD
#define MOTOR_DRIVER_1(var) StepperDriver<26,28,24,0,0> var(107,40) // 定义第一个电机驱动器的参数，包括步进引脚，方向引脚，使能引脚，最小限位开关引脚，最大限位开关引脚，每转步数，每毫米步数
=======
#define MOTOR_DRIVER_1(var) StepperDriver<26,28,24,0,0> var(107,50) // 定义第一个电机驱动器的参数，包括步进引脚，方向引脚，使能引脚，最小限位开关引脚，最大限位开关引脚，每转步数，每毫米步数
>>>>>>> 0d3481ff61b80fc504cba6a890a236fafdded1bc



#endif // 结束条件编译

/* Below you will find the configuration string, that created this Configuration.h

========== Start configuration string ==========
{
    "editMode": 2,
    "processor": 0,
    "baudrate": 115200,
    "bluetoothSerial": -1,
    "bluetoothBaudrate": 115200,
    "xStepsPerMM": 80,
    "yStepsPerMM": 80,
    "zStepsPerMM": 80,
    "xInvert": 0,
    "xInvertEnable": 0,
    "eepromMode": 2,
    "yInvert": 0,
    "yInvertEnable": 0,
    "zInvert": "1",
    "zInvertEnable": 0,
    "extruder": [],
    "uiLanguage": 0,
    "uiController": 0,
    "xMinEndstop": 1,
    "yMinEndstop": 1,
    "zMinEndstop": 1,
    "xMaxEndstop": 0,
    "yMaxEndstop": 0,
    "zMaxEndstop": 0,
    "motherboard": 33,
    "driveSystem": 1,
    "xMaxSpeed": 200,
    "xHomingSpeed": 40,
    "xTravelAcceleration": 1000,
    "xPrintAcceleration": 1000,
    "yMaxSpeed": 200,
    "yHomingSpeed": 40,
    "yTravelAcceleration": 1000,
    "yPrintAcceleration": 1000,
    "zMaxSpeed": 2,
    "zHomingSpeed": 2,
    "zTravelAcceleration": 100,
    "zPrintAcceleration": 100,
    "xMotor": {
        "name": "X motor",
        "step": "ORIG_X_STEP_PIN",
        "dir": "ORIG_X_DIR_PIN",
        "enable": "ORIG_X_ENABLE_PIN"
    },
    "yMotor": {
        "name": "Y motor",
        "step": "ORIG_Y_STEP_PIN",
        "dir": "ORIG_Y_DIR_PIN",
        "enable": "ORIG_Y_ENABLE_PIN"
    },
    "zMotor": {
        "name": "Z motor",
        "step": "ORIG_Z_STEP_PIN",
        "dir": "ORIG_Z_DIR_PIN",
        "enable": "ORIG_Z_ENABLE_PIN"
    },
    "enableBacklash": "0",
    "backlashX": 0,
    "backlashY": 0,
    "backlashZ": 0,
    "stepperInactiveTime": 360,
    "maxInactiveTime": 0,
    "xMinPos": 0,
    "yMinPos": 0,
    "zMinPos": 0,
    "xLength": 300,
    "yLength": 160,
    "zLength": 80,
    "alwaysCheckEndstops": "1",
    "disableX": "0",
    "disableY": "0",
    "disableZ": "0",
    "disableE": "0",
    "xHomeDir": "-1",
    "yHomeDir": "-1",
    "zHomeDir": "-1",
    "xEndstopBack": 1,
    "yEndstopBack": 1,
    "zEndstopBack": 0,
    "deltaSegmentsPerSecondPrint": 180,
    "deltaSegmentsPerSecondTravel": 70,
    "deltaDiagonalRod": 445,
    "deltaHorizontalRadius": 209.25,
    "deltaAlphaA": 210,
    "deltaAlphaB": 330,
    "deltaAlphaC": 90,
    "deltaDiagonalCorrA": 0,
    "deltaDiagonalCorrB": 0,
    "deltaDiagonalCorrC": 0,
    "deltaMaxRadius": 150,
    "deltaFloorSafetyMarginMM": 15,
    "deltaRadiusCorrA": 0,
    "deltaRadiusCorrB": 0,
    "deltaRadiusCorrC": 0,
    "deltaXOffsetSteps": 0,
    "deltaYOffsetSteps": 0,
    "deltaZOffsetSteps": 0,
    "deltaSegmentsPerLine": 24,
    "stepperHighDelay": 1,
    "directionDelay": 0,
    "stepDoublerFrequency": 12000,
    "allowQuadstepping": "1",
    "doubleStepDelay": 0,
    "maxJerk": 20,
    "maxZJerk": 0.3,
    "moveCacheSize": 16,
    "moveCacheLow": 10,
    "lowTicksPerMove": 250000,
    "enablePowerOnStartup": "1",
    "echoOnExecute": "1",
    "sendWaits": "1",
    "ackWithLineNumber": "1",
    "killMethod": 1,
    "useAdvance": "0",
    "useQuadraticAdvance": "0",
    "powerInverting": 0,
    "mirrorX": 0,
    "mirrorXMotor": {
        "name": "Extruder 1",
        "step": "ORIG_E1_STEP_PIN",
        "dir": "ORIG_E1_DIR_PIN",
        "enable": "ORIG_E1_ENABLE_PIN"
    },
    "mirrorY": 0,
    "mirrorYMotor": {
        "name": "Extruder 1",
        "step": "ORIG_E1_STEP_PIN",
        "dir": "ORIG_E1_DIR_PIN",
        "enable": "ORIG_E1_ENABLE_PIN"
    },
    "mirrorZ": "0",
    "mirrorZMotor": {
        "name": "Extruder 1",
        "step": "ORIG_E1_STEP_PIN",
        "dir": "ORIG_E1_DIR_PIN",
        "enable": "ORIG_E1_ENABLE_PIN"
    },
    "mirrorZ3": "0",
    "mirrorZ3Motor": {
        "name": "Extruder 2",
        "step": "ORIG_E2_STEP_PIN",
        "dir": "ORIG_E2_DIR_PIN",
        "enable": "ORIG_E2_ENABLE_PIN"
    },
    "dittoPrinting": "0",
    "featureServos": "0",
    "servo0Pin": 11,
    "servo1Pin": -1,
    "servo2Pin": -1,
    "servo3Pin": -1,
    "featureWatchdog": "0",
    "hasHeatedBed": "0",
    "enableZProbing": "0",
    "extrudeMaxLength": 160,
    "homeOrder": "HOME_ORDER_ZXY",
    "featureController": 0,
    "uiPrinterName": "RepRap",
    "uiPrinterCompany": "Home made",
    "uiPagesDuration": 4000,
    "uiAnimation": "0",
    "uiDisablePageswitch": "1",
    "uiAutoReturnAfter": 30000,
    "featureKeys": "0",
    "uiEncoderSpeed": 1,
    "uiReverseEncoder": "0",
    "uiKeyBouncetime": 10,
    "uiKeyFirstRepeat": 500,
    "uiKeyReduceRepeat": 50,
    "uiKeyMinRepeat": 50,
    "featureBeeper": "0",
    "uiPresetBedTempPLA": 60,
    "uiPresetBedABS": 110,
    "uiPresetExtruderPLA": 190,
    "uiPresetExtruderABS": 240,
    "uiMinHeatedBed": 30,
    "uiMaxHeatedBed": 120,
    "uiMinEtxruderTemp": 170,
    "uiMaxExtruderTemp": 260,
    "uiExtruderFeedrate": 2,
    "uiExtruderRetractDistance": 3,
    "uiSpeeddependentPositioning": "0",
    "maxBedTemperature": 120,
    "bedSensorType": 1,
    "bedSensorPin": "TEMP_1_PIN",
    "bedHeaterPin": "HEATER_1_PIN",
    "bedHeatManager": 0,
    "bedUpdateInterval": 5000,
    "bedPidDriveMin": 80,
    "bedPidDriveMax": 255,
    "bedPidP": 196,
    "bedPidI": 33,
    "bedPidD": 290,
    "bedPidMax": 255,
    "bedDecoupleTestPeriod": 300,
    "caseLightPin": -1,
    "caseLightDefaultOn": "1",
    "bedSkipIfWithin": 3,
    "gen1T0": 25,
    "gen1R0": 100000,
    "gen1Beta": 4036,
    "gen1MinTemp": -20,
    "gen1MaxTemp": 300,
    "gen1R1": 0,
    "gen1R2": 4700,
    "gen2T0": 25,
    "gen2R0": 100000,
    "gen2Beta": 4036,
    "gen2MinTemp": -20,
    "gen2MaxTemp": 300,
    "gen2R1": 0,
    "gen2R2": 4700,
    "gen3T0": 25,
    "gen3R0": 100000,
    "gen3Beta": 4036,
    "gen3MinTemp": -20,
    "gen3MaxTemp": 300,
    "gen3R1": 0,
    "gen3R2": 4700,
    "userTable0": {
        "r1": 0,
        "r2": 4700,
        "temps": [],
        "numEntries": 0
    },
    "userTable1": {
        "r1": 0,
        "r2": 4700,
        "temps": [],
        "numEntries": 0
    },
    "userTable2": {
        "r1": 0,
        "r2": 4700,
        "temps": [],
        "numEntries": 0
    },
    "tempHysteresis": 0,
    "pidControlRange": 20,
    "skipM109Within": 2,
    "extruderFanCoolTemp": 50,
    "minTemp": 150,
    "maxTemp": 275,
    "minDefectTemp": -10,
    "maxDefectTemp": 290,
    "arcSupport": "1",
    "featureMemoryPositionWatchdog": "1",
    "forceChecksum": "0",
    "sdExtendedDir": "1",
    "featureFanControl": "1",
    "fanPin": "ORIG_FAN_PIN",
    "featureFan2Control": "0",
    "fan2Pin": "ORIG_FAN2_PIN",
    "fanThermoPin": -1,
    "fanThermoMinPWM": 128,
    "fanThermoMaxPWM": 255,
    "fanThermoMinTemp": 45,
    "fanThermoMaxTemp": 60,
    "fanThermoThermistorPin": -1,
    "fanThermoThermistorType": 1,
    "scalePidToMax": 0,
    "zProbePin": -1,
    "zProbeBedDistance": 10,
    "zProbePullup": "0",
    "zProbeOnHigh": "0",
    "zProbeXOffset": 0,
    "zProbeYOffset": 0,
    "zProbeWaitBeforeTest": "0",
    "zProbeSpeed": 2,
    "zProbeXYSpeed": 150,
    "zProbeHeight": 40,
    "zProbeStartScript": "",
    "zProbeFinishedScript": "",
    "featureAutolevel": "1",
    "zProbeX1": 20,
    "zProbeY1": 20,
    "zProbeX2": 160,
    "zProbeY2": 20,
    "zProbeX3": 100,
    "zProbeY3": 160,
    "zProbeSwitchingDistance": 1,
    "zProbeRepetitions": 1,
    "sdSupport": "0",
    "sdCardDetectPin": -1,
    "sdCardDetectInverted": "0",
    "uiStartScreenDelay": 1000,
    "xEndstopBackMove": 5,
    "yEndstopBackMove": 5,
    "zEndstopBackMove": 0,
    "xEndstopRetestFactor": 3,
    "yEndstopRetestFactor": 3,
    "zEndstopRetestFactor": 3,
    "xMinPin": "ORIG_X_MIN_PIN",
    "yMinPin": "ORIG_Y_MIN_PIN",
    "zMinPin": "ORIG_Z_MIN_PIN",
    "xMaxPin": "ORIG_X_MAX_PIN",
    "yMaxPin": "ORIG_Y_MAX_PIN",
    "zMaxPin": "ORIG_Z_MAX_PIN",
    "deltaHomeOnPower": "0",
    "fanBoardPin": -1,
    "heaterPWMSpeed": 0,
    "featureBabystepping": "1",
    "babystepMultiplicator": 1,
    "pdmForHeater": "0",
    "pdmForCooler": "0",
    "psOn": "ORIG_PS_ON_PIN",
    "mixingExtruder": "0",
    "decouplingTestMaxHoldVariance": 20,
    "decouplingTestMinTempRise": 1,
    "featureAxisComp": "0",
    "axisCompTanXY": 0,
    "axisCompTanXZ": 0,
    "axisCompTanYZ": 0,
    "retractOnPause": 2,
    "pauseStartCommands": "",
    "pauseEndCommands": "",
    "distortionCorrection": "0",
    "distortionCorrectionPoints": 5,
    "distortionCorrectionR": 100,
    "distortionPermanent": "1",
    "distortionUpdateFrequency": 15,
    "distortionStartDegrade": 0.5,
    "distortionEndDegrade": 1,
    "distortionExtrapolateCorners": "0",
    "distortionXMin": 10,
    "distortionXMax": 190,
    "distortionYMin": 10,
    "distortionYMax": 190,
    "sdRunOnStop": "",
    "sdStopHeaterMotorsOnStop": "1",
    "featureRetraction": "1",
    "autoretractEnabled": "0",
    "retractionLength": 3,
    "retractionLongLength": 13,
    "retractionSpeed": 40,
    "retractionZLift": 0,
    "retractionUndoExtraLength": 0,
    "retractionUndoExtraLongLength": 0,
    "retractionUndoSpeed": 20,
    "filamentChangeXPos": 0,
    "filamentChangeYPos": 0,
    "filamentChangeZAdd": 2,
    "filamentChangeRehome": 1,
    "filamentChangeShortRetract": 5,
    "filamentChangeLongRetract": 50,
    "fanKickstart": 200,
    "servo0StartPos": -1,
    "servo1StartPos": -1,
    "servo2StartPos": -1,
    "servo3StartPos": -1,
    "uiDynamicEncoderSpeed": "1",
    "uiServoControl": 0,
    "killIfSensorDefect": "0",
    "jamSteps": 220,
    "jamSlowdownSteps": 320,
    "jamSlowdownTo": 70,
    "jamErrorSteps": 500,
    "jamMinSteps": 10,
    "jamAction": 1,
    "primaryPort": 0,
    "numMotorDrivers": 1,
    "motorDrivers": [
        {
            "t": "Stepper",
            "s": "StepperDriver<26,28,24,0,0> var(80,10)",
            "invertEnable": "0",
            "invertDirection": "0",
            "stepsPerMM": 80,
            "speed": 10,
            "dirPin": 28,
            "stepPin": 26,
            "enablePin": 24
        },
        {
            "t": "None",
            "s": "",
            "invertEnable": "0",
            "invertDirection": "0",
            "stepsPerMM": 100,
            "speed": 10,
            "dirPin": -1,
            "stepPin": -1,
            "enablePin": -1
        },
        {
            "t": "None",
            "s": "",
            "invertEnable": "0",
            "invertDirection": "0",
            "stepsPerMM": 100,
            "speed": 10,
            "dirPin": -1,
            "stepPin": -1,
            "enablePin": -1
        },
        {
            "t": "None",
            "s": "",
            "invertEnable": "0",
            "invertDirection": "0",
            "stepsPerMM": 100,
            "speed": 10,
            "dirPin": -1,
            "stepPin": -1,
            "enablePin": -1
        },
        {
            "t": "None",
            "s": "",
            "invertEnable": "0",
            "invertDirection": "0",
            "stepsPerMM": 100,
            "speed": 10,
            "dirPin": -1,
            "stepPin": -1,
            "enablePin": -1
        },
        {
            "t": "None",
            "s": "",
            "invertEnable": "0",
            "invertDirection": "0",
            "stepsPerMM": 100,
            "speed": 10,
            "dirPin": -1,
            "stepPin": -1,
            "enablePin": -1
        }
    ],
    "manualConfig": "",
    "zHomeMinTemperature": 0,
    "zHomeXPos": 999999,
    "zHomeYPos": 999999,
    "zHomeHeatHeight": 20,
    "zHomeHeatAll": "1",
    "zProbeZOffsetMode": 0,
    "zProbeZOffset": 0,
    "uiBedCoating": "1",
    "langEN": "1",
    "langDE": "1",
    "langNL": "0",
    "langPT": "1",
    "langIT": "1",
    "langES": "1",
    "langFI": "0",
    "langSE": "0",
    "langFR": "1",
    "langCZ": "0",
    "langPL": "1",
    "langTR": "1",
    "interpolateAccelerationWithZ": 0,
    "accelerationFactorTop": 100,
    "bendingCorrectionA": 0,
    "bendingCorrectionB": 0,
    "bendingCorrectionC": 0,
    "preventZDisableOnStepperTimeout": "0",
    "supportLaser": "1",
    "laserPin": "HEATER_0_PIN",
    "laserOnHigh": "1",
    "defaultPrinterMode": 1,
    "supportCNC": "0",
    "cncWaitOnEnable": 300,
    "cncWaitOnDisable": 0,
    "cncEnablePin": -1,
    "cncEnableWith": "1",
    "cncDirectionPin": -1,
    "cncDirectionCW": "1",
    "startupGCode": "",
    "jsonOutput": "0",
    "bedLevelingMethod": 0,
    "bedCorrectionMethod": 0,
    "bedLevelingGridSize": 5,
    "bedLevelingRepetitions": 5,
    "bedMotor1X": 0,
    "bedMotor1Y": 0,
    "bedMotor2X": 200,
    "bedMotor2Y": 0,
    "bedMotor3X": 100,
    "bedMotor3Y": 200,
    "zProbeRequiresHeating": "0",
    "zProbeMinTemperature": 150,
    "adcKeypadPin": -1,
    "sharedExtruderHeater": "0",
    "extruderSwitchXYSpeed": 100,
    "dualXAxis": "0",
    "boardFanSpeed": 255,
    "hasMAX6675": false,
    "hasMAX31855": false,
    "hasGeneric1": false,
    "hasGeneric2": false,
    "hasGeneric3": false,
    "hasUser0": false,
    "hasUser1": false,
    "hasUser2": false,
    "numExtruder": 0,
    "version": 92.9,
    "primaryPortName": ""
}
========== End configuration string ==========

*/