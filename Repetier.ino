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

    This firmware is a nearly complete rewrite of the sprinter firmware
    by kliment (https://github.com/kliment/Sprinter)
    which based on Tonokip RepRap firmware rewrite based off of Hydra-mmm firmware.

    Main author: repetier

*/

//这里是协议介绍，采用GNU许可

/**
\mainpage Repetier-Firmware for Arduino based RepRaps
<CENTER>Copyright &copy; 2011-2013 by repetier
</CENTER>

\section Intro Introduction 简介


\section GCodes Implemented GCodes  部分g代码实现参考这里的g代码描述

 look here for descriptions of gcodes: http://linuxcnc.org/handbook/gcode/g-code.html
 and http://objects.reprap.org/wiki/Mendel_User_Manual:_RepRapGCodes

Implemented Codes

//G0和G1完全等价，直线运动
- G0  -> G1 
- G1  - Coordinated Movement X Y Z E, S1 disables boundary check, S0 enables it
//延时S秒或者P毫秒
- G4  - Dwell S<seconds> or P<milliseconds> 

//数据设置，根据储存的设置。这里是回抽
- G10 S<1 = 长回抽, 0 = 短回抽 = 默认> 根据存储的设置回抽丝料
- G10 S<1 = long retract, 0 = short retract = default> retracts filament according to stored setting
- G11 S<1 = 长回抽, 0 = 短回抽 = 默认> = 撤销回抽根据存储的设置
- G11 S<1 = long retract, 0 = short retract = default> = Undo retraction according to stored setting

//子程序调用
- G20 - Units for G0/G1 are inches.
- G21 - Units for G0/G1 are mm.

//归零所有轴或指定轴。
- G28 - Home all axis or named axis.
//在三个定义的探针点处进行 Z 探针。S = 1 测量平均 zHeight，S = 2 存储平均 zHeight
- G29 S<0..2> - Z-Probe at the 3 defined probe points. S = 1 measure avg. zHeight, S = 2 store avg zHeight
//S<0…2> - 在三个定义的探针点处进行 Z 探针。S = 1 测量平均 zHeight，S = 2 存储平均 zHeight
- G30 P<0..3> - Single z-probe at current position P = 1 first measurement, P = 2 Last measurement P = 0 or 3 first and last measurement
//写入探针传感器的信号
- G31 - Write signal of probe sensor
//自动平整打印床。S = 1 测量 zLength，S = 2 测量并存储新的 zLength
- G32 S<0..2> P<0..1> - Autolevel print bed. S = 1 measure zLength, S = 2 Measure and store new zLength
//使用绝对坐标
- G90 - Use absolute coordinates
//使用相对坐标
- G91 - Use relative coordinates
//将当前位置设置为给定坐标
- G92 - Set current position to coordinates given
//将挤出机偏移位置设置为 0 - 校准时需要与 G132 配合使用
- G131 - set extruder offset position to 0 - needed for calibration with G132
//校准终点位置。在调用 G131 并将挤出机支架居中之后调用此命令。
- G132 - calibrate endstop positions. Call this, after calling G131 and after centering the extruder holder.
//测量达到最大终点的步数。可以用来检测步进电机是否丢步。
- G133 - measure steps until max endstops for deltas. Can be used to detect lost steps within tolerances of endstops.
//校准喷嘴高度差（需要在喷嘴中有 z 探针！）Px = 参考挤出机，Sx = 只测量挤出机 x 相对于参考挤出机，Zx = 添加到测量的 z 距离上以校正 Sx。
- G134 Px Sx Zx - Calibrate nozzle height difference (need z probe in nozzle!) Px = reference extruder, Sx = only measure extrude x against reference, Zx = add to measured z distance for Sx for correction.

RepRap M Codes

//设置挤出机目标温度
- M104 - Set extruder target temp
//读取当前温度
- M105 - Read current temp
// 风扇开启速度 = 0…255, P = 0 或 1, 0 是默认值并且可以省略
- M106 S<speed> P<fan> - Fan on speed = 0..255, P = 0 or 1, 0 is default and can be omitted
//风扇关闭, P = 0 或 1, 0 是默认值并且可以省略
- M107 P<fan> - Fan off, P = 0 or 1, 0 is default and can be omitted
//等待挤出机当前温度达到目标温度。
- M109 - Wait for extruder current temp to reach target temp.
//显示当前位置
- M114 - Display current position

Custom M Codes

- M20  - List SD card
- M21  - Init SD card
- M22  - Release SD card
- M23  - Select SD file (M23 filename.g)
- M24  - Start/resume SD print 开始/恢复 SD 打印
- M25  - Pause SD print  暂停 SD 打印
- M26  - Set SD position in bytes (M26 S12345) 设置 SD 字节位置 (M26 S12345)
- M27  - Report SD print status 报告 SD 打印状态
- M28  - Start SD write (M28 filename.g) 开始 SD 写入 (M28 filename.g)
- M29  - Stop SD write 停止 SD 写入
- M30 <filename> - Delete file on sd card 删除 sd 卡上的文件
- M32 <dirname> create subdirectory 创建子目录
- M42 P<pin number> S<value 0..255> - Change output of pin P to S. Does not work on most important pins. 改变引脚 P 的输出为 S. 不适用于大多数重要引脚。
- M80  - Turn on power supply 打开电源供应器
- M81  - Turn off power supply 关闭电源供应器
- M82  - Set E codes absolute (default) 设置 E 码为绝对值 (默认)
- M83  - Set E codes relative while in Absolute Coordinates (G90) mode 在绝对坐标 (G90) 模式下设置 E 码为相对值
//禁用步进电机直到下一次移动， 或者使用 S<秒> 指定一个不活动超时时间，在超时后步进电机将被禁用。S0 禁用超时。
- M84  - Disable steppers until next move, 
        or use S<seconds> to specify an inactivity timeout, after which the steppers will be disabled.  S0 to disable the timeout.
//使用参数 S<秒> 设置不活动关闭定时器。要禁用设置为零 (默认)
- M85  - Set inactivity shutdown timer with parameter S<seconds>. To disable set zero (default)
//设置 axisStepsPerMM 和 G92 的相同语法
- M92  - Set axisStepsPerMM - same syntax as G92
//禁用给定轴的电机 S 秒 (默认 10)。
- M99 S<delayInSec> X0 Y0 Z0 - Disable motors for S seconds (default 10) for given axis.
//设置温度而不等待。P1 = 等待移动完成，F1 = 温度达到第一次时响铃
- M104 S<temp> T<extruder> P1 F1 - Set temperature without wait. P1 = wait for moves to finish, F1 = beep when temp. reached first time
//- 获取温度。如果添加了 X0，则还会写入原始模拟值。
- M105 X0 - Get temperatures. If X0 is added, the raw analog values are also written.
- M112 - Emergency kill 紧急停止
- M115- Capabilities string 功能字符串
- M116 - Wait for all temperatures in a +/- 1 degree range  等待所有温度在 +/- 1 度范围内
- M117 <message> - Write message in status row on lcd  <消息> 在液晶显示屏上的状态行上写消息
- M119 - Report endstop status 报告终点状态
//设置热床目标温度，F1 在温度第一次达到时响铃
- M140 S<temp> F1 - Set bed target temp, F1 makes a beep when temperature is reached the first time
//设置混合挤出机驱动的重量
- M163 S<extruderNum> P<weight>  - Set weight for this mixing extruder drive
//S<虚拟编号> P<0 = 不存储到 eeprom,1 = 存储到 eeprom> - 将重量存储为虚拟挤出机 S
- M164 S<virtNum> P<0 = dont store eeprom,1 = store to eeprom> - Store weights as virtual extruder S
//等待热床当前温度达到目标温度。 S60代表等待热床达到60
- M190 - Wait for bed current temp to reach target temp.
//T<挤出机> D<直径> - 使用体积挤出。设置 D0 或省略 D 来禁用体积挤出。省略 T 为当前挤出机。
- M200 T<extruder> D<diameter> - Use volumetric extrusion. Set D0 or omit D to disable volumetric extr. Omit T for current extruder.
// 设置打印移动的最大加速度 (M201 X1000 Y1000)
- M201 - Set max acceleration in units/s^2 for print moves (M201 X1000 Y1000)
//设置空程移动的最大加速度 (M202 X1000 Y1000)
- M202 - Set max acceleration in units/s^2 for travel moves (M202 X1000 Y1000)
//设置温度监控为 Sx
- M203 - Set temperture monitor to Sx
//设置 PID 参数 X => Kp Y => Ki Z => Kd S<挤出机> 默认为当前挤出机。NUM_EXTRUDER=加热床
- M204 - Set PID parameter X => Kp Y => Ki Z => Kd S<extruder> Default is current extruder. NUM_EXTRUDER=Heated bed
//输出 EEPROM 设置
- M205 - Output EEPROM settings
//设置 EEPROM 值
- M206 - Set EEPROM value
//X<XY 抖动> Z<Z 抖动> E<挤出机抖动> - 改变当前抖动值，但不存储到 eeprom 中。
- M207 X<XY jerk> Z<Z Jerk> E<ExtruderJerk> - Changes current jerk values, but do not store them in eeprom.
//启用/禁用自动回抽
- M209 S<0/1> - Enable/disable autoretraction
//S<进给率百分比> - 增加/减少给定的进给率
- M220 S<Feedrate multiplier in percent> - Increase/decrease given feedrate
//S<挤出流量百分比> - 增加/减少给定的流量
- M221 S<Extrusion flow multiplier in percent> - Increase/decrease given flow rate
// P<引脚> S<状态 0/1> - 等待引脚获取状态 S. 添加 X0 以初始化为输入无上拉电阻，X1 为输入有上拉电阻。
- M226 P<pin> S<state 0/1> - Wait for pin getting state S. Add X0 to init as input without pullup and X1 for input with pullup.
//S<OPS_MODE> X<最小距离> Y<回抽> Z<反向间隙> F<回抽移动> - 设置 OPS 参数
- M231 S<OPS_MODE> X<Min_Distance> Y<Retract> Z<Backlash> F<ReatrctMove> - Set OPS parameter
//读取并重置最大提前值
- M232 - Read and reset max. advance values
//X<AdvanceK> Y<AdvanceL> - 将临时提前 K 值设置为 X，线性项 advanceL 设置为 Y
- M233 X<AdvanceK> Y<AdvanceL> - Set temporary advance K-value to X and linear term advanceL to Y
//测量从归零终点开始的 Z 步数 (Delta 打印机)。S0 - 重置，S1 - 打印，S2 - 存储到 Z 长度 (如果启用了 EEPROM 也存储到 EEPROM)
- M251 Measure Z steps from homing stop (Delta printers). S0 - Reset, S1 - Print, S2 - Store to Z length (also EEPROM if enabled)
//设置 ditto 打印模式。模式: 0 = 关闭，1 = 1 个额外的挤出机，2 = 2 个额外的挤出机，3 = 3 个额外的挤出机
- M280 S<mode> - Set ditto printing mode. mode: 0 = off, 1 = 1 extra extruder, 2 = 2 extra extruder, 3 = 3 extra extruders
//测试看门狗是否运行和工作。使用 M281 X0 在 AVR 板上禁用看门狗。有时对于有旧引导程序的板子来说需要这样做才能重新刷写。
- M281 Test if watchdog is running and working. Use M281 X0 to disable watchdog on AVR boards. Sometimes needed for boards with old bootloaders to allow reflashing.
//M300 S<频率> P<持续毫秒数> 播放频率
- M300 S<Frequency> P<DurationMillis> play frequency
//M302 S<0 或 1> - 允许冷挤出。没有 S 参数则允许。S1 则禁止。
- M302 S<0 or 1> - allow cold extrusion. Without S parameter it will allow. S1 will disallow.
//M303 P<挤出机/热床> S<打印温度> X0 R<重复次数>- 自动检测 pid 值。使用 P<NUM_EXTRUDER> 来表示加热床。X0 将结果保存在 EEPROM 中。R 是循环次数。
- M303 P<extruder/bed> S<printTemerature> X0 R<Repetitions>- Autodetect pid values. Use P<NUM_EXTRUDER> for heated bed. X0 saves result in EEPROM. R is number of cycles.
//S<0/1> - 激活自动平整，S1 将它存储在 eeprom 中
- M320 S<0/1> - Activate autolevel, S1 stores it in eeprom
//S<0/1> - 取消自动平整，S1 将它存储在 eeprom 中
- M321 S<0/1> - Deactivate autolevel, S1 stores it in eeprom
- M322 - Reset autolevel matrix 重置自动平整矩阵
//S0/S1 启用禁用失真校正 P0 = 不永久，P1 = 永久 = 默认值
- M323 S0/S1 enable disable distortion correction P0 = not permanent, P1 = permanent = default
//P<舵机编号> S<pulseInUS> R<自动关闭毫秒数>: 舵机编号 = 0…3, 舵机由一个通常在 500 和 2500 之间的脉冲控制，1500ms 是中心位置。0 关闭舵机。R 允许一段时间后自动关闭。
- M340 P<servoId> S<pulseInUS> R<autoOffIn ms>: servoID = 0..3, Servos are controlled by a pulse with normally between 500 and 2500 with 1500ms in center position. 0 turns servo off. R allows automatic disabling after a while.
//M350 S<mstepsAll> X<mstepsX> Y<mstepsY> Z<mstepsZ> E<mstepsE0> P<mstespE1>: 在 RAMBO 板上设置微步进
- M350 S<mstepsAll> X<mstepsX> Y<mstepsY> Z<mstepsZ> E<mstepsE0> P<mstespE1> : Set microstepping on RAMBO board
//M355 S<0/1> - 打开/关闭外壳灯光，没有 S = 报告状态
- M355 S<0/1> - Turn case light on/off, no S = report status
- M360 - show configuration 显示配置信息
- M400 - Wait until move buffers empty. 等待移动缓冲区为空。
- M401 - Store x, y and z position. 存储 x, y 和 z 的位置。
//转到存储的位置。如果指定了 X, Y 或 Z，则只使用这些坐标。F 改变该移动的进给率。
- M402 - Go to stored position. If X, Y or Z is specified, only these coordinates are used. F changes feedrate fo rthat move.
//报告打印机模式
- M450 - Reports printer mode
- M451 - Set printer mode to FFF 将打印机模式设置为 FFF
- M452 - Set printer mode to laser 将打印机模式设置为激光器
- M453 - Set printer mode to CNC 将打印机模式设置为 CNC
//M460 X<最低温度> Y<最高温度>: 设置热敏电阻控制风扇的温度范围
- M460 X<minTemp> Y<maxTemp> : Set temperature range for thermistor controlled fan
- M500 Store settings to EEPROM 将设置存储到 EEPROM
- M501 Load settings from EEPROM 从 EEPROM 加载设置
//将设置重置为 configuration.h 中的值。不会将值存储在 EEPROM 中！
- M502 Reset settings to the one in configuration.h. Does not store values in EEPROM! 
- M513 - Clear all jam marker.清除所有堵塞标记。
- M600 Change filament 更换丝料
//S<1/0> - 暂停挤出器。暂停的挤出器会关闭加热器和电机。恢复后重新加热挤出器到旧温度。
- M601 S<1/0> - Pause extruders. Paused extrudes disable heaters and motor. Unpausing reheats extruder to old temp.
//M602 S<1/0> P<1/0>- 调试堵塞控制 (S) 禁用堵塞控制 (P)。如果启用它将记录信号变化并且不会触发堵塞错误！
- M602 S<1/0> P<1/0>- Debug jam control (S) Disable jam control (P). If enabled it will log signal changes and will not trigger jam errors!
//M908 P<地址> S<值>: 设置 digipot (RAMBO 板) 的步进电流
- M908 P<address> S<value> : Set stepper current for digipot (RAMBO board)
//M999 继续从致命错误中恢复。M999 S1 将创建一个致命错误用于测试。
- M999 - Continue from fatal error. M999 S1 will create a fatal error for testing.
*/

#include "Repetier.h"
#include <SPI.h>

#if UI_DISPLAY_TYPE == DISPLAY_ARDUINO_LIB //表示使用 Arduino 库来控制显示器
//这是 Arduino 的液晶显示库，如果你使用的是液晶显示器，可以取消下一行的注释 
//#include <LiquidCrystal.h> // Uncomment this if you are using liquid crystal library
#endif

void setup()
{
    Printer::setup();
}

void loop()
{
    //调用 Commands 类的 commandLoop() 方法，进行 GCode 命令的解析和执行 }
    Commands::commandLoop();
}








