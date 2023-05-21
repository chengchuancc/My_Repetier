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

  Functions in this file are used to communicate using ascii or repetier protocol.
*/

#ifndef COMMANDS_H_INCLUDED
#define COMMANDS_H_INCLUDED

class Commands
{
public:
    static void commandLoop();//命令循环函数，用于从串口或者SD卡读取并解析g代码
    static void checkForPeriodicalActions(bool allowNewMoves);//检查周期性动作函数，用于执行温度控制、看门狗、UI更新等任务，参数表示是否允许新的移动命令加入缓冲区
    static void processArc(GCode *com);//处理圆弧函数，用于将g代码中的圆弧命令转换为直线插补命令，参数为g代码指针
    static void processGCode(GCode *com);//处理g代码函数，用于根据g代码中的G命令执行相应的操作，参数为g代码指针
    static void processMCode(GCode *com);//处理m代码函数，用于根据g代码中的M命令执行相应的操作，参数为g代码指针
    static void executeGCode(GCode *com);//执行g代码函数，用于将g代码中的坐标和速度参数转换为实际的移动命令，并加入缓冲区等待执行，参数为g代码指针
    static void waitUntilEndOfAllMoves();//等待所有移动结束函数，用于阻塞当前线程直到缓冲区中的所有移动命令都执行完毕
    static void waitUntilEndOfAllBuffers();//等待所有缓冲区结束函数，用于阻塞当前线程直到缓冲区中的所有命令都执行完毕
    static void printCurrentPosition(FSTRINGPARAM(s));//打印当前位置函数，用于向串口输出当前挤出头的坐标和步进电机的位置，参数为字符串常量指针
    static void printTemperatures(bool showRaw = false);//打印温度函数，用于向串口输出当前挤出头和热床的温度，参数表示是否显示原始值，默认为否
    //打印温度函数，用于向串口输出当前挤出头和热床的温度，参数表示是否显示原始值，默认为否
    static void setFanSpeed(int speed, bool immediately = false); /// Set fan speed 0..255
    //置第二个风扇速度函数，参数为速度值。该函数只在有第二个风扇时有效，并且总是立即生效
    static void setFan2Speed(int speed); /// Set fan speed 0..255
    // 改变进给速率倍数函数，参数为百分比因子。该函数会影响后续移动命令的速度，并且总是立即生效[^2^][2]
    static void changeFeedrateMultiply(int factorInPercent);// 改变进给速率倍数函数，参数为百分比因子。该函数会影响后续移动命令的速度，并且总是立即生效[^2^][2]
    static void changeFlowrateMultiply(int factorInPercent);// 改变流量倍数函数，参数为百分比因子。该函数会影响后续挤出命令的流量，并且总是立即生效
    static void reportPrinterUsage(); // 报告打印机使用情况函数，用于向串口输出打印机的累计打印时间和耗材长度
    static void emergencyStop();// 紧急停止函数，用于在发生错误或者收到紧急停止命令时停止打印机的所有动作，并重置固件状态
    static void checkFreeMemory();// 检查空闲内存函数，用于向串口输出当前可用内存和最低可用内存
    static void writeLowestFreeRAM(); // 写入最低空闲内存函数，用于将最低可用内存写入EEPROM中保存
private:
    static int lowestRAMValue;
    static int lowestRAMValueSend;
};

#endif // COMMANDS_H_INCLUDED
