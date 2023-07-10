#ifndef EVENTS_H_INCLUDED
#define EVENTS_H_INCLUDED

/*
Event system in a nutshell:

All printers are different and my need additions in th eone or other place.
It is not very convenient to add these code parts across the firmware. For this
reason repetier-firmware uses a simple event system that comes at no cost if
a event is not used.

- simple: Only one subscriber is possible
- cost effective: Macros work as event caller. By default all macros are empty

How to use the system:

1. In Configuration.h add
#define CUSTOM_EVENTS
2. Add a file "CustomEvents.h" which overrides all event macros you need.
   It shoudl also include the function declarations used.
3. Add a file "CustomEventsImpl.h" which includes all function definitions.
   Also it is named .h it will be included inside a cpp file only once.
   This is to compile only when selected and still keep ArduinoIDE happy.

Each of the following events describe the parameter and when it is called.
事件系统简介:

所有的打印机都有不同的特点，可能需要在某些地方添加额外的代码。
在固件中分散添加这些代码并不方便。为了解决这个问题，repetier-firmware使用了一个简单的事件系统，如果一个事件没有被使用，就不会有任何开销。

- 简单: 只能有一个订阅者
- 低成本: 宏作为事件调用者。默认情况下，所有的宏都是空的

如何使用这个系统:

1. 在Configuration.h中添加
#define CUSTOM_EVENTS
2. 添加一个文件"CustomEvents.h"，它覆盖了你需要的所有事件宏。
 它还应该包含使用的函数声明。
3. 添加一个文件"CustomEventsImpl.h"，它包含了所有函数定义。
 尽管它也是以.h结尾，但它只会在一个cpp文件中被包含一次。
 这是为了只在选择时编译，并且保持ArduinoIDE的兼容性。

以下每个事件都描述了参数和调用时机。
*/

// Catch heating events. id is extruder id or -1 for heated bed.
//捕获加热事件，id是挤出机id或-1表示加热床。
#define EVENT_WAITING_HEATER(id) {}
#define EVENT_HEATING_FINISHED(id) {}

// This gets called every 0.1 second
// 这个每0.1秒被调用一次
#define EVENT_TIMER_100MS {}
// This gets called every 0.5 second
// 这个每0.5秒被调用一次
#define EVENT_TIMER_500MS {}
// Gets called on a regular basis as time allows
// 在时间允许的情况下定期被调用
#define EVENT_PERIODICAL {}
// Gets called when kill gets called. only_steppes = true -> we only want to disable steppers, not everything.
// 当kill被调用时被调用。only_steppes = true -> 我们只想禁用步进电机，而不是所有的东西。
#define EVENT_KILL(only_steppers) {}
// Gets called when a jam was detected.
// 当检测到卡塞时被调用。
#define EVENT_JAM_DETECTED {}
// Gets called every time the jam detection signal switches. Steps are the extruder steps since last change.
// 每次卡塞检测信号切换时被调用。Steps是自上次变化以来的挤出机步数。
#define EVENT_JAM_SIGNAL_CHANGED(extruderId,steps) {}
// Gets called if a heater decoupling is detected.
// 当检测到加热器解耦时被调用。
#define EVENT_HEATER_DECOUPLED(id) {}
// Gets called if a missing/shorted thermistor is detected.
// 当检测到缺失/短路的温度计时被调用。
#define EVENT_HEATER_DEFECT(id) {}
// Gets called if a action in ui.cpp okAction gets executed.
// 当在ui.cpp中执行一个动作okAction时被调用。
#define EVENT_START_UI_ACTION(shortAction) {}
// Gets called if a nextPrevius actions gets executed.
// 当执行一个nextPrevius动作时被调用。
#define EVENT_START_NEXTPREVIOUS(action,increment) {}
// Gets called before a move is queued. Gives the ability to limit moves.
// 在排队移动之前被调用。可以用来限制移动。
#define EVENT_CONTRAIN_DESTINATION_COORDINATES
// Gets called when a fatal error occurs and all actions should be stopped
// 当发生致命错误并且所有动作都应该停止时被调用
#define EVENT_FATAL_ERROR_OCCURED
// Gets called after a M999 to continue from fatal errors
// 在M999之后被调用，从致命错误中继续
#define EVENT_CONTINUE_FROM_FATAL_ERROR

// Called to initialize laser pins. Return false to prevent default initialization.
// 被调用来初始化激光器引脚。返回false以阻止默认初始化。
#define EVENT_INITALIZE_LASER true
// Set laser to intensity level 0 = off, 255 = full. Return false if you have overridden the setting routine.
// with true the default solution will set it as digital value.
// 将激光器设置为强度级别 0 = 关闭, 255 = 全开。如果你已经覆盖了设置程序，返回false。
// 如果返回true，则默认解决方案将其设置为数字值。
#define EVENT_SET_LASER(intensity) true

// Called to initialize CNC pins. Return false to prevent default initialization.
// 被调用来初始化CNC引脚。返回false以阻止默认初始化。
#define EVENT_INITALIZE_CNC true
// Turn off spindle
// 关闭主轴
#define EVENT_SPINDLE_OFF true
// Turn spindle clockwise
// 顺时针转动主轴
#define EVENT_SPINDLE_CW(rpm) true
// Turn spindle counter clockwise
// 逆时针转动主轴
#define EVENT_SPINDLE_CCW(rpm) true

// Allow adding new G and M codes. To implement it create a function
// bool eventUnhandledGCode(GCode *com)
// that returns true if it handled the code, otherwise false.
// Event define would then be
// #define EVENT_UNHANDLED_G_CODE(c) eventUnhandledGCode(c)
// 允许添加新的G和M代码。要实现它，创建一个函数
// bool eventUnhandledGCode(GCode *com)
// 如果它处理了代码，就返回true，否则返回false。
// 事件定义将是
// #define EVENT_UNHANDLED_G_CODE(c) eventUnhandledGCode(c)
#define EVENT_UNHANDLED_G_CODE(c) false
#define EVENT_UNHANDLED_M_CODE(c) false

// This gets called every time the user has saved a value to eeprom
// or any other reason why dependent values may need recomputation.
// 每次用户将一个值保存到eeprom时被调用
// 或者任何其他原因导致依赖值可能需要重新计算。
#define EVENT_UPDATE_DERIVED {}

// This gets called after the basic firmware functions have initialized.
// Use this to initalize your hardware etc.
// 在基本固件功能初始化之后被调用。
// 使用这个来初始化你的硬件等等。
#define EVENT_INITIALIZE {}

#endif // EVENTS_H_INCLUDED
