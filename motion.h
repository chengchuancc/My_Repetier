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
#
  Functions in this file are used to communicate using ascii or repetier protocol.
*/

#ifndef MOTION_H_INCLUDED // 如果没有定义MOTION_H_INCLUDED宏
#define MOTION_H_INCLUDED // 定义MOTION_H_INCLUDED宏，避免重复包含

/** 标记一个新移动的第一步 */
#define FLAG_WARMUP 1 // 定义FLAG_WARMUP宏为1，表示移动开始时的预热阶段
#define FLAG_NOMINAL 2 // 定义FLAG_NOMINAL宏为2，表示移动达到最大速度时的匀速阶段
#define FLAG_DECELERATING 4 // 定义FLAG_DECELERATING宏为4，表示移动结束时的减速阶段
#define FLAG_ACCELERATION_ENABLED 8 // 定义FLAG_ACCELERATION_ENABLED宏为8，表示是否启用加速度控制（未使用）
#define FLAG_CHECK_ENDSTOPS 16 // 定义FLAG_CHECK_ENDSTOPS宏为16，表示是否检查限位开关
#define FLAG_ALL_E_MOTORS 32 // 定义FLAG_ALL_E_MOTORS宏为32，表示对于混合挤出机移动所有电机而不是选定电机
#define FLAG_SKIP_DEACCELERATING 64 // 定义FLAG_SKIP_DEACCELERATING宏为64，表示是否跳过减速阶段（未使用）
#define FLAG_BLOCKED 128 // 定义FLAG_BLOCKED宏为128，表示是否阻塞移动

/** 步进参数是否已计算 */
#define FLAG_JOIN_STEPPARAMS_COMPUTED 1 // 定义FLAG_JOIN_STEPPARAMS_COMPUTED宏为1，表示是否已计算步进参数
/** 右边速度是固定的。不要检查这个块或左边的任何块。 */
#define FLAG_JOIN_END_FIXED 2 // 定义FLAG_JOIN_END_FIXED宏为2，表示右边速度是固定的
/** 左边速度是固定的。不要检查左边的块。 */
#define FLAG_JOIN_START_FIXED 4 // 定义FLAG_JOIN_START_FIXED宏为4，表示左边速度是固定的
/** 在移动开始时开始丝料回缩 */
#define FLAG_JOIN_START_RETRACT 8 // 定义FLAG_JOIN_START_RETRACT宏为8，表示在移动开始时开始丝料回缩
/** 在结束移动之前，等待丝料回推 */
#define FLAG_JOIN_END_RETRACT 16 // 定义FLAG_JOIN_END_RETRACT宏为16，表示在结束移动之前，等待丝料回推
/** 禁用这条线的回缩 */
#define FLAG_JOIN_NO_RETRACT 32 // 定义FLAG_JOIN_NO_RETRACT宏为32，表示禁用这条线的回缩
/** 等待挤出机完成它的上升运动 */
#define FLAG_JOIN_WAIT_EXTRUDER_UP 64 // 定义FLAG_JOIN_WAIT_EXTRUDER_UP宏为64，表示等待挤出机完成它的上升运动
/** 等待挤出机完成它的下降运动 */
#define FLAG_JOIN_WAIT_EXTRUDER_DOWN 128 // 定义FLAG_JOIN_WAIT_EXTRUDER_DOWN宏为128，表示等待挤出机完成它的下降运动
// 打印相关数据
#if NONLINEAR_SYSTEM // 如果定义了NONLINEAR_SYSTEM宏，表示使用非线性系统（如三角洲或SCARA）
// Allow the delta cache to store segments for every line in line cache. Beware this gets big ... fast.

class PrintLine;
typedef struct
{
    flag8_t dir; 									///< Direction of delta movement.
    uint16_t deltaSteps[TOWER_ARRAY];   				    ///< Number of steps in move.
    inline bool checkEndstops(PrintLine *cur,bool checkall);
    inline void setXMoveFinished()
    {
        dir &= ~XSTEP;
    }
    inline void setYMoveFinished()
    {
        dir &= ~YSTEP;
    }
    inline void setZMoveFinished()
    {
        dir &= ~ZSTEP;
    }
    inline void setXYMoveFinished()
    {
        dir &= ~XY_STEP;
    }
    inline bool isXPositiveMove()
    {
        return (dir & X_STEP_DIRPOS) == X_STEP_DIRPOS;
    }
    inline bool isXNegativeMove()
    {
        return (dir & X_STEP_DIRPOS) == XSTEP;
    }
    inline bool isYPositiveMove()
    {
        return (dir & Y_STEP_DIRPOS) == Y_STEP_DIRPOS;
    }
    inline bool isYNegativeMove()
    {
        return (dir & Y_STEP_DIRPOS) == YSTEP;
    }
    inline bool isZPositiveMove()
    {
        return (dir & Z_STEP_DIRPOS) == Z_STEP_DIRPOS;
    }
    inline bool isZNegativeMove()
    {
        return (dir & Z_STEP_DIRPOS) == ZSTEP;
    }
    inline bool isEPositiveMove()
    {
        return (dir & E_STEP_DIRPOS) == E_STEP_DIRPOS;
    }
    inline bool isENegativeMove()
    {
        return (dir & E_STEP_DIRPOS) == ESTEP;
    }
    inline bool isXMove()
    {
        return (dir & XSTEP);
    }
    inline bool isYMove()
    {
        return (dir & YSTEP);
    }
    inline bool isXOrYMove()
    {
        return dir & XY_STEP;
    }
    inline bool isZMove()
    {
        return (dir & ZSTEP);
    }
    inline bool isEMove()
    {
        return (dir & ESTEP);
    }
    inline bool isEOnlyMove()
    {
        return (dir & XYZE_STEP)==ESTEP;
    }
    inline bool isNoMove()
    {
        return (dir & XYZE_STEP) == 0;
    }
    inline bool isXYZMove()
    {
        return dir & XYZ_STEP;
    }
    inline bool isMoveOfAxis(uint8_t axis)
    {
        return (dir & (XSTEP<<axis));
    }
    inline void setMoveOfAxis(uint8_t axis)
    {
        dir |= XSTEP << axis;
    }
    inline void setPositiveMoveOfAxis(uint8_t axis)
    {
        dir |= X_STEP_DIRPOS << axis;
    }
    inline void setPositiveDirectionForAxis(uint8_t axis)
    {
        dir |= X_DIRPOS << axis;
    }
} NonlinearSegment;
extern uint8_t lastMoveID;
#endif
class UIDisplay;
class PrintLine   // RAM usage: 24*4+15 = 113 Byte
{
    friend class UIDisplay;
#if CPU_ARCH == ARCH_ARM
    static volatile bool nlFlag;
#endif
public:
    static ufast8_t linesPos; // Position for executing line movement // 执行线段移动的位置
    static PrintLine lines[]; // 静态数组，存储缓存的线段对象
    static ufast8_t linesWritePos; // Position where we write the next cached line move // 写入下一个缓存线段移动的位置
    ufast8_t joinFlags; // 用于标记线段之间的连接状态和特性的标志位
    volatile ufast8_t flags; // 用于标记线段的移动状态和特性的标志位
    uint8_t secondSpeed; // for laser intensity or fan control // 用于激光强度或风扇控制的第二速度
private:
    fast8_t primaryAxis; // 主要的移动轴，取决于哪个轴的步数最多
    ufast8_t dir;                       ///< Direction of movement. 1 = X+, 2 = Y+, 4= Z+, values can be combined. // 移动方向。1 = X+，2 = Y+，4 = Z+，值可以组合。
    int32_t timeInTicks; // 线段移动所需的总时间，以时钟周期为单位
    int32_t delta[E_AXIS_ARRAY];                  ///< Steps we want to move. // 我们想要移动的步数。
    int32_t error[E_AXIS_ARRAY];                  ///< Error calculation for Bresenham algorithm // Bresenham算法的误差计算
    float speedX;                   ///< Speed in x direction at fullInterval in mm/s // 在fullInterval时x方向的速度，以mm/s为单位
    float speedY;                   ///< Speed in y direction at fullInterval in mm/s // 在fullInterval时y方向的速度，以mm/s为单位
    float speedZ;                   ///< Speed in z direction at fullInterval in mm/s // 在fullInterval时z方向的速度，以mm/s为单位
    float speedE;                   ///< Speed in E direction at fullInterval in mm/s // 在fullInterval时E方向的速度，以mm/s为单位
    float fullSpeed;                ///< Desired speed mm/s // 期望的速度，以mm/s为单位
    float invFullSpeed;             ///< 1.0/fullSpeed for faster computation // 1.0/fullSpeed，用于更快的计算
    float accelerationDistance2;    ///< Real 2.0*distance*acceleration mm²/s² // 真实的2.0*distance*acceleration，以mm²/s²为单位
    float maxJunctionSpeed;         ///< Max. junction speed between this and next segment // 这个和下一个线段之间的最大连接速度
    float startSpeed;               ///< Starting speed in mm/s // 起始速度，以mm/s为单位
    float endSpeed;                 ///< Exit speed in mm/s // 结束速度，以mm/s为单位
    float minSpeed;
    float distance; // 线段移动的距离，以mm为单位
#if NONLINEAR_SYSTEM // 如果定义了NONLINEAR_SYSTEM宏，表示使用非线性系统（如三角洲或SCARA）
    uint8_t numNonlinearSegments;		///< Number of delta segments left in line. Decremented by stepper timer. // 线段中剩余的三角洲分段数。由步进定时器递减。
    uint8_t moveID;					///< ID used to identify moves which are all part of the same line // 用于识别属于同一条线的所有移动的ID
    int32_t numPrimaryStepPerSegment;	///< Number of primary Bresenham axis steps in each delta segment // 每个三角洲分段中主要Bresenham轴步数
    NonlinearSegment segments[DELTASEGMENTS_PER_PRINTLINE]; // 非线性分段数组，存储每个分段的位置和速度信息
#endif
    ticks_t fullInterval;     ///< interval at full speed in ticks/step. // 在全速时的间隔，以时钟周期/步为单位。
    uint16_t accelSteps;        ///< How much steps does it take, to reach the plateau. // 达到平台速度需要多少步。
    uint16_t decelSteps;        ///< How much steps does it take, to reach the end speed. // 达到结束速度需要多少步。
    uint32_t accelerationPrim; ///< Acceleration along primary axis // 沿着主要轴的加速度
    uint32_t fAcceleration;    ///< accelerationPrim*262144/F_CPU // 加速度乘以一个系数，用于更快的计算
    speed_t vMax;              ///< Maximum reached speed in steps/s. // 达到的最大速度，以步/秒为单位。
    speed_t vStart;            ///< Starting speed in steps/s. // 起始速度，以步/秒为单位。
    speed_t vEnd;              ///< End speed in steps/s // 结束速度，以步/秒为单位

#if USE_ADVANCE
#if ENABLE_QUADRATIC_ADVANCE
    int32_t advanceRate;               ///< Advance steps at full speed
    int32_t advanceFull;               ///< Maximum advance at fullInterval [steps*65536]
    int32_t advanceStart;
    int32_t advanceEnd;
#endif
    uint16_t advanceL;         ///< Recomputed L value
#endif
#ifdef DEBUG_STEPCOUNT
    int32_t totalStepsRemaining;
#endif
public:
    int32_t stepsRemaining;            ///< Remaining steps, until move is finished // 移动完成前剩余的步数
    static PrintLine *cur; // 静态指针，指向当前正在执行的线段对象
    static volatile ufast8_t linesCount; // Number of lines cached 0 = nothing to do // 缓存的线段数，0 = 没有要做的事情
    inline bool areParameterUpToDate()
    {
        return joinFlags & FLAG_JOIN_STEPPARAMS_COMPUTED; // 返回是否已计算步进参数的布尔值
    }
    inline void invalidateParameter()
    {
        joinFlags &= ~FLAG_JOIN_STEPPARAMS_COMPUTED; // 将步进参数计算标志位清零，表示需要重新计算
    }
    inline void setParameterUpToDate()
    {
        joinFlags |= FLAG_JOIN_STEPPARAMS_COMPUTED; // 将步进参数计算标志位置一，表示已经计算
    }
    inline bool isStartSpeedFixed()
    {
        return joinFlags & FLAG_JOIN_START_FIXED; // 返回起始速度是否固定的布尔值
    }
    inline void setStartSpeedFixed(bool newState)
    {
        joinFlags = (newState ? joinFlags | FLAG_JOIN_START_FIXED : joinFlags & ~FLAG_JOIN_START_FIXED); // 根据新状态设置起始速度固定标志位
    }
    inline void fixStartAndEndSpeed()
    {
        joinFlags |= FLAG_JOIN_END_FIXED | FLAG_JOIN_START_FIXED; // 将起始速度和结束速度固定标志位都置一，表示不需要调整
    }
    inline bool isEndSpeedFixed()
    {
        return joinFlags & FLAG_JOIN_END_FIXED; // 返回结束速度是否固定的布尔值
    }
    inline void setEndSpeedFixed(bool newState)
    {
        joinFlags = (newState ? joinFlags | FLAG_JOIN_END_FIXED : joinFlags & ~FLAG_JOIN_END_FIXED); // 根据新状态设置结束速度固定标志位
    }
    inline bool isWarmUp()
    {
        return flags & FLAG_WARMUP; // 返回是否是预热阶段的布尔值
    }
    inline uint8_t getWaitForXLinesFilled()
    {
        return primaryAxis; // 返回等待填充的线段数（用primaryAxis变量暂存）
    }
    inline void setWaitForXLinesFilled(uint8_t b)
    {
        primaryAxis = b; // 设置等待填充的线段数（用primaryAxis变量暂存）
    }
    inline bool isExtruderForwardMove()
    {
        return (dir & E_STEP_DIRPOS)==E_STEP_DIRPOS; // 返回挤出机是否是正向移动的布尔值（根据dir变量中的E_STEP_DIRPOS位判断）
    }
    inline void block()
    {
        flags |= FLAG_BLOCKED; // 将阻塞标志位置一，表示线段不能执行
    }
    inline void unblock()
    {
        flags &= ~FLAG_BLOCKED; // 将阻塞标志位清零，表示线段可以执行
    }
    inline bool isBlocked()
    {
        return flags & FLAG_BLOCKED; // 返回线段是否被阻塞的布尔值
    }
    inline bool isAllEMotors() {
        return flags & FLAG_ALL_E_MOTORS; // 返回是否移动所有挤出机电机的布尔值（对于混合挤出机）
    }
    inline bool isCheckEndstops()
    {
        return flags & FLAG_CHECK_ENDSTOPS; // 返回是否检查限位开关的布尔值
    }
    inline bool isNominalMove()
    {
        return flags & FLAG_NOMINAL; // 返回是否是匀速移动的布尔值
    }
    inline void setNominalMove()
    {
        flags |= FLAG_NOMINAL; // 将匀速移动标志位置一，表示达到最大速度
    }
	// 检查限位开关的函数，如果触发了限位开关，就停止相应的轴的移动
    inline void checkEndstops()
    {
        if(isCheckEndstops()) // 如果需要检查限位开关
        {
			Endstops::update(); // 更新限位开关的状态
            if(isXNegativeMove() && Endstops::xMin()) // 如果是X轴负向移动并且触发了X轴最小限位开关
                setXMoveFinished(); // 停止X轴的移动
            else if(isXPositiveMove() && Endstops::xMax()) // 如果是X轴正向移动并且触发了X轴最大限位开关
                setXMoveFinished(); // 停止X轴的移动
            if(isYNegativeMove() && Endstops::yMin()) // 如果是Y轴负向移动并且触发了Y轴最小限位开关
                setYMoveFinished(); // 停止Y轴的移动
            else if(isYPositiveMove() && Endstops::yMax()) // 如果是Y轴正向移动并且触发了Y轴最大限位开关
                setYMoveFinished(); // 停止Y轴的移动
#if FEATURE_Z_PROBE // 如果启用了Z探针功能
            if(Printer::isZProbingActive() && isZNegativeMove() && Endstops::zProbe()) // 如果正在进行Z探针活动并且是Z轴负向移动并且触发了Z探针
            {
                setZMoveFinished(); // 停止Z轴的移动
                Printer::stepsRemainingAtZHit = stepsRemaining; // 记录Z探针触发时剩余的步数
            }
            else
#endif
                if(isZNegativeMove() && Endstops::zMin()) // 如果是Z轴负向移动并且触发了Z轴最小限位开关
                {
                    setZMoveFinished(); // 停止Z轴的移动
                }
                else if(isZPositiveMove() && Endstops::zMax()) // 如果是Z轴正向移动并且触发了Z轴最大限位开关
                {
#if MAX_HARDWARE_ENDSTOP_Z // 如果启用了Z轴最大硬件限位开关
                    Printer::stepsRemainingAtZHit = stepsRemaining; // 记录Z轴最大限位开关触发时剩余的步数
#endif
                    setZMoveFinished(); // 停止Z轴的移动
                }
        }
#if FEATURE_Z_PROBE // 如果启用了Z探针功能
        else if(Printer::isZProbingActive() && isZNegativeMove()) { // 如果正在进行Z探针活动并且是Z轴负向移动
			Endstops::update(); // 更新限位开关的状态
			if(Endstops::zProbe()) // 如果触发了Z探针
			{
				setZMoveFinished(); // 停止Z轴的移动
				Printer::stepsRemainingAtZHit = stepsRemaining; // 记录Z探针触发时剩余的步数
			}
        }
#endif
    }

    inline void setXMoveFinished()
    {
#if DRIVE_SYSTEM==XY_GANTRY || DRIVE_SYSTEM==YX_GANTRY
        dir &= ~48;
#elif DRIVE_SYSTEM==XZ_GANTRY || DRIVE_SYSTEM==ZX_GANTRY		
		dir &= ~80
#else
        dir &= ~16;
#endif
    }
    inline void setYMoveFinished()
    {
#if DRIVE_SYSTEM==XY_GANTRY || DRIVE_SYSTEM==YX_GANTRY
        dir &= ~48;
#else
        dir &= ~32;
#endif
    }
    inline void setZMoveFinished()
    {
#if DRIVE_SYSTEM==XZ_GANTRY || DRIVE_SYSTEM==ZX_GANTRY		
		dir &= ~80
#else		
        dir &= ~64;
#endif		
    }
    inline void setXYMoveFinished()
    {
        dir &= ~48;
    }
    inline bool isXPositiveMove()
    {
        return (dir & X_STEP_DIRPOS) == X_STEP_DIRPOS;
    }
    inline bool isXNegativeMove()
    {
        return (dir & X_STEP_DIRPOS) == XSTEP;
    }
    inline bool isYPositiveMove()
    {
        return (dir & Y_STEP_DIRPOS) == Y_STEP_DIRPOS;
    }
    inline bool isYNegativeMove()
    {
        return (dir & Y_STEP_DIRPOS) == YSTEP;
    }
    inline bool isZPositiveMove()
    {
        return (dir & Z_STEP_DIRPOS) == Z_STEP_DIRPOS;
    }
    inline bool isZNegativeMove()
    {
        return (dir & Z_STEP_DIRPOS) == ZSTEP;
    }
    inline bool isEPositiveMove()
    {
        return (dir & E_STEP_DIRPOS) == E_STEP_DIRPOS;
    }
    inline bool isENegativeMove()
    {
        return (dir & E_STEP_DIRPOS) == ESTEP;
    }
    inline bool isXMove()
    {
        return (dir & XSTEP);
    }
    inline bool isYMove()
    {
        return (dir & YSTEP);
    }
    inline bool isXOrYMove()
    {
        return dir & XY_STEP;
    }
    inline bool isXOrZMove()
    {
        return dir & (XSTEP | YSTEP);
    }
    inline bool isZMove()
    {
        return (dir & ZSTEP);
    }
    inline bool isEMove()
    {
        return (dir & ESTEP);
    }
    inline bool isEOnlyMove()
    {
        return (dir & XYZE_STEP) == ESTEP;
    }
    inline bool isNoMove()
    {
        return (dir & XYZE_STEP) == 0;
    }
    inline bool isXYZMove()
    {
        return dir & XYZ_STEP;
    }
    inline bool isMoveOfAxis(uint8_t axis)
    {
        return (dir & (XSTEP << axis));
    }
    inline void setMoveOfAxis(uint8_t axis)
    {
        dir |= XSTEP << axis;
    }
    inline void setPositiveDirectionForAxis(uint8_t axis)
    {
        dir |= X_DIRPOS << axis;
    }
    inline static void resetPathPlanner()
    {
        linesCount = 0;
        linesPos = linesWritePos;
        Printer::setMenuMode(MENU_MODE_PRINTING, false);
    }
    // Only called from bresenham -> inside interrupt handle
    inline void updateAdvanceSteps(speed_t v, uint8_t max_loops, bool accelerate)
    {
#if USE_ADVANCE
        if(!Printer::isAdvanceActivated()) return;
#if ENABLE_QUADRATIC_ADVANCE
        long advanceTarget = Printer::advanceExecuted;
        if(accelerate)
        {
            for(uint8_t loop = 0; loop < max_loops; loop++) advanceTarget += advanceRate;
            if(advanceTarget > advanceFull)
                advanceTarget = advanceFull;
        }
        else
        {
            for(uint8_t loop = 0; loop < max_loops; loop++) advanceTarget -= advanceRate;
            if(advanceTarget < advanceEnd)
                advanceTarget = advanceEnd;
        }
        long h = HAL::mulu16xu16to32(v, advanceL);
        int tred = ((advanceTarget + h) >> 16);
        HAL::forbidInterrupts();
        Printer::extruderStepsNeeded += tred - Printer::advanceStepsSet;
        if(tred > 0 && Printer::advanceStepsSet <= 0)
            Printer::extruderStepsNeeded += Extruder::current->advanceBacklash;
        else if(tred < 0 && Printer::advanceStepsSet >= 0)
            Printer::extruderStepsNeeded -= Extruder::current->advanceBacklash;
        Printer::advanceStepsSet = tred;
        HAL::allowInterrupts();
        Printer::advanceExecuted = advanceTarget;
#else
        int tred = HAL::mulu6xu16shift16(v, advanceL);
        HAL::forbidInterrupts();
        Printer::extruderStepsNeeded += tred - Printer::advanceStepsSet;
        if(tred > 0 && Printer::advanceStepsSet <= 0)
            Printer::extruderStepsNeeded += (Extruder::current->advanceBacklash << 1);
        else if(tred < 0 && Printer::advanceStepsSet >= 0)
            Printer::extruderStepsNeeded -= (Extruder::current->advanceBacklash << 1);
        Printer::advanceStepsSet = tred;
        HAL::allowInterrupts();
#endif
#endif
    }
    INLINE bool moveDecelerating()//判断是否需要减速
    {
        if(stepsRemaining <= decelSteps)
        {
            if (!(flags & FLAG_DECELERATING))
            {
                Printer::timer = 0;
                flags |= FLAG_DECELERATING;
            }
            return true;
        }
        else return false;
    }
    INLINE bool moveAccelerating()//判断是否需要加速
    {
        return Printer::stepNumber <= accelSteps;
    }
    INLINE void startXStep()
    {
#if !(GANTRY) || defined(FAST_COREXYZ)
        Printer::startXStep();
#else
#if DRIVE_SYSTEM == XY_GANTRY || DRIVE_SYSTEM == XZ_GANTRY
        if(isXPositiveMove())
        {
            Printer::motorX++;
            Printer::motorYorZ++;
        }
        else
        {
            Printer::motorX--;
            Printer::motorYorZ--;
        }
#endif
#if DRIVE_SYSTEM == YX_GANTRY || DRIVE_SYSTEM == ZX_GANTRY
        if(isXPositiveMove())
        {
            Printer::motorX++;
            Printer::motorYorZ--;
        }
        else
        {
            Printer::motorX--;
            Printer::motorYorZ++;
        }
#endif
#endif
#ifdef DEBUG_STEPCOUNT
        totalStepsRemaining--;
#endif
    }
    INLINE void startYStep()
    {
#if !(GANTRY) || DRIVE_SYSTEM == ZX_GANTRY || DRIVE_SYSTEM == XZ_GANTRY || defined(FAST_COREXYZ)
        Printer::startYStep();
#else
#if DRIVE_SYSTEM == XY_GANTRY
        if(isYPositiveMove())
        {
            Printer::motorX++;
            Printer::motorYorZ--;
        }
        else
        {
            Printer::motorX--;
            Printer::motorYorZ++;
        }
#endif
#if DRIVE_SYSTEM == YX_GANTRY
        if(isYPositiveMove())
        {
            Printer::motorX++;
            Printer::motorYorZ++;
        }
        else
        {
            Printer::motorX--;
            Printer::motorYorZ--;
        }
#endif
#endif // GANTRY
#ifdef DEBUG_STEPCOUNT
        totalStepsRemaining--;
#endif

    }
    INLINE void startZStep()
    {
#if !(GANTRY) || DRIVE_SYSTEM == YX_GANTRY || DRIVE_SYSTEM == XY_GANTRY || defined(FAST_COREXYZ)
        Printer::startZStep();
#else
#if DRIVE_SYSTEM == XZ_GANTRY
        if(isZPositiveMove())
        {
            Printer::motorX++;
            Printer::motorYorZ--;
        }
        else
        {
            Printer::motorX--;
            Printer::motorYorZ++;
        }
#endif
#if DRIVE_SYSTEM == ZX_GANTRY
        if(isZPositiveMove())
        {
            Printer::motorX++;
            Printer::motorYorZ++;
        }
        else
        {
            Printer::motorX--;
            Printer::motorYorZ--;
        }
#endif
#endif
#ifdef DEBUG_STEPCOUNT
        totalStepsRemaining--;
#endif
    }
    void updateStepsParameter();
    float safeSpeed(fast8_t drivingAxis);
    void calculateMove(float axis_diff[],uint8_t pathOptimize,fast8_t distanceBase);
    void logLine();
    INLINE long getWaitTicks()
    {
        return timeInTicks;
    }
    INLINE void setWaitTicks(long wait)
    {
        timeInTicks = wait;
    }

    static INLINE bool hasLines()
    {
        return linesCount;
    }
    static INLINE void setCurrentLine()
    {
        cur = &lines[linesPos];
#if CPU_ARCH==ARCH_ARM
        PrintLine::nlFlag = true;
#endif
    }
    // Only called from within interrupts
    static INLINE void removeCurrentLineForbidInterrupt()
    {
        linesPos++;
        if(linesPos >= PRINTLINE_CACHE_SIZE) linesPos = 0;
        cur = NULL;
#if CPU_ARCH == ARCH_ARM
        nlFlag = false;
#endif
        HAL::forbidInterrupts();
        --linesCount;
        if(!linesCount)
            Printer::setMenuMode(MENU_MODE_PRINTING, false);
    }
    static INLINE void pushLine()
    {
        linesWritePos++;
        if(linesWritePos >= PRINTLINE_CACHE_SIZE) linesWritePos = 0;
        Printer::setMenuMode(MENU_MODE_PRINTING, true);
        InterruptProtectedBlock noInts;
        linesCount++;
    }
    static uint8_t getLinesCount()
    {
        InterruptProtectedBlock noInts;
        return linesCount;
    }
    static PrintLine *getNextWriteLine()
    {
        return &lines[linesWritePos];
    }
    static inline void computeMaxJunctionSpeed(PrintLine *previous,PrintLine *current);
    static int32_t bresenhamStep();
    static void waitForXFreeLines(uint8_t b=1, bool allowMoves = false);
    static inline void forwardPlanner(ufast8_t p);
    static inline void backwardPlanner(ufast8_t p,ufast8_t last);
    static void updateTrapezoids();
    static uint8_t insertWaitMovesIfNeeded(uint8_t pathOptimize, uint8_t waitExtraLines);
#if !NONLINEAR_SYSTEM
    static void queueCartesianMove(uint8_t check_endstops,uint8_t pathOptimize);
#if DISTORTION_CORRECTION
	static void queueCartesianSegmentTo(uint8_t check_endstops, uint8_t pathOptimize);
#endif
#endif	
    static void moveRelativeDistanceInSteps(int32_t x,int32_t y,int32_t z,int32_t e,float feedrate,bool waitEnd,bool check_endstop,bool pathOptimize = true);
    static void moveRelativeDistanceInStepsReal(int32_t x,int32_t y,int32_t z,int32_t e,float feedrate,bool waitEnd,bool pathOptimize = true);
#if ARC_SUPPORT
    static void arc(float *position, float *target, float *offset, float radius, uint8_t isclockwise);
#endif
    static INLINE void previousPlannerIndex(ufast8_t &p)
    {
        p = (p ? p - 1 : PRINTLINE_CACHE_SIZE - 1);
    }
    static INLINE void nextPlannerIndex(ufast8_t& p)
    {
        p = (p == PRINTLINE_CACHE_SIZE - 1 ? 0 : p + 1);
    }
#if NONLINEAR_SYSTEM
    static uint8_t queueNonlinearMove(uint8_t check_endstops,uint8_t pathOptimize, uint8_t softEndstop);
    static inline void queueEMove(int32_t e_diff,uint8_t check_endstops,uint8_t pathOptimize);
    inline uint16_t calculateNonlinearSubSegments(uint8_t softEndstop);
    static inline void calculateDirectionAndDelta(int32_t difference[], ufast8_t *dir, int32_t delta[]);
    static inline uint8_t calculateDistance(float axis_diff[], uint8_t dir, float *distance);
#if SOFTWARE_LEVELING && DRIVE_SYSTEM == DELTA
    static void calculatePlane(int32_t factors[], int32_t p1[], int32_t p2[], int32_t p3[]);
    static float calcZOffset(int32_t factors[], int32_t pointX, int32_t pointY);
#endif
#endif
};



#endif // MOTION_H_INCLUDED
