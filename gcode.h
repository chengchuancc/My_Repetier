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
#ifndef _GCODE_H
#define _GCODE_H

#define MAX_CMD_SIZE 96
#define ARRAY_SIZE(_x)	(sizeof(_x)/sizeof(_x[0]))
class SDCard;
class GCode   // 52 uint8_ts per command needed
{
    uint16_t params; // 用于存储指令是否有M、N、G、X、Y、Z、E、F、T、S、P等参数的标志位
    uint16_t params2; // 用于存储指令是否有I、J、R、D、C、H、A、B、K、L、O等参数以及是否是V2格式或有格式错误的标志位
public:
    uint16_t N; // Line number // 行号
    uint16_t M; // M指令的编号
    uint16_t G; // G指令的编号
    float X; // X坐标或参数
    float Y; // Y坐标或参数
    float Z; // Z坐标或参数
    float E; // E坐标或参数
    float F; // F参数，表示进给速度
    int32_t S; // S参数，表示温度或其他值
    int32_t P; // P参数，表示时间或其他值
    float I; // I参数，表示圆弧中心的X偏移量或其他值
    float J; // J参数，表示圆弧中心的Y偏移量或其他值
    float R; // R参数，表示圆弧半径或其他值
    float D; // D参数，表示挤出机直径或其他值
    float C; // C参数，表示圆弧角度或其他值
    float H; // H参数，表示Z探针高度或其他值
    float A; // A参数，表示旋转角度或其他值（对于非笛卡尔系统）
    float B; // B参数，表示旋转角度或其他值（对于非笛卡尔系统）
    float K; // K参数，表示旋转角度或其他值（对于非笛卡尔系统）
    float L; // L参数，表示LED亮度或其他值
    float O; // O参数，表示激光功率或其他值

    char *text; //text[17];
    //moved the byte to the end and aligned ints on short boundary
    // Old habit from PC, which require alignments for data types such as int and long to be on 2 or 4 byte boundary
    // Otherwise, the compiler adds padding, wasted space.
    uint8_t T; // This may not matter on any of these controllers, but it can't hurt
    // True if origin did not come from serial console. That way we can send status messages to
    // a host only if he would normally not know about the mode switch.
    bool internalCommand;
    inline bool hasM()
    {
        return ((params & 2)!=0);
    }
    inline bool hasN()
    {
        return ((params & 1)!=0);
    }
    inline bool hasG()
    {
        return ((params & 4)!=0);
    }
    inline bool hasX()
    {
        return ((params & 8)!=0);
    }
    inline bool hasY()
    {
        return ((params & 16)!=0);
    }
    inline bool hasZ()
    {
        return ((params & 32)!=0);
    }
    inline bool hasNoXYZ()
    {
        return ((params & 56)==0);
    }
    inline bool hasE()
    {
        return ((params & 64)!=0);
    }
    inline bool hasF()
    {
        return ((params & 256)!=0);
    }
    inline bool hasT()
    {
        return ((params & 512)!=0);
    }
    inline bool hasS()
    {
        return ((params & 1024)!=0);
    }
    inline bool hasP()
    {
        return ((params & 2048)!=0);
    }
    inline bool isV2()
    {
        return ((params & 4096)!=0);
    }
    inline bool hasString()
    {
        return ((params & 32768)!=0);
    }
    inline bool hasI()
    {
        return ((params2 & 1)!=0);
    }
    inline bool hasJ()
    {
        return ((params2 & 2)!=0);
    }
    inline bool hasR()
    {
        return ((params2 & 4)!=0);
    }
    inline bool hasD()
    {
        return ((params2 & 8)!=0);
    }
    inline bool hasC()
    {
        return ((params2 & 16)!=0);
    }
    inline bool hasH()
    {
        return ((params2 & 32)!=0);
    }
    inline bool hasA()
    {
        return ((params2 & 64)!=0);
    }
    inline bool hasB()
    {
        return ((params2 & 128)!=0);
    }
    inline bool hasK()
    {
        return ((params2 & 256)!=0);
    }
    inline bool hasL()
    {
        return ((params2 & 512)!=0);
    }
    inline bool hasO()
    {
        return ((params2 & 1024)!=0);
    }
    inline long getS(long def)
    {
        return (hasS() ? S : def);
    }
    inline long getP(long def)
    {
        return (hasP() ? P : def);
    }
    inline void setFormatError() {
        params2 |= 32768;
    }
    inline bool hasFormatError() {
        return ((params2 & 32768)!=0);
    }
    void printCommand();
    bool parseBinary(uint8_t *buffer,bool fromSerial);
    bool parseAscii(char *line,bool fromSerial);
    void popCurrentCommand();
    void echoCommand();
    /** Get next command in command buffer. After the command is processed, call gcode_command_finished() */
    static GCode *peekCurrentCommand();
    /** Frees the cache used by the last command fetched. */
    static void readFromSerial();
    static void pushCommand();
    static void executeFString(FSTRINGPARAM(cmd));
    static uint8_t computeBinarySize(char *ptr);
	static void fatalError(FSTRINGPARAM(message));
	static void reportFatalError();
	static void resetFatalError();
	inline static bool hasFatalError() {
		return fatalErrorMsg != NULL;
	}
    friend class SDCard;
    friend class UIDisplay;
private:
    void debugCommandBuffer();
    void checkAndPushCommand();
    static void requestResend();
    inline float parseFloatValue(char *s)
    {
        char *endPtr;
        while(*s == 32) s++; // skip spaces
        float f = (strtod(s, &endPtr));
        if(s == endPtr) f=0.0; // treat empty string "x " as "x0"
        return f;
    }
    inline long parseLongValue(char *s)
    {
        char *endPtr;
        while(*s == 32) s++; // skip spaces
        long l = (strtol(s, &endPtr, 10));
        if(s == endPtr) l=0; // treat empty string argument "p " as "p0"
        return l;
    }

	static FSTRINGPARAM(fatalErrorMsg);
    static GCode commandsBuffered[GCODE_BUFFER_SIZE]; ///< Buffer for received commands.
    static uint8_t bufferReadIndex; ///< Read position in gcode_buffer.
    static uint8_t bufferWriteIndex; ///< Write position in gcode_buffer.
    static uint8_t commandReceiving[MAX_CMD_SIZE]; ///< Current received command.
    static uint8_t commandsReceivingWritePosition; ///< Writing position in gcode_transbuffer.
    static uint8_t sendAsBinary; ///< Flags the command as binary input.
    static uint8_t wasLastCommandReceivedAsBinary; ///< Was the last successful command in binary mode?
    static uint8_t commentDetected; ///< Flags true if we are reading the comment part of a command.
    static uint8_t binaryCommandSize; ///< Expected size of the incoming binary command.
    static bool waitUntilAllCommandsAreParsed; ///< Don't read until all commands are parsed. Needed if gcode_buffer is misused as storage for strings.
    static uint32_t lastLineNumber; ///< Last line number received.
    static uint32_t actLineNumber; ///< Line number of current command.
    static int8_t waitingForResend; ///< Waiting for line to be resend. -1 = no wait.
    static volatile uint8_t bufferLength; ///< Number of commands stored in gcode_buffer
    static millis_t timeOfLastDataPacket; ///< Time, when we got the last data packet. Used to detect missing uint8_ts.
    static uint8_t formatErrors; ///< Number of sequential format errors
};

#if JSON_OUTPUT
#include "SdFat.h"
// Struct to hold Gcode file information 32 bytes
#define GENBY_SIZE 16
class GCodeFileInfo {
public:
    void init(SdBaseFile &file);

    unsigned long fileSize;
    float objectHeight;
    float layerHeight;
    float filamentNeeded;
    char generatedBy[GENBY_SIZE];

    bool findGeneratedBy(char *buf, char *genBy);
    bool findLayerHeight(char *buf, float &layerHeight);
    bool findFilamentNeed(char *buf, float &filament);
    bool findTotalHeight(char *buf, float &objectHeight);
};
#endif

#endif

