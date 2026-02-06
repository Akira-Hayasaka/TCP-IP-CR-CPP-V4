#pragma once

#include "DobotClient.h"
#include "DescartesPoint.h"
#include "JointPoint.h"
#include <mutex>

namespace Dobot
{
class CDashboard : public CDobotClient
{
public:
    CDashboard();
    virtual ~CDashboard();
    std::string SendRecvMsg(std::string& str);
    std::string SendRecvMsg(char* cmd);
    /// <summary>
    /// リセット（エラーをクリア）
    /// </summary>
    /// <returns>実行結果の説明メッセージ</returns>
    std::string ClearError();

    /// <summary>
    /// ロボット電源ON
    /// </summary>
    /// <returns>実行結果の説明メッセージ</returns>
    std::string PowerOn();

    /// <summary>
    /// 非常停止
    /// </summary>
    /// <returns>実行結果の説明メッセージ</returns>
    std::string EmergencyStop(int);

    /// <summary>
    /// ロボットをイネーブル
    /// </summary>
    /// <returns>実行結果の説明メッセージ</returns>
    std::string EnableRobot();
    template <typename... Args>
    std::string EnableRobot(Args... args);

    /// <summary>
    /// ロボットをディスエーブル
    /// </summary>
    /// <returns>実行結果の説明メッセージ</returns>
    std::string DisableRobot();

    /// <summary>
    /// グローバル速度比率を設定
    /// </summary>
    /// <param name="ratio">速度比率（範囲: 1~100）</param>
    /// <returns>実行結果の説明メッセージ</returns>
    std::string SpeedFactor(int ratio);

    /// <summary>
    /// デジタル出力ポート状態を設定（キュー指令）
    /// </summary>
    /// <param name="index">デジタル出力インデックス（範囲: 1~16 または 100~1000）</param>
    /// <param name="status">デジタル出力状態（1: High / 0: Low）</param>
    /// <returns>実行結果の説明メッセージ</returns>
    std::string DO(int index, int status);
    std::string DO(int index, int status, int time);

    /// <summary>
    /// エラーID取得
    /// </summary>
    /// <returns>実行結果の説明メッセージ</returns>
    std::string GetErrorID();

    std::string User(int index);
    std::string Tool(int index);

    // 軌跡リカバリ指令
    std::string SetResumeOffset(double distance);
    std::string PathRecovery();
    std::string PathRecoveryStop();
    std::string PathRecoveryStatus();

    // ログエクスポート指令
    std::string LogExportUSB(int range);
    std::string GetExportStatus();

    // 力制御指令
    std::string EnableFTSensor(int status);
    std::string SixForceHome();
    std::string GetForce(int tool);
    std::string ForceDriveMode(const CDescartesPoint& pt,int user);
    std::string ForceDriveSpeed(int speed);
    std::string FCForceMode(const CDescartesPoint& pt,const CForcePoint& force,int reference, int user, int tool);
    std::string FCSetDeviation(const CDescartesPoint& pt, int controltype);
    std::string FCSetForceLimit(const CDescartesPoint& pt);
    std::string FCSetMass(const CDescartesPoint& pt);
    std::string FCSetDamping(const CDescartesPoint& pt);
    std::string FCOff();
    std::string FCSetForceSpeedLimit(const CDescartesPoint& pt);
    std::string FCSetForce(const CDescartesPoint& pt);

    // 衝突
    std::string SetFCCollision(double force, double torque);
    std::string FCCollisionSwitch(int enable);

    // 460 で追加された動作指令
    std::string RelPointTool(const CDescartesPoint& pt,const COffsetPoint& pt2);
    std::string RelPointTool(const CJointPoint& pt,const COffsetPoint& pt2);
    std::string RelPointUser(const CDescartesPoint& pt,const COffsetPoint& pt2);
    std::string RelPointUser(const CJointPoint& pt,const COffsetPoint& pt2);
    template <typename... Args>
    std::string RelJoint(const CJointPoint& pt,Args... args);



    std::string RobotMode();

    /// <summary>
    /// TCPモード切替要求
    /// </summary>
    /// <returns>実行結果の説明メッセージ</returns>
    std::string RequestControl();

    std::string SetPayload(float load);
    template <typename... Args>
    std::string SetPayload(float load, Args... args);
    std::string SetPayload(std::string name);

    std::string ToolDOInstant(int index, int status);
    std::string AOInstant(int index, int status);
    std::string VelJ(int Rprecent);
    std::string VelL(int Rprecent);
    std::string RunScript(std::string projectName);
    std::string Stop();
    std::string Pause();
    std::string Continue();
    

    std::string PayLoad(double weight, double inertia);
    std::string DOInstant(int index, int status);
    std::string ToolDO(int index, int status);
    std::string AO(int index, int status);
    std::string AccJ(int Rprecent);
    std::string AccL(int Rprecent);
    std::string CP(int Rprecent);

    std::string SetArmOrientation(int LorR);
    std::string SetArmOrientation(int LorR, int UorD, int ForN, int Config6);
    std::string PositiveKin(const CDescartesPoint& pt, int User, int Tool);
    std::string InverseKin(const CDescartesPoint& pt, int User, int Tool);

    template <typename... Args>
    std::string InverseKin(const CDescartesPoint& pt, int User, int Tool, Args... args);

    std::string SetCollisionLevel(int level);
    std::string GetAngle();
    std::string GetPose();
    std::string GetPose(int user, int tool);

    std::string ModbusRTUCreate(int slave_id, int baud);
    template <typename... Args>
    std::string ModbusRTUCreate(int slave_id, int baud, Args... args);

    std::string ModbusCreate(std::string ip, int port, int slave_id);
    std::string ModbusCreate(std::string ip, int port, int slave_id, int isRTU);    //  オーバーロード（オプション引数）

    std::string ModbusClose(int index);
    std::string GetInBits(int index, int addr, int count);

    std::string GetInRegs(int index, int addr, int count);
    std::string GetInRegs(int index, int addr, int count, std::string valType);    //  オーバーロード（オプション引数）

    std::string GetCoils(int index, int addr, int count);
    std::string SetCoils(int index, int addr, int count, std::string valTab);
    std::string GetHoldRegs(int index, int addr, int count);
    std::string GetHoldRegs(int index, int addr, int count, std::string valType);    //  オーバーロード（オプション引数）
    std::string SetHoldRegs(int index, int addr, int count, std::string valTab);
    std::string SetHoldRegs(int index, int addr, int count, std::string valTab,
                            std::string valType);    //  オーバーロード（オプション引数）
    std::string DI(int index);
    std::string ToolDI(int index);
    std::string AI(int index);
    std::string ToolAI(int index);

    std::string DIGroup(int index, int value);
    template <typename... Args>
    std::string DIGroup(Args... args);

    std::string DOGroup(int index, int value);
    template <typename... Args>
    std::string DOGroup(Args... args);
    std::string BrakeControl(int axisID, int value);
    std::string StartDrag();
    std::string StopDrag();
    std::string DragSensivity(int index, int value);
    std::string GetDO(int index);
    std::string GetAO(int index);

    template <typename... Args>
    std::string GetDOGroup(Args... args);

    std::string SetTool485(int baudrate);
    template <typename... Args>
    std::string SetTool485(int baudrate, Args... args);

    std::string SetSafeWallEnable(int index, int value);
    std::string SetToolPower(int status);

    std::string SetToolMode(int mode);
    template <typename... Args>
    std::string SetToolMode(int mode, Args... args);

    std::string SetBackDistance(double distance);
    std::string SetPostCollisionMode(int mode);

    std::string SetUser(int index, std::string value);
    std::string SetTool(int index, std::string value);
    std::string CalcUser(int index, int matrix, std::string offset);
    std::string CalcTool(int index, int matrix, std::string offset);
    std::string GetInputBool(int address);
    std::string GetInputInt(int address);
    std::string GetInputFloat(int address);
    std::string GetOutputBool(int address);
    std::string GetOutputInt(int address);
    std::string GetOutputFloat(int address);
    std::string SetOutputBool(int address, int value);
    std::string SetOutputInt(int address, int value);
    std::string SetOutputFloat(int address, int value);

    /// 点到点動作（目標点はデカルト座標）
    /// <param name="pt">デカルト座標の目標点</param>
    /// <returns>実行結果の説明メッセージ</returns>
    std::string MovJ(const CDescartesPoint& pt);
    template <typename... Args>
    std::string MovJ(const CDescartesPoint& pt, Args... args);

    std::string MovJ(const CJointPoint& pt);
    template <typename... Args>
    std::string MovJ(const CJointPoint& pt, Args... args);

    /// 直線動作（目標点はデカルト座標）
    /// <param name="pt">デカルト座標の目標点</param>
    /// <returns>実行結果の説明メッセージ</returns>
    std::string MovL(const CDescartesPoint& pt);
    template <typename... Args>
    std::string MovL(const CDescartesPoint& pt, Args... args);

    std::string MovL(const CJointPoint& pt);
    template <typename... Args>
    std::string MovL(const CJointPoint& pt, Args... args);

    std::string MovLIO(const CDescartesPoint& pt, ModeDistanceIndexStatus& mdis);
    template <typename... Args>
    std::string MovLIO(const CDescartesPoint& pt, ModeDistanceIndexStatus& mdis, Args... args);

    std::string MovLIO(const CJointPoint& pt, ModeDistanceIndexStatus& mdis);
    template <typename... Args>
    std::string MovLIO(const CJointPoint& pt, ModeDistanceIndexStatus& mdis, Args... args);

    std::string MovJIO(const CDescartesPoint& pt, ModeDistanceIndexStatus& mdis);
    template <typename... Args>
    std::string MovJIO(const CDescartesPoint& pt, ModeDistanceIndexStatus& mdis, Args... args);

    std::string MovJIO(const CJointPoint& pt, ModeDistanceIndexStatus& mdis);
    template <typename... Args>
    std::string MovJIO(const CJointPoint& pt, ModeDistanceIndexStatus& mdis, Args... args);

    std::string Arc(const CDescartesPoint& pt, const CDescartesPoint& pt2);
    template <typename... Args>
    std::string Arc(const CDescartesPoint& pt, const CDescartesPoint& pt2, Args... args);

    std::string Arc(const CJointPoint& pt, const CJointPoint& pt2);
    template <typename... Args>
    std::string Arc(const CJointPoint& pt, const CJointPoint& pt2, Args... args);

    std::string Circle(const CDescartesPoint& pt, const CDescartesPoint& pt2, int count);
    template <typename... Args>
    std::string Circle(const CDescartesPoint& pt, const CDescartesPoint& pt2, int count, Args... args);

    std::string Circle(const CJointPoint& pt, const CJointPoint& pt2, int count);
    template <typename... Args>
    std::string Circle(const CJointPoint& pt, const CJointPoint& pt2, int count, Args... args);

    /// 関節ジョグ動作（距離固定ではない）
    /// <param name="s">ジョグ対象の軸</param>
    /// <returns>実行結果の説明メッセージ</returns>
    std::string MoveJog(std::string traceName);
    template <typename... Args>
    std::string MoveJog(std::string traceName, Args... args);

    /// 関節ジョグ動作を停止
    /// <returns>実行結果の説明メッセージ</returns>
    std::string StopMoveJog();

    // 軌跡再生（軌跡ファイル: 関節）
    /// <param traceName="s">軌跡ファイル名（拡張子含む）。軌跡は /dobot/userdata/project/process/trajectory/ に保存</param>
    /// <param constInt="s">constInt=1: 等速再生（停止やデッドゾーンは除去） / constInt=0: 元の速度で再生</param>
    /// <param cart="s">cart=1: デカルト経路で再生 / cart=0: 関節経路で再生</param>
    /// <returns>実行結果の説明メッセージ</returns>
    std::string StartPath(std::string traceName);
    template <typename... Args>
    std::string StartPath(std::string traceName, Args... args);

    std::string GetStartPose(std::string traceName);
    std::string RelMovJTool(const CDescartesPoint& pt);
    template <typename... Args>
    std::string RelMovJTool(const CDescartesPoint& pt, Args... args);

    std::string RelMovLTool(const CDescartesPoint& pt);
    template <typename... Args>
    std::string RelMovLTool(const CDescartesPoint& pt, Args... args);

    std::string RelMovJUser(const CDescartesPoint& pt);
    template <typename... Args>
    std::string RelMovJUser(const CDescartesPoint& pt, Args... args);

    std::string RelMovLUser(const CDescartesPoint& pt);
    template <typename... Args>
    std::string RelMovLUser(const CDescartesPoint& pt, Args... args);

    std::string RelJointMovJ(const CJointPoint& pt);
    template <typename... Args>
    std::string RelJointMovJ(const CJointPoint& pt, Args... args);

    std::string GetCurrentCommandId();
    std::string EnableSafeSkin(int result);
    std::string SetSafeSkin(int part, int status);

    std::string ServoJ(const CJointPoint& pt);
    template <typename... Args>
    std::string ServoJ(const CJointPoint& pt, Args... args);

    std::string ServoP(const CDescartesPoint& pt);
    template <typename... Args>
    std::string ServoP(const CDescartesPoint& pt, Args... args);

protected:
    void OnConnected() override;
    void OnDisconnected() override;

private:
    // クラス外で定義するメンバ関数
    template <typename T>
    void printArg(T arg);

    template <typename T, typename... Args>
    void printArg(T arg, Args... args);

    template <typename T>
    std::string toString(T arg);
    std::string strSend{ "" };
    std::mutex m_mutex;
    std::mutex m_mutexSend;
};

// テンプレート関数は .h 内で定義する必要があります
// オプション引数の出力
template <typename T>
void CDashboard::printArg(T arg)
{
    strSend = strSend + toString(arg) + ")";
}

// 再帰的にオプション引数を出力
template <typename T, typename... Args>
void CDashboard::printArg(T arg, Args... args)
{
    strSend = strSend + toString(arg) + ",";
    printArg(args...);
}

// 補助関数：引数を文字列へ変換
template <typename T>
std::string CDashboard::toString(T arg)
{
    std::stringstream ss;
    ss << arg;
    return ss.str();
}

template <typename... Args>
std::string CDashboard::EnableRobot(Args... args)
{
    std::unique_lock<std::mutex> lockValue(m_mutexSend);
    std::string str = "EnableRobot(";
    strSend = str;
    printArg(args...);
    str = strSend;
    strSend.clear();

    return SendRecvMsg(str);
}

template <typename... Args>
std::string CDashboard::SetPayload(float load, Args... args)
{
    std::unique_lock<std::mutex> lockValue(m_mutexSend);
    std::ostringstream oss;
    oss << "SetPayload(" << std::to_string(load) << ',';
    strSend = oss.str();
    printArg(args...);
    std::string str = strSend;
    strSend.clear();

    return SendRecvMsg(str);
}

template <typename... Args>
std::string CDashboard::DIGroup(Args... args)
{
    std::unique_lock<std::mutex> lockValue(m_mutexSend);
    std::ostringstream oss;
    oss << "DIGroup(";
    strSend = oss.str();
    printArg(args...);
    std::string str = strSend;
    strSend.clear();

    return SendRecvMsg(str);
}

template <typename... Args>
std::string CDashboard::DOGroup(Args... args)
{
    std::unique_lock<std::mutex> lockValue(m_mutexSend);
    std::ostringstream oss;
    oss << "DOGroup(";
    strSend = oss.str();
    printArg(args...);
    std::string str = strSend;
    strSend.clear();

    return SendRecvMsg(str);
}

template <typename... Args>
std::string CDashboard::InverseKin(const CDescartesPoint& pt, int User, int Tool, Args... args)
{
    std::unique_lock<std::mutex> lockValue(m_mutexSend);
    std::ostringstream oss;
    oss << "InverseKin(" + std::to_string(pt.x) + "," + std::to_string(pt.y) + "," + std::to_string(pt.z) + "," +
               std::to_string(pt.rx) + "," + std::to_string(pt.ry) + "," + std::to_string(pt.rz) + "," +
               std::to_string(User) + "," + std::to_string(Tool);
    strSend = oss.str();
    printArg(args...);
    std::string str = strSend;
    strSend.clear();

    return SendRecvMsg(str);
}

template <typename... Args>
std::string CDashboard::ModbusRTUCreate(int slave_id, int baud, Args... args)
{
    std::unique_lock<std::mutex> lockValue(m_mutexSend);
    std::ostringstream oss;
    oss << "ModbusRTUCreate(" + std::to_string(slave_id) + "," + std::to_string(baud) + ",";
    strSend = oss.str();
    printArg(args...);
    std::string str = strSend;
    strSend.clear();

    return SendRecvMsg(str);
}

template <typename... Args>
std::string CDashboard::GetDOGroup(Args... args)
{
    std::unique_lock<std::mutex> lockValue(m_mutexSend);
    std::ostringstream oss;
    oss << "GetDOGroup(";
    strSend = oss.str();
    printArg(args...);
    std::string str = strSend;
    strSend.clear();

    return SendRecvMsg(str);
}

template <typename... Args>
std::string CDashboard::SetTool485(int baudrate, Args... args)
{
    std::unique_lock<std::mutex> lockValue(m_mutexSend);
    std::ostringstream oss;
    oss << "SetTool485(" << std::to_string(baudrate) << ',';
    strSend = oss.str();
    printArg(args...);
    std::string str = strSend;
    strSend.clear();

    return SendRecvMsg(str);
}

template <typename... Args>
std::string CDashboard::SetToolMode(int mode, Args... args)
{
    std::unique_lock<std::mutex> lockValue(m_mutexSend);
    std::ostringstream oss;
    oss << "SetToolMode(" << std::to_string(mode) << ',';
    strSend = oss.str();
    printArg(args...);
    std::string str = strSend;
    strSend.clear();

    return SendRecvMsg(str);
}

template <typename... Args>
std::string CDashboard::MovJ(const CDescartesPoint& pt, Args... args)
{
    std::unique_lock<std::mutex> lockValue(m_mutexSend);
    char cmd[200];
    snprintf(cmd, sizeof(cmd), "MovJ(pose={%0.3f,%0.3f,%0.3f,%0.3f,%0.3f,%0.3f},", pt.x, pt.y, pt.z, pt.rx, pt.ry,
             pt.rz);
    strSend = cmd;
    printArg(args...);
    strcpy(cmd, strSend.c_str());
    strSend.clear();
    return SendRecvMsg(cmd);
}

template <typename... Args>
std::string CDashboard::MovJ(const CJointPoint& pt, Args... args)
{
    std::unique_lock<std::mutex> lockValue(m_mutexSend);
    char cmd[200];
    snprintf(cmd, sizeof(cmd), "MovJ(joint={%0.3f,%0.3f,%0.3f,%0.3f,%0.3f,%0.3f},", pt.j1, pt.j2, pt.j3, pt.j4, pt.j5,
             pt.j6);
    strSend = cmd;
    printArg(args...);
    strcpy(cmd, strSend.c_str());
    strSend.clear();

    return SendRecvMsg(cmd);
}

template <typename... Args>
std::string CDashboard::MovL(const CDescartesPoint& pt, Args... args)
{
    std::unique_lock<std::mutex> lockValue(m_mutexSend);
    char cmd[200];
    snprintf(cmd, sizeof(cmd), "MovL(pose={%0.3f,%0.3f,%0.3f,%0.3f,%0.3f,%0.3f},", pt.x, pt.y, pt.z, pt.rx, pt.ry,
             pt.rz);
    strSend = cmd;
    printArg(args...);
    strcpy(cmd, strSend.c_str());
    strSend.clear();
    return SendRecvMsg(cmd);
}

template <typename... Args>
std::string CDashboard::MovL(const CJointPoint& pt, Args... args)
{
    std::unique_lock<std::mutex> lockValue(m_mutexSend);
    char cmd[200];
    snprintf(cmd, sizeof(cmd), "MovL(joint={%0.3f,%0.3f,%0.3f,%0.3f,%0.3f,%0.3f},", pt.j1, pt.j2, pt.j3, pt.j4, pt.j5,
             pt.j6);
    strSend = cmd;
    printArg(args...);
    strcpy(cmd, strSend.c_str());
    strSend.clear();

    return SendRecvMsg(cmd);
}

template <typename... Args>
std::string CDashboard::MovLIO(const CDescartesPoint& pt, ModeDistanceIndexStatus& mdis, Args... args)
{
    std::unique_lock<std::mutex> lockValue(m_mutexSend);
    char cmd[200];
    snprintf(cmd, sizeof(cmd), "MovLIO(pose={%0.3f,%0.3f,%0.3f,%0.3f,%0.3f,%0.3f},%s,", pt.x, pt.y, pt.z, pt.rx, pt.ry,
             pt.rz, mdis.ToString().c_str());
    strSend = cmd;
    printArg(args...);
    strcpy(cmd, strSend.c_str());
    strSend.clear();
    return SendRecvMsg(cmd);
}

template <typename... Args>
std::string CDashboard::MovLIO(const CJointPoint& pt, ModeDistanceIndexStatus& mdis, Args... args)
{
    std::unique_lock<std::mutex> lockValue(m_mutexSend);
    char cmd[200];
    snprintf(cmd, sizeof(cmd), "MovLIO(joint={%0.3f,%0.3f,%0.3f,%0.3f,%0.3f,%0.3f},%s,", pt.j1, pt.j2, pt.j3, pt.j4,
             pt.j5, pt.j6, mdis.ToString().c_str());
    strSend = cmd;
    printArg(args...);
    strcpy(cmd, strSend.c_str());
    strSend.clear();
    return SendRecvMsg(cmd);
}

template <typename... Args>
std::string CDashboard::MovJIO(const CDescartesPoint& pt, ModeDistanceIndexStatus& mdis, Args... args)
{
    std::unique_lock<std::mutex> lockValue(m_mutexSend);
    char cmd[200];
    snprintf(cmd, sizeof(cmd), "MovJIO(pose={%0.3f,%0.3f,%0.3f,%0.3f,%0.3f,%0.3f},%s,", pt.x, pt.y, pt.z, pt.rx, pt.ry,
             pt.rz, mdis.ToString().c_str());
    strSend = cmd;
    printArg(args...);
    strcpy(cmd, strSend.c_str());
    strSend.clear();
    return SendRecvMsg(cmd);
}

template <typename... Args>
std::string CDashboard::MovJIO(const CJointPoint& pt, ModeDistanceIndexStatus& mdis, Args... args)
{
    std::unique_lock<std::mutex> lockValue(m_mutexSend);
    char cmd[200];
    snprintf(cmd, sizeof(cmd), "MovJIO(joint={%0.3f,%0.3f,%0.3f,%0.3f,%0.3f,%0.3f},%s,", pt.j1, pt.j2, pt.j3, pt.j4,
             pt.j5, pt.j6, mdis.ToString().c_str());
    strSend = cmd;
    printArg(args...);
    strcpy(cmd, strSend.c_str());
    strSend.clear();
    return SendRecvMsg(cmd);
}

template <typename... Args>
std::string CDashboard::Arc(const CDescartesPoint& pt, const CDescartesPoint& pt2, Args... args)
{
    std::unique_lock<std::mutex> lockValue(m_mutexSend);
    char cmd[200];
    snprintf(cmd, sizeof(cmd),
             "Arc(pose={%0.3f,%0.3f,%0.3f,%0.3f,%0.3f,%0.3f},pose={%0.3f,%0.3f,%0.3f,%0.3f,%0.3f,%0.3f},", pt.x, pt.y,
             pt.z, pt.rx, pt.ry, pt.rz, pt2.x, pt2.y, pt2.z, pt2.rx, pt2.ry, pt2.rz);
    strSend = cmd;
    printArg(args...);
    strcpy(cmd, strSend.c_str());
    strSend.clear();
    return SendRecvMsg(cmd);
}

template <typename... Args>
std::string CDashboard::Arc(const CJointPoint& pt, const CJointPoint& pt2, Args... args)
{
    std::unique_lock<std::mutex> lockValue(m_mutexSend);
    char cmd[200];
    snprintf(cmd, sizeof(cmd),
             "Arc(joint={%0.3f,%0.3f,%0.3f,%0.3f,%0.3f,%0.3f},joint={%0.3f,%0.3f,%0.3f,%0.3f,%0.3f,%0.3f},", pt.j1,
             pt.j2, pt.j3, pt.j4, pt.j5, pt.j6, pt2.j1, pt2.j2, pt2.j3, pt2.j4, pt2.j5, pt2.j6);
    strSend = cmd;
    printArg(args...);
    strcpy(cmd, strSend.c_str());
    strSend.clear();
    return SendRecvMsg(cmd);
}

template <typename... Args>
std::string CDashboard::Circle(const CDescartesPoint& pt, const CDescartesPoint& pt2, int count, Args... args)
{
    std::unique_lock<std::mutex> lockValue(m_mutexSend);
    char cmd[200];
    snprintf(cmd, sizeof(cmd),
             "Circle(pose={%0.3f,%0.3f,%0.3f,%0.3f,%0.3f,%0.3f},pose={%0.3f,%0.3f,%0.3f,%0.3f,%0.3f,%0.3f},%d,", pt.x,
             pt.y, pt.z, pt.rx, pt.ry, pt.rz, pt2.x, pt2.y, pt2.z, pt2.rx, pt2.ry, pt2.rz, count);
    strSend = cmd;
    printArg(args...);
    strcpy(cmd, strSend.c_str());
    strSend.clear();
    return SendRecvMsg(cmd);
}

template <typename... Args>
std::string CDashboard::Circle(const CJointPoint& pt, const CJointPoint& pt2, int count, Args... args)
{
    std::unique_lock<std::mutex> lockValue(m_mutexSend);
    char cmd[200];
    snprintf(cmd, sizeof(cmd),
             "Circle(joint={%0.3f,%0.3f,%0.3f,%0.3f,%0.3f,%0.3f},joint={%0.3f,%0.3f,%0.3f,%0.3f,%0.3f,%0.3f},%d,",
             pt.j1, pt.j2, pt.j3, pt.j4, pt.j5, pt.j6, pt2.j1, pt2.j2, pt2.j3, pt2.j4, pt2.j5, pt2.j6, count);
    strSend = cmd;
    printArg(args...);
    strcpy(cmd, strSend.c_str());
    strSend.clear();
    return SendRecvMsg(cmd);
}

template <typename... Args>
std::string CDashboard::MoveJog(std::string traceName, Args... args)
{
    std::unique_lock<std::mutex> lockValue(m_mutexSend);
    std::string str = "MoveJog(" + std::string(traceName) + ",";
    strSend = str;
    printArg(args...);
    str = strSend;
    strSend.clear();

    return SendRecvMsg(str);
}

template <typename... Args>
std::string CDashboard::StartPath(std::string traceName, Args... args)
{
    std::unique_lock<std::mutex> lockValue(m_mutexSend);
    std::string str = "StartPath(" + std::string(traceName) + ",";
    strSend = str;
    printArg(args...);
    str = strSend;
    strSend.clear();

    return SendRecvMsg(str);
}

template <typename... Args>
std::string CDashboard::RelMovJTool(const CDescartesPoint& pt, Args... args)
{
    std::unique_lock<std::mutex> lockValue(m_mutexSend);
    std::ostringstream oss;
    oss << "RelMovJTool(" << pt.x << ',' << pt.y << ',' << pt.z << ',' << pt.rx << "," << pt.ry << "," << pt.rz << ",";
    strSend = oss.str();
    printArg(args...);
    std::string str = strSend;
    strSend.clear();

    return SendRecvMsg(str);
}

template <typename... Args>
std::string CDashboard::RelMovLTool(const CDescartesPoint& pt, Args... args)
{
    std::unique_lock<std::mutex> lockValue(m_mutexSend);
    std::ostringstream oss;
    oss << "RelMovLTool(" << pt.x << ',' << pt.y << ',' << pt.z << ',' << pt.rx << "," << pt.ry << "," << pt.rz << ",";
    strSend = oss.str();
    printArg(args...);
    std::string str = strSend;
    strSend.clear();

    return SendRecvMsg(str);
}

template <typename... Args>
std::string CDashboard::RelMovJUser(const CDescartesPoint& pt, Args... args)
{
    std::unique_lock<std::mutex> lockValue(m_mutexSend);
    std::ostringstream oss;
    oss << "RelMovJUser(" << pt.x << ',' << pt.y << ',' << pt.z << ',' << pt.rx << "," << pt.ry << "," << pt.rz << ",";
    strSend = oss.str();
    printArg(args...);
    std::string str = strSend;
    strSend.clear();

    return SendRecvMsg(str);
}

template <typename... Args>
std::string CDashboard::RelMovLUser(const CDescartesPoint& pt, Args... args)
{
    std::unique_lock<std::mutex> lockValue(m_mutexSend);
    std::ostringstream oss;
    oss << "RelMovLUser(" << pt.x << ',' << pt.y << ',' << pt.z << ',' << pt.rx << "," << pt.ry << "," << pt.rz << ",";
    strSend = oss.str();
    printArg(args...);
    std::string str = strSend;
    strSend.clear();

    return SendRecvMsg(str);
}

template <typename... Args>
std::string CDashboard::RelJointMovJ(const CJointPoint& pt, Args... args)
{
    std::unique_lock<std::mutex> lockValue(m_mutexSend);
    std::ostringstream oss;
    oss << "RelJointMovJ(" << pt.j1 << ',' << pt.j2 << ',' << pt.j3 << ',' << pt.j4 << "," << pt.j5 << "," << pt.j6
        << ",";
    strSend = oss.str();
    printArg(args...);
    std::string str = strSend;
    strSend.clear();

    return SendRecvMsg(str);
}

template <typename... Args>
std::string CDashboard::ServoJ(const CJointPoint& pt, Args... args)
{
    std::unique_lock<std::mutex> lockValue(m_mutexSend);
    std::ostringstream oss;
    oss << "ServoJ(" << pt.j1 << ',' << pt.j2 << ',' << pt.j3 << ',' << pt.j4 << ',' << pt.j5 << ',' << pt.j6 << ',';
    strSend = oss.str();
    printArg(args...);
    std::string str = strSend;
    strSend.clear();

    return SendRecvMsg(str);
}

template <typename... Args>
std::string CDashboard::ServoP(const CDescartesPoint& pt, Args... args)
{
    std::unique_lock<std::mutex> lockValue(m_mutexSend);
    std::ostringstream oss;
    oss << "ServoP(" << pt.x << ',' << pt.y << ',' << pt.z << ',' << pt.rx << ',' << pt.ry << ',' << pt.rz << ',';
    strSend = oss.str();
    printArg(args...);
    std::string str = strSend; 
    strSend.clear();

    return SendRecvMsg(str);
}




}    // namespace Dobot
