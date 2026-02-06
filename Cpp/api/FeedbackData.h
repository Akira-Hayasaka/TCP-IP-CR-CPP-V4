#pragma once
#include <cstdint>

namespace Dobot
{
class CFeedbackData
{
public:
    unsigned short MessageSize = 0;    // メッセージ全体のバイト長

    short Reserved1[3];    // 予約領域

    long DigitalInputs = 0;     // デジタル入力
    long DigitalOutputs = 0;    // デジタル出力
    long RobotMode = 0;         // ロボットモード（9 はエラー）
    long TimeStamp = 0;         // タイムスタンプ（単位: ms）
    long RunTime = 0;         // ロボット起動後の稼働時間（単位: ms）
    long TestValue = 0;      // メモリ構造テスト用の基準値 0x0123 4567 89AB CDEF

    
    unsigned char Reserved2[8];    // 予約領域

    double SpeedScaling = 0;          // 速度比率

    unsigned char Reserved3[16];    // 予約領域

    double VRobot = 0;                // ロボット電圧
    double IRobot = 0;                // ロボット電流
    double ProgramState = 0;          // スクリプト実行状態
    char SafetyOIn[2];                // 安全IO入力状態
    char SafetyOOut[2];               // 安全IO出力状態

    unsigned char Reserved4[76];    // 予約領域

    double QTarget[6];              // 目標関節位置
    double QdTarget[6];             // 目標関節速度
    double QddTarget[6];            // 目標関節加速度
    double ITarget[6];              // 目標関節電流
    double MTarget[6];              // 目標関節トルク
    double QActual[6];              // 実際の関節位置
    double QDActual[6];             // 実際の関節速度
    double IActual[6];              // 実際の関節電流

    double ActualTCPForce[6];       // TCP 各軸の力（6軸力センサ生データから算出）
    double ToolVectorActual[6];     // TCP デカルト実座標
    double ToolSpeedActual[6];     // TCP デカルト実速度
    double TCPForce[6];             // TCP 力
    double ToolVectorTarget[6];     // TCP デカルト目標座標
    double TCPSpeedTarget[6];       // TCP デカルト目標速度
    double MotorTempetatures[6];    // 関節温度
    double JointModes[6];           // 関節制御モード
    double VActual[6];              // 関節電圧

    unsigned char Handtype[4];                 // ハンドタイプ
    unsigned char User = 0;                    // ユーザー座標
    unsigned char Tool = 0;                    // ツール座標
    unsigned char RunQueuedCmd = 0;            // アルゴリズムキュー実行フラグ
    unsigned char PauseCmdFlag = 0;            // アルゴリズムキュー一時停止フラグ
    unsigned char VelocityRatio = 0;           // 関節速度比率 (0~100)
    unsigned char AccelerationRatio = 0;       // 関節加速度比率 (0~100)
    unsigned char Reserved5 = 0;               // 予約領域
    unsigned char XYZVelocityRatio = 0;        // デカルト位置の速度比率 (0~100)
    unsigned char RVelocityRatio = 0;          // デカルト姿勢の速度比率 (0~100)
    unsigned char XYZAccelerationRatio = 0;    // デカルト位置の加速度比率 (0~100)
    unsigned char RAccelerationRatio = 0;      // デカルト姿勢の加速度比率 (0~100)

    unsigned char Reserved6[2];                // 予約領域

    unsigned char BrakeStatus = 0;             // ロボットブレーキ状態
    unsigned char EnableStatus = 0;            // ロボットイネーブル状態
    unsigned char DragStatus = 0;              // ロボットドラッグ状態
    unsigned char RunningStatus = 0;           // ロボット実行状態
    unsigned char ErrorStatus = 0;             // ロボット警報状態
    unsigned char JogStatusCR = 0;               // ロボットジョグ状態
    unsigned char CRRobotType = 0;               // ロボット機種
    unsigned char DragButtonSignal = 0;        // ボタンパネル：ドラッグ信号
    unsigned char EnableButtonSignal = 0;      // ボタンパネル：イネーブル信号
    unsigned char RecordButtonSignal = 0;      // ボタンパネル：記録信号
    unsigned char ReappearButtonSignal = 0;    // ボタンパネル：再生（復元）信号
    unsigned char JawButtonSignal = 0;         // ボタンパネル：グリッパ制御信号
    unsigned char SixForceOnline = 0;          // 6軸力センサ オンライン状態
    unsigned char CollisionStatus =  0;        // 衝突状態
    unsigned char ArmApproachSatus = 0;        // 前腕安全スキン接近E-Stop
    unsigned char J4ApproachSatus = 0;         // J4 安全スキン接近E-Stop
    unsigned char J5ApproachSatus = 0;         // J5 安全スキン接近E-Stop
    unsigned char J6ApproachSatus = 0;         // J6 安全スキン接近E-Stop

    unsigned char Reserved7[61];    // 予約領域

    double VibrationDisZ = 0;      // 加速度計で測定した Z 軸振動変位
    long CurrentCommandId = 0;     // 現在の動作キューID
    double MActual[6];             // 6関節の実トルク
    double Load = 0;               // 負荷重量 (kg)
    double CenterX = 0;            // X 方向の偏心距離 (mm)
    double CenterY = 0;            // Y 方向の偏心距離 (mm)
    double CenterZ = 0;            // Z 方向の偏心距離 (mm)
    double UserValue[6];            // ユーザー座標値
    double ToolValue[6];               // ツール座標値

    unsigned char Reserved8[8];    // 予約領域

    double SixForceValue[6];       // 現在の6軸力センサ生データ
    double TargetQuaternion[4];    // [qw,qx,qy,qz] 目標クォータニオン
    double ActualQuaternion[4];    // [qw,qx,qy,qz] 実クォータニオン
    short AutoManualMode = 0;       // 自動/手動モード
    unsigned short ExportStatus = 0;         // USBエクスポート状態
    char SafetyState =0;                
    /* 安全状態 1420
        1420:0 非常停止状態（Low 有効）
        1420:1 保護停止状態（Low 有効）
        1420:2 縮減モード状態（Low 有効）
        1420:3 非停止状態（Low 有効）
        1420:4 動作中状態（Low 有効）
        1420:5 システム非常停止状態（Low 有効）
        1420:6 ユーザー非常停止状態（Low 有効）
        1420:7 安全原点出力状態（Low 有効。安全原点外のとき有効）
    */
    char SafetyState_Resevered =0;  // 安全状態の予約領域
    unsigned char Reserved9[18];    // 予約領域
};
}    // namespace Dobot
