#pragma once
#include <sstream>
#include <string>

namespace Dobot
{
struct CDescartesPoint
{
public:
    /// <summary>
    /// X 軸位置（単位: mm）
    /// </summary>
    double x;

    /// <summary>
    /// Y 軸位置（単位: mm）
    /// </summary>
    double y;

    /// <summary>
    /// Z 軸位置（単位: mm）
    /// </summary>
    double z;

    /// <summary>
    /// Rx 軸位置（単位: 度）
    /// </summary>
    double rx;

    /// <summary>
    /// Ry 軸位置（単位: 度）
    /// </summary>
    double ry;

    /// <summary>
    /// Rz 軸位置（単位: 度）
    /// </summary>
    double rz;

    std::string ToString()
    {
        std::ostringstream oss;
        oss << x << ',' << y << ',' << z << ',' << rx << ',' << ry << ',' << rz;
        return oss.str();
    }
};

struct CForcePoint
{
    public:
        /// <summary>
        /// X 軸目標力（単位: mm）
        /// </summary>
        double fx;

        /// <summary>
        /// Y 軸目標力（単位: mm）
        /// </summary>
        double fy;

        /// <summary>
        /// Z 軸目標力（単位: mm）
        /// </summary>
        double fz;

        /// <summary>
        /// Rx 軸目標力（単位: 度）
        /// </summary>
        double frx;

        /// <summary>
        /// Ry 軸目標力（単位: 度）
        /// </summary>
        double fry;

        /// <summary>
        /// Rz 軸目標力（単位: 度）
        /// </summary>
        double frz;

        std::string ToString()
        {
            std::ostringstream oss;
            oss << fx << ',' << fy << ',' << fz << ',' << frx << ',' << fry << ',' << frz;
            return oss.str();
        }
};

struct COffsetPoint
{
    public:
        /// <summary>
        /// X 軸オフセット量
        /// </summary>
        double offsetX;

        /// <summary>
        /// Y 軸オフセット量
        /// </summary>
        double offsetY;

        /// <summary>
        /// Z 軸オフセット量
        /// </summary>
        double offsetZ;

        /// <summary>
        /// Rx 軸オフセット量
        /// </summary>
        double offsetRx;

        /// <summary>
        /// Ry 軸オフセット量
        /// </summary>
        double offsetRy;

        /// <summary>
        /// Rz 軸オフセット量
        /// </summary>
        double offsetRz;

        std::string ToString()
        {
            std::ostringstream oss;
            oss << offsetX << ',' << offsetY << ',' << offsetZ << ',' << offsetRx << ',' << offsetRy << ',' << offsetRz;
            return oss.str();
        }
};

struct ModeDistanceIndexStatus
{
public:
    // Distance モードを設定
    int Mode;

    // 指定距離で実行
    int Distance;

    // デジタル出力インデックス
    int Index;

    // デジタル出力状態
    int Status;
    std::string ToString()
    {
        std::ostringstream oss;
        oss << "{" << Mode << ',' << Distance << ',' << Index << ',' << Status << "}";
        return oss.str();
    }
};
}    // namespace Dobot
