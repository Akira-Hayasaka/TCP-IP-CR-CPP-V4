#pragma once

#include <thread>
#include "DobotClient.h"
#include "FeedbackData.h"

namespace Dobot
{
    class CFeedback : public CDobotClient
    {
    public:
        CFeedback();
        virtual ~CFeedback();

        const CFeedbackData& GetFeedbackData() const;
        inline bool IsDataHasRead() const{
            return m_IsDataHasRead;
        }
        inline void SetDataHasRead(bool bValue)
        {
            m_IsDataHasRead = bValue;
        }
        std::string ConvertRobotMode();

    protected:
        void OnConnected() override;
        void OnDisconnected() override;

        /// <summary>
        /// 受信データを取得し、解析・処理する
        /// </summary>
        void OnRecvData();

        /// <summary>
        /// データ解析
        /// </summary>
        /// <param name="buffer">1パケット分の完全なデータ</param>
        void ParseData(char* pBuffer);

    private:
        std::thread m_thd;
        bool m_IsDataHasRead = false;

        CFeedbackData m_feedbackData;
    };
}

