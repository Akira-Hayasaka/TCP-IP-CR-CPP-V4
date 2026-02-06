#pragma once

#if defined(_WIN32)||defined(_WIN64)
#include <WinSock2.h>
#include <Windows.h>
#include <WS2tcpip.h>
#pragma comment(lib, "ws2_32.lib")
#else
#include <unistd.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/select.h>
#include <netdb.h>
#include <string.h>

#define SOCKET int
#define INVALID_SOCKET -1
#define SOCKET_ERROR -1
#define ADDR_ANY 0
#define SD_BOTH SHUT_RDWR

#define closesocket close
#define Sleep(t) sleep(t)
#endif

#include <string>

namespace Dobot
{
    class CDobotClient
    {
    public:
        CDobotClient();
        virtual ~CDobotClient();

        static bool InitNet();
        static void UinitNet();

        std::string GetIp() const;
        unsigned short GetPort() const;

        /// <summary>
        /// デバイスへ接続
        /// </summary>
        /// <param name="strIp">デバイスのアドレス</param>
        /// <param name="iPort">接続先ポート</param>
        /// <returns>true: 成功 / false: 失敗</returns>
        bool Connect(std::string strIp, unsigned short iPort);

        /// <summary>
        /// 切断
        /// </summary>
        void Disconnect();

        bool IsConnected() const;

    protected:
        virtual void OnConnected() = 0;
        virtual void OnDisconnected() = 0;

        void Construct();
        void DestroyConstruct();

        /// <summary>
        /// データ送信
        /// </summary>
        /// <param name="str">送信内容</param>
        /// <returns>true: 成功 / false: 失敗</returns>

        bool SendData(std::string str);

        /// <summary>
        /// 応答待ち
        /// </summary>
        /// <param name="iTimeoutMillsecond">待機時間（ミリ秒）</param>
        /// <returns>応答内容</returns>
        std::string WaitReply(int iTimeoutMillsecond);

        int Receive(char* pBuffer, int iLen);

    private:
        std::string m_strIp;
        unsigned short m_iPort;
        SOCKET m_sockListen;
        bool m_bIsConnect;
    };
}

