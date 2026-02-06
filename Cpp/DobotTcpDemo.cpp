
#include "DobotTcpDemo.h"
#include <fstream>

DobotTcpDemo::DobotTcpDemo()
{
    std::string robotIp = "192.168.5.1";
    unsigned int controlPort = 29999;
    unsigned int feekPort = 30004;

    std::cout << "接続開始" << std::endl;
    m_Dashboard.Connect(robotIp, controlPort);
    m_CFeedback.Connect(robotIp, feekPort);
    std::cout << "接続成功" << std::endl;
    m_CErrorInfoHelper.ParseControllerJsonFile("../alarmController.json");
    m_CErrorInfoHelper.ParseServoJsonFile("../alarmServo.json");
    threadGetFeedBackInfo = std::thread(&DobotTcpDemo::getFeedBackInfo, this);
    threadGetFeedBackInfo.detach();
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    threadClearRobotError = std::thread(&DobotTcpDemo::clearRobotError, this);
    threadClearRobotError.detach();
}

DobotTcpDemo::~DobotTcpDemo()
{
    m_Dashboard.Disconnect();
    m_CFeedback.Disconnect();
}

void DobotTcpDemo::moveRobot()
{
    double pointa[] = { -90, 20, 0, 0, 0, 0 };
    double pointb[] = { 90, 20, 0, 0, 0, 0 };
    m_Dashboard.EnableRobot();
    Dobot::CJointPoint ptPointa;
    Dobot::CJointPoint ptPointb;
    memcpy(&ptPointa, pointa, sizeof(ptPointa));
    memcpy(&ptPointb, pointb, sizeof(ptPointb));
    int currentCommandID = 0;
    while (true) {
        getCurrentCommandID(m_Dashboard.MovJ(ptPointa), currentCommandID);
        moveArriveFinish(ptPointa, currentCommandID);
        getCurrentCommandID(m_Dashboard.MovJ(ptPointb), currentCommandID);
        moveArriveFinish(ptPointb, currentCommandID);
    }
}

void DobotTcpDemo::getCurrentCommandID(std::string recvData, int& currentCommandID)
{
    std::cout << "recvData " << recvData << std::endl;
    currentCommandID = 2147483647;    // 初期値（int の最大値）
    if (recvData.find("device does not connected") != std::string::npos) {
        std::cout << "device does not connected " << std::endl;
        return;
    }

    if (recvData.find("send error") != std::string::npos) {
        std::cout << "send error" << std::endl;
        return;
    }

    // recvData 例: 0,{2},MovJ(joint={-90, 20, 0, 0, 0, 0})
    // vecRecv は文字列中の数値だけを抽出した配列例: [0, 2, -90, 20, 0, 0, 0, 0]
    std::vector<std::string> vecRecv = regexRecv(recvData);

    // vecRecv[0]: 指令送信成否（0=成功） / vecRecv[1]: 返却される動作指令の currentCommandID
    if (vecRecv.size() >= 2U && std::stoi(vecRecv[0]) == 0) {
        currentCommandID = std::stoi(vecRecv[1]);
    }
}

void DobotTcpDemo::getFeedBackInfo()
{
    std::cout << "Start GetFeedBackInfo" << std::endl;
    while (true) {
        {
            std::unique_lock<std::mutex> lockValue(m_mutexValue);
            feedbackData = m_CFeedback.GetFeedbackData();
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
}

void DobotTcpDemo::moveArriveFinish(const Dobot::CJointPoint& pt, int currentCommandID)
{
    std::cout << "Wait moveArriveFinish" << std::endl;
    while (true) {
        {
            std::unique_lock<std::mutex> lockValue(m_mutexValue);
            if (feedbackData.CurrentCommandId > currentCommandID) {
                break;
            }
            if (feedbackData.CurrentCommandId == currentCommandID && feedbackData.RobotMode == 5) {
                break;
            }
        }

        {
            std::unique_lock<std::mutex> lockValue(m_mutexState);
            if (finishState) {
                finishState = false;
                break;
            }
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
};

std::vector<std::string> DobotTcpDemo::regexRecv(std::string getRecvInfo)
{
    std::regex pattern("-?\\d+");
    std::smatch matches;
    std::string::const_iterator searchStart(getRecvInfo.cbegin());
    std::vector<std::string> vecErrorId;
    while (std::regex_search(searchStart, getRecvInfo.cend(), matches, pattern)) {
        for (auto& match : matches) {
            vecErrorId.push_back(match.str());
        }
        searchStart = matches.suffix().first;
    }
    return vecErrorId;
};

void DobotTcpDemo::clearRobotError()
{
    std::cout << "Start CheckRobotError" << std::endl;
    while (true) {
        {
            std::unique_lock<std::mutex> lockValue(m_mutexValue);
            if (feedbackData.ErrorStatus) {
                std::vector<std::string> errorIdVec = regexRecv(m_Dashboard.GetErrorID());
                for (int i = 1; i < errorIdVec.size(); i++) {
                    Dobot::CErrorInfoBean beanController;
                    Dobot::CErrorInfoBean beanServo;
                    if (std::stoi(errorIdVec[i]) != 0) {
                        printf("アラームコード: %s\n", errorIdVec[i].c_str());
                        if (m_CErrorInfoHelper.FindController(std::stoi(errorIdVec[i]), beanController)) {
                            printf("コントローラアラーム: %d, 原因: %s\n", beanController.id,
                                   beanController.en.description.c_str());
                        } else {
                            if (m_CErrorInfoHelper.FindServo(std::stoi(errorIdVec[i]), beanServo)) {
                                printf("サーボアラーム: %d, 原因: %s\n", beanServo.id,
                                       beanServo.en.description.c_str());
                            }
                        }
                    }
                }
                char choose[50] = { "" };
                std::cout << "1を入力するとエラーをクリアして動作を継続します:" << std::endl;
                std::cin >> choose;
                std::cout << "入力値: " << choose << std::endl;
                try {
                    int result = std::stoi(choose);
                    if (result == 1) {
                        std::cout << "エラーをクリアしました。動作を継続します。" << std::endl;
                        m_Dashboard.ClearError();
                    }
                } catch (const std::exception& e) {
                    std::cerr << "Exception caught: " << e.what() << std::endl;
                } catch (...) {
                    std::cerr << "Unknown exception caught." << std::endl;
                }
            } else {
                if (feedbackData.RobotMode == 11) {
                    std::cout << "ロボットが衝突を検知しました" << std::endl;
                    char choose[50] = { "" };
                    std::cout << "1を入力すると衝突状態をクリアして動作を継続します: " << std::endl;
                    std::cin >> choose;
                    std::cout << "入力値: " << choose << std::endl;
                    try {
                        int result = std::stoi(choose);
                        if (result == 1) {
                            std::cout << "エラーをクリアしました。動作を継続します。" << std::endl;
                            m_Dashboard.ClearError();
                        }
                    } catch (const std::exception& e) {
                        std::cerr << "Exception caught: " << e.what() << std::endl;
                    } catch (...) {
                        std::cerr << "Unknown exception caught." << std::endl;
                    }
                }

                if (!feedbackData.EnableStatus) {
                    std::cout << "ロボットがイネーブルされていません" << std::endl;
                    char choose[50] = { "" };
                    std::cout << "1を入力するとロボットをイネーブルします: " << std::endl;
                    std::cin >> choose;
                    std::cout << "入力値: " << choose << std::endl;
                    try {
                        int result = std::stoi(choose);
                        if (result == 1) {
                            std::cout << "ロボットをイネーブルしました！" << std::endl;
                            m_Dashboard.EnableRobot();
                        }
                    } catch (const std::exception& e) {
                        std::cerr << "Exception caught: " << e.what() << std::endl;
                    } catch (...) {
                        std::cerr << "Unknown exception caught." << std::endl;
                    }
                }

                if (!feedbackData.ErrorStatus && feedbackData.EnableStatus && feedbackData.RobotMode == 5) {
                    std::unique_lock<std::mutex> lockValue(m_mutexState);
                    finishState = true;
                }
            }
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(5000));
    }
}
