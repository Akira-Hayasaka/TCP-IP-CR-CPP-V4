# CMake の最低バージョンを指定
cmake_minimum_required(VERSION 3.10)

# プロジェクト名を設定
project(DobotTcpDemo)

# C++14 標準を有効化
set(CMAKE_CXX_STANDARD 14)
add_compile_options(-std=c++14)
# ビルドタイプを Release に設定
set(CMAKE_BUILD_TYPE Release)


# 実行ファイルを追加し、ソースファイルを指定
add_executable(DobotTcpDemo main.cpp DobotTcpDemo.cpp api/BitConverter.cpp api/Dashboard.cpp api/DobotClient.cpp api/ErrorInfoBean.cpp api/ErrorInfoHelper.cpp api/Feedback.cpp)

# スレッドライブラリをリンク
find_package(Threads REQUIRED)
target_link_libraries(DobotTcpDemo Threads::Threads)