#include <iostream>
#include <librealsense2/rs.hpp>
#include <opencv2/opencv.hpp>
#include <unistd.h>
#include <cmath>
#include <vector>

#include "/home/unitree/unitree_legged_sdk/include/unitree_legged_sdk/unitree_legged_sdk.h"

using namespace UNITREE_LEGGED_SDK;

// A1通信用グローバルオブジェクト
UDP udp(8080, "192.168.123.161", 8082, sizeof(HighCmd), sizeof(HighState));
HighCmd high_cmd = {0};
HighState high_state = {0};

// スレッド間通信用のスレッドセーフなグローバル制御変数
float target_body_height = 0.0f;
int target_mode = 0;

// 肌色閾値（検知を容易にするためかなり広く設定）
cv::Scalar SKIN_MIN(0, 10, 20);
cv::Scalar SKIN_MAX(40, 180, 255);

// 端末への視覚化インジケータ
void drawTerminalMap(const cv::Mat& mask) {
    cv::Mat small_mask;
    cv::resize(mask, small_mask, cv::Size(40, 12));
    
    std::cout << "\033[2J\033[1;1H"; // 画面クリア
    std::cout << "=== [RealSense 視覚化デバッグモニター] ===" << std::endl;
    for (int y = 0; y < small_mask.rows; ++y) {
        for (int x = 0; x < small_mask.cols; ++x) {
            if (small_mask.at<uchar>(y, x) > 128) {
                std::cout << "#"; // 肌色
            } else {
                std::cout << "."; // 背景
            }
        }
        std::cout << std::endl;
    }
    std::cout << "==========================================" << std::endl;
}

int countFingers(const cv::Mat& frame) {
    cv::Mat hsv, mask;
    cv::cvtColor(frame, hsv, cv::COLOR_BGR2HSV);
    cv::inRange(hsv, SKIN_MIN, SKIN_MAX, mask);

    cv::blur(mask, mask, cv::Size(5, 5));
    
    // アスキーアートを強制描写
    drawTerminalMap(mask);

    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

    if (contours.empty()) {
        std::cout << "【ログ】輪郭（白い塊）がありません。" << std::endl;
        return 0;
    }

    size_t max_idx = 0;
    double max_area = 0;
    for (size_t i = 0; i < contours.size(); i++) {
        double area = cv::contourArea(contours[i]);
        if (area > max_area) {
            max_area = area;
            max_idx = i;
        }
    }

    std::cout << "【ログ】最大オブジェクトの面積: " << (int)max_area << " (基準値: 5000)" << std::endl;

    if (max_area < 5000) {
        std::cout << "【ログ】オブジェクトが小さすぎます。カメラに近づけてください。" << std::endl;
        return 0;
    }

    std::vector<int> hull_ints;
    cv::convexHull(contours[max_idx], hull_ints, false);

    std::vector<cv::Vec4i> defects;
    if (hull_ints.size() > 3) {
        cv::convexityDefects(contours[max_idx], hull_ints, defects);
    }

    int finger_count = 0;
    for (const auto& defect : defects) {
        int far_idx = defect[2];
        float depth = defect[3] / 256.0f;

        if (depth > 15.0f) {
            cv::Point p_start = contours[max_idx][defect[0]];
            cv::Point p_end = contours[max_idx][defect[1]];
            cv::Point p_far = contours[max_idx][far_idx];

            float a = std::sqrt(std::pow(p_end.x - p_start.x, 2) + std::pow(p_end.y - p_start.y, 2));
            float b = std::sqrt(std::pow(p_far.x - p_start.x, 2) + std::pow(p_far.y - p_start.y, 2));
            float c = std::sqrt(std::pow(p_end.x - p_far.x, 2) + std::pow(p_far.y - p_far.y, 2));
            float angle = std::acos((b*b + c*c - a*a) / (2*b*c)) * 57.29f;

            if (angle < 90.0f) {
                finger_count++;
            }
        }
    }

    std::cout << "【ログ】指の股の数: " << finger_count << " (1 = チョキ)" << std::endl;

    if (finger_count == 1) return 2; 
    return 0;
}

// A1の通信タスク（高速定周期スレッド）
void UDPSend() { udp.Send(); }
void UDPRecv() { udp.Recv(); }

// ロボット制御タスク（メインスレッドが計算した値を安全に詰め替えて送信するだけにする）
void RobotControlLoop() {
    udp.GetRecv(high_state);

    high_cmd.mode = target_mode;
    high_cmd.bodyHeight = target_body_height;
    high_cmd.levelFlag = HIGHLEVEL;

    udp.SetSend(high_cmd);
}

int main() {
    // --- 1. RealSense 初期化 ---
    rs2::pipeline rs_pipe;
    rs2::config cfg;
    cfg.enable_stream(RS2_STREAM_COLOR, 640, 480, RS2_FORMAT_BGR8, 30);
    try {
        rs_pipe.start(cfg);
    } catch (const rs2::error & e) {
        std::cerr << "RealSense起動失敗: " << e.what() << std::endl;
        return -1;
    }

    // --- 2. Unitree UDP 初期設定とスレッド開始 ---
    udp.InitCmdData(high_cmd);

    LoopFunc loop_udpSend("udp_send", 0.002, UDPSend);
    LoopFunc loop_udpRecv("udp_recv", 0.002, UDPRecv);
    LoopFunc loop_control("control_loop", 0.01, RobotControlLoop); // 10ms周期

    loop_udpSend.start();
    loop_udpRecv.start();
    loop_control.start();

    std::cout << "A1 通信スレッド起動完了。メインループによる画像処理を開始します。" << std::endl;

    // --- 3. メインスレッドでの画像処理（スレッド競合を100%回避） ---
    while (true) {
        try {
            rs2::frameset frames = rs_pipe.wait_for_frames(1000); // タイムアウト1秒
            rs2::frame color_frame = frames.get_color_frame();
            if (!color_frame) continue;

            cv::Mat frame(cv::Size(640, 480), CV_8UC3, (void*)color_frame.get_data(), cv::Mat::AUTO_STEP);

            // 指の数（チョキ）を判定
            int fingers = countFingers(frame);

            if (fingers == 2) {
                // チョキ認識時：グローバル変数を書き換え
                target_mode = 1;
                target_body_height = -0.12f;
            } else {
                // 通常時
                target_mode = 0;
                target_body_height = 0.0f;
            }
        } catch (const std::exception& e) {
            std::cout << "【警告】メインループでのRealSenseフレーム取得タイムアウト" << std::endl;
        }
    }

    rs_pipe.stop();
    return 0;
}
