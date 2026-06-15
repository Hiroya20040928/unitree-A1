#include <librealsense2/rs.hpp>
#include <opencv2/opencv.hpp>
#include <iostream>
#include <vector>
#include <cmath>
#include <unistd.h>

int main() {
    rs2::context ctx;
    auto list = ctx.query_devices();
    if (list.size() == 0) {
        std::cerr << "RealSenseが見つかりません。" << std::endl;
        return -1;
    }
    std::cout << "【物理層】カメラASICを強制リセット中..." << std::endl;
    list[0].hardware_reset();
    sleep(3);

    rs2::pipeline pipe;
    rs2::config cfg;
    
    // 視差が最も安定する 848x480 プロファイル
    cfg.enable_stream(RS2_STREAM_DEPTH, 848, 480, RS2_FORMAT_Z16, 30);
    
    // ポストプロセッシングフィルタの初期化
    rs2::disparity_transform depth_to_disparity(true);
    rs2::disparity_transform disparity_to_depth(false);
    rs2::spatial_filter spatial;
    spatial.set_option(RS2_OPTION_FILTER_MAGNITUDE, 2);
    spatial.set_option(RS2_OPTION_FILTER_SMOOTH_ALPHA, 0.5);
    spatial.set_option(RS2_OPTION_HOLES_FILL, 2);

    try {
        rs2::pipeline_profile profile = pipe.start(cfg);
        
        // 【修正】D435i用の正しい近距離プリセット列挙型を注入
        auto depth_sensor = profile.get_device().first<rs2::depth_sensor>();
        if (depth_sensor.supports(RS2_OPTION_VISUAL_PRESET)) {
            std::cout << "【最適化】D435i専用 Short Range プリセットをASICに注入します。" << std::endl;
            depth_sensor.set_option(RS2_OPTION_VISUAL_PRESET, RS2_RS400_VISUAL_PRESET_SHORT_RANGE);
        }

        std::cout << "=== 完全に修正されたチョキ認識エンジン始動 ===" << std::endl;

        for (int frame_idx = 0; frame_idx < 500; frame_idx++) {
            rs2::frameset frames;
            if (!pipe.poll_for_frames(&frames)) {
                usleep(10000);
                continue;
            }

            rs2::frame depth_frame = frames.get_depth_frame();
            if (!depth_frame) continue;

            // 視差フィルタパイプライン
            depth_frame = depth_to_disparity.process(depth_frame);
            depth_frame = spatial.process(depth_frame);
            depth_frame = disparity_to_depth.process(depth_frame);
            
            rs2::depth_frame filtered_depth = depth_frame.as<rs2::depth_frame>();

            // 高速なポインタアクセスのためのバッファ取得
            const uint16_t* depth_ptr = (const uint16_t*)filtered_depth.get_data();
            
            cv::Mat hand_mask = cv::Mat::zeros(480, 848, CV_8UC1);
            int valid_pixels = 0;
            long long sum_x = 0, sum_y = 0;

            // 15cm (150mm) 〜 80cm (800mm) をミリメートル整数型で超高速スキャン
            for (int y = 0; y < 480; y++) {
                for (int x = 0; x < 848; x++) {
                    uint16_t d_value = depth_ptr[y * 848 + x];
                    if (d_value > 150 && d_value < 800) {
                        hand_mask.at<uchar>(y, x) = 255;
                        sum_x += x;
                        sum_y += y;
                        valid_pixels++;
                    }
                }
            }

            if (frame_idx % 3 == 0) {
                std::cout << "\033[2J\033[1;1H";
                std::cout << "=== [修正版 視差フィルタ・モニター] ===" << std::endl;
                
                // 確実に手が切り出されていれば起動する適正しきい値（画素数2000以上）
                if (valid_pixels > 2000) {
                    int cx = sum_x / valid_pixels;
                    int cy = sum_y / valid_pixels;

                    int box_size = 220;
                    int x_start = std::max(0, cx - box_size/2);
                    int y_start = std::max(0, cy - box_size/2);
                    x_start = std::min(x_start, 848 - box_size);
                    y_start = std::min(y_start, 480 - box_size);

                    cv::Mat cropped = hand_mask(cv::Rect(x_start, y_start, box_size, box_size));
                    
                    cv::Mat kernel = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(7, 7));
                    cv::morphologyEx(cropped, cropped, cv::MORPH_CLOSE, kernel);

                    cv::Mat roi;
                    cv::resize(cropped, roi, cv::Size(40, 15));

                    // アスキーアート描写
                    for (int y = 0; y < roi.rows; ++y) {
                        for (int x = 0; x < roi.cols; ++x) {
                            if (roi.at<uchar>(y, x) > 128) std::cout << "#";
                            else                           std::cout << ".";
                        }
                        std::cout << std::endl;
                    }
                    std::cout << "---------------------------------------" << std::endl;
                    std::cout << "【ログ】有効画素数: " << valid_pixels << std::endl;
                    std::cout << "【ログ】手までの実距離: " << filtered_depth.get_distance(cx, cy) << " メートル" << std::endl;
                    
                    // 幾何学的形状解析による最終判定
                    std::vector<std::vector<cv::Point>> contours;
                    cv::findContours(cropped, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
                    
                    if (!contours.empty()) {
                        size_t max_i = 0; double max_a = 0;
                        for(size_t i=0; i<contours.size(); i++) {
                            double a = cv::contourArea(contours[i]);
                            if(a > max_a) { max_a = a; max_i = i; }
                        }
                        
                        std::vector<int> hull;
                        cv::convexHull(contours[max_i], hull, false);
                        std::vector<cv::Vec4i> defects;
                        if(hull.size() > 3) cv::convexityDefects(contours[max_i], hull, defects);
                        
                        int fingers = 0;
                        cv::Moments m = cv::moments(contours[max_i]);
                        cv::Point center(m.m10 / m.m00, m.m01 / m.m00);

                        for(const auto& d : defects) {
                            if((d[3]/256.0f) > 15.0f) { 
                                cv::Point p_far = contours[max_i][d[2]];
                                if (p_far.y < (center.y + 30)) {
                                    fingers++;
                                }
                            }
                        }
                        
                        if (fingers == 1) {
                            std::cout << "🚀 【確実認識】チョキ（A1通信接続可能）" << std::endl;
                        } else {
                            std::cout << "💤 待機中（手をレンズに向けてチョキにしてください）" << std::endl;
                        }
                    }
                } else {
                    std::cout << "\n\n   [ カメラの前（20cm〜50cm）に手を出してください ]\n" << std::endl;
                    std::cout << "【ログ】有効画素数: " << valid_pixels << " (2000以上で起動)" << std::endl;
                }
                usleep(30000);
            }
        }
    } catch (const std::exception& e) {
        std::cerr << "エラー: " << e.what() << std::endl;
        return -1;
    }

    pipe.stop();
    return 0;
}
