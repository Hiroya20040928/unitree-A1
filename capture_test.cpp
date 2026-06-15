#include <librealsense2/rs.hpp>
#include <opencv2/opencv.hpp>
#include <iostream>
#include <unistd.h>

int main() {
    rs2::context ctx;
    auto list = ctx.query_devices();
    if (list.size() == 0) {
        std::cerr << "エラー: RealSenseデバイスが見つかりません。" << std::endl;
        return -1;
    }
    
    // 1. ビューワー起動時と同じ「ハードウェア強制リセット」をシステム命令で送る
    std::cout << "【対策1】RealSenseハードウェアをリセット中..." << std::endl;
    auto dev = list[0];
    dev.hardware_reset();
    
    // リセット後の再認識を確実に待つため、3秒スリープ
    std::cout << "再起動を待っています（3秒スリープ）..." << std::endl;
    sleep(3);

    rs2::pipeline pipe;
    rs2::config cfg;
    
    // 最もシンプルで確実に通るプロファイル
    cfg.enable_stream(RS2_STREAM_COLOR, 0, 640, 480, RS2_FORMAT_BGR8, 30);
    
    try {
        std::cout << "パイプラインを開始します..." << std::endl;
        rs2::pipeline_profile profile = pipe.start(cfg);
        
        // 2. メタデータの不整合によるフリーズを避けるため、センサーオプションを調整
        auto color_sensor = profile.get_device().first<rs2::color_sensor>();
        if (color_sensor.supports(RS2_OPTION_ENABLE_AUTO_EXPOSURE)) {
            color_sensor.set_option(RS2_OPTION_ENABLE_AUTO_EXPOSURE, 1); // あえてONにして揺さぶりをかける
        }

        std::cout << "【対策2】ノンブロッキング取得（15秒間ループ）でフレームを強引に引きずり出します..." << std::endl;
        
        rs2::frameset frames;
        bool success = false;
        
        // wait_for_frames を一切使わず、15秒間超高速でバッファを叩き続ける
        for (int i = 0; i < 500; i++) {
            if (pipe.poll_for_frames(&frames)) {
                rs2::frame color = frames.get_color_frame();
                if (color) {
                    std::cout << "★パケットの着信を検知しました！フレームを展開します。" << std::endl;
                    cv::Mat mat(cv::Size(640, 480), CV_8UC3, (void*)color.get_data(), cv::Mat::AUTO_STEP);
                    cv::imwrite("/home/unitree/test_hand.png", mat);
                    success = true;
                    break;
                }
            }
            usleep(30000); // 30msごとにバッファをスキャン
        }
        
        if (success) {
            std::cout << "【完全勝利】/home/unitree/test_hand.png の保存に成功しました！" << std::endl;
        } else {
            std::cout << "【警告】リセットを行いましたが、データストリームのパケットが到達しません。" << std::endl;
        }
        
    } catch (const std::exception& e) {
        std::cerr << "エラー原因: " << e.what() << std::endl;
        return -1;
    }
    
    try { pipe.stop(); } catch(...) {}
    return 0;
}
