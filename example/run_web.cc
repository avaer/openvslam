/* #ifdef USE_PANGOLIN_VIEWER
#include "pangolin_viewer/viewer.h"
#elif USE_SOCKET_PUBLISHER
#include "socket_publisher/publisher.h"
#endif */
#include <emscripten.h>

#include "openvslam/system.h"
#include "openvslam/config.h"
#include "openvslam/util/stereo_rectifier.h"

#include <iostream>
#include <chrono>
#include <numeric>

#include <opencv2/core/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
// #include <opencv2/videoio.hpp>
// #include <spdlog/spdlog.h>
#include <popl.hpp>

#include <socket_publisher/data_serializer2.h>

/* #define CV_CN_MAX     512
#define CV_CN_SHIFT   3
#define CV_DEPTH_MAX  (1 << CV_CN_SHIFT)

#define CV_8U   0
#define CV_8S   1
#define CV_16U  2
#define CV_16S  3
#define CV_32S  4
#define CV_32F  5
#define CV_64F  6
#define CV_USRTYPE1 7

#define CV_MAT_DEPTH_MASK       (CV_DEPTH_MAX - 1)
#define CV_MAT_DEPTH(flags)     ((flags) & CV_MAT_DEPTH_MASK)

#define CV_MAKETYPE(depth,cn) (CV_MAT_DEPTH(depth) + (((cn)-1) << CV_CN_SHIFT))
#define CV_MAKE_TYPE CV_MAKETYPE

#define CV_8UC1 CV_MAKETYPE(CV_8U,1)
#define CV_8UC2 CV_MAKETYPE(CV_8U,2)
#define CV_8UC3 CV_MAKETYPE(CV_8U,3)
#define CV_8UC4 CV_MAKETYPE(CV_8U,4)
#define CV_8UC(n) CV_MAKETYPE(CV_8U,(n)) */

// detection

class MonoState {
  std::shared_ptr<openvslam::config> cfg;
  openvslam::system SLAM;
  cv::Mat mask;
  cv::Mat frame;
  double timestamp;
  // std::vector<double> track_times;
  unsigned int num_frame;
  bool is_not_end;
  std::unique_ptr<socket_publisher::data_serializer2> data_serializer2_;

public:
  MonoState(
    const int rows, const int cols, const int type,
    const std::string& config_file_data,
    const std::string& vocab_file_data
  ) :
    cfg(std::make_shared<openvslam::config>(config_file_data)),
    SLAM(cfg, vocab_file_data),
    frame(rows, cols, type),
    mask{},
    timestamp(0.0),
    num_frame(0),
    is_not_end(true)
  {
    // startup the SLAM process
    SLAM.startup();

    auto frame_publisher = SLAM.get_frame_publisher();
    auto map_publisher = SLAM.get_map_publisher();
    const auto camera = cfg->camera_;
    const auto img_cols = (camera->cols_ < 1) ? 640 : camera->cols_;
    const auto img_rows = (camera->rows_ < 1) ? 480 : camera->rows_;
    data_serializer2_.reset(new socket_publisher::data_serializer2(frame_publisher, map_publisher, img_cols, img_rows));

    // create a viewer object
    // and pass the frame_publisher and the map_publisher
/* #ifdef USE_PANGOLIN_VIEWER
    pangolin_viewer::viewer viewer(cfg, &SLAM, SLAM.get_frame_publisher(), SLAM.get_map_publisher());
#elif USE_SOCKET_PUBLISHER
    socket_publisher::publisher publisher(cfg, &SLAM, SLAM.get_frame_publisher(), SLAM.get_map_publisher());
#endif */

    /* auto video = cv::VideoCapture(cam_num);
    if (!video.isOpened()) {
        spdlog::critical("cannot open a camera {}", cam_num);
        SLAM.shutdown();
        return;
    } */

    /* cv::Mat frame(rows, cols, type);
    double timestamp = 0.0;
    std::vector<double> track_times;

    unsigned int num_frame = 0;

    bool is_not_end = true;
    // run the SLAM in another thread
    std::thread thread([&]() {
        if (is_not_end) {
            // check if the termination of SLAM system is requested or not
            if (SLAM.terminate_is_requested()) {
                break;
            }

            // is_not_end = video.read(frame); // XXX
            if (frame.empty()) {
                continue;
            }
            if (scale != 1.0) {
                cv::resize(frame, frame, cv::Size(), scale, scale, cv::INTER_LINEAR);
            }

            const auto tp_1 = std::chrono::steady_clock::now();

            // input the current frame and estimate the camera pose
            SLAM.feed_monocular_frame(frame, timestamp, mask);

            const auto tp_2 = std::chrono::steady_clock::now();

            const auto track_time = std::chrono::duration_cast<std::chrono::duration<double>>(tp_2 - tp_1).count();
            track_times.push_back(track_time);

            timestamp += 1.0 / cfg->camera_->fps_;
            ++num_frame;
        }
    }); */

    // run the viewer in the current thread
/* #ifdef USE_PANGOLIN_VIEWER
    viewer.run();
#elif USE_SOCKET_PUBLISHER
    publisher.run();
#endif */

    // thread.join();
  }
  ~MonoState() {
    // wait until the loop BA is finished
    /* while (SLAM.loop_BA_is_running()) {
      std::this_thread::sleep_for(std::chrono::microseconds(5000));
    } */

    SLAM.shutdown();
  }
  unsigned char *getFrameBuf() {
    return frame.ptr();
  }
  void pushFrame() {
    // if (is_not_end) {
        // check if the termination of SLAM system is requested or not
        /* if (SLAM.terminate_is_requested()) {
            return;
        } */

        // is_not_end = video.read(frame); // XXX
        // memcpy(frame.ptr(), frameBuf, frame.total() * frame.elemSize());
        /* if (frame.empty()) {
            continue;
        } */
        /*if (scale != 1.0) {
            cv::resize(frame, frame, cv::Size(), scale, scale, cv::INTER_LINEAR);
        } */

        // const auto tp_1 = std::chrono::steady_clock::now();

        // input the current frame and estimate the camera pose
        SLAM.feed_monocular_frame(frame, timestamp, mask);

        /* const auto tp_2 = std::chrono::steady_clock::now();

        const auto track_time = std::chrono::duration_cast<std::chrono::duration<double>>(tp_2 - tp_1).count();
        track_times.push_back(track_time); */

        timestamp += 1.0 / cfg->camera_->fps_;
        ++num_frame;
    // }
  }
  void pullUpdate(unsigned char *data, unsigned int *length) {
    data_serializer2_->serialize_map_diff_binary(data, length);
  }
};

EMSCRIPTEN_KEEPALIVE MonoState *create_mono(
    const int rows, const int cols, const int type,
    const char *config_file_data,
    unsigned int config_file_data_size,
    const char *vocab_file_data,
    unsigned int vocab_file_data_size
) {
    std::string config_file_data_string(config_file_data, config_file_data_size);
    std::string vocab_file_data_string(vocab_file_data, vocab_file_data_size);
    free((void *)config_file_data);
    free((void *)vocab_file_data);
    return new MonoState(rows, cols, type, config_file_data_string, vocab_file_data_string);
}
EMSCRIPTEN_KEEPALIVE unsigned char *get_framebuf_mono(MonoState *monoState) {
    return monoState->getFrameBuf();
}
EMSCRIPTEN_KEEPALIVE void push_frame_mono(MonoState *monoState) {
    monoState->pushFrame();
}
EMSCRIPTEN_KEEPALIVE void pull_update_mono(MonoState *monoState, unsigned char *data, unsigned int *length) {
    monoState->pullUpdate(data, length);
}

EMSCRIPTEN_KEEPALIVE void stereo_tracking(
    const int rows, const int cols, const int type,
    const std::string& config_file_data,
    const std::string& vocab_file_data,
    const float scale
) {
    std::shared_ptr<openvslam::config> cfg = std::make_shared<openvslam::config>(config_file_data);

    const cv::Mat mask;// = mask_img_path.empty() ? cv::Mat{} : cv::imread(mask_img_path, cv::IMREAD_GRAYSCALE);
    // build a SLAM system
    openvslam::system SLAM(cfg, vocab_file_data);
    // startup the SLAM process
    SLAM.startup();

    // create a viewer object
    // and pass the frame_publisher and the map_publisher
/* #ifdef USE_PANGOLIN_VIEWER
    pangolin_viewer::viewer viewer(cfg, &SLAM, SLAM.get_frame_publisher(), SLAM.get_map_publisher());
#elif USE_SOCKET_PUBLISHER
    socket_publisher::publisher publisher(cfg, &SLAM, SLAM.get_frame_publisher(), SLAM.get_map_publisher());
#endif */

    /* cv::VideoCapture videos[2];
    for (int i = 0; i < 2; i++) {
        videos[i] = cv::VideoCapture(cam_num + i);
        if (!videos[i].isOpened()) {
            spdlog::critical("cannot open a camera {}", cam_num + i);
            SLAM.shutdown();
            return;
        }
    } */

    const openvslam::util::stereo_rectifier rectifier(cfg);

    cv::Mat frames[2];
    cv::Mat frames_rectified[2];
    double timestamp = 0.0;
    std::vector<double> track_times;
    unsigned int num_frame = 0;

    bool is_not_end = true;
    // run the SLAM in another thread
    // std::thread thread([&]() {
        while (is_not_end) {
            // check if the termination of SLAM system is requested or not
            if (SLAM.terminate_is_requested()) {
                break;
            }

            // is_not_end = videos[0].read(frames[0]) && videos[1].read(frames[1]); // XXX
            if (frames[0].empty() || frames[1].empty()) {
                continue;
            }
            for (int i = 0; i < 2; i++) {
                if (scale != 1.0) {
                    cv::resize(frames[i], frames[i], cv::Size(), scale, scale, cv::INTER_LINEAR);
                }
            }
            rectifier.rectify(frames[0], frames[1], frames_rectified[0], frames_rectified[1]);

            const auto tp_1 = std::chrono::steady_clock::now();

            // input the current frame and estimate the camera pose
            SLAM.feed_stereo_frame(frames_rectified[0], frames_rectified[1], timestamp, mask);

            const auto tp_2 = std::chrono::steady_clock::now();

            const auto track_time = std::chrono::duration_cast<std::chrono::duration<double>>(tp_2 - tp_1).count();
            track_times.push_back(track_time);

            timestamp += 1.0 / cfg->camera_->fps_;
            ++num_frame;
        }

        // wait until the loop BA is finished
        /* while (SLAM.loop_BA_is_running()) {
            std::this_thread::sleep_for(std::chrono::microseconds(5000));
        } */
    // });

    // run the viewer in the current thread
/* #ifdef USE_PANGOLIN_VIEWER
    viewer.run();
#elif USE_SOCKET_PUBLISHER
    publisher.run();
#endif */

    // thread.join();

    // shutdown the SLAM process
    SLAM.shutdown();

    /* if (!map_db_path.empty()) {
        // output the map database
        SLAM.save_map_database(map_db_path);
    } */

    std::sort(track_times.begin(), track_times.end());
    const auto total_track_time = std::accumulate(track_times.begin(), track_times.end(), 0.0);
    std::cout << "median tracking time: " << track_times.at(track_times.size() / 2) << "[s]" << std::endl;
    std::cout << "mean tracking time: " << total_track_time / track_times.size() << "[s]" << std::endl;
}

/* int main(int argc, char* argv[]) {
    // create options
    popl::OptionParser op("Allowed options");
    auto help = op.add<popl::Switch>("h", "help", "produce help message");
    auto vocab_file_path = op.add<popl::Value<std::string>>("v", "vocab", "vocabulary file path");
    auto cam_num = op.add<popl::Value<unsigned int>>("n", "number", "camera number");
    auto config_file_path = op.add<popl::Value<std::string>>("c", "config", "config file path");
    auto mask_img_path = op.add<popl::Value<std::string>>("", "mask", "mask image path", "");
    auto scale = op.add<popl::Value<float>>("s", "scale", "scaling ratio of images", 1.0);
    auto map_db_path = op.add<popl::Value<std::string>>("p", "map-db", "store a map database at this path after SLAM", "");
    auto debug_mode = op.add<popl::Switch>("", "debug", "debug mode");
    try {
        op.parse(argc, argv);
    }
    catch (const std::exception& e) {
        std::cerr << e.what() << std::endl;
        std::cerr << std::endl;
        std::cerr << op << std::endl;
        return EXIT_FAILURE;
    }

    // check validness of options
    if (help->is_set()) {
        std::cerr << op << std::endl;
        return EXIT_FAILURE;
    }
    if (!vocab_file_path->is_set() || !cam_num->is_set()
        || !config_file_path->is_set()) {
        std::cerr << "invalid arguments" << std::endl;
        std::cerr << std::endl;
        std::cerr << op << std::endl;
        return EXIT_FAILURE;
    }

    // setup logger
    spdlog::set_pattern("[%Y-%m-%d %H:%M:%S.%e] %^[%L] %v%$");
    if (debug_mode->is_set()) {
        spdlog::set_level(spdlog::level::debug);
    }
    else {
        spdlog::set_level(spdlog::level::info);
    }

    // load configuration
    std::shared_ptr<openvslam::config> cfg;
    try {
        cfg = std::make_shared<openvslam::config>(config_file_path->value());
    }
    catch (const std::exception& e) {
        std::cerr << e.what() << std::endl;
        return EXIT_FAILURE;
    }

    // run tracking
    if (cfg->camera_->setup_type_ == openvslam::camera::setup_type_t::Monocular) {
        mono_tracking(cfg, vocab_file_path->value(), cam_num->value(), mask_img_path->value(),
                      scale->value(), map_db_path->value());
    }
    else if (cfg->camera_->setup_type_ == openvslam::camera::setup_type_t::Stereo) {
        stereo_tracking(cfg, vocab_file_path->value(), cam_num->value(), mask_img_path->value(),
                        scale->value(), map_db_path->value());
    }
    else {
        throw std::runtime_error("Invalid setup type: " + cfg->camera_->get_setup_type_string());
    }

    return EXIT_SUCCESS;
} */