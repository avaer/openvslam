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
#include <spdlog/spdlog.h>
#include <popl.hpp>

#include <socket_publisher/data_serializer2.h>

/* #define CV_8U   0
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
    while (SLAM.loop_BA_is_running()) {
      std::this_thread::sleep_for(std::chrono::microseconds(5000));
    }

    SLAM.shutdown();
  }
  unsigned char *getFrameBuf() {
    return frame.ptr();
  }
  void pushFrame() {
    // if (is_not_end) {
        // check if the termination of SLAM system is requested or not
        if (SLAM.terminate_is_requested()) {
            return;
        }

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
    std::thread thread([&]() {
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
        while (SLAM.loop_BA_is_running()) {
            std::this_thread::sleep_for(std::chrono::microseconds(5000));
        }
    });

    // run the viewer in the current thread
/* #ifdef USE_PANGOLIN_VIEWER
    viewer.run();
#elif USE_SOCKET_PUBLISHER
    publisher.run();
#endif */

    thread.join();

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

// calibration

// #include <opencv2/core/utility.hpp>
#include <opencv2/calib3d.hpp>

#include <cctype>
#include <stdio.h>
#include <string.h>
#include <time.h>
#include <iostream>

using namespace cv;
using namespace std;

/* const char * usage =
" \nexample command line for calibration from a live feed.\n"
"   calibration  -w=4 -h=5 -s=0.025 -o=camera.yml -op -oe\n"
" \n"
" example command line for calibration from a list of stored images:\n"
"   imagelist_creator image_list.xml *.png\n"
"   calibration -w=4 -h=5 -s=0.025 -o=camera.yml -op -oe image_list.xml\n"
" where image_list.xml is the standard OpenCV XML/YAML\n"
" use imagelist_creator to create the xml or yaml list\n"
" file consisting of the list of strings, e.g.:\n"
" \n"
"<?xml version=\"1.0\"?>\n"
"<opencv_storage>\n"
"<images>\n"
"view000.png\n"
"view001.png\n"
"<!-- view002.png -->\n"
"view003.png\n"
"view010.png\n"
"one_extra_view.jpg\n"
"</images>\n"
"</opencv_storage>\n";




const char* liveCaptureHelp =
    "When the live video from camera is used as input, the following hot-keys may be used:\n"
        "  <ESC>, 'q' - quit the program\n"
        "  'g' - start capturing images\n"
        "  'u' - switch undistortion on/off\n";

static void help()
{
    printf( "This is a camera calibration sample.\n"
        "Usage: calibration\n"
        "     -w=<board_width>         # the number of inner corners per one of board dimension\n"
        "     -h=<board_height>        # the number of inner corners per another board dimension\n"
        "     [-pt=<pattern>]          # the type of pattern: chessboard or circles' grid\n"
        "     [-n=<number_of_frames>]  # the number of frames to use for calibration\n"
        "                              # (if not specified, it will be set to the number\n"
        "                              #  of board views actually available)\n"
        "     [-d=<delay>]             # a minimum delay in ms between subsequent attempts to capture a next view\n"
        "                              # (used only for video capturing)\n"
        "     [-s=<squareSize>]       # square size in some user-defined units (1 by default)\n"
        "     [-o=<out_camera_params>] # the output filename for intrinsic [and extrinsic] parameters\n"
        "     [-op]                    # write detected feature points\n"
        "     [-oe]                    # write extrinsic parameters\n"
        "     [-oo]                    # write refined 3D object points\n"
        "     [-zt]                    # assume zero tangential distortion\n"
        "     [-a=<aspectRatio>]      # fix aspect ratio (fx/fy)\n"
        "     [-p]                     # fix the principal point at the center\n"
        "     [-v]                     # flip the captured images around the horizontal axis\n"
        "     [-V]                     # use a video file, and not an image list, uses\n"
        "                              # [input_data] string for the video file name\n"
        "     [-su]                    # show undistorted images after calibration\n"
        "     [-ws=<number_of_pixel>]  # Half of search window for cornerSubPix (11 by default)\n"
        "     [-dt=<distance>]         # actual distance between top-left and top-right corners of\n"
        "                              # the calibration grid. If this parameter is specified, a more\n"
        "                              # accurate calibration method will be used which may be better\n"
        "                              # with inaccurate, roughly planar target.\n"
        "     [input_data]             # input data, one of the following:\n"
        "                              #  - text file with a list of the images of the board\n"
        "                              #    the text file can be generated with imagelist_creator\n"
        "                              #  - name of video file with a video of the board\n"
        "                              # if input_data not specified, a live view from the camera is used\n"
        "\n" );
    printf("\n%s",usage);
    printf( "\n%s", liveCaptureHelp );
} */

enum { DETECTION = 0, CAPTURING = 1, CALIBRATED = 2 };
enum Pattern { CHESSBOARD, CIRCLES_GRID, ASYMMETRIC_CIRCLES_GRID };

static double computeReprojectionErrors(
        const vector<vector<Point3f> >& objectPoints,
        const vector<vector<Point2f> >& imagePoints,
        const vector<Mat>& rvecs, const vector<Mat>& tvecs,
        const Mat& cameraMatrix, const Mat& distCoeffs,
        vector<float>& perViewErrors )
{
    vector<Point2f> imagePoints2;
    int i, totalPoints = 0;
    double totalErr = 0, err;
    perViewErrors.resize(objectPoints.size());

    for( i = 0; i < (int)objectPoints.size(); i++ )
    {
        projectPoints(Mat(objectPoints[i]), rvecs[i], tvecs[i],
                      cameraMatrix, distCoeffs, imagePoints2);
        err = norm(Mat(imagePoints[i]), Mat(imagePoints2), NORM_L2);
        int n = (int)objectPoints[i].size();
        perViewErrors[i] = (float)std::sqrt(err*err/n);
        totalErr += err*err;
        totalPoints += n;
    }

    return std::sqrt(totalErr/totalPoints);
}

static void calcChessboardCorners(Size boardSize, float squareSize, vector<Point3f>& corners, Pattern patternType = CHESSBOARD)
{
    corners.resize(0);

    switch(patternType)
    {
      case CHESSBOARD:
      case CIRCLES_GRID:
        for( int i = 0; i < boardSize.height; i++ )
            for( int j = 0; j < boardSize.width; j++ )
                corners.push_back(Point3f(float(j*squareSize),
                                          float(i*squareSize), 0));
        break;

      case ASYMMETRIC_CIRCLES_GRID:
        for( int i = 0; i < boardSize.height; i++ )
            for( int j = 0; j < boardSize.width; j++ )
                corners.push_back(Point3f(float((2*j + i % 2)*squareSize),
                                          float(i*squareSize), 0));
        break;

      default:
        CV_Error(Error::StsBadArg, "Unknown pattern type\n");
    }
}

static bool runCalibration( vector<vector<Point2f> > imagePoints,
                    Size imageSize, Size boardSize, Pattern patternType,
                    float squareSize, float aspectRatio,
                    float grid_width, bool release_object,
                    int flags, Mat& cameraMatrix, Mat& distCoeffs,
                    vector<Mat>& rvecs, vector<Mat>& tvecs,
                    vector<float>& reprojErrs,
                    vector<Point3f>& newObjPoints,
                    double& totalAvgErr)
{
    cameraMatrix = Mat::eye(3, 3, CV_64F);
    if( flags & CALIB_FIX_ASPECT_RATIO )
        cameraMatrix.at<double>(0,0) = aspectRatio;

    distCoeffs = Mat::zeros(8, 1, CV_64F);

    vector<vector<Point3f> > objectPoints(1);
    calcChessboardCorners(boardSize, squareSize, objectPoints[0], patternType);
    objectPoints[0][boardSize.width - 1].x = objectPoints[0][0].x + grid_width;
    newObjPoints = objectPoints[0];

    objectPoints.resize(imagePoints.size(),objectPoints[0]);

    double rms;
    int iFixedPoint = -1;
    if (release_object)
        iFixedPoint = boardSize.width - 1;
    rms = calibrateCameraRO(objectPoints, imagePoints, imageSize, iFixedPoint,
                            cameraMatrix, distCoeffs, rvecs, tvecs, newObjPoints,
                            flags | CALIB_FIX_K3 | CALIB_USE_LU);
    printf("RMS error reported by calibrateCamera: %g\n", rms);

    bool ok = checkRange(cameraMatrix) && checkRange(distCoeffs);

    if (release_object) {
        cout << "New board corners: " << endl;
        cout << newObjPoints[0] << endl;
        cout << newObjPoints[boardSize.width - 1] << endl;
        cout << newObjPoints[boardSize.width * (boardSize.height - 1)] << endl;
        cout << newObjPoints.back() << endl;
    }

    objectPoints.clear();
    objectPoints.resize(imagePoints.size(), newObjPoints);
    totalAvgErr = computeReprojectionErrors(objectPoints, imagePoints,
                rvecs, tvecs, cameraMatrix, distCoeffs, reprojErrs);

    return ok;
}


/* static void saveCameraParams( const string& filename,
                       Size imageSize, Size boardSize,
                       float squareSize, float aspectRatio, int flags,
                       const Mat& cameraMatrix, const Mat& distCoeffs,
                       const vector<Mat>& rvecs, const vector<Mat>& tvecs,
                       const vector<float>& reprojErrs,
                       const vector<vector<Point2f> >& imagePoints,
                       const vector<Point3f>& newObjPoints,
                       double totalAvgErr )
{
    FileStorage fs( filename, FileStorage::WRITE );

    time_t tt;
    time( &tt );
    struct tm *t2 = localtime( &tt );
    char buf[1024];
    strftime( buf, sizeof(buf)-1, "%c", t2 );

    fs << "calibration_time" << buf;

    if( !rvecs.empty() || !reprojErrs.empty() )
        fs << "nframes" << (int)std::max(rvecs.size(), reprojErrs.size());
    fs << "image_width" << imageSize.width;
    fs << "image_height" << imageSize.height;
    fs << "board_width" << boardSize.width;
    fs << "board_height" << boardSize.height;
    fs << "square_size" << squareSize;

    if( flags & CALIB_FIX_ASPECT_RATIO )
        fs << "aspectRatio" << aspectRatio;

    if( flags != 0 )
    {
        sprintf( buf, "flags: %s%s%s%s",
            flags & CALIB_USE_INTRINSIC_GUESS ? "+use_intrinsic_guess" : "",
            flags & CALIB_FIX_ASPECT_RATIO ? "+fix_aspectRatio" : "",
            flags & CALIB_FIX_PRINCIPAL_POINT ? "+fix_principal_point" : "",
            flags & CALIB_ZERO_TANGENT_DIST ? "+zero_tangent_dist" : "" );
        //cvWriteComment( *fs, buf, 0 );
    }

    fs << "flags" << flags;

    fs << "camera_matrix" << cameraMatrix;
    fs << "distortion_coefficients" << distCoeffs;

    fs << "avg_reprojection_error" << totalAvgErr;
    if( !reprojErrs.empty() )
        fs << "per_view_reprojection_errors" << Mat(reprojErrs);

    if( !rvecs.empty() && !tvecs.empty() )
    {
        CV_Assert(rvecs[0].type() == tvecs[0].type());
        Mat bigmat((int)rvecs.size(), 6, rvecs[0].type());
        for( int i = 0; i < (int)rvecs.size(); i++ )
        {
            Mat r = bigmat(Range(i, i+1), Range(0,3));
            Mat t = bigmat(Range(i, i+1), Range(3,6));

            CV_Assert(rvecs[i].rows == 3 && rvecs[i].cols == 1);
            CV_Assert(tvecs[i].rows == 3 && tvecs[i].cols == 1);
            // *.t() is MatExpr (not Mat) so we can use assignment operator
            r = rvecs[i].t();
            t = tvecs[i].t();
        }
        //cvWriteComment( *fs, "a set of 6-tuples (rotation vector + translation vector) for each view", 0 );
        fs << "extrinsic_parameters" << bigmat;
    }

    if( !imagePoints.empty() )
    {
        Mat imagePtMat((int)imagePoints.size(), (int)imagePoints[0].size(), CV_32FC2);
        for( int i = 0; i < (int)imagePoints.size(); i++ )
        {
            Mat r = imagePtMat.row(i).reshape(2, imagePtMat.cols);
            Mat imgpti(imagePoints[i]);
            imgpti.copyTo(r);
        }
        fs << "image_points" << imagePtMat;
    }

    if( !newObjPoints.empty() )
    {
        fs << "grid_points" << newObjPoints;
    }
}

static bool readStringList( const string& filename, vector<string>& l )
{
    l.resize(0);
    FileStorage fs(filename, FileStorage::READ);
    if( !fs.isOpened() )
        return false;
    size_t dir_pos = filename.rfind('/');
    if (dir_pos == string::npos)
        dir_pos = filename.rfind('\\');
    FileNode n = fs.getFirstTopLevelNode();
    if( n.type() != FileNode::SEQ )
        return false;
    FileNodeIterator it = n.begin(), it_end = n.end();
    for( ; it != it_end; ++it )
    {
        string fname = (string)*it;
        if (dir_pos != string::npos)
        {
            string fpath = samples::findFile(filename.substr(0, dir_pos + 1) + fname, false);
            if (fpath.empty())
            {
                fpath = samples::findFile(fname);
            }
            fname = fpath;
        }
        else
        {
            fname = samples::findFile(fname);
        }
        l.push_back(fname);
    }
    return true;
}


static bool runAndSave(const string& outputFilename,
                const vector<vector<Point2f> >& imagePoints,
                Size imageSize, Size boardSize, Pattern patternType, float squareSize,
                float grid_width, bool release_object,
                float aspectRatio, int flags, Mat& cameraMatrix,
                Mat& distCoeffs, bool writeExtrinsics, bool writePoints, bool writeGrid )
{
    vector<Mat> rvecs, tvecs;
    vector<float> reprojErrs;
    double totalAvgErr = 0;
    vector<Point3f> newObjPoints;

    bool ok = runCalibration(imagePoints, imageSize, boardSize, patternType, squareSize,
                   aspectRatio, grid_width, release_object, flags, cameraMatrix, distCoeffs,
                   rvecs, tvecs, reprojErrs, newObjPoints, totalAvgErr);
    printf("%s. avg reprojection error = %.7f\n",
           ok ? "Calibration succeeded" : "Calibration failed",
           totalAvgErr);

    if( ok )
        saveCameraParams( outputFilename, imageSize,
                         boardSize, squareSize, aspectRatio,
                         flags, cameraMatrix, distCoeffs,
                         writeExtrinsics ? rvecs : vector<Mat>(),
                         writeExtrinsics ? tvecs : vector<Mat>(),
                         writeExtrinsics ? reprojErrs : vector<float>(),
                         writePoints ? imagePoints : vector<vector<Point2f> >(),
                         writeGrid ? newObjPoints : vector<Point3f>(),
                         totalAvgErr );
    return ok;
} */

class Calibrator {
  Mat view;
  Size boardSize;
  vector<vector<Point2f> > imagePoints;
  clock_t prevTimestamp = 0;
  Pattern pattern;

public:
  Calibrator(int width, int height, int type, int boardWidth, int boardHeight) :
    view(width, height, type),
    boardSize(boardWidth, boardHeight),
    pattern(CHESSBOARD)
  {}
  unsigned char *getFrameBuf() {
    return view.ptr();
  }
  bool update() {
    int winSize = 11;
    int delay = 100;
    int mode = CAPTURING;

    Mat viewGray;
    bool blink = false;

    vector<Point2f> pointbuf;
    cvtColor(view, viewGray, COLOR_BGR2GRAY);

    bool found;
    switch( pattern )
    {
        case CHESSBOARD:
            found = findChessboardCorners( view, boardSize, pointbuf,
                CALIB_CB_ADAPTIVE_THRESH | CALIB_CB_FAST_CHECK | CALIB_CB_NORMALIZE_IMAGE);
            break;
        case CIRCLES_GRID:
            found = findCirclesGrid( view, boardSize, pointbuf );
            break;
        case ASYMMETRIC_CIRCLES_GRID:
            found = findCirclesGrid( view, boardSize, pointbuf, CALIB_CB_ASYMMETRIC_GRID );
            break;
        default:
            return fprintf( stderr, "Unknown pattern type\n" ), -1;
    }

   // improve the found corners' coordinate accuracy
    if( pattern == CHESSBOARD && found) cornerSubPix( viewGray, pointbuf, Size(winSize,winSize),
        Size(-1,-1), TermCriteria( TermCriteria::EPS+TermCriteria::COUNT, 30, 0.0001 ));

    if( mode == CAPTURING && found &&
       (clock() - prevTimestamp > delay*1e-3*CLOCKS_PER_SEC) )
    {
        imagePoints.push_back(pointbuf);
        prevTimestamp = clock();
        blink = true;
    }

    return blink;
  }
  bool finish(unsigned char *data) {
    Size imageSize = view.size();
    float squareSize, aspectRatio = 1;
    float grid_width = squareSize * (boardSize.width - 1);
    bool release_object = false;
    int flags = 0;

    Mat cameraMatrix, distCoeffs;
    vector<Mat> rvecs, tvecs;
    vector<float> reprojErrs;
    vector<Point3f> newObjPoints;
    double totalAvgErr = 0;

    bool ok = runCalibration(imagePoints, imageSize, boardSize, pattern, squareSize,
                   aspectRatio, grid_width, release_object, flags, cameraMatrix, distCoeffs,
                   rvecs, tvecs, reprojErrs, newObjPoints, totalAvgErr);
    if (ok) {
      unsigned int index = 0;
      memcpy(data + index, cameraMatrix.ptr(), 3*3*sizeof(double));
      index += 3*3*sizeof(double);
      memcpy(data + index, cameraMatrix.ptr(), 8*1*sizeof(double));
      index += 8*1*sizeof(double);
    }
    return ok;
  }
};

EMSCRIPTEN_KEEPALIVE Calibrator *make_calibrator(int width, int height, int type, int boardWidth, int boardHeight) {
  return new Calibrator(width, height, type, boardWidth, boardHeight);
}

EMSCRIPTEN_KEEPALIVE bool update_calibrator(Calibrator *calibrator) {
  return calibrator->update();
}

EMSCRIPTEN_KEEPALIVE bool finish_calibrator(Calibrator *calibrator, unsigned char *data) {
  return calibrator->finish(data);
}

/* int main( int argc, char** argv )
{
    Size boardSize, imageSize;
    float squareSize, aspectRatio = 1;
    Mat cameraMatrix, distCoeffs;
    string outputFilename;
    string inputFilename = "";

    int i, nframes;
    bool writeExtrinsics, writePoints;
    bool undistortImage = false;
    int flags = 0;
    VideoCapture capture;
    bool flipVertical;
    bool showUndistorted;
    bool videofile;
    int delay;
    clock_t prevTimestamp = 0;
    int mode = DETECTION;
    int cameraId = 0;
    vector<vector<Point2f> > imagePoints;
    vector<string> imageList;
    Pattern pattern = CHESSBOARD;

    cv::CommandLineParser parser(argc, argv,
        "{help ||}{w||}{h||}{pt|chessboard|}{n|10|}{d|1000|}{s|1|}{o|out_camera_data.yml|}"
        "{op||}{oe||}{zt||}{a||}{p||}{v||}{V||}{su||}"
        "{oo||}{ws|11|}{dt||}"
        "{@input_data|0|}");
    if (parser.has("help"))
    {
        help();
        return 0;
    }
    boardSize.width = parser.get<int>( "w" );
    boardSize.height = parser.get<int>( "h" );
    if ( parser.has("pt") )
    {
        string val = parser.get<string>("pt");
        if( val == "circles" )
            pattern = CIRCLES_GRID;
        else if( val == "acircles" )
            pattern = ASYMMETRIC_CIRCLES_GRID;
        else if( val == "chessboard" )
            pattern = CHESSBOARD;
        else
            return fprintf( stderr, "Invalid pattern type: must be chessboard or circles\n" ), -1;
    }
    squareSize = parser.get<float>("s");
    nframes = parser.get<int>("n");
    delay = parser.get<int>("d");
    writePoints = parser.has("op");
    writeExtrinsics = parser.has("oe");
    bool writeGrid = parser.has("oo");
    if (parser.has("a")) {
        flags |= CALIB_FIX_ASPECT_RATIO;
        aspectRatio = parser.get<float>("a");
    }
    if ( parser.has("zt") )
        flags |= CALIB_ZERO_TANGENT_DIST;
    if ( parser.has("p") )
        flags |= CALIB_FIX_PRINCIPAL_POINT;
    flipVertical = parser.has("v");
    videofile = parser.has("V");
    if ( parser.has("o") )
        outputFilename = parser.get<string>("o");
    showUndistorted = parser.has("su");
    if ( isdigit(parser.get<string>("@input_data")[0]) )
        cameraId = parser.get<int>("@input_data");
    else
        inputFilename = parser.get<string>("@input_data");
    int winSize = parser.get<int>("ws");
    float grid_width = squareSize * (boardSize.width - 1);
    bool release_object = false;
    if (parser.has("dt")) {
        grid_width = parser.get<float>("dt");
        release_object = true;
    }
    if (!parser.check())
    {
        help();
        parser.printErrors();
        return -1;
    }
    if ( squareSize <= 0 )
        return fprintf( stderr, "Invalid board square width\n" ), -1;
    if ( nframes <= 3 )
        return printf("Invalid number of images\n" ), -1;
    if ( aspectRatio <= 0 )
        return printf( "Invalid aspect ratio\n" ), -1;
    if ( delay <= 0 )
        return printf( "Invalid delay\n" ), -1;
    if ( boardSize.width <= 0 )
        return fprintf( stderr, "Invalid board width\n" ), -1;
    if ( boardSize.height <= 0 )
        return fprintf( stderr, "Invalid board height\n" ), -1;

    if( !inputFilename.empty() )
    {
        if( !videofile && readStringList(samples::findFile(inputFilename), imageList) )
            mode = CAPTURING;
        else
            capture.open(samples::findFileOrKeep(inputFilename));
    }
    else
        capture.open(cameraId);

    if( !capture.isOpened() && imageList.empty() )
        return fprintf( stderr, "Could not initialize video (%d) capture\n",cameraId ), -2;

    if( !imageList.empty() )
        nframes = (int)imageList.size();

    if( capture.isOpened() )
        printf( "%s", liveCaptureHelp );

    namedWindow( "Image View", 1 );

    for(i = 0;;i++)
    {
        Mat view, viewGray;
        bool blink = false;

        if( capture.isOpened() )
        {
            Mat view0;
            capture >> view0;
            view0.copyTo(view);
        }
        else if( i < (int)imageList.size() )
            view = imread(imageList[i], 1);

        if(view.empty())
        {
            if( imagePoints.size() > 0 )
                runAndSave(outputFilename, imagePoints, imageSize,
                           boardSize, pattern, squareSize, grid_width, release_object, aspectRatio,
                           flags, cameraMatrix, distCoeffs,
                           writeExtrinsics, writePoints, writeGrid);
            break;
        }

        imageSize = view.size();

        if( flipVertical )
            flip( view, view, 0 );

        vector<Point2f> pointbuf;
        cvtColor(view, viewGray, COLOR_BGR2GRAY);

        bool found;
        switch( pattern )
        {
            case CHESSBOARD:
                found = findChessboardCorners( view, boardSize, pointbuf,
                    CALIB_CB_ADAPTIVE_THRESH | CALIB_CB_FAST_CHECK | CALIB_CB_NORMALIZE_IMAGE);
                break;
            case CIRCLES_GRID:
                found = findCirclesGrid( view, boardSize, pointbuf );
                break;
            case ASYMMETRIC_CIRCLES_GRID:
                found = findCirclesGrid( view, boardSize, pointbuf, CALIB_CB_ASYMMETRIC_GRID );
                break;
            default:
                return fprintf( stderr, "Unknown pattern type\n" ), -1;
        }

       // improve the found corners' coordinate accuracy
        if( pattern == CHESSBOARD && found) cornerSubPix( viewGray, pointbuf, Size(winSize,winSize),
            Size(-1,-1), TermCriteria( TermCriteria::EPS+TermCriteria::COUNT, 30, 0.0001 ));

        if( mode == CAPTURING && found &&
           (!capture.isOpened() || clock() - prevTimestamp > delay*1e-3*CLOCKS_PER_SEC) )
        {
            imagePoints.push_back(pointbuf);
            prevTimestamp = clock();
            blink = capture.isOpened();
        }

        if(found)
            drawChessboardCorners( view, boardSize, Mat(pointbuf), found );

        string msg = mode == CAPTURING ? "100/100" :
            mode == CALIBRATED ? "Calibrated" : "Press 'g' to start";
        int baseLine = 0;
        Size textSize = getTextSize(msg, 1, 1, 1, &baseLine);
        Point textOrigin(view.cols - 2*textSize.width - 10, view.rows - 2*baseLine - 10);

        if( mode == CAPTURING )
        {
            if(undistortImage)
                msg = format( "%d/%d Undist", (int)imagePoints.size(), nframes );
            else
                msg = format( "%d/%d", (int)imagePoints.size(), nframes );
        }

        putText( view, msg, textOrigin, 1, 1,
                 mode != CALIBRATED ? Scalar(0,0,255) : Scalar(0,255,0));

        if( blink )
            bitwise_not(view, view);

        if( mode == CALIBRATED && undistortImage )
        {
            Mat temp = view.clone();
            undistort(temp, view, cameraMatrix, distCoeffs);
        }

        imshow("Image View", view);
        char key = (char)waitKey(capture.isOpened() ? 50 : 500);

        if( key == 27 )
            break;

        if( key == 'u' && mode == CALIBRATED )
            undistortImage = !undistortImage;

        if( capture.isOpened() && key == 'g' )
        {
            mode = CAPTURING;
            imagePoints.clear();
        }

        if( mode == CAPTURING && imagePoints.size() >= (unsigned)nframes )
        {
            if( runAndSave(outputFilename, imagePoints, imageSize,
                       boardSize, pattern, squareSize, grid_width, release_object, aspectRatio,
                       flags, cameraMatrix, distCoeffs,
                       writeExtrinsics, writePoints, writeGrid))
                mode = CALIBRATED;
            else
                mode = DETECTION;
            if( !capture.isOpened() )
                break;
        }
    }

    if( !capture.isOpened() && showUndistorted )
    {
        Mat view, rview, map1, map2;
        initUndistortRectifyMap(cameraMatrix, distCoeffs, Mat(),
                                getOptimalNewCameraMatrix(cameraMatrix, distCoeffs, imageSize, 1, imageSize, 0),
                                imageSize, CV_16SC2, map1, map2);

        for( i = 0; i < (int)imageList.size(); i++ )
        {
            view = imread(imageList[i], 1);
            if(view.empty())
                continue;
            //undistort( view, rview, cameraMatrix, distCoeffs, cameraMatrix );
            remap(view, rview, map1, map2, INTER_LINEAR);
            imshow("Image View", rview);
            char c = (char)waitKey();
            if( c == 27 || c == 'q' || c == 'Q' )
                break;
        }
    }

    return 0;
} */