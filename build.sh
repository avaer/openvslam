export FLAGS="-I./src/ -I./eigen-eigen-5a0156e40feb/ -I./opencv/ -I./opencv/modules/core/include -I./opencv/build_wasm/ -I./opencv/build/modules/core/ -I./opencv/modules/core/src/ -I./opencv/3rdparty/include/opencl/1.2/ -I./DBoW2/include -I./yaml-cpp/include/ -I./3rd/json/include/ -I./opencv/modules/imgcodecs/include/ -I./opencv/modules/imgproc/include/ -I./opencv/modules/videoio/include/ -I./3rd/spdlog/include/ -I./3rd/popl/include/ -I./g2o -I./g2o/build -I./opencv/modules/features2d/include/ -I./opencv/modules/flann/include/ -I./opencv/modules/calib3d/include/ -I./g2o/EXTERNAL/csparse/ -DUSE_DBOW2=1";

# cmake -DMODULE_NAME=core -DOUTPUT=opencl_kernels_core.cpp -DOUTPUT_HPP=opencl_kernels_core.hpp -DCL_DIR=modules/core/src/opencl/ -P ./cmake/cl2cpp.cmake
# emcc $FLAGS -std=c++11 -o opencv.o -D__OPENCV_BUILD=1 opencv-3.4.0/modules/core/src/*.cpp
EMSCRIPTEN=/home/ubuntu/emsdk/upstream/emscripten/ python2 ./platforms/js/build_js.py build_wasm --build_wasm

emcc $FLAGS -std=c++11 -O3 -s FILESYSTEM=0 -o dbow.o DBoW2/src/*.cpp;

emcc $FLAGS -std=c++11 -O3 -s FILESYSTEM=0 -o openvslam.o src/openvslam/system.cc src/openvslam/tracking_module.cc src/openvslam/mapping_module.cc src/openvslam/global_optimization_module.cc src/openvslam/data/*.cc src/openvslam/module/*.cc src/openvslam/feature/*.cc src/openvslam/publish/*.cc src/openvslam/optimize/*.cc src/openvslam/match/*.cc src/openvslam/util/*.cc src/openvslam/optimize/g2o/*.cc src/openvslam/optimize/g2o/se3/*.cc src/openvslam/optimize/g2o/sim3/*.cc src/openvslam/config.cc src/openvslam/solve/*.cc src/openvslam/io/*.cc src/openvslam/initialize/*.cc src/openvslam/camera/*.cc src/socket_publisher/data_serializer2.cc

emcc $FLAGS -std=c++11 -O3 -s FILESYSTEM=0 -o yaml.o yaml-cpp/src/*.cpp yaml-cpp/include/yaml-cpp/exceptions.cc yaml-cpp/include/yaml-cpp/node/impl.cc

emcc $FLAGS -O3 -s FILESYSTEM=0 -o csparse.o g2o/EXTERNAL/csparse/*.c;
emcc $FLAGS -std=c++11 -O3 -s FILESYSTEM=0 -o g2o.o g2o/g2o/core/*.cpp g2o/g2o/stuff/*.cpp g2o/g2o/solvers/csparse/*.cpp

emcc $FLAGS -std=c++11 -O3 -s FILESYSTEM=0 -s TOTAL_MEMORY=100MB -o run_web.js csparse.o dbow.o openvslam.o yaml.o g2o.o opencv/build_wasm/lib/*.a opencv/build_wasm/3rdparty/lib/libzlib.a ./example/run_web.cc ./example/run_calibration.cc ./example/run_malloc.cc