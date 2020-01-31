// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2015-2017 Intel Corporation. All Rights Reserved.

#include <librealsense2/rs.hpp> // Include RealSense Cross Platform API
#include "example.hpp"          // Include short list of convenience functions for rendering

#include <algorithm>            // std::min, std::max

#include <opencv2/opencv.hpp>
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"

#include <experimental/filesystem>

#include <stdio.h>
#include <stdlib.h>
#include <vector>
#include <sstream>

namespace filesystem = std::experimental::filesystem;

// Helper functions
void initialize(int argc, char * argv[]);
inline void initializeParameter(int argc, char * argv[]);
inline void initializeSensor();
void register_glfw_callbacks(window& app, glfw_state& app_state);
cv::Mat frame_to_mat(const rs2::frame& f);
cv::Mat depth_frame_to_meters(const rs2::pipeline& pipe, const rs2::depth_frame& f);

//global variables
rs2::pointcloud pc;
rs2::points points;
rs2::align *align_to_color;
rs2::pipeline pipeline;
rs2::pipeline_profile pipeline_profile;
rs2::frameset frameset;

rs2::frame color_frame;
uint32_t color_width;
uint32_t color_height;

rs2::frame depth_frame;
uint32_t depth_width;
uint32_t depth_height;

filesystem::path bag_file;
filesystem::path directory;
std::vector<int32_t> params;
bool scaling = false;

int cnt = 0;


cv::Mat findHoles(cv::Mat img);

int main(int argc, char * argv[]) {

try {
    // Create a simple OpenGL window for rendering:
    //window app(1280, 720, "pointcloud");
    // Construct an object to manage view state
    //glfw_state app_state;
    // register callbacks to allow manipulation of the pointcloud
    //register_glfw_callbacks(app, app_state);

    initialize(argc, argv);
   
    // Retrieve Last Position
    uint64_t last_position = pipeline_profile.get_device().as<rs2::playback>().get_position();

    bool run = true;

    while(run) // Application still alive?
    {

        // Wait for the next set of frames from the camera
        frameset = pipeline.wait_for_frames();

        // Align all frames to color viewport
        frameset = align_to_color->process(frameset);
        
        // Retrieve Color Flame
        color_frame = frameset.get_color_frame();
        if( !color_frame ){
            run = false;
            break;
        }

        // Retrive Frame Size
        color_width = color_frame.as<rs2::video_frame>().get_width();
        color_height = color_frame.as<rs2::video_frame>().get_height();

        // Creating OpenCV Matrix from a color image
        cv::Mat color_mat = frame_to_mat(color_frame);
        
        
        
        // Tell pointcloud object to map to this color frame
        //pc.map_to(color_frame);

        // Retrieve Depth Flame
        depth_frame = frameset.get_depth_frame();

        if( !depth_frame ){
            run = false;
            break;
        }

        // Retrive Frame Size
        depth_width = depth_frame.as<rs2::video_frame>().get_width();
        depth_height = depth_frame.as<rs2::video_frame>().get_height();

        // Creating OpenCV Matrix from a depth image
        cv::Mat depth_mat = depth_frame_to_meters(pipeline, depth_frame);

        // Generate the pointcloud and texture mappings
        //points = pc.calculate(depth_frame);

        // Upload the color frame to OpenGL
        //app_state.tex.upload(color_frame);

        // Draw the pointcloud
        //draw_pointcloud(app.width(), app.height(), app_state, points);


        cv::Mat color_mat_flipped(color_mat.size(), color_mat.type()); 
        cv::flip(color_mat, color_mat_flipped, -1);
        cv::Mat depth_mat_flipped(depth_mat.size(), depth_mat.type());
        cv::flip(depth_mat, depth_mat_flipped, -1);
        cv::Mat drawing = findHoles(color_mat_flipped);

        /*
        std::ostringstream ss;
  	ss << std::setw(6) << std::setfill('0') << std::to_string(cnt++);
        std::string number(ss.str());
        std::string savingName = "./images/" + number + ".jpg";
        cv::imwrite(savingName, color_mat_flipped);
        */

        cv::imshow("color", color_mat_flipped);
        cv::imshow("depth", depth_mat_flipped);
        cv::imshow("Contours", drawing);
        cv::waitKey(3);
    }

    return EXIT_SUCCESS;
}
catch (const rs2::error & e)
{
    std::cerr << "RealSense error calling " << e.get_failed_function() << "(" << e.get_failed_args() << "):\n    " << e.what() << std::endl;
    return EXIT_FAILURE;
}
catch (const std::exception & e)
{
    std::cerr << e.what() << std::endl;
    return EXIT_FAILURE;
}
}//main


// Initialize
void initialize(int argc, char* argv[])
{
    cv::setUseOptimized(true);

    // Initialize Parameter
    initializeParameter(argc, argv);

    // Initialize Sensor
    initializeSensor();

    align_to_color = new rs2::align(RS2_STREAM_COLOR);    
}

// Initialize Parameter
inline void initializeParameter(int argc, char * argv[])
{
    // Create Command Line Parser
    const std::string keys =
        "{ help h    |       | print this message.                                                      }"
        "{ bag b     |       | path to input bag file. (required)                                       }"
        "{ scaling s | false | enable depth scaling for visualization. false is raw 16bit image. (bool) }";

    cv::CommandLineParser parser( argc, argv, keys );

    if( parser.has("help") ){
        parser.printMessage();
        std::exit( EXIT_SUCCESS );
    }

    // Check Parsing Error
    if(!parser.check()){
        parser.printErrors();
        throw std::runtime_error( "failed command arguments" );
    }

    // Retrieve Bag File Path (Required)
    if(!parser.has( "bag" ) ){
        throw std::runtime_error( "failed cannot find input bag file" );
    }
    else{

        bag_file = parser.get<cv::String>("bag").c_str();
        
        if( !filesystem::is_regular_file(bag_file)) {
            throw std::runtime_error("failed bag file corrupt");
        }
        else if(bag_file.extension() != ".bag") {
            throw std::runtime_error("failed can't find input bag file");
        }

    }

    // Retrieve Scaling Flag (Option)
    if(!parser.has("scaling")){
        scaling = false;
    }
    else{
        scaling = parser.get<bool>( "scaling" );
    }
}

// Initialize Sensor
inline void initializeSensor()
{
    // Retrieve Each Streams that contain in File
    rs2::config config;
    rs2::context context;
    const rs2::playback playback = context.load_device( bag_file.string() );
    const std::vector<rs2::sensor> sensors = playback.query_sensors();
    for( const rs2::sensor& sensor : sensors ){
        const std::vector<rs2::stream_profile> stream_profiles = sensor.get_stream_profiles();
        for( const rs2::stream_profile& stream_profile : stream_profiles ){
            config.enable_stream( stream_profile.stream_type(), stream_profile.stream_index() );
        }
    }

    // Start Pipeline
    config.enable_device_from_file(playback.file_name());
    pipeline_profile = pipeline.start(config);

    // Set Non Real Time Playback
    pipeline_profile.get_device().as<rs2::playback>().set_real_time( false );

    // Show Enable Streams
    const std::vector<rs2::stream_profile> stream_profiles = pipeline_profile.get_streams();
    std::cout << "Streams:" << std::endl;
    for( const rs2::stream_profile stream_profile : stream_profiles ){
        std::cout << "  " << stream_profile.stream_name() << std::endl;
    }
}

// Convert rs2::frame to cv::Mat
cv::Mat frame_to_mat(const rs2::frame& f)
{
    using namespace cv;
    using namespace rs2;

    auto vf = f.as<video_frame>();
    const int w = vf.get_width();
    const int h = vf.get_height();

    if (f.get_profile().format() == RS2_FORMAT_BGR8)
    {
        return Mat(Size(w, h), CV_8UC3, (void*)f.get_data(), Mat::AUTO_STEP);
    }
    else if (f.get_profile().format() == RS2_FORMAT_RGB8)
    {
        auto r = Mat(Size(w, h), CV_8UC3, (void*)f.get_data(), Mat::AUTO_STEP);
        cvtColor(r, r, COLOR_RGB2BGR);
        return r;
    }
    else if (f.get_profile().format() == RS2_FORMAT_Z16)
    {
        return Mat(Size(w, h), CV_16UC1, (void*)f.get_data(), Mat::AUTO_STEP);
    }
    else if (f.get_profile().format() == RS2_FORMAT_Y8)
    {
        return Mat(Size(w, h), CV_8UC1, (void*)f.get_data(), Mat::AUTO_STEP);
    }
    else if (f.get_profile().format() == RS2_FORMAT_DISPARITY32)
    {
        return Mat(Size(w, h), CV_32FC1, (void*)f.get_data(), Mat::AUTO_STEP);
    }

    throw std::runtime_error("Frame format is not supported yet!");
}

// Converts depth frame to a matrix of doubles with distances in meters
cv::Mat depth_frame_to_meters(const rs2::pipeline& pipe, const rs2::depth_frame& f)
{
    using namespace cv;
    using namespace rs2;

    Mat dm = frame_to_mat(f);
    dm.convertTo(dm, CV_64F);
    auto depth_scale = pipe.get_active_profile()
        .get_device()
        .first<depth_sensor>()
        .get_depth_scale();
    dm = dm * depth_scale;
    return dm;
}

cv::Mat findHoles(cv::Mat img) {
 
    using namespace cv;
    using namespace std;
    
    Mat img_gray;
    /// Convert image to gray and blur it
    cvtColor(img, img_gray, CV_BGR2GRAY);
    blur(img_gray, img_gray, Size(3,3));
    
    Mat canny_output;
    vector<vector<Point> > contours;
    vector<Vec4i> hierarchy;

    int thresh = 50;
    /// Detect edges using canny
    Canny(img_gray, canny_output, thresh, thresh*2, 3 );
    /// Find contours
    findContours(canny_output, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, Point(0, 0) );

    /// Draw contours
    Mat drawing = Mat::zeros(canny_output.size(), CV_8UC3 );
    for(int i = 0; i< contours.size(); i++)
    {
        drawContours( drawing, contours, i, CV_RGB(255,0,0), 1, 8, hierarchy, 0, Point() );
    }

    return drawing;
}




