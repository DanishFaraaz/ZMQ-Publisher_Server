#include "CMT.h"
#include "gui.h"
#include <stdio.h>
#include <errno.h>

#include "opencv2/opencv.hpp"
#include "withrobot_camera.hpp"

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core/core.hpp>

#include "opencv2/features2d.hpp"
#include "opencv2/xfeatures2d.hpp"

#include <iostream>
#include <fstream>
#include <sstream>
#include <cstdio>

#include <zmq.hpp>
#include <stdio.h>
#include <stdlib.h>
#include <string>
#include <iostream>
#ifndef _WIN32
#include <unistd.h>
#else
#include <windows.h>

#define sleep(n)    Sleep(n)
#endif

#include <cmath>

#ifdef __GNUC__
#include <getopt.h>
#else
#include "geui.h"
#include <stdio.h>
#include <errno.h>

#include "opencv2/opencv.hpp"
#include "withrobot_camera.hpp"

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core/core.hpp>

#include "opencv2/features2d.hpp"
#include "opencv2/xfeatures2d.hpp"
//topt/getopt.h"
#endif

using namespace cv;
using namespace cv::xfeatures2d;

using namespace std;

using cmt::CMT;
using cv::imread;
using cv::namedWindow;
using cv::Scalar;
using cv::VideoCapture;
using cv::waitKey;
using std::cerr;
using std::istream;
using std::ifstream;
using std::stringstream;
using std::ofstream;
using std::cout;
using std::min_element;
using std::max_element;
using std::endl;
using ::atof;

int x_t, y_t, widt, heigh;
int nf = 0;
int array_ind = 0;
int array_s[7] = {};

static string WIN_NAME = "CMT";
static string OUT_FILE_COL_HEADERS =
    "Frame,Timestamp (ms),Active points,"\
    "Bounding box centre X (px),Bounding box centre Y (px),"\
    "Bounding box width (px),Bounding box height (px),"\
    "Bounding box rotation (degrees),"\
    "Bounding box vertex 1 X (px),Bounding box vertex 1 Y (px),"\
    "Bounding box vertex 2 X (px),Bounding box vertex 2 Y (px),"\
    "Bounding box vertex 3 X (px),Bounding box vertex 3 Y (px),"\
    "Bounding box vertex 4 X (px),Bounding box vertex 4 Y (px)";

vector<float> getNextLineAndSplitIntoFloats(istream& str)
{
    vector<float>   result;
    string                line;
    getline(str,line);

    stringstream          lineStream(line);
    string                cell;
    while(getline(lineStream,cell,','))
    {
        result.push_back(atof(cell.c_str()));
    }
    return result;
}


int display(Mat im, CMT & cmt, int p, int nf, VideoWriter video)
{
    //  Prepare our context and socket
    zmq::context_t context (1);
    zmq::socket_t publisher (context, ZMQ_PUB);
    publisher.bind ("tcp://127.0.0.1:5556");

    //Visualize the output
    //It is ok to draw on im itself, as CMT only uses the grayscale image
    for(size_t i = 0; i < cmt.points_active.size(); i++)
    {
        circle(im, cmt.points_active[i], 2, Scalar(255,0,0));
    }

    Point2f vertices[4];
    cmt.bb_rot.points(vertices);
    for (int i = 0; i < 4; i++)
    {
        if (int(cmt.points_active.size())>0.3*p){
            line(im, vertices[i], vertices[(i+1)%4], Scalar(255,0,0));
            nf = 0;

            zmq::message_t message(20);

            int rot_center_x = cmt.bb_rot.center.x;
            snprintf((char *) message.data(), 20, "%d", rot_center_x);
            
            publisher.send(message);
        }
    }

    if (int(cmt.points_active.size())<0.3*p) {
    nf = nf+1;
    }


    video.write(im);
    imshow(WIN_NAME, im);
    waitKey(5);

    return nf;

    // return waitKey(5);
}

Rect surf_bb(Mat frame)
{
    //--------------------------------------------------
    //-- Step 1: Detect the keypoints using SURF Detector
    Mat gray;
    int m = 0;
    int ind;

    cvtColor(frame,gray,CV_BGR2GRAY);
    // gray = gray(cv::Rect(0,0,570,410));
    int minHessian = 1000;
    Ptr<SURF> detector = SURF::create(minHessian);
    std::vector<KeyPoint> keypoints;
    detector->detect( gray, keypoints );
    //-- Draw keypoints
    Mat img_keypoints;
    drawKeypoints( frame, keypoints, img_keypoints );
    for (int i=1; i<= int(sizeof(keypoints)); i++){
        if (int(keypoints[i].size)>m){
            m = keypoints[i].size;
            ind = i;
        }
    }
    //-- Show detected (drawn) keypoints
    // imshow("SURF Keypoints", img_keypoints );
    // waitKey(1);

    x_t = keypoints[ind].pt.x  - (keypoints[ind].size)/2;
    y_t = keypoints[ind].pt.y - (keypoints[ind].size)/2;
    widt = 2*keypoints[ind].size;
    heigh = 2*keypoints[ind].size;


    //return Rect(x_tl, y_tl, width, height);
    Rect rect = Rect(x_t, y_t, widt, heigh);

    // cv::rectangle(frame, rect, cv::Scalar(0, 255, 0));
    // imshow("SURF bounding box", frame );
        
    //rect = getRect(colorImg, WIN_NAME);

    return rect;
}

string write_rotated_rect(RotatedRect rect)
{
    Point2f verts[4];
    rect.points(verts);
    stringstream coords;

    coords << rect.center.x << "," << rect.center.y << ",";
    coords << rect.size.width << "," << rect.size.height << ",";
    coords << rect.angle << ",";

    for (int i = 0; i < 4; i++)
    {
        coords << verts[i].x << "," << verts[i].y;
        if (i != 3) coords << ",";
    }

    return coords.str();
}


int main()
{
    //Create a CMT object
    CMT cmt;

    //Initialization bounding box
    Rect rect;

    //Parse args
    int challenge_flag = 0;
    int loop_flag = 0;
    int verbose_flag = 0;
    int bbox_flag = 0;
    int skip_frames = 0;
    int skip_msecs = 0;
    int output_flag = 0;

    bool donot = true;

    string input_path;
    string output_path;

    const int detector_cmd = 1000;
    const int descriptor_cmd = 1001;
    const int bbox_cmd = 1002;
    const int no_scale_cmd = 1003;
    const int with_rotation_cmd = 1004;
    const int skip_cmd = 1005;
    const int skip_msecs_cmd = 1006;
    const int output_file_cmd = 1007;


    //Normal mode

    // Create window
    // namedWindow(WIN_NAME);

    Mat img1 = imread( "bottle_latest4.jpeg", IMREAD_GRAYSCALE );
    resize(img1, img1, cv::Size(320, 240));

    VideoCapture cap("latest1.mp4");
    int frame_width = cap.get(CV_CAP_PROP_FRAME_WIDTH);
    int frame_heigth = cap.get(CV_CAP_PROP_FRAME_HEIGHT);

    VideoWriter video("tracker.avi",CV_FOURCC('M','J','P','G'),10, Size(frame_width,frame_heigth),true);


    // bool show_preview = true;

    // // If no input was specified
    // const char* devPath = "/dev/video0";
    // Withrobot::Camera camera(devPath);
        
    // //cap.open(0); //Open default camera device

    // /* bayer RBG 640 x 480 80 fps */
    // camera.set_format(640, 480, Withrobot::fourcc_to_pixformat('G','B','G','R'), 1, 80);

    // /*
    //  * get current camera format (image size and frame rate)
    //  */
    // Withrobot::camera_format camFormat;
    // camera.get_current_format(camFormat);

    // /*
    //  * Print infomations
    //  */
    // std::string camName = camera.get_dev_name();
    // std::string camSerialNumber = camera.get_serial_number();

    // printf("dev: %s, serial number: %s\n", camName.c_str(), camSerialNumber.c_str());
    // printf("----------------- Current format informations -----------------\n");
    // camFormat.print();
    // printf("---------------------------------------------------------------\n");

    // int brightness = camera.get_control("Gain");
    // int exposure = camera.get_control("Exposure (Absolute)");

    // //camera.set_control("Gain", 70);
    // camera.set_control("Gain", brightness);
    // //camera.set_control("Exposure (Absolute)", 15);
    // camera.set_control("Exposure (Absolute)", exposure);


    // //Get initial image
    // // Mat im0;
    // // cap >> im0;

    // std::string windowName = camName + " " + camSerialNumber;
    // cv::Mat srcImg(cv::Size(camFormat.width, camFormat.height), CV_8UC1);
    // cv::Mat frame(cv::Size(camFormat.width, camFormat.height), CV_8UC3);

    // int size = camera.get_frame(srcImg.data, camFormat.image_size, 1);
    // cout << "Receiving bounding box" << endl;
    // cv::cvtColor(srcImg, colorImg, cv::COLOR_BayerGB2BGR);

    // while (1){
    //     int size = camera.get_frame(srcImg.data, camFormat.image_size, 1);
    //     if (size == -1) {
    //         printf("error number: %d\n", errno);
    //         perror("Cannot get image from camera");
    //         camera.stop();
    //         camera.start();
    //         continue;
    //     }
    //     cout << "Receiving bounding box" << endl;
    //     cv::cvtColor(srcImg, colorImg, cv::COLOR_BayerGB2BGR);

    //     imshow("Color Image", colorImg);
    //     waitKey(1);

    // }


    //If no bounding was specified, get it from user

    while (!bbox_flag && donot)
    {
        Mat frame;
        cap >> frame;


        // int size = camera.get_frame(srcImg.data, camFormat.image_size, 1);
        // if (size == -1) {
        //     printf("error number: %d\n", errno);
        //     perror("Cannot get image from camera");
        //     camera.stop();
        //     camera.start();
        //     continue;
        // }
        // // cout << "Receiving bounding box" << endl;
        // cv::cvtColor(srcImg, frame, cv::COLOR_BayerGB2BGR);

        // cout << "Converted to Opencv format" << endl;
        // rect = getRect(im0, WIN_NAME);

        int maximum, ind, i;
        int m = 0;

        Mat gray;
        cvtColor(frame, gray, CV_BGR2GRAY);
        resize(gray, gray, cv::Size(320, 240));

        //-- Step 1: Detect the keypoints using SURF Detector, compute the descriptors
        int minHessian = 0;
        // Ptr<ORB> detector = ORB::create(500,1.2f,8,31,0,3);
        // Ptr<SIFT> detector = SIFT::create(0,3,0.01);
        Ptr<SIFT> detector = SIFT::create();
        std::vector<KeyPoint> keypoints1, keypoints2;

        Mat descriptors1, descriptors2;
        detector->detectAndCompute( img1, noArray(), keypoints1, descriptors1 );
        detector->detectAndCompute( gray, noArray(), keypoints2, descriptors2 );
        //-- Step 2: Matching descriptor vectors with a FLANN based matcher
        // Since SURF is a floating-point descriptor NORM_L2 is used
        Ptr<DescriptorMatcher> matcher = DescriptorMatcher::create(DescriptorMatcher::BRUTEFORCE);
        std::vector< std::vector<DMatch> > knn_matches;
        matcher->knnMatch( descriptors1, descriptors2, knn_matches, 2 );
        //-- Filter matches using the Lowe's ratio test
        const float ratio_thresh = 0.87f;
        std::vector<DMatch> good_matches;
        for (size_t i = 0; i < knn_matches.size(); i++)
        {
            if (knn_matches[i][0].distance < ratio_thresh * knn_matches[i][1].distance)
            {
                good_matches.push_back(knn_matches[i][0]);
            }
        }

        //-- Draw matches
        Mat img_matches;
        drawMatches( img1, keypoints1, gray, keypoints2, good_matches, img_matches, Scalar::all(-1),
                    Scalar::all(-1), std::vector<char>(), DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS );

        imshow("Good Matches", img_matches );
        // imshow("Camera", frame);
        waitKey(1);

        // -- Show detected matches
        if (good_matches.size() > 0.2*knn_matches.size()){
            array_s[array_ind] = 1;
            good_matches.clear();
            keypoints1.clear();
            keypoints2.clear();
            descriptors1.release();
            descriptors2.release();
            knn_matches.clear();
        }
        else {
            array_s[array_ind] = 0;
            good_matches.clear();
            keypoints1.clear();
            keypoints2.clear();
            descriptors1.release();
            descriptors2.release();
            knn_matches.clear();
        }

        array_ind = array_ind+1;

        if (array_ind % 6 == 0){
            array_ind = 0;
            int sum = 0;
            for (int y = 0; y<6; y++){
                sum = sum + array_s[y];
            }
            cout << sum << endl;
            if (int(sum) == 6)
            {
                cap >> frame;

                // int size = camera.get_frame(srcImg.data, camFormat.image_size, 1);
                // cv::cvtColor(srcImg, frame, cv::COLOR_BayerGB2BGR);


                rect = surf_bb(frame);

                // std::vector<float> t_l_x;
                // std::vector<float> t_l_y;

                // for (int i=0; i <= good_matches.size(); i++) {
                //     t_l_x.push_back(keypoints2[good_matches[i].trainIdx].pt.x); 
                //     t_l_y.push_back(keypoints2[good_matches[i].trainIdx].pt.y); 
                // }

                // float sum_x = 0;
                // float sum_y = 0;

                // for (int i=0; i<= t_l_x.size(); i++){
                //     sum_x = sum_x + t_l_x[i];
                //     sum_y = sum_y + t_l_y[i];
                // }

                // float t_l_x_avg = sum_x/t_l_x.size();
                // float t_l_y_avg = sum_y/t_l_y.size();

                // cout << "x coordinate: " << t_l_x_avg << endl;
                // cout << "y coordinate: " << t_l_y_avg << endl;

                // rect = Rect(t_l_x_avg, t_l_y_avg, 200, 200); 
                break;
            }
        }
    }

    // FILE_LOG(logINFO) << "Using " << rect.x << "," << rect.y << "," << rect.width << "," << rect.height
    //     << " as initial bounding box.";


    Mat frame1;
    cap >> frame1;

    // cv::Mat srcImg1(cv::Size(camFormat.width, camFormat.height), CV_8UC1);
    // cv::Mat frame1(cv::Size(camFormat.width, camFormat.height), CV_8UC3);

    // int size = camera.get_frame(srcImg1.data, camFormat.image_size, 1);
    // cv::cvtColor(srcImg1, frame1, cv::COLOR_BayerGB2BGR);


    // //Convert im0 to grayscale
    Mat im0_gray;
    
    cvtColor(frame1, im0_gray, CV_BGR2GRAY);

    //Initialize CMT
    cmt.initialize(im0_gray, rect);
    int p = cmt.points_active.size();

    // int frame = skip_frames;

    //Open output file.
    ofstream output_file;

    if (output_flag)
    {
        //int msecs = (int) cap.get(CV_CAP_PROP_POS_MSEC);

        output_file.open(output_path.c_str());
        output_file << OUT_FILE_COL_HEADERS << endl;
        //output_file << frame << "," << msecs << ",";
        output_file << cmt.points_active.size() << ",";
        output_file << write_rotated_rect(cmt.bb_rot) << endl;
    }

    //Main loop
    while (true)
    {


        Mat im;
        cap >> im;

        // cv::Mat frame(cv::Size(camFormat.width, camFormat.height), CV_8UC1);
        // cv::Mat im(cv::Size(camFormat.width, camFormat.height), CV_8UC3);

        // int size = camera.get_frame(frame.data, camFormat.image_size, 1);
        // cv::cvtColor(frame, im, cv::COLOR_BayerGB2BGR);

        // If loop flag is set, reuse initial image (for debugging purposes)
        // if (loop_flag) im0.copyTo(im);
        // else cap >> im; //Else use next image in stream

        // if (im.empty()) break; //Exit at end of video stream

        Mat im_gray;
        
        cvtColor(im, im_gray, CV_BGR2GRAY);
        // im_gray = im_gray(cv::Rect(0,0,570,410));

        //Let CMT process the frame
        cmt.processFrame(im_gray);

        //Output.
        if (output_flag)
        {
            //int msecs = (int) cap.get(CV_CAP_PROP_POS_MSEC);
            //output_file << frame << "," << msecs << ",";
            output_file << cmt.points_active.size() << ",";
            output_file << write_rotated_rect(cmt.bb_rot) << endl;
        }
        else
        {
            //TODO: Provide meaningful output
            //FILE_LOG(logINFO) << "#" << frame << " active: " << cmt.points_active.size();
        }

        //Display image and then quit if requested.
        // char key = display(im, cmt, p);
        nf = display(im, cmt, p, nf, video);
        cout << "Number of times not in frame: " << nf << endl;
        cout << "+++++++++++++++++++++++++++++++++++++++++++++++++" << endl;

        if (nf>180){
            destroyAllWindows();
            bbox_flag = false;

            while (!bbox_flag)
            {
                Mat frame;
                cap >> frame;

                // int size = camera.get_frame(srcImg.data, camFormat.image_size, 1);
                // if (size == -1) {
                //     printf("error number: %d\n", errno);
                //     perror("Cannot get image from camera");
                //     camera.stop();
                //     camera.start();
                //     continue;
                // }
                // cv::cvtColor(srcImg, frame, cv::COLOR_BayerGB2BGR);

                // cout << "Converted to Opencv format" << endl;
                // rect = getRect(im0, WIN_NAME);

                int maximum, ind, i;
                int m = 0;

                Mat gray;
                cvtColor(frame, gray, CV_BGR2GRAY);
                resize(gray, gray, cv::Size(320, 240));

                //-- Step 1: Detect the keypoints using SURF Detector, compute the descriptors
                int minHessian = 0;
                // Ptr<KAZE> detector = KAZE::create(false,false,0.0001);
                // Ptr<ORB> detector = ORB::create(500,1.2f,8,31,0,3);
                // Ptr<SIFT> detector = SIFT::create(0,3,0.01);
                Ptr<SIFT> detector = SIFT::create(0,3,0.01);
                std::vector<KeyPoint> keypoints1, keypoints2;

                Mat descriptors1, descriptors2;
                detector->detectAndCompute(img1, noArray(), keypoints1, descriptors1);
                detector->detectAndCompute(gray, noArray(), keypoints2, descriptors2);
                // detector->detect(img1,keypoints1,noArray());
                // detector->compute(img1,keypoints1,descriptors1);
                // detector->detect(gray,keypoints2,noArray());
                // detector->compute(gray,keypoints2,descriptors2);
                //-- Step 2: Matching descriptor vectors with a FLANN based matcher
                // Since SURF is a floating-point descriptor NORM_L2 is used
                Ptr<DescriptorMatcher> matcher = DescriptorMatcher::create(DescriptorMatcher::BRUTEFORCE);
                std::vector<std::vector<DMatch>> knn_matches;
                matcher->knnMatch(descriptors1, descriptors2, knn_matches, 2);
                //-- Filter matches using the Lowe's ratio testdescrip
                const float ratio_thresh = 0.85f;
                std::vector<DMatch> good_matches;
                for (size_t i = 0; i < knn_matches.size(); i++)
                {
                    if (knn_matches[i][0].distance < ratio_thresh * knn_matches[i][1].distance)
                    {
                        good_matches.push_back(knn_matches[i][0]);
                    }
                }

                //-- Draw matches
                Mat img_matches;
                drawMatches(img1, keypoints1, gray, keypoints2, good_matches, img_matches, Scalar::all(-1),
                            Scalar::all(-1), std::vector<char>(), DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS);
                imshow("Good Matches", img_matches);
                // imshow("Camera", frame);
                waitKey(1);

                // -- Show detected matches
                if (good_matches.size() > 0.1 * knn_matches.size())
                {
                    array_s[array_ind] = 1;
                    good_matches.clear();
                    keypoints1.clear();
                    keypoints2.clear();
                    descriptors1.release();
                    descriptors2.release();
                    knn_matches.clear();
                }
                else
                {
                    array_s[array_ind] = 0;
                    good_matches.clear();
                    keypoints1.clear();
                    keypoints2.clear();
                    descriptors1.empty();
                    descriptors2.empty();
                    knn_matches.clear();
                }

                array_ind = array_ind + 1;

                if (array_ind % 6 == 0)
                {
                    array_ind = 0;
                    int sum = 0;
                    for (int y = 0; y < 6; y++)
                    {
                        sum = sum + array_s[y];
                    }
                    cout << int(sum) << endl;
                    if (int(sum) == 6)
                    {
                        cap >> frame;
                        // int size = camera.get_frame(srcImg.data, camFormat.image_size, 1);
                        // cv::cvtColor(srcImg, frame, cv::COLOR_BayerGB2BGR);
                        rect = surf_bb(frame);
                        nf = 0;
                        break;
                    }
                }
            }
        }
    }

    //Close output file.
    if (output_flag) output_file.close();

    return 0;
}
