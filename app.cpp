#include <stdio.h>
#include <string>
#include <iostream>
#include <sys/types.h>
#include <dirent.h>
#include <ctime>
#include "orb.hpp"

#include <opencv2/core/core.hpp>
#include <opencv2/features2d.hpp>
// #include <opencv2/xfeatures2d.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/calib3d.hpp>

using namespace cv;
// using namespace cv::xfeatures2d;

#define USE_ORB

void read_directory(const std::string &name, std::vector<std::string> &v)
{
    DIR *dirp = opendir(name.c_str());
    struct dirent *dp;
    while ((dp = readdir(dirp)) != NULL)
    {
        v.push_back(dp->d_name);
    }
    closedir(dirp);
}

static bool endsWith(const std::string &str, const std::string &suffix)
{
    return str.size() >= suffix.size() && 0 == str.compare(str.size() - suffix.size(), suffix.size(), suffix);
}

static bool startsWith(const std::string &str, const std::string &prefix)
{
    return str.size() >= prefix.size() && 0 == str.compare(0, prefix.size(), prefix);
}

typedef std::vector<std::string> CommandLineStringArgs;

void findParams(std::vector<cv::Point2f> &obj, std::vector<cv::Point2f> &scene, float *mat_object)
{
    float a1, a2, a3, a4;
    int n = obj.size();
    float x_avg = 0, y_avg = 0, x_d_avg = 0, y_d_avg = 0, x_t_x = 0, x_t_x_d = 0, x_t_y_d = 0, y_t_y = 0, y_t_y_d = 0, y_t_x_d = 0;
    for (int i = 0; i < n; i++)
    {
        float x = obj[i].x, y = obj[i].y, x_d = scene[i].x, y_d = scene[i].y;
        x_avg += x;
        y_avg += y;
        x_d_avg += x_d;
        y_d_avg += y_d;
        x_t_x += x * x;
        y_t_y += y * y;
        x_t_x_d += x * x_d;
        x_t_y_d += x * y_d;
        y_t_x_d += y * x_d;
        y_t_y_d += y * y_d;
    }
    x_avg /= n;
    x_d_avg /= n;
    y_avg /= n;
    y_d_avg /= n;
    a1 = ((x_t_x_d - n * x_avg * x_d_avg) + (y_t_y_d - n * y_avg * y_d_avg)) / ((x_t_x - n * x_avg * x_avg) + (y_t_y - n * y_avg * y_avg));
    a2 = ((x_t_y_d - n * x_avg * y_d_avg) - (y_t_x_d - n * y_avg * x_d_avg)) / ((x_t_x - n * x_avg * x_avg) + (y_t_y - n * y_avg * y_avg));
    a3 = x_d_avg - a1 * x_avg + a2 * y_avg;
    a4 = y_d_avg - a2 * x_avg - a1 * y_avg;
    mat_object[0] = a1;
    mat_object[1] = -a2;
    mat_object[2] = a3;
    mat_object[3] = a2;
    mat_object[4] = a1;
    mat_object[5] = a4;
    mat_object[6] = 0;
    mat_object[7] = 0;
    mat_object[8] = 1;
}

int main(int argc, char **argv)
{

    if (argc != 3)
    {
        std::cout << "Not enough arguments. Required 2 provided " << argc - 1 << std::endl;
        return -1;
    }

    CommandLineStringArgs cmdlineStringArgs(&argv[0], &argv[0 + argc]);

    std::string scene_filename = cmdlineStringArgs[1];

    // Template feature extraction
    std::string template_filename = cmdlineStringArgs[2];

    Mat img_1 = imread(template_filename, IMREAD_GRAYSCALE); // Read the file

    std::vector<std::vector<int>> data(img_1.rows, std::vector<int>(img_1.cols, 0));
    for(int i=0; i < img_1.rows; i++){
        for(int j = 0; j < img_1.cols; j++){
            data[i][j] = img_1.at<uchar>(i,j);
        }
    }


    // bitwise_not ( img_1, img_1 );

    if (!img_1.data)
    {
        std::cout << " --(!) Error reading template image " << std::endl;
        return -1;
    }

    Image img_template(img_1.cols, img_1.rows, data);
    std::vector<xy> corners = orb_detect_compute(img_template);

    Mat displayImage = img_1.clone();

    cvtColor(displayImage, displayImage, CV_GRAY2BGR);

    for(xy corner:corners){
        circle(displayImage, Point2d(corner.x, corner.y), 1, Scalar(0,255,0));
    }

    std::cout << corners.size() << std::endl;

    cv::imshow("Fast features template", displayImage);

    Mat opencv_img_1 = img_1.clone();
    std::vector<KeyPoint> keypointsD;
    Ptr<FastFeatureDetector> detector_fast=FastFeatureDetector::create();

    detector_fast->detect(opencv_img_1,keypointsD,Mat());
    drawKeypoints(opencv_img_1, keypointsD, opencv_img_1);
    imshow("Fast keypoints",opencv_img_1);


    /// Scene

    Mat img_2 = imread(scene_filename, IMREAD_GRAYSCALE); // Read the file

    // cv::resize(img_2, img_2, cv::Size(img_2.cols * 0.8,img_2.rows * 0.8), 0, 0, CV_INTER_LINEAR);

    std::vector<std::vector<int>> data_scene(img_2.rows, std::vector<int>(img_2.cols, 0));
    for(int i=0; i < img_2.rows; i++){
        for(int j = 0; j < img_2.cols; j++){
            data_scene[i][j] = img_2.at<uchar>(i,j);
        }
    }      

    Image img_scene(img_2.cols, img_2.rows, data_scene);
    std::vector<xy> corners_scene = orb_detect_compute(img_scene);

    Mat displayImageScene = img_2.clone();

    cvtColor(displayImageScene, displayImageScene, CV_GRAY2BGR);

    for(xy corner:corners_scene){
        circle(displayImageScene, Point2d(corner.x, corner.y), 1, Scalar(0,255,0));
    }

    cv::imshow("Fast features scene", displayImageScene);

    std::cout << corners_scene.size() << std::endl;

    Mat opencv_img_2 = img_2.clone();
    std::vector<KeyPoint> keypointsD_scene;

    detector_fast->detect(opencv_img_2,keypointsD_scene,Mat());
    drawKeypoints(opencv_img_2, keypointsD_scene, opencv_img_2);
    imshow("Fast keypoints scene",opencv_img_2);

    char key = waitKey(0);


    return 0;
}