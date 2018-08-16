#include <stdio.h>
#include <string>
#include <iostream>
#include <sys/types.h>
#include <dirent.h>
#include <ctime>
#include "orb.hpp"
#include <cstdlib>

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

void plotMatches(Mat &displayImage, Mat &displayImageScene, std::vector<KeyPoints> &corners,std::vector<KeyPoints> &corners_scene, std::vector<Match> &matches){
    
    Mat templateImage = displayImage.clone();
    Mat seceneImage = displayImageScene.clone();

    if(templateImage.rows > seceneImage.rows){
        copyMakeBorder(seceneImage, seceneImage, 0, templateImage.rows - seceneImage.rows, 0, 0, BORDER_CONSTANT, Scalar(0));
    }else{
        copyMakeBorder(templateImage, templateImage, 0, displayImageScene.rows - templateImage.rows, 0, 0, BORDER_CONSTANT, Scalar(0));
    }

    Mat displayTogether;
    displayTogether.push_back(templateImage.t());
    displayTogether.push_back(seceneImage.t());

    displayTogether = displayTogether.t();

    for(Match match:matches){
        // if(match.distance > 55)
        //     continue;
        Scalar color = Scalar(rand()%255, rand()%255, rand()%255);
        xy corner1 = corners[match.template_coor].coord;
        xy corner2 = corners_scene[match.scene_coor].coord;
        corner2.x += displayImage.cols;
        circle(displayTogether, Point2d(corner1.x, corner1.y), 2, color);
        circle(displayTogether, Point2d(corner2.x, corner2.y), 2, color);
        line(displayTogether, Point2d(corner1.x, corner1.y), Point2d(corner2.x, corner2.y), color);        
    }

    std::vector<Point2f> obj;
    std::vector<Point2f> scene;

    for (int i = 0; i < matches.size(); i++)
    {
        //-- Get the keypoints from the good matches
        obj.push_back(Point2f(corners[matches[i].template_coor].coord.x, corners[matches[i].template_coor].coord.y));
        scene.push_back(Point2f(corners_scene[matches[i].scene_coor].coord.x, corners_scene[matches[i].scene_coor].coord.y));
    }

    float mat_object[9];
    findParams(obj, scene, mat_object);

    Mat H_1 = cv::Mat(3, 3, CV_32F, mat_object);

    std::cout << H_1 << std::endl;

    //-- Get the corners from the image_1 ( the object to be "detected" )
    std::vector<Point2f> obj_corners(4);
    obj_corners[0] = cvPoint(15, 15);
    obj_corners[1] = cvPoint(displayImage.cols - 15, 15);
    obj_corners[2] = cvPoint(displayImage.cols - 15, displayImage.rows - 15);
    obj_corners[3] = cvPoint(15, displayImage.rows - 15);
    std::vector<Point2f> scene_corners(4);

    cv::perspectiveTransform(obj_corners, scene_corners, H_1);

    //-- Draw lines between the corners (the mapped object in the scene - image_2 )
    cv::line(displayTogether, scene_corners[0] + Point2f(displayImage.cols, 0), scene_corners[1] + Point2f(displayImage.cols, 0), Scalar(0, 255, 0), 4);
    cv::line(displayTogether, scene_corners[1] + Point2f(displayImage.cols, 0), scene_corners[2] + Point2f(displayImage.cols, 0), Scalar(0, 255, 0), 4);
    cv::line(displayTogether, scene_corners[2] + Point2f(displayImage.cols, 0), scene_corners[3] + Point2f(displayImage.cols, 0), Scalar(0, 255, 0), 4);
    cv::line(displayTogether, scene_corners[3] + Point2f(displayImage.cols, 0), scene_corners[0] + Point2f(displayImage.cols, 0), Scalar(0, 255, 0), 4);


    imshow("Matched features",displayTogether);
}

void calculateBestMatches(std::vector<KeyPoints> &corners, std::vector<KeyPoints> &corners_scene, std::vector<Match> &good_matches, std::vector<Match> &best_matches){
    std::vector<Point2f> obj, transfor_obj;
    std::vector<Point2f> scene;

    for (int i = 0; i < good_matches.size(); i++)
    {
        //-- Get the keypoints from the good matches
        obj.push_back(Point2f(corners[good_matches[i].template_coor].coord.x, corners[good_matches[i].template_coor].coord.y));
        scene.push_back(Point2f(corners_scene[good_matches[i].scene_coor].coord.x, corners_scene[good_matches[i].scene_coor].coord.y));
    }
    
    std::clock_t start;
    start = std::clock();
    Mat H = findHomography(obj, scene, CV_RANSAC);
    std::cout << "Time RANSAC: " << (std::clock() - start) / (double)(CLOCKS_PER_SEC / 1000) << " ms" << std::endl;

    std::cout << H << std::endl;

    cv::perspectiveTransform(obj, transfor_obj, H);

    for (int i = 0; i < obj.size(); i++)
    {
        float d = pow(scene[i].x - transfor_obj[i].x, 2) + pow(scene[i].y - transfor_obj[i].y, 2);
        if (d < 9)
        {
            best_matches.push_back(good_matches[i]);
        }
    }
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

    copyMakeBorder(img_1, img_1, 15, 15, 15, 15,  BORDER_CONSTANT, Scalar(img_1.at<uchar>(0, 0)));
    Mat2<byte> img_template(img_1.cols, img_1.rows);

    int k=0;
    for(int i=0; i < img_1.rows; i++){
        for(int j = 0; j < img_1.cols; j++){
            img_template.data[k++] = img_1.at<uchar>(i,j);
        }
    }


    // bitwise_not ( img_1, img_1 );

    if (!img_1.data)
    {
        std::cout << " --(!) Error reading template image " << std::endl;
        return -1;
    }

    std::clock_t start;
    start = std::clock();
    std::vector<KeyPoints> corners;
    std::vector<Mat2<int32_t>> decriptors;
    orb_detect_compute(img_template, corners, decriptors);
    std::cout << "Time: " << (std::clock() - start) / (double)(CLOCKS_PER_SEC / 1000) << " ms" << std::endl;

    Mat displayImage = img_1.clone();

    cvtColor(displayImage, displayImage, CV_GRAY2BGR);


    std::cout << corners.size() << std::endl;


    Mat opencv_img_1 = img_1.clone();
    std::vector<KeyPoint> keypointsD;
    Ptr<FastFeatureDetector> detector_fast=FastFeatureDetector::create();

    detector_fast->detect(opencv_img_1,keypointsD,Mat());
    drawKeypoints(opencv_img_1, keypointsD, opencv_img_1);
    imshow("Fast keypoints",opencv_img_1);


    /// Scene

    Mat img_2 = imread(scene_filename, IMREAD_GRAYSCALE); // Read the file

    Mat2<byte> img_scene(img_2.cols, img_2.rows);

    k = 0;
    for(int i=0; i < img_2.rows; i++){
        for(int j = 0; j < img_2.cols; j++){
            img_scene.data[k++] = img_2.at<uchar>(i,j);
        }
    }      


    start = std::clock();
    std::vector<KeyPoints> corners_scene;
    std::vector<Mat2<int32_t>> decriptors_scene;
    orb_detect_compute(img_scene, corners_scene, decriptors_scene);
    std::cout << "Time: " << (std::clock() - start) / (double)(CLOCKS_PER_SEC / 1000) << " ms" << std::endl;

    Mat displayImageScene = img_2.clone();

    cvtColor(displayImageScene, displayImageScene, CV_GRAY2BGR);

    std::vector<std::vector<Match>> matches;
    matchFeatures(decriptors, decriptors_scene, matches);
    std::vector<Match> good_matches;

    for(std::vector<Match> nnMatches:matches){
        float ratio = 0.75;
        if(nnMatches[0].distance < ratio*nnMatches[1].distance){
            good_matches.push_back(nnMatches[0]);
        }
        // good_matches.push_back(nnMatches[0]);
    }

    std::vector<Match> best_matches;
    calculateBestMatches(corners, corners_scene, good_matches, best_matches);
    plotMatches(displayImage, displayImageScene, corners, corners_scene, best_matches);

    for(KeyPoints corner:corners){
        Scalar color = Scalar(rand()%255, rand()%255, rand()%255);
        circle(displayImage, Point2d(corner.coord.x, corner.coord.y), 2, color);
    }
    cv::imshow("Fast features template", displayImage);

    for(KeyPoints corner:corners_scene){
        Scalar color = Scalar(rand()%255, rand()%255, rand()%255);
        circle(displayImageScene, Point2d(corner.coord.x, corner.coord.y), 2, color);
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