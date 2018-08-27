#include <stdio.h>
#include <string>
#include <iostream>
#include <sys/types.h>
#include <dirent.h>
#include <ctime>
#include "orb.hpp"
#include <cstdlib>
// #include <flann/flann.hpp>

#include <opencv2/core/core.hpp>
#include <opencv2/features2d.hpp>
// #include <opencv2/xfeatures2d.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/calib3d.hpp>

using namespace cv;
using namespace ORB;
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

/**
 * @brief The function is used to plot the matches found between the template features and the scene features. 
 * It does not find or draw the template area in the scene according to the matches found.
 * 
 * @param displayImage Template Image
 * @param displayImageScene Scene Image
 * @param corners Features detected in the template image
 * @param corners_scene Features detected in the scene image
 * @param matches Matches found between the template and scene features
 */
void plotMatchesWithoutBox(Mat &displayImage, Mat &displayImageScene, std::vector<KeyPoints> &corners,std::vector<KeyPoints> &corners_scene, std::vector<Match> &matches){
    
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

    imshow("Matched features flann",displayTogether);
}

/**
 * @brief The function is used to plot the matches found between the template features and the scene features. 
 * It also finds and draws the template area in the scene according to the matches found.
 * 
 * @param displayImage 
 * @param displayImageScene 
 * @param corners 
 * @param corners_scene 
 * @param matches 
 * @param name 
 */
void plotMatches(Mat &displayImage, Mat &displayImageScene, std::vector<KeyPoints> &corners,std::vector<KeyPoints> &corners_scene, std::vector<Match> &matches, std::string name="Matched features"){
    
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

#ifdef NO_NONMAX
    std::vector<::ORB::Point<int>> obj, transfor_obj;
    std::vector<::ORB::Point<int>> scene;

    for (int i = 0; i < matches.size(); i++)
    {
        obj.push_back(::ORB::Point<int>(corners[matches[i].template_coor].coord.x, corners[matches[i].template_coor].coord.y));
        scene.push_back(::ORB::Point<int>(corners_scene[matches[i].scene_coor].coord.x, corners_scene[matches[i].scene_coor].coord.y));
    }

    std::vector<float> a;
    findParams(obj, scene, a);

    // std::cout << H_1 << std::endl;

    //-- Get the corners from the image_1 ( the object to be "detected" )
    std::vector<::ORB::Point<int>> obj_corners(4);
    obj_corners[0] = ::ORB::Point<int>(15, 15);
    obj_corners[1] = ::ORB::Point<int>(displayImage.cols - 15, 15);
    obj_corners[2] = ::ORB::Point<int>(displayImage.cols - 15, displayImage.rows - 15);
    obj_corners[3] = ::ORB::Point<int>(15, displayImage.rows - 15);
    std::vector<::ORB::Point<int>> scene_corners;

    perspectiveTransform(obj_corners, scene_corners, a);

    //-- Draw lines between the corners (the mapped object in the scene - image_2 )
    cv::line(displayTogether, 
            Point2f(scene_corners[0].x + displayImage.cols, scene_corners[0].y), 
            Point2f(scene_corners[1].x + displayImage.cols, scene_corners[1].y), Scalar(0, 255, 0), 4);
    cv::line(displayTogether, 
            Point2f(scene_corners[1].x, scene_corners[1].y)  + Point2f(displayImage.cols, 0), 
            Point2f(scene_corners[2].x, scene_corners[2].y) + Point2f(displayImage.cols, 0), Scalar(0, 255, 0), 4);
    cv::line(displayTogether, 
            Point2f(scene_corners[2].x, scene_corners[2].y)  + Point2f(displayImage.cols, 0), 
            Point2f(scene_corners[3].x, scene_corners[3].y) + Point2f(displayImage.cols, 0), Scalar(0, 255, 0), 4);
    cv::line(displayTogether, 
            Point2f(scene_corners[3].x, scene_corners[3].y)  + Point2f(displayImage.cols, 0), 
            Point2f(scene_corners[0].x, scene_corners[0].y) + Point2f(displayImage.cols, 0), Scalar(0, 255, 0), 4);

#endif
    imshow(name, displayTogether);
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
    orb_detect(img_template, corners, false);
    orb_compute(img_template, corners, decriptors);
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
    orb_detect(img_scene, corners_scene);
    std::cout << "Time ORB detect : " << (std::clock() - start) / (double)(CLOCKS_PER_SEC / 1000) << " ms" << std::endl;
    
    start = std::clock();
    orb_compute(img_scene, corners_scene, decriptors_scene);
    std::cout << "Time ORB compute : " << (std::clock() - start) / (double)(CLOCKS_PER_SEC / 1000) << " ms" << std::endl;

    std::vector<Mat2<byte>> descriptor_temp_byte;
    std::vector<Mat2<byte>> descriptor_scene_byte;

    orb_compute(img_template, corners, descriptor_temp_byte);
    start = std::clock();
    orb_compute(img_scene, corners_scene, descriptor_scene_byte);
    std::cout << "Time ORB compute byte : " << (std::clock() - start) / (double)(CLOCKS_PER_SEC / 1000) << " ms" << std::endl;

    start = std::clock();
    std::vector<std::vector<Match>> matches;
    matchFeatures(decriptors, decriptors_scene, matches);
    std::vector<Match> good_matches;

    for(std::vector<Match> nnMatches:matches){
        // float ratio = 0.75;
        // if(nnMatches[0].distance < ratio*nnMatches[1].distance){
        //     good_matches.push_back(nnMatches[0]);
        // }
        good_matches.push_back(nnMatches[0]);
    }
    std::cout << "Time Brute Force : " << (std::clock() - start) / (double)(CLOCKS_PER_SEC / 1000) << " ms" << std::endl;
    Mat displayImageScene = img_2.clone();

    cvtColor(displayImageScene, displayImageScene, CV_GRAY2BGR);

#ifdef NO_NONMAX
    std::vector<Match> best_matches;
    calculateBestMatches(corners, corners_scene, good_matches, best_matches);
    plotMatches(displayImage, displayImageScene, corners, corners_scene, best_matches);
#else
    plotMatches(displayImage, displayImageScene, corners, corners_scene, good_matches);
#endif



    // typedef ::flann::Hamming<byte> Distance;
	// typedef Distance::ElementType ElementType;
	// typedef Distance::ResultType DistanceType;
	// ::flann::Matrix<byte> query(new byte[descriptor_temp_byte.size()*descriptor_temp_byte[0].xsize], descriptor_temp_byte.size(), descriptor_temp_byte[0].xsize);
	// ::flann::Matrix<byte> data(new byte[descriptor_scene_byte.size()*descriptor_scene_byte[0].xsize], descriptor_scene_byte.size(), descriptor_scene_byte[0].xsize);
	// ::flann::Matrix<size_t> gt_indices;
	// // ::flann::Matrix<DistanceType> dists;
	// ::flann::Matrix<DistanceType> gt_dists;
	// // ::flann::Matrix<size_t> indices;
	// unsigned int k_nn_ = 2;

    // byte *matrix_ptr = query.ptr();
    // for(int i=0; i<descriptor_temp_byte.size(); i++){
    //     byte *mat2_ptr =  descriptor_temp_byte[i].data;
    //     for(int j=0; j<descriptor_temp_byte[i].xsize; j++){
    //         *matrix_ptr = *mat2_ptr;
    //         matrix_ptr++;
    //         mat2_ptr++;
    //     }
    // }

    // matrix_ptr = data.ptr();
    // for(int i=0; i<descriptor_scene_byte.size(); i++){
    //     byte *mat2_ptr =  descriptor_scene_byte[i].data;
    //     for(int j=0; j<descriptor_scene_byte[i].xsize; j++){
    //         *matrix_ptr = *mat2_ptr;
    //         matrix_ptr++;
    //         mat2_ptr++;
    //     }
    // }
    
    
    // start = std::clock();
    // ::flann::Index<Distance> index(data, ::flann::LshIndexParams(12, 20, 2));
    // index.buildIndex();

    // // std::vector< std::vector<int> > indices;
    // // std::vector<std::vector<DistanceType> > dists;
    // gt_indices = ::flann::Matrix<size_t>(new size_t[query.rows * k_nn_], query.rows, k_nn_);
    // gt_dists = ::flann::Matrix<DistanceType>(new DistanceType[query.rows * k_nn_], query.rows, k_nn_);
    // index.knnSearch(query, gt_indices, gt_dists, k_nn_, ::flann::SearchParams(-1));
    
    // std::cout << "Time FLANN : " << (std::clock() - start) / (double)(CLOCKS_PER_SEC / 1000) << " ms" << std::endl;
    // // index.knnSearch(query, indices, dists, k_nn_, ::flann::SearchParams(-1));

    // size_t *index_ptr = gt_indices.ptr();
    // DistanceType *dist_ptr_1 = gt_dists.ptr();
    // DistanceType *dist_ptr_2 = dist_ptr_1 + 1;

    // std::vector<Match> good_matches_flann;
    // for(int i=0; i < gt_indices.rows; i++){
    //     if(*dist_ptr_1 < 0.7 * *dist_ptr_2){
    //         good_matches_flann.push_back(Match(i, *index_ptr, *dist_ptr_1));
    //     }
    //     dist_ptr_1 += 2;
    //     dist_ptr_2 += 2;
    //     index_ptr += 2;
    // }
    // std::vector<Match> best_matches_flann;
    // calculateBestMatches(corners, corners_scene, good_matches_flann, best_matches_flann);
    // plotMatches(displayImage, displayImageScene, corners, corners_scene, best_matches_flann, "Matched Features FLANN");
    // plotMatchesWithoutBox(displayImage, displayImageScene, corners, corners_scene, best_matches_flann);





    // for(std::vector<int> knn_indices:indices){
    //     for(int index:knn_indices){
    //         std::cout << index << std::endl;
    //     }
    // }
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