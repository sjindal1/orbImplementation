#ifndef ORB_H
#define ORB_H

#include<vector>
#include <algorithm>
#include<unordered_map>
#include<math.h>

namespace ORB
{
typedef struct { int x, y; } xy; 
typedef unsigned char byte;

#define NO_NONMAX
// #define GAUSSIAN_BLUR

#ifdef NO_NONMAX
    #define FAST_THRESHOLD 20
#else
    #define FAST_THRESHOLD 20
#endif
#define NUM_KEY_POINTS 2000
#define CORNER_WIDTH 17
#define HARRIS_BLOCK_SIZE 7
#define PATCH_SIZE 7
#define PI 3.14159265

/**
 * @brief Stuct is used to represent a point in 2D plane. The coordinates can be int, float or double.
 * 
 * @tparam T 
 */
template<class T>
struct Point{
    T x;
    T y;

    inline Point()
        :x(0), y(0) {}

    inline Point(T in_x, T in_y)
        :x(in_x), y(in_y) {}
};

/**
 * @brief Struct is used to store the information of the features detected in a image.
 * It has the xy coordinates of the features and the angle information.
 * 
 */
struct KeyPoints
{
    /// The xy coordinates in the image
    xy coord;
    // The angle calculated from the intensity info
    float theta;

    /**
     * @brief Construct a new Key Points object
     * 
     */
    KeyPoints() {}

    /**
     * @brief Construct a new Key Points object
     * 
     * @param in_coor xy coordinate
     * @param in_theta the angle information
     */
    KeyPoints(xy in_coor, float in_theta)
        :coord(in_coor), theta(in_theta) {}

    /**
     * @brief Construct a new Key Points object without angle information
     * 
     * @param in_coor xy coordinates
     */
    KeyPoints(xy in_coor)
        :coord(in_coor) {}
    
    /**
     * @brief Construct a new Key Points object from another KeyPoint
     * 
     * @param other The other keypoint
     */
    KeyPoints(const KeyPoints &other)
        :coord(other.coord), theta(other.theta) {}
};

/**
 * @brief Struct is used to store the information about a feature match.
 * 
 */
struct Match
{
    /// The index in template features
    int template_coor;

    /// The index in scene features
    int scene_coor;

    /// The distance calculated using BRIEF information
    float distance;

    /**
     * @brief Construct a new Match object
     * 
     */
    Match() {}

    /**
     * @brief Construct a new Match object
     * 
     * @param tem_coor index in template features
     * @param in_scene_coor index in scene features
     * @param in_distance distance calculated using BRIEF information
     */
    Match(int tem_coor, int in_scene_coor, float in_distance)
        :template_coor(tem_coor), scene_coor(in_scene_coor), distance(in_distance) {}
};

/**
 * @brief General Matrix class
 * 
 * @tparam T 
 */
template <class T>
struct Mat2
{
    /// mat size in x direction
    int xsize;

    /// mat size in y direction
    int ysize;

    /// stride since we use a 1-D array to store all the information
    int stride;

    /// Matrix data
    T *data;

    /**
     * @brief Construct a new Mat 2 object
     * 
     */
    Mat2() {}

    /**
     * @brief Construct a new Mat 2 object and create an empty array of size in_xsize*in_ysize and stride = x_size
     * 
     * @param in_xsize size in x direction
     * @param in_ysize size in y direction
     */
    Mat2(int in_xsize, int in_ysize)
        : xsize(in_xsize), ysize(in_ysize), stride(in_xsize), data(new T[in_xsize*in_ysize]) {}

    /**
     * @brief Construct a new Mat 2 object where stride = x_size
     * 
     * @param in_xsize size in x direction
     * @param in_ysize size in y direction
     * @param in_data mat data
     */
    Mat2(int in_xsize, int in_ysize, T *in_data)
        : xsize(in_xsize), ysize(in_ysize), stride(in_xsize), data(in_data) {}
    
    /**
     * @brief Construct a new Mat 2 object 
     * 
     * @param in_xsize size in x direction
     * @param in_ysize size in y direction
     * @param in_stride mat stride
     * @param in_data mat data 
     */ 
    Mat2(int in_xsize, int in_ysize, int in_stride, T *in_data)
        : xsize(in_xsize), ysize(in_ysize), stride(in_stride), data(in_data) {}

    /**
     * @brief Construct a new Mat 2 object from another Mat2 object
     * 
     * @param other 
     */
    Mat2(const Mat2 &other)
        : xsize(other.xsize), ysize(other.ysize), stride(other.stride), data(other.data) {}
};

/**
 * @brief Calculate the fast 9 score of one feature
 * 
 * @param p 
 * @param pixel 
 * @param bstart 
 * @return int 
 */
int fast9_corner_score(const byte* p, const int pixel[], int bstart);

/**
 * @brief Calculate the fast9 features
 * 
 * @param im Input image
 * @param xsize number of columns
 * @param ysize number of rows
 * @param stride stride length of the image
 * @param b FAST Threshold
 * @param ret_num_corners number of features detected
 * @return xy* List of coordinates of features detected
 */
xy* fast9_detect(byte* im, int xsize, int ysize, int stride, int b, int* ret_num_corners, Mat2<short> &gradient_x, Mat2<short> &gradient_y, bool use_grad_indfo);

/**
 * @brief Calculate FAST 9 score for all the features detected
 * 
 * @param i Input Image
 * @param stride stride length of the image
 * @param corners List of coordinates of features detected
 * @param num_corners number of features detected
 * @param b FAST Threshold
 * @return int* FAST 9 score for all the features detected
 */
int* fast9_score(const byte* i, int stride, xy* corners, int num_corners, int b);

/**
 * @brief Detect fast 9 features in an image using the library nonmax suppression
 * 
 * @param img Input Image
 * @param b FAST 9 Threshold
 * @return std::vector<KeyPoints> List of features detected
 */
std::vector<KeyPoints> fast9_detect_nonmax(Mat2<byte> &img, int b, Mat2<short> &gradient_x, Mat2<short> &gradient_y, bool use_grad_indfo);

/**
 * @brief Detect fast 9 features in an image using the custom nonmax suppression
 * 
 * @param img Input Image
 * @param b FAST 9 Threshold
 * @param numKeyPoints Number of features you want 
 * @return std::vector<KeyPoints> List of features detected
 */
std::vector<KeyPoints> fast9_detect_corners(Mat2<byte> &img, int b, int numKeyPoints, Mat2<short> &gradient_x, Mat2<short> &gradient_y, bool use_grad_indfo);

/**
 * @brief Library nonmax suppression function
 * 
 * @param corners List of features detected
 * @param scores Corresponding scores for all the features
 * @param num_corners Number of features detected
 * @param ret_num_nonmax Number of features after nonmax suppression
 * @return xy* List of features after nonmax suppression
 */
xy* nonmax_suppression(const xy* corners, const int* scores, int num_corners, int* ret_num_nonmax);

/**
 * @brief Detects the ORB features in the given image.
 * Function first finds the FAST 9 features and then ranks them based on the Harris Score
 * It then chooses the top numKeyPoints as the desired set of features. Since FAST 9 provide
 * no orienation information the function also calculates the orientation for each feature
 * 
 * @param img Input Image
 * @param good_corners The vector where the features will be stored
 * @param fastThreshold FAST 9 Threshold
 * @param numKeyPoints Maximum number of keypoints to be used
 * @param edge_width Keypoints within edge width from the corners are ignored
 */
void orb_detect(Mat2<byte> &img, std::vector<KeyPoints> &good_corners, bool use_grad_indfo = false, int fastThreshold = FAST_THRESHOLD, int numKeyPoints = NUM_KEY_POINTS, int edge_width = CORNER_WIDTH);

/**
 * @brief Computes the BRIEF descriptor for each feature detected by orb_detect.
 * This function aligns the descriptor as int32_t array
 * 
 * @param img Input Image
 * @param good_corners Features detected
 * @param decriptors BRIEF Descriptors
 */
void orb_compute(Mat2<byte> &img, std::vector<KeyPoints> &good_corners, std::vector<Mat2<int32_t>> &decriptors);

/**
 * @brief Computes the BRIEF descriptor for each feature detected by orb_detect.
 * This function aligns the descriptor as unsigned char array
 * 
 * @param img Input Image
 * @param good_corners Features detected
 * @param decriptors BRIEF Descriptors
 */
void orb_compute(Mat2<byte> &img, std::vector<KeyPoints> &good_corners, std::vector<Mat2<byte>> &decriptors);

/**
 * @brief Calculated the harris score for all the FAST9 points and keeps the top numPoints
 * 
 * @param img Input Image
 * @param corners Detected FAST9 features
 * @param gradient_x Intensities in x direction
 * @param gradient_y Intensities in y direction
 * @param numKeyPoints Maximum number of keypoints to be used
 * @param edge_width edge width
 * @return std::vector<KeyPoints> Updated set of KeyPoints 
 */
std::vector<KeyPoints> calculateHarrisAndKeepGood(Mat2<byte> &img, std::vector<KeyPoints> &corners, Mat2<short> &gradient_x, Mat2<short> &gradient_y, int numKeyPoints,int edge_width);

/**
 * @brief Calculates the orientation of the FAST9 features detcted since 
 * fast 9 does not provide orientaion information.
 * 
 * @param img Input Image
 * @param key_points Detected FAST9 features
 * @param gradient_x Intensities in x direction
 * @param gradient_y Intensities inyx direction
 */
void calculateOrientationOfCorners(Mat2<byte> &img, std::vector<KeyPoints> &key_points, Mat2<short> &gradient_x, Mat2<short> &gradient_y);

/**
 * @brief Calculates the ORB descriptor as int32_t array
 * 
 * @param img Input Image
 * @param corners Detected ORB features
 * @param descriptors Output descriptors
 */
void computerOrbDescriptor(const Mat2<byte> &img, std::vector<KeyPoints> & corners, std::vector<Mat2<int32_t>> &descriptors);

/**
 * @brief Calculates the ORB descriptor as unsigned char array
 * 
 * @param img Input Image
 * @param corners Detected ORB features
 * @param descriptors Output descriptors
 */
void computerOrbDescriptor(const Mat2<byte> &img, std::vector<KeyPoints> & corners, std::vector<Mat2<byte>> &descriptors);

/**
 * @brief Remove all the keypoints that are too close to the edge so that we don't have to worry about the calculations
 * 
 * @param img Input Image
 * @param corners Detected FAST9 features
 * @param edge_width 
 */
void remove_edge_keypoints(const Mat2<byte> &img, std::vector<KeyPoints> &corners, int edge_width);

/**
 * @brief Brute force matching of features.
 * 
 * @param descriptors_template BRIEF Descriptors in template image
 * @param descriptors_scene BRIEF Descriptors in scene image
 * @param matches 2 best matched for each template descriptor in scene
 */
void matchFeatures(std::vector<Mat2<int32_t>> &descriptors_template, std::vector<Mat2<int32_t>> &descriptors_scene, std::vector<std::vector<Match>> &matches);

/**
 * @brief The  function is used for custom nonmax suppression. It checks if the corner is max in 3x3 or 5x5 block
 * 
 * @param im Image data
 * @param corner Detected ORB features
 * @param stride image stride
 * @param kernelSize the kernel size can be 3 or 5
 * @return true is max
 * @return false is not max
 */
bool corner_max(byte *im, xy corner, int stride, int kernelSize);

/**
 * @brief Find the vector a using random sampling of the mathces in template and scene(RANSAC)
 * uses the equations 
 * ***********************
 * x' = a1*x - a2*y + a3 *
 * y' = a2*x + a1*y + a4 *
 * ***********************
 * to find a1, a2, a3, a4
 * 
 * @param temp 
 * @param scene 
 * @param a 
 */
void findHomography(std::vector<Point<int>> &temp, std::vector<Point<int>> &scene, std::vector<float> &a);

/**
 * @brief Finds the transform of all the src points using the a vector 
 * uses the equations 
 * ***********************
 * x' = a1*x - a2*y + a3 *
 * y' = a2*x + a1*y + a4 *
 * ***********************
 * 
 * @param src 
 * @param transform 
 * @param a 
 */
void perspectiveTransform(const std::vector<Point<int>> &src, std::vector<Point<int>> &transform, const std::vector<float> &a);

/**
 * @brief Find the vector a using average of all the good matches in template and scene
 * uses the equations 
 * ***********************
 * x' = a1*x - a2*y + a3 *
 * y' = a2*x + a1*y + a4 *
 * ***********************
 * to find a1, a2, a3, a4
 * 
 * @param temp 
 * @param scene 
 * @param a 
 */
void findParams(std::vector<::ORB::Point<int>> &obj, std::vector<::ORB::Point<int>> &scene, std::vector<float> &a);
 /**
 * @brief Calculates the best matches using RANSAC 
 * 
 * @param corners ORB Template feature
 * @param corners_scene ORB Scene feature
 * @param good_matches Matches using brute force
 * @param best_matches The output matches
 */
void calculateBestMatches(std::vector<KeyPoints> &corners, std::vector<KeyPoints> &corners_scene, std::vector<Match> &good_matches, std::vector<Match> &best_matches);
 /**
 * @brief collects the Harris and angle calculation into one function and doesn't use previously calculated intensity informations thus saving space
 * 
 * @param img Input image
 * @param key_points FAST9 features
 * @param good_features output features
 * @param numKeyPoints Maximum features to be preserved.
 */
void calculateBestCornersAndOrientation(Mat2<byte> &img, std::vector<KeyPoints> &key_points, std::vector<KeyPoints> &good_features, int numKeyPoints);
} //namespace orb
#endif