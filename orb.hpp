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
#define NUM_KEY_POINTS 1000
#define CORNER_WIDTH 17
#define HARRIS_BLOCK_SIZE 7
#define PATCH_SIZE 7
#define PI 3.14159265

template<class T>
struct Point{
    T x;
    T y;

    inline Point()
        :x(0), y(0) {}

    inline Point(T in_x, T in_y)
        :x(in_x), y(in_y) {}
};

struct KeyPoints
{
    xy coord;
    float theta;

    KeyPoints() {}

    KeyPoints(xy in_coor, float in_theta)
        :coord(in_coor), theta(in_theta) {}

    KeyPoints(xy in_coor)
        :coord(in_coor) {}
    
    KeyPoints(const KeyPoints &other)
        :coord(other.coord), theta(other.theta) {}
};

struct Match
{
    int template_coor;
    int scene_coor;
    float distance;

    Match() {}

    Match(int tem_coor, int in_scene_coor, float in_distance)
        :template_coor(tem_coor), scene_coor(in_scene_coor), distance(in_distance) {}
};

template <class T>
struct Mat2
{
    int xsize;
    int ysize;
    int stride;
    T *data;

    Mat2() {}

    Mat2(int in_xsize, int in_ysize)
        : xsize(in_xsize), ysize(in_ysize), stride(in_xsize), data(new T[in_xsize*in_ysize]) {}

    Mat2(int in_xsize, int in_ysize, T *in_data)
        : xsize(in_xsize), ysize(in_ysize), stride(in_xsize), data(in_data) {}
    
    Mat2(int in_xsize, int in_ysize, int in_stride, T *in_data)
        : xsize(in_xsize), ysize(in_ysize), stride(in_stride), data(in_data) {}

    Mat2(const Mat2 &other)
        : xsize(other.xsize), ysize(other.ysize), stride(other.stride), data(other.data) {}
};

int fast9_corner_score(const byte* p, const int pixel[], int bstart);

xy* fast9_detect(byte* im, int xsize, int ysize, int stride, int b, int* ret_num_corners, Mat2<short> &gradient_x, Mat2<short> &gradient_y, bool use_grad_indfo);

int* fast9_score(const byte* i, int stride, xy* corners, int num_corners, int b);

std::vector<KeyPoints> fast9_detect_nonmax(Mat2<byte> &img, int b, Mat2<short> &gradient_x, Mat2<short> &gradient_y, bool use_grad_indfo);

std::vector<KeyPoints> fast9_detect_corners(Mat2<byte> &img, int b, int numKeyPoints, Mat2<short> &gradient_x, Mat2<short> &gradient_y, bool use_grad_indfo);

xy* nonmax_suppression(const xy* corners, const int* scores, int num_corners, int* ret_num_nonmax);

void orb_detect(Mat2<byte> &img, std::vector<KeyPoints> &good_corners, bool use_grad_indfo = false, int fastThreshold = FAST_THRESHOLD, int numKeyPoints = NUM_KEY_POINTS, int edge_width = CORNER_WIDTH);

void orb_compute(Mat2<byte> &img, std::vector<KeyPoints> &good_corners, std::vector<Mat2<int32_t>> &decriptors);

void orb_compute(Mat2<byte> &img, std::vector<KeyPoints> &good_corners, std::vector<Mat2<byte>> &decriptors);

std::vector<KeyPoints> calculateHarrisAndKeepGood(Mat2<byte> &img, std::vector<KeyPoints> &corners, Mat2<short> &gradient_x, Mat2<short> &gradient_y, int numKeyPoints,int edge_width);

void calculateOrientationOfCorners(Mat2<byte> &img, std::vector<KeyPoints> &key_points, Mat2<short> &gradient_x, Mat2<short> &gradient_y);

void computerOrbDescriptor(const Mat2<byte> &img, std::vector<KeyPoints> & corners, std::vector<Mat2<int32_t>> &descriptors);

void computerOrbDescriptor(const Mat2<byte> &img, std::vector<KeyPoints> & corners, std::vector<Mat2<byte>> &descriptors);

void remove_edge_keypoints(const Mat2<byte> &img, std::vector<KeyPoints> &corners, int edge_width);

void matchFeatures(std::vector<Mat2<int32_t>> &descriptors_template, std::vector<Mat2<int32_t>> &descriptors_scene, std::vector<std::vector<Match>> &matches);

bool corner_max(byte *im, xy corner, int stride, int kernelSize);

void findHomography(std::vector<Point<int>> &temp, std::vector<Point<int>> &scene, std::vector<float> &a);

void perspectiveTransform(const std::vector<Point<int>> &src, std::vector<Point<int>> &transform, const std::vector<float> &a);
} //namespace orb
#endif