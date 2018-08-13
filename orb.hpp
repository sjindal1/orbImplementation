#ifndef ORB_H
#define ORB_H

#include<vector>
#include <algorithm>

typedef struct { int x, y; } xy; 
typedef unsigned char byte;

#define FAST_THRESHOLD 5
#define NUM_KEY_POINTS 1000
#define CORNER_WIDTH 5

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

struct Image
{
    int xsize;
    int ysize;
    int stride;
    std::vector<std::vector<int>> data;

    Image() {}

    Image(int in_xsize, int in_ysize, std::vector<std::vector<int>> in_data)
        : xsize(in_xsize), ysize(in_ysize), stride(in_xsize), data(in_data) {}
    
    Image(int in_xsize, int in_ysize, int in_stride, std::vector<std::vector<int>> in_data)
        : xsize(in_xsize), ysize(in_ysize), stride(in_stride), data(in_data) {}

    Image(const Image &other)
        : xsize(other.xsize), ysize(other.ysize), stride(other.stride), data(other.data) {}
};

int fast9_corner_score(const byte* p, const int pixel[], int bstart);

xy* fast9_detect(const byte* im, int xsize, int ysize, int stride, int b, int* ret_num_corners);

int* fast9_score(const byte* i, int stride, xy* corners, int num_corners, int b);

std::vector<KeyPoints> fast9_detect_nonmax(Image img, int b);

xy* nonmax_suppression(const xy* corners, const int* scores, int num_corners, int* ret_num_nonmax);

std::vector<KeyPoints> orb_detect_compute(Image img, int fastThreshold = FAST_THRESHOLD, int numKeyPoints = NUM_KEY_POINTS, int edge_width = CORNER_WIDTH);

std::vector<KeyPoints> calculateHarrisAndKeepGood(Image img, std::vector<KeyPoints> &corners, int numKeyPoints,int edge_width);

void calculateOrientationOfCorners(Image img, std::vector<xy> corners, std::vector<KeyPoints> key_points);

#endif