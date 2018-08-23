#include <stdlib.h>
#include <iostream>
#include "orb.hpp"
#include <ctime>
namespace ORB
{
alignas(32) char gaussian_bit_pattern_31_x_a[256] = { 8,4,-11,7,2,1,-2,-13,-13,10,-13,-11,7,-4,-13,-9,12,-3,-6,11,4,5,3,-8,-2,-13,-7,-4,-10,5,5,1,9,4,2,-4,-8,4,0,-13,-3,-6,8,0,7,-13,10,-6,10,-13,-13,3,5,-1,3,2,-13,-13,-13,-7,6,-9,-2,-12,3,-7,-3,2,-11,-1,5,-4,-9,-12,10,7,-7,-4,7,-7,-13,-3,7,-13,1,2,-4,-1,7,1,9,-1,-13,7,12,6,5,2,3,2,9,-8,-11,1,6,2,6,3,7,-11,-10,-5,-10,8,4,-10,4,-2,-5,7,-9,-5,8,-9,1,7,-2,11,-12,3,5,0,-9,0,-1,5,3,-13,-5,-4,6,-7,-13,1,4,-2,2,-2,4,-6,-3,7,4,-13,7,7,-7,-8,-13,2,10,-6,8,2,-11,-12,-11,5,-2,-1,-13,-10,-3,2,-9,-4,-4,-6,6,-13,11,7,-1,-4,-7,-13,-7,-8,-5,-13,1,1,9,5,-1,-9,-1,-13,8,2,7,-10,-10,4,3,-4,5,4,-9,0,-12,3,-10,8,-8,2,10,6,-7,-3,-1,-3,-8,4,2,6,3,11,-3,4,2,-10,-13,-13,6,0,-13,-9,-13,5,2,-1,9,11,3,-1,3,-13,5,8,7,-10,7,9,7,-1};
alignas(32) char gaussian_bit_pattern_31_y_a[256] = {-3,2,9,-12,-13,-7,-10,-13,-3,4,-8,7,7,-5,2,0,-6,6,-13,-13,7,-3,-7,-7,11,12,3,2,-12,-12,-6,0,11,7,-1,-12,-5,11,-8,-2,-2,9,12,9,-5,-6,7,-3,-9,8,0,3,7,7,-10,-4,0,-7,3,12,-10,-1,-5,5,-10,-7,-2,9,-13,6,-3,-13,-6,-10,2,12,-13,9,-1,6,11,7,-8,-7,-3,-6,3,-13,1,-1,1,-9,-13,7,-5,3,-13,-12,8,6,-12,4,12,12,-9,3,3,-3,8,-5,11,-8,5,-1,-6,12,-2,0,-8,-6,-13,-13,-8,-11,-8,-4,1,-6,-9,7,5,-4,12,7,2,11,5,-4,9,-7,5,6,6,-10,1,-2,-12,-13,1,-10,-13,5,-2,9,1,-8,-4,11,6,4,-5,-5,-3,-12,-2,-13,0,-3,-13,-8,-11,-2,9,-3,-13,6,12,-11,-3,11,11,-5,12,-8,1,-12,-2,5,-1,7,5,0,12,-8,11,-3,-10,1,-11,-13,-13,-10,-8,-6,12,2,-13,-13,9,3,1,2,-10,-13,-12,2,6,8,10,-9,-13,-7,-2,2,-5,-9,-1,-1,0,-11,-4,-6,7,12,0,-1,3,8,-6,-9,7,-6,5,-3,0,4,-6,0,8,9,-4,4,3,-7,0,-6};
alignas(32) char gaussian_bit_pattern_31_x_b[256] = {9,7,-8,12,2,1,-2,-11,-12,11,-8,-9,12,-3,-12,-7,12,-2,-4,12,5,10,6,-6,-1,-8,-5,-3,-6,6,7,4,11,4,4,-2,-7,9,1,-8,-2,-4,10,1,11,-11,12,-6,12,-8,-8,7,10,1,5,3,-13,-12,-11,-4,12,-7,0,-7,8,-4,-1,5,-5,0,5,-4,-9,-8,12,12,-6,-3,12,-5,-12,-2,12,-11,12,3,-2,1,8,3,12,-1,-10,10,12,7,6,2,4,12,10,-7,-4,2,7,3,11,8,9,-6,-5,-3,-9,12,6,-8,6,-2,-5,10,-8,-5,9,-9,1,9,-1,12,-6,7,10,2,-5,2,1,7,6,-8,-3,-3,8,-6,-5,3,8,2,12,0,9,-3,-1,12,5,-9,8,7,-7,-7,-12,3,12,-6,9,2,-10,-7,-10,11,-1,0,-12,-10,-2,3,-4,-3,-2,-4,6,-5,12,12,0,-3,-6,-8,-6,-6,-4,-8,5,10,10,10,1,-6,1,-8,10,3,12,-5,-8,8,8,-3,10,5,-4,3,-6,4,-10,12,-6,3,11,8,-6,-3,-1,-3,-8,12,3,11,7,12,-3,4,2,-8,-11,-11,11,1,-9,-6,-8,8,3,-1,11,12,3,0,4,-10,12,9,8,-10,12,10,12,0};
alignas(32) char gaussian_bit_pattern_31_y_b[256] = {5,-12,2,-13,12,6,-4,-8,-9,9,-9,12,6,0,-3,5,-1,12,-8,-8,1,-3,12,-2,-10,10,-3,7,11,-7,-1,-5,-13,12,4,7,-10,12,-13,2,3,-9,7,3,-10,0,1,12,-4,-12,-4,8,-7,-12,6,-10,5,12,8,7,8,-6,12,5,-13,5,-7,-11,-13,-1,2,12,6,-4,-3,12,5,4,2,1,5,-6,-7,-12,12,0,-13,9,-6,12,6,3,5,12,9,11,10,3,-6,-13,3,9,-6,-8,-4,-2,0,-8,3,-4,10,12,0,-6,-11,7,7,12,2,12,-8,-2,-13,0,-2,1,-4,-11,4,12,8,8,-13,12,7,-9,-8,9,-3,-12,0,12,-2,10,-4,-13,12,-6,3,-5,1,-11,-7,-5,6,6,1,-8,-8,9,3,7,-8,8,3,-9,-5,8,12,9,-5,11,-13,2,0,-10,-7,9,11,5,6,-2,7,-2,7,-13,-8,-9,5,10,-13,-13,-1,-9,-13,2,12,-10,-6,-6,-9,-7,-13,5,-13,-3,-12,-1,3,-9,1,-8,9,12,-5,7,-8,-12,5,9,5,4,3,12,11,-13,12,4,6,12,1,1,1,-13,-13,4,-2,-3,-2,10,-9,-1,-2,-8,5,10,5,5,11,-6,-12,9,4,-2,-2,-11};


#define Compare(X, Y) ((X) >= (Y))

std::vector<KeyPoints> calculateHarrisAndKeepGood(Mat2<byte> &img, std::vector<KeyPoints> &corners, Mat2<short> &gradient_x, Mat2<short> &gradient_y, int numKeyPoints, int edge_width)
{

	std::vector<std::pair<float, int>> harris_score;
	int n = corners.size();
	for (int i = 0; i < n; i++)
	{
		xy corner = corners[i].coord;
		// // Ignore the corner if it lies close to the edge
		// if (corner.x <= edge_width || corner.y < edge_width || corner.x > img.xsize - edge_width || corner.y > img.ysize - edge_width)
		// 	continue;

		float intensiy_x_2 = 0, intensiy_y_2 = 0, intensiy_x_y = 0;
		int r = HARRIS_BLOCK_SIZE / 2;
		for (int i = -r; i <= r; i++)
		{
            int row_center = (corner.y + i)*gradient_x.stride + corner.x;
			for (int j = -r; j <= r; j++)
			{
				// using sobel
				float intensiy_x = gradient_x.data[row_center + j];
				float intensiy_y = gradient_y.data[row_center + j];
				intensiy_x_2 += intensiy_x * intensiy_x;
				intensiy_y_2 += intensiy_y * intensiy_y;
				intensiy_x_y += intensiy_x * intensiy_y;
			}
		}

		float score = ((intensiy_x_2 * intensiy_y_2) - (intensiy_x_y * intensiy_x_y)) - 0.04 * (intensiy_x_2 + intensiy_y_2) * (intensiy_x_2 + intensiy_y_2);

		harris_score.push_back(std::make_pair(score, i));
	}

	std::sort(harris_score.rbegin(), harris_score.rend());

	std::vector<KeyPoints> good_corners;
    numKeyPoints = std::min(n, numKeyPoints);
	for (int i = 0; i < numKeyPoints; i++)
	{
		good_corners.push_back(corners[harris_score[i].second]);
	}

	return good_corners;
}

void calculateSobel(Mat2<byte> &img, Mat2<short> &gradient_x, Mat2<short> &gradient_y)
{
    Mat2<short> row_gradient_x(img.xsize, img.ysize);
    Mat2<short> row_gradient_y(img.xsize, img.ysize);
    byte* center = img.data, *center_end;
    short* center_x = row_gradient_x.data;
    short* center_y = row_gradient_y.data;

    for (int i = 0; i < img.ysize; i++)
	{
        center_end = center + img.stride - 2;
		while (center < center_end)
		{
            //byte *center_j = center + j - 1;
            *(center_x) = *(center) - *(center + 2);
            *(center_y) = *(center) + *(center + 1)*2 + *(center + 2);
            center++;
            center_x++;
            center_y++;
        }
        center += 2;
        center_x += 2;
        center_y += 2;
    }

    center_x = row_gradient_x.data + 1;
    center_y = row_gradient_y.data + 1;
    short *grad_x_ptr = gradient_x.data + img.stride + 1;
    short *grad_y_ptr = gradient_y.data + img.stride + 1;
    short *center_x_end; 
    for (int i = 0; i < img.ysize - 2; i++)
	{
        center_x_end = center_x + img.stride - 2;
		while(center_x < center_x_end)
		{
            *(grad_x_ptr) = *(center_x) + 2* *(center_x + img.stride) + *(center_x + 2*img.stride);
            *(grad_y_ptr) = *(center_y) - *(center_y + 2*img.stride);
            grad_x_ptr++;
            grad_y_ptr++;
            center_x++;
            center_y++;
		}
        grad_x_ptr+= 2;
        grad_y_ptr+= 2;
        center_x+= 2;
        center_y+= 2;
	}
}

void calculateOrientationOfCorners(Mat2<byte> &img, std::vector<KeyPoints> &key_points, Mat2<short> &gradient_x, Mat2<short> &gradient_y)
{
	for (int i = 0; i < key_points.size(); i++)
	{
		xy corner = key_points[i].coord;

		int path_sizes[] = {corner.x - 1, img.xsize - corner.x - 1, corner.y - 1, img.ysize - corner.y - 1, PATCH_SIZE / 2};
		int r = *std::min_element(path_sizes, path_sizes + 5);
		float m_01 = 0, m_10 = 0;
		for (int i = -r; i <= r; i++)
		{
            int row_center = (corner.y + i)*gradient_y.stride + corner.x;
			for (int j = -r; j <= r; j++)
			{
				int x = corner.x + j, y = corner.y + i;
				m_01 += y * gradient_y.data[row_center + j];
				m_10 += x * gradient_x.data[row_center + j];
                // m_01 += *(img.data + y*img.stride + x) * y;
                // m_10 += *(img.data + y*img.stride + x) * x;
                // float gradient = std::sqrt(std::pow(gradients[y][x].second, 2) + std::pow(gradients[y][x].first, 2));
                // m_01 += y*gradient;
                // m_10 += x*gradient;
			}
		}
		float theta = atan2(m_01, m_10) * 180 / PI;
        // float theta = atan(m_01/m_10) * 180 / PI;
		key_points[i].theta = theta;
	}
}

Mat2<short> row_intensity_x(PATCH_SIZE, PATCH_SIZE + 2);
Mat2<short> row_intensity_y(PATCH_SIZE, PATCH_SIZE + 2);
void calculateXYIntensities(Mat2<byte> &img,xy &corner, Mat2<short> &intensity_x, Mat2<short> &intensity_y){

    int r = (PATCH_SIZE + 2)/2;
    byte* row_start = img.data + (corner.y-r)*img.stride + corner.x - r, *row_end;
    short* in_row_x = row_intensity_x.data;
    short* in_row_y = row_intensity_y.data;

    for(int i=0; i<PATCH_SIZE + 2; i++){
        row_end = row_start + PATCH_SIZE;
        while(row_start < row_end){
            *(in_row_x) = *(row_start) - *(row_start + 2);
            *(in_row_y) = *(row_start) + *(row_start + 1)*2 + *(row_start + 2);
            row_start++;
            in_row_x++;
            in_row_y++;
        }
        row_start += img.stride - PATCH_SIZE;
    }

    in_row_x = row_intensity_x.data;
    in_row_y = row_intensity_y.data;
    short *in_x = intensity_x.data, *in_y = intensity_y.data;
    short *in_row_x_end, *in_row_table_end = in_row_x + PATCH_SIZE*PATCH_SIZE;
    while(in_row_x < in_row_table_end){
        in_row_x_end = in_row_x + PATCH_SIZE;
        while(in_row_x < in_row_x_end){
            *(in_x) = *(in_row_x) + 2* *(in_row_x + PATCH_SIZE) + *(in_row_x + 2*PATCH_SIZE);
            *(in_y) = *(in_row_y) - *(in_row_y + 2*PATCH_SIZE);
            in_row_x++;
            in_row_y++;
            in_x++;
            in_y++;
        }
    }
}

void calculateBestCornersAndOrientation(Mat2<byte> &img, std::vector<KeyPoints> &key_points, std::vector<KeyPoints> &good_features, int numKeyPoints)
{
    Mat2<short> intensity_x(PATCH_SIZE, PATCH_SIZE);
    Mat2<short> intensity_y(PATCH_SIZE, PATCH_SIZE);
    int r = PATCH_SIZE/2;
    std::vector<std::pair<float, int>> harris_score;
    for (int i = 0; i < key_points.size(); i++)
	{
		xy corner = key_points[i].coord;

        calculateXYIntensities(img, corner, intensity_x, intensity_y);

        short *in_x_ptr = intensity_x.data;
        short *in_y_ptr = intensity_y.data;

        float intensity_x_2 = 0, intensity_y_2 = 0, intensity_x_y = 0;
        float m_01 = 0, m_10 = 0;
        for (int i = -r; i <= r; i++)
		{
            int y = corner.y + i;
			for (int j = -r; j <= r; j++)
			{
				int x = corner.x + j;
                m_01 += y* *(in_y_ptr);
                m_10 += x* *(in_x_ptr);
                intensity_x_2 += *(in_x_ptr) * *(in_x_ptr);
                intensity_y_2 += *(in_y_ptr) * *(in_y_ptr);
                intensity_x_y += *(in_x_ptr) * *(in_y_ptr);
                in_y_ptr++;
                in_x_ptr++;
            }
        }

        
		float score = ((intensity_x_2 * intensity_y_2) - (intensity_x_y * intensity_x_y)) - 0.04 * (intensity_x_2 + intensity_y_2) * (intensity_x_2 + intensity_y_2);

        float theta = atan2(m_01, m_10) * 180 / PI;
		key_points[i].theta = theta;
        harris_score.push_back(std::make_pair(score, i));
	}

	std::sort(harris_score.rbegin(), harris_score.rend());
    int n = key_points.size();
    numKeyPoints = std::min(n, numKeyPoints);
	for (int i = 0; i < numKeyPoints; i++)
	{
		good_features.push_back(key_points[harris_score[i].second]);
	}

}

#define ROUND(value) value >=0 ? (int32_t)(value + 0.5):(int32_t)(value - 0.5)

void computerOrbDescriptor(const Mat2<byte> &img, std::vector<KeyPoints> & corners, std::vector<Mat2<int32_t>> &descriptors){
    for(KeyPoints corner:corners){
        float cos_angle = std::cos(corner.theta);
        float sin_angle = std::sin(corner.theta);

        byte *image_center = img.data + corner.coord.y*img.stride + corner.coord.x;

        alignas(16) int32_t ia_x[256];
        alignas(16) int32_t ia_y[256];
        alignas(16) int32_t ib_x[256];
        alignas(16) int32_t ib_y[256];

        for (int i = 0 ; i< 256; i++)  {
            ia_x[i] = ROUND((gaussian_bit_pattern_31_x_a[i]*cos_angle - gaussian_bit_pattern_31_y_a[i]*sin_angle));
            ia_y[i] = ROUND((gaussian_bit_pattern_31_x_a[i]*sin_angle + gaussian_bit_pattern_31_y_a[i]*cos_angle));
            ib_x[i] = ROUND((gaussian_bit_pattern_31_x_b[i]*cos_angle - gaussian_bit_pattern_31_y_b[i]*sin_angle));
            ib_y[i] = ROUND((gaussian_bit_pattern_31_x_b[i]*sin_angle + gaussian_bit_pattern_31_y_b[i]*cos_angle));
        }

        Mat2<int32_t> descriptor(8, 1);

        descriptor.data[0] = 0;
        descriptor.data[1] = 0;
        descriptor.data[2] = 0;
        descriptor.data[3] = 0;
        descriptor.data[4] = 0;
        descriptor.data[5] = 0;
        descriptor.data[6] = 0;
        descriptor.data[7] = 0;


        int32_t *ia_x_ptr = ia_x, *ia_y_ptr = ia_y, *ib_x_ptr = ib_x, *ib_y_ptr = ib_y;
        for(int i=0; i<8; i++){

            for(int j=0; j<32; j++){
                descriptor.data[i] |= ((*(image_center + *ia_y_ptr*img.stride + *ia_x_ptr) < *(image_center + *ib_y_ptr*img.stride + *ib_x_ptr)) << j);
                ia_x_ptr++;
                ia_y_ptr++;
                ib_x_ptr++;
                ib_y_ptr++;
            }
        }        


        // for(int i=0; i<8; i++){
        //     descriptor.data[i] = f[i];
        // }

        descriptors.push_back(descriptor);
	       
    }

}

void computerOrbDescriptor(const Mat2<byte> &img, std::vector<KeyPoints> & corners, std::vector<Mat2<byte>> &descriptors){
    for(KeyPoints corner:corners){
        float cos_angle = std::cos(corner.theta);
        float sin_angle = std::sin(corner.theta);

        byte *image_center = img.data + corner.coord.y*img.stride + corner.coord.x;

        alignas(16) int32_t ia_x[256];
        alignas(16) int32_t ia_y[256];
        alignas(16) int32_t ib_x[256];
        alignas(16) int32_t ib_y[256];

        for (int i = 0 ; i< 256; i++)  {
            ia_x[i] = ROUND((gaussian_bit_pattern_31_x_a[i]*cos_angle - gaussian_bit_pattern_31_y_a[i]*sin_angle));
            ia_y[i] = ROUND((gaussian_bit_pattern_31_x_a[i]*sin_angle + gaussian_bit_pattern_31_y_a[i]*cos_angle));
            ib_x[i] = ROUND((gaussian_bit_pattern_31_x_b[i]*cos_angle - gaussian_bit_pattern_31_y_b[i]*sin_angle));
            ib_y[i] = ROUND((gaussian_bit_pattern_31_x_b[i]*sin_angle + gaussian_bit_pattern_31_y_b[i]*cos_angle));
        }

        #define GET_BYTE_VALUE(i, j) (*(image_center + ia_y[i*8 + j]*img.stride + ia_x[i*8 + j]) < *(image_center + ib_y[i*8 + j]*img.stride + ib_x[i*8 + j])) << j
	
        alignas(32) byte f[32] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0} ;    

        for(int i=0; i<32; i++){
            f[i] |= GET_BYTE_VALUE(i, 0);
            f[i] |= GET_BYTE_VALUE(i, 1);
            f[i] |= GET_BYTE_VALUE(i, 2);
            f[i] |= GET_BYTE_VALUE(i, 3);
            f[i] |= GET_BYTE_VALUE(i, 4);
            f[i] |= GET_BYTE_VALUE(i, 5);
            f[i] |= GET_BYTE_VALUE(i, 6);
            f[i] |= GET_BYTE_VALUE(i, 7);
        }    

        Mat2<byte> descriptor(32, 1);

        for(int i=0; i<32; i++){
            descriptor.data[i] = f[i];
        }

        descriptors.push_back(descriptor);
	       
    }

}

void remove_edge_keypoints(const Mat2<byte> &img, std::vector<KeyPoints> &corners, int edge_width){
    for(int i=0; i<corners.size(); i++){
        xy corner = corners[i].coord;
        if (corner.x <= edge_width || corner.y < edge_width || corner.x > img.xsize - edge_width || corner.y > img.ysize - edge_width){
            corners.erase(corners.begin() + i);
            i--;
        }
    }
}

void calculateSummedAreaTable(Mat2<byte> &img, Mat2<int> &summedAreaTable){
    summedAreaTable.data[0] = img.data[0];
    for(int i=1; i < img.xsize; i++){
        summedAreaTable.data[i] = img.data[i] + summedAreaTable.data[i - 1];
    }

    for(int i=1; i < img.ysize; i++){
        summedAreaTable.data[i*img.stride] = img.data[i*img.stride] + summedAreaTable.data[(i-1)*img.stride];
    }

    for(int i=1; i < img.ysize; i++){
        for(int j=1; j < img.xsize; j++){
            summedAreaTable.data[i*img.stride + j] = img.data[i*img.stride + j] + 
                                        summedAreaTable.data[i*img.stride + j - 1] + 
                                        summedAreaTable.data[(i-1)*img.stride + j] - 
                                        summedAreaTable.data[(i - 1)*img.stride + j - 1];
        }
    }
}

void gaussianBlur(Mat2<byte> &img, Mat2<byte> &gaussian){
    // int kernel_1[3] = {1, 0, -1}, kernel_2[3] = {1, 2, 1};
    Mat2<int> row_gradients(img.xsize, img.ysize);

    byte *center = img.data, *center_end;
    int *row_center = row_gradients.data + 2, *row_center_end;
    for (int i = 0; i < img.ysize; i++)
	{
        center_end = center + img.stride - 4;
		while(center < center_end)
		{
            *(row_center) = *center + 4 * *(center + 1) + 6* *(center + 2) + 4* *(center + 3) + *(center + 4);
            center++;
            row_center++;
        }
        center += 4;
        row_center += 4;
    }
    row_center = row_gradients.data + 2;
    center = gaussian.data + 2*img.stride + 2;
    for (int i = 0; i < img.ysize - 4; i++)
	{
        row_center_end = row_center + img.stride - 2;
		while(row_center < row_center_end)
		{
            *(center) = (*(row_center) + 
                        4 * *(row_center + img.stride) + 
                        6 * *(row_center + 2*img.stride) + 
                        4 * *(row_center + 3*img.stride) + 
                        *(row_center + 4*img.stride))/256;
            center++;
            row_center++;
		}
        center += 4;
        row_center += 4;
	}
}

void orb_detect(Mat2<byte> &img, std::vector<KeyPoints> &good_corners, int fastThreshold, int numKeyPoints, int edge_width)
{   
    std::clock_t start;

    //FAST 9 features
    start = std::clock();
#ifdef NONMAX_CUSTOM
    std::vector<KeyPoints> corners = fast9_detect_corners(img, fastThreshold, 2*numKeyPoints);
#else
	std::vector<KeyPoints> corners = fast9_detect_nonmax(img, fastThreshold);
#endif
    std::cout << "Time fast: " << (std::clock() - start) / (double)(CLOCKS_PER_SEC / 1000) << " ms" << std::endl;

    remove_edge_keypoints(img, corners, edge_width);

    calculateBestCornersAndOrientation(img, corners, good_corners, numKeyPoints);
}

void orb_compute(Mat2<byte> &img, std::vector<KeyPoints> &good_corners, std::vector<Mat2<byte>> &decriptors)
{   
    std::clock_t start;
#ifdef GAUSSIAN_BLUR
    Mat2<byte> gaussian(img.xsize, img.ysize);

    start = std::clock();
    gaussianBlur(img, gaussian);
    std::cout << "Time GAUSSIAN: " << (std::clock() - start) / (double)(CLOCKS_PER_SEC / 1000) << " ms" << std::endl;

    computerOrbDescriptor(gaussian, good_corners, decriptors);
#else
    computerOrbDescriptor(img, good_corners, decriptors);
#endif

}

void orb_compute(Mat2<byte> &img, std::vector<KeyPoints> &good_corners, std::vector<Mat2<int32_t>> &decriptors)
{   
    std::clock_t start;
#ifdef GAUSSIAN_BLUR
    Mat2<byte> gaussian(img.xsize, img.ysize);

    start = std::clock();
    gaussianBlur(img, gaussian);
    std::cout << "Time GAUSSIAN: " << (std::clock() - start) / (double)(CLOCKS_PER_SEC / 1000) << " ms" << std::endl;

    start = std::clock();
    computerOrbDescriptor(gaussian, good_corners, decriptors);
    std::cout << "Time Descriptor Calculation: " << (std::clock() - start) / (double)(CLOCKS_PER_SEC / 1000) << " ms" << std::endl;
#else
    computerOrbDescriptor(img, good_corners, decriptors);
#endif

}

std::vector<KeyPoints> fast9_detect_corners(Mat2<byte> &img, int b, int numKeyPoints)
{
	byte *im = img.data;
	int xsize = img.xsize;
	int ysize = img.ysize;
	int stride = img.stride;
	xy *corners;
	int num_corners;
	// int *scores;
	// xy *nonmax;

	corners = fast9_detect(im, xsize, ysize, stride, b, &num_corners);
    
    std::cout << "No of FAST 9 Features " << num_corners << std::endl;
    std::vector<KeyPoints> corners_nomax;

   
    for (int i = 0; i < num_corners; i++)
    {
        if(corner_max(im, corners[i], stride, 3))
            corners_nomax.push_back(corners[i]);
    }
	free(corners);

    std::cout << "No of FAST 9 Features after non max " << corners_nomax.size() << std::endl;

	// free(nonmax);
	return corners_nomax;
}

bool corner_max(byte *im, xy corner, int stride, int kernelSize){
    byte *p = im + corner.y*stride + corner.x;
    float thresh = 1.5;
    if(kernelSize == 3){
        byte *p0 = p - stride, *p1 = p + stride;
        if(*(p0 - 1) > thresh * *p) return false;
        if(*(p0) > thresh * *p) return false;
        if(*(p0 + 1) > thresh * *p) return false;
        if(*(p - 1) > thresh * *p) return false;
        if(*(p + 1) > thresh * *p) return false;
        if(*(p1 - 1) > thresh * *p) return false;
        if(*(p1) > thresh * *p) return false;
        if(*(p1 + 1) > thresh * *p) return false;
    }else if(kernelSize == 5){
        byte *p0 = p - 2*stride, *p1 = p - stride, *p2 = p + stride, *p3 = p + 2*stride;
        if(*(p0 - 2) > *p) return false;
        if(*(p0 - 1) > *p) return false;
        if(*(p0) > *p) return false;
        if(*(p0 + 1) > *p) return false;
        if(*(p0 + 2) > *p) return false;

        if(*(p1 - 2) > *p) return false;
        if(*(p1 - 1) > *p) return false;
        if(*(p1) > *p) return false;
        if(*(p1 + 1) > *p) return false;
        if(*(p1 + 2) > *p) return false;

        if(*(p - 2) > *p) return false;
        if(*(p - 1) > *p) return false;
        if(*(p + 1) > *p) return false;
        if(*(p + 2) > *p) return false;

        if(*(p2 - 2) > *p) return false;
        if(*(p2 - 1) > *p) return false;
        if(*(p2) > *p) return false;
        if(*(p2 + 1) > *p) return false;
        if(*(p2 + 2) > *p) return false;

        if(*(p3 - 2) > *p) return false;
        if(*(p3 - 1) > *p) return false;
        if(*(p3) > *p) return false;
        if(*(p3 + 1) > *p) return false;
        if(*(p3 + 2) > *p) return false;
    }

    return true;
}


std::vector<KeyPoints> fast9_detect_nonmax(Mat2<byte> &img, int b)
{
	byte *im = img.data;
	int xsize = img.xsize;
	int ysize = img.ysize;
	int stride = img.stride;
	xy *corners;
	int num_corners;
	int *scores;
	xy *nonmax;

	corners = fast9_detect(im, xsize, ysize, stride, b, &num_corners);
	scores = fast9_score(im, stride, corners, num_corners, b);
	nonmax = nonmax_suppression(corners, scores, num_corners, &num_corners);

	free(corners);
	free(scores);

	std::vector<KeyPoints> corners_nomax;
	for (int i = 0; i < num_corners; i++)
	{
		corners_nomax.push_back(nonmax[i]);
	}

	free(nonmax);
	return corners_nomax;
}

xy *nonmax_suppression(const xy *corners, const int *scores, int num_corners, int *ret_num_nonmax)
{
	int num_nonmax = 0;
	int last_row;
	int *row_start;
	int i, j;
	xy *ret_nonmax;
	const int sz = (int)num_corners;

	/*Point above points (roughly) to the pixel above the one of interest, if there
    is a feature there.*/
	int point_above = 0;
	int point_below = 0;

	if (num_corners < 1)
	{
		*ret_num_nonmax = 0;
		return 0;
	}

	ret_nonmax = (xy *)malloc(num_corners * sizeof(xy));

	/* Find where each row begins
	   (the corners are output in raster scan order). A beginning of -1 signifies
	   that there are no corners on that row. */
	last_row = corners[num_corners - 1].y;
	row_start = (int *)malloc((last_row + 1) * sizeof(int));

	for (i = 0; i < last_row + 1; i++)
		row_start[i] = -1;

	{
		int prev_row = -1;
		for (i = 0; i < num_corners; i++)
			if (corners[i].y != prev_row)
			{
				row_start[corners[i].y] = i;
				prev_row = corners[i].y;
			}
	}

	for (i = 0; i < sz; i++)
	{
		int score = scores[i];
		xy pos = corners[i];

		/*Check left */
		if (i > 0)
			if (corners[i - 1].x == pos.x - 1 && corners[i - 1].y == pos.y && Compare(scores[i - 1], score))
				continue;

		/*Check right*/
		if (i < (sz - 1))
			if (corners[i + 1].x == pos.x + 1 && corners[i + 1].y == pos.y && Compare(scores[i + 1], score))
				continue;

		/*Check above (if there is a valid row above)*/
		if (pos.y != 0 && row_start[pos.y - 1] != -1)
		{
			/*Make sure that current point_above is one
			  row above.*/
			if (corners[point_above].y < pos.y - 1)
				point_above = row_start[pos.y - 1];

			/*Make point_above point to the first of the pixels above the current point,
			  if it exists.*/
			for (; corners[point_above].y < pos.y && corners[point_above].x < pos.x - 1; point_above++)
			{
			}

			for (j = point_above; corners[j].y < pos.y && corners[j].x <= pos.x + 1; j++)
			{
				int x = corners[j].x;
				if ((x == pos.x - 1 || x == pos.x || x == pos.x + 1) && Compare(scores[j], score))
					goto cont;
			}
		}

		/*Check below (if there is anything below)*/
		if (pos.y != last_row && row_start[pos.y + 1] != -1 && point_below < sz) /*Nothing below*/
		{
			if (corners[point_below].y < pos.y + 1)
				point_below = row_start[pos.y + 1];

			/* Make point below point to one of the pixels belowthe current point, if it
			   exists.*/
			for (; point_below < sz && corners[point_below].y == pos.y + 1 && corners[point_below].x < pos.x - 1; point_below++)
			{
			}

			for (j = point_below; j < sz && corners[j].y == pos.y + 1 && corners[j].x <= pos.x + 1; j++)
			{
				int x = corners[j].x;
				if ((x == pos.x - 1 || x == pos.x || x == pos.x + 1) && Compare(scores[j], score))
					goto cont;
			}
		}

		ret_nonmax[num_nonmax++] = corners[i];
	cont:;
	}

	free(row_start);
	*ret_num_nonmax = num_nonmax;
	return ret_nonmax;
}


void matchFeatures(std::vector<Mat2<int32_t>> &descriptors_template, std::vector<Mat2<int32_t>> &descriptors_scene, std::vector<std::vector<Match>> &matches){
    int template_size = descriptors_template.size();
    int scene_size = descriptors_scene.size();

    for(int i=0; i < template_size; i++){
        int32_t *template_despriptor = descriptors_template[i].data;
        int match_index_1 = -1, match_index_2 = -1; float distance_1 = 512, distance_2 = 512; 
        for(int j=0; j < scene_size; j++){
            int32_t *scene_despriptor = descriptors_scene[j].data;
            float score = 0;
            for(int k=0; k <8; k++){
                score += __builtin_popcount(template_despriptor[k]^scene_despriptor[k]);
            }

            if(score < distance_1){
                distance_2 = distance_1;
                match_index_2 = match_index_1;
                distance_1 = score;
                match_index_1 = j;
            }else if(score < distance_2){
                distance_2 = score;
                match_index_2 = j;
            }
        }

        Match match_1(i, match_index_1, distance_1);
        Match match_2(i, match_index_2, distance_2);
        std::vector<Match> nnMatch;
        nnMatch.push_back(match_1);
        nnMatch.push_back(match_2);
        matches.push_back(nnMatch);
    }
}

std::pair<int, int> getRandonPair(int size){
    int i = rand()%size;
    int j = rand()%(size-1);

    if(j >= i)
        j++;
    
    return std::make_pair(i, j);
}

void findHomography(std::vector<Point<int>> &templ, std::vector<Point<int>> &scene, std::vector<float> &a){
    int max = -1, size = templ.size();
    if(a.size() < 4){
        for(int i = a.size(); i<4; i++)
            a.push_back(0);
    }
    float a1=0, a2=0, a3=0, a4=0;
    for(int i=0; i<500; i++){
        std::pair<int, int> randPair = getRandonPair(templ.size() - 1);
        Point<int> p1 = templ[randPair.first];
        Point<int> p2 = templ[randPair.second];
        Point<int> p1_d = scene[randPair.first];
        Point<int> p2_d = scene[randPair.second];
        int x_diff = p1.x - p2.x;
        int y_diff = p1.y - p2.y;
        int x_d_diff = p1_d.x - p2_d.x;
        int y_d_diff = p1_d.y - p2_d.y;
        int den = x_diff*x_diff + y_diff*y_diff;
        a1 = (float)(x_diff*x_d_diff + y_diff*y_d_diff)/den;
        a2 = (float)(x_diff*y_d_diff + y_diff*x_d_diff)/den;
        a3 = p1_d.x - a1*p1.x + a2*p1.y;
        a4 = p1_d.y - a2*p1.x - a1*p1.y;
        int count = 0;
        for(int j=0; j<size; j++){
            Point<int> templ_point = templ[j];
            Point<float> trans;
            trans.x = a1*templ_point.x - a2*templ_point.y + a3;
            trans.y = a2*templ_point.x + a1*templ_point.y + a4;
            float d = std::pow(trans.x - scene[j].x,2) + std::pow(trans.y - scene[j].y, 2);
            if(d < 9)
                count++;
        }

        if(count > max){
            max = count;
            a[0] = a1;
            a[1] = a2;
            a[2] = a3;
            a[3] = a4;
        }
    }

}

void perspectiveTransform(const std::vector<Point<int>> &src, std::vector<Point<int>> &transform, const std::vector<float> &a){
    for(Point<int> p:src){
        Point<int> trans;
        trans.x = a[0]*p.x - a[1]*p.y + a[2];
        trans.y = a[1]*p.x + a[0]*p.y + a[3];
        transform.push_back(trans);
    }
}

void findParams(std::vector<::ORB::Point<int>> &obj, std::vector<::ORB::Point<int>> &scene, std::vector<float> &a)
{
    if(a.size() < 4){
        for(int i = a.size(); i<4; i++)
            a.push_back(0);
    }
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

    a[0] = a1;
    a[1] = a2;
    a[2] = a3;
    a[3] = a4;
}

void calculateBestMatches(std::vector<KeyPoints> &corners, std::vector<KeyPoints> &corners_scene, std::vector<Match> &good_matches, std::vector<Match> &best_matches){
    std::vector<::ORB::Point<int>> obj, transfor_obj;
    std::vector<::ORB::Point<int>> scene;

    for (int i = 0; i < good_matches.size(); i++)
    {
        //-- Get the keypoints from the good matches
        obj.push_back(::ORB::Point<int>(corners[good_matches[i].template_coor].coord.x, corners[good_matches[i].template_coor].coord.y));
        scene.push_back(::ORB::Point<int>(corners_scene[good_matches[i].scene_coor].coord.x, corners_scene[good_matches[i].scene_coor].coord.y));
    }

    std::vector<float> a;
    
    std::clock_t start;
    start = std::clock();
    findHomography(obj, scene, a);
    std::cout << "Time RANSAC: " << (std::clock() - start) / (double)(CLOCKS_PER_SEC / 1000) << " ms" << std::endl;

    // std::cout << H << std::endl;

    perspectiveTransform(obj, transfor_obj, a);

    for (int i = 0; i < obj.size(); i++)
    {
        float d = pow(scene[i].x - transfor_obj[i].x, 2) + pow(scene[i].y - transfor_obj[i].y, 2);
        if (d < 9)
        {
            best_matches.push_back(good_matches[i]);
        }
    }
}

} //namespace ORB