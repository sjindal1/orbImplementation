#include <stdlib.h>
#include "orb.hpp"

#define Compare(X, Y) ((X) >= (Y))

std::vector<KeyPoints> calculateHarrisAndKeepGood(Image &img, std::vector<KeyPoints> &corners, std::vector<std::vector<std::pair<float, float>>> &gradients, int numKeyPoints, int edge_width)
{

	std::vector<std::pair<float, int>> harris_score;
	int n = corners.size();
	for (int i = 0; i < n; i++)
	{
		xy corner = corners[i].coord;
		// Ignore the corner if it lies close to the edge
		if (corner.x <= edge_width || corner.y < edge_width || corner.x > img.xsize - edge_width || corner.y > img.ysize - edge_width)
			continue;

		float intensiy_x_2 = 0, intensiy_y_2 = 0, intensiy_x_y = 0;
		int r = HARRIS_BLOCK_SIZE / 2;
		for (int i = -r; i <= r; i++)
		{
			for (int j = -r; j <= r; j++)
			{
				// using sobel
				float intensiy_x = gradients[corner.y + i][corner.x + j].first;
				float intensiy_y = gradients[corner.y + i][corner.x + j].second;
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
	for (int i = 0; i < numKeyPoints; i++)
	{
		good_corners.push_back(corners[harris_score[i].second]);
	}

	return good_corners;
}

void calculateSobel(Image &img, std::vector<std::vector<std::pair<float, float>>> &gradients)
{
    // int kernel_1[3] = {1, 0, -1}, kernel_2[3] = {1, 2, 1};
    std::vector<std::vector<std::pair<float, float>>> row_gradients(img.ysize, std::vector<std::pair<float, float>>(img.xsize));
    for (int i = 0; i < img.ysize; i++)
	{
		for (int j = 1; j < img.xsize - 1; j++)
		{
            row_gradients[i][j].first = img.data[i][j-1] - img.data[i][j+1];
            row_gradients[i][j].second = img.data[i][j-1] + img.data[i][j]*2 + img.data[i][j+1];
        }
    }
    for (int i = 1; i < img.xsize - 1; i++)
	{
		for (int j = 1; j < img.ysize - 1; j++)
		{
            gradients[j][i].first = row_gradients[j-1][i].first + 2*row_gradients[j][i].first + row_gradients[j+1][i].first;
            gradients[j][i].second = row_gradients[j-1][i].second - row_gradients[j+1][i].second;
		}
	}
}

void calculateOrientationOfCorners(Image &img, std::vector<KeyPoints> &key_points, std::vector<std::vector<std::pair<float, float>>> &gradients)
{
	for (int i = 0; i < key_points.size(); i++)
	{
		xy corner = key_points[i].coord;

		int path_sizes[] = {corner.x - 1, img.xsize - corner.x - 1, corner.y - 1, img.ysize - corner.y - 1, PATCH_SIZE / 2};
		int r = *std::min_element(path_sizes, path_sizes + 5);
		float m_01 = 0, m_10 = 0;
		for (int i = -r; i <= r; i++)
		{
			for (int j = -r; j <= r; j++)
			{
				int x = corner.x + j, y = corner.y + i;
				m_01 += y * gradients[y][x].second;
				m_10 += x * gradients[y][x].first;
			}
		}
		float theta = atan2(m_01, m_10) * 180 / PI;
		key_points[i].theta = theta;
	}
}

std::vector<KeyPoints> orb_detect_compute(Image &img, int fastThreshold, int numKeyPoints, int edge_width)
{

	std::vector<KeyPoints> corners = fast9_detect_nonmax(img, fastThreshold);

    std::vector<std::vector<std::pair<float, float>>> gradients(img.ysize, std::vector<std::pair<float, float>>(img.xsize));
	calculateSobel(img, gradients);

	std::vector<KeyPoints> good_corners;

	if (corners.size() > numKeyPoints)
	{
		good_corners = calculateHarrisAndKeepGood(img, corners, gradients, numKeyPoints, edge_width);
	}
	else
	{
		good_corners = corners;
	}

	calculateOrientationOfCorners(img, good_corners, gradients);

	return good_corners;
}

std::vector<KeyPoints> fast9_detect_nonmax(Image &img, int b)
{
	byte *im = new byte[img.xsize * img.ysize];
	int k = 0;
	for (int i = 0; i < img.ysize; i++)
	{
		for (int j = 0; j < img.xsize; j++)
		{
			im[k++] = img.data[i][j];
		}
	}
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