/*
 * color_segmenter.h
 *
 *  Created on: Nov 19, 2010
 *      Author: gerow
 */

#ifndef COLOR_SEGMENTER_H_
#define COLOR_SEGMENTER_H_
#include <base_image_proc/base_image_proc.h>
#include <color_defs/colors.h>
#include <color_segmenter/FindBlobs.h>
#include "color_segmenter.h"
#include <queue>
#include <vector>

//void makeImageBinary(cv::Mat &, int);
std::vector<color_segmenter::ColorBlob> blobFindingMethod(cv::Mat &, int, cv::Vec3b &, bool);
std::vector<color_segmenter::ColorBlob> blobFindingMethod(cv::Mat &, int, cv::Vec3b &);
bool getBinPixelValue(cv::Mat &, cv::Vec3b &, int, int);
void drawSegments(cv::Mat &, std::vector<color_segmenter::ColorBlob> &);
std::string weightString(int, double);
std::vector<color_segmenter::ColorBlob> floodFillMethod(cv::Mat, int);
cv::Vec3b getColorVector(int);
color_segmenter::ColorBlob findBlob(cv::Mat & cv_img_, std::vector<std::vector<bool> > & table, int initx, int inity, bool include_diagonals, cv::Vec3b & color);

#endif /* COLOR_SEGMENTER_H_ */
