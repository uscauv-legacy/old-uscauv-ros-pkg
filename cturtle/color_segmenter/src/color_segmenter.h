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

void makeImageBinary(cv::Mat &, int);
std::vector<color_segmenter::ColorBlob> blobFindingMethod(cv::Mat &, int);
bool getBinPixelValue(cv::Mat &, int, int);
color_segmenter::ColorBlob findBlob(cv::Mat &, std::vector<std::vector<bool> >, int, int);
void drawSegments(cv::Mat &, std::vector<color_segmenter::ColorBlob> &);
std::string weightString(int, double);
std::vector<color_segmenter::ColorBlob> floodFillMethod(cv::Mat, int);

#endif /* COLOR_SEGMENTER_H_ */
