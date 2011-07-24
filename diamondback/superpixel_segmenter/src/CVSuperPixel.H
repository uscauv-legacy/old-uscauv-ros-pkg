//! Segment an image using Super Pixel Segmentation.
/*!
 * Returns an integer image where the value at each pixel denotes the group it
 * belongs to.  Additionally, if a groups vector is supplied, it will be filled
 * with vectors of points belonging to each group.
 *
 * \param im image to segment.
 * \param sigma to smooth the image.
 * \param c constant for treshold function.
 * \param min_size minimum component size (enforced by post-processing stage).
 */
#ifndef CVSUPERPIXEL_H
#define CVSUPERPIXEL_H

#include <cstddef>
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <vector>

std::vector<std::vector<cv::Point> >
SuperPixelSegment(cv::Mat im, int sigma, float c, int min_size);

cv::Mat SuperPixelDebugImage(
    std::vector<std::vector<cv::Point > > const& groups, cv::Mat originalImage);

#endif // CVSUPERPIXEL_H
