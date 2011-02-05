/*
 * landmark_projector.h
 *
 *  Created on: Feb 4, 2011
 *      Author: gerow
 */

#ifndef LANDMARK_PROJECTOR_H_
#define LANDMARK_PROJECTOR_H_

#include <landmark_map/Landmark.h>
#include <color_segmenter/FindBlobs.h>

class LandmarkProjector
{
public:
	static Landmark projectBuoy( color_segmenter::ColorBlob &b, double dist_measured_from = 0.3048, double width_at_dist_measured = 374, double buoy_actual_width = 0.19404187 );
	static void projectBin( Landmark &l );
	static void projectPipe( Landmark &l );
};

#endif /* LANDMARK_PROJECTOR_H_ */
