/*
 * landmark_projector.h
 *
 *  Created on: Feb 4, 2011
 *      Author: gerow
 */

#ifndef LANDMARK_PROJECTOR_H_
#define LANDMARK_PROJECTOR_H_

#include <landmark_map/Landmark.h>

Landmark projectLandmark(Landmark l);
void projectBouy(Landmark &l);
void projectBin(Landmark &l);
void projectPipe(Landmark &l);

#endif /* LANDMARK_PROJECTOR_H_ */
