/*******************************************************************************
 *
 *      landmark_projector
 *
 *      Copyright (c) 2010
 *
 *      Mike Gerow  (gerow.mike@gmail.com)
 *
 *      All rights reserved.
 *
 *      Redistribution and use in source and binary forms, with or without
 *      modification, are permitted provided that the following conditions are
 *      met:
 *
 *      * Redistributions of source code must retain the above copyright
 *        notice, this list of conditions and the following disclaimer.
 *      * Redistributions in binary form must reproduce the above
 *        copyright notice, this list of conditions and the following disclaimer
 *        in the documentation and/or other materials provided with the
 *        distribution.
 *      * Neither the name of "seabee3-ros-pkg" nor the names of its
 *        contributors may be used to endorse or promote products derived from
 *        this software without specific prior written permission.
 *
 *      THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *      "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *      LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 *      A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 *      OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 *      SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 *      LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 *      DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 *      THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 *      (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 *      OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 *******************************************************************************/
#include <math.h>

#include <landmark_map/Landmark.h>
#include <color_segmenter/FindBlobs.h>

#include <landmark_projector/landmark_projector.h>

Landmark LandmarkProjector::projectBuoy( color_segmenter::ColorBlob &b, double dist_measured_from, double width_at_dist_measured, double buoy_actual_width )
{
	//REMEMBER BOUY SIZE!
	double radius = sqrt( b.mass / M_PI );
	double dist = ( width_at_dist_measured / ( radius * 2 ) ) * dist_measured_from;
	//If this doesn't work, it's totally not my fault.
	double scalar = ( width_at_dist_measured / ( radius * 2 ) ) * buoy_actual_width;

	double x = dist;
	double y = -b.x * scalar;
	double z = b.y * scalar;

	Landmark l( cv::Point3d( x, y, z ) );
	l.shape_type_ = Landmark::LandmarkType::Buoy;

	return l;
}

void LandmarkProjector::projectBin( Landmark &l )
{

}

void LandmarkProjector::projectPipe( Landmark &l )
{

}
