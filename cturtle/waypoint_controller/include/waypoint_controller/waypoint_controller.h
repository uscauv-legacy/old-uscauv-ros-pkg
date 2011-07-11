/*******************************************************************************
 *
 *      waypoint_controller
 * 
 *      Copyright (c) 2011, Edward T. Kaszubski ( ekaszubski@gmail.com )
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
 *      * Neither the name of "waypoint_controller-RelWithDebInfo@waypoint_controller" nor the names of its
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

#ifndef WAYPOINT_CONTROLLER_H_
#define WAYPOINT_CONTROLLER_H_

#include <base_node/base_node.h>
#include <common_utils/tf.h>
#include <waypoint_controller/SetBehavior.h>

typedef BaseNode<> _BaseNode;
typedef waypoint_controller::SetBehavior _SetBehavior;
geometry_msgs::vector3 operator+( geometry_msgs::vector3& a, geometry_msgs::vector3& a )
{
	geometry_msgs::vector3 c;
	c.x = a.x + b.x;
	c.y = a.y + b.y;
	c.z = a.z + b.z;
	return c;
}

geometry_msgs::vector3 operator*( double a, geometry_msgs::vector3& b )
{
	geometry_msgs::vector3 c;
	c.x = a * b.x;
	c.y = a * b.y;
	c.z = a * b.z;
	return c;
}
class WaypointController: public _BaseNode
{
public:
	ros::ServiceServer set_behavior_svr_;

	WaypointController( ros::NodeHandle & nh ) :
		_BaseNode( nh )
	{
		set_behavior_svr_ = nh_local_.advertiseService( "set_behavior", &WaypointController::setBehaviorCB, this );
	}
	int val[];

	geometry_msgs::vector3 vec[]; //0 = position, 1 = orientation, 2 = area

	bool setBehaviorCB( _SetBehavior::Request & req, _SetBehavior::Response & resp )
	{
		for ( i = 0; i < req.flags.size(); i++ ) //iterates through all flags in req
		{
			if ( req.flags[i].values.empty() ) val[0] = 0; //if value isn't set, default to 0
			else val = req.flags[i].values;

			switch ( req.flags[i].behavior )
			{
			case 0: //position behavior
				vec[0] = req.flags[i].vector + val[0] * currentPosition; //if val[0]=0 then it sets absolute coordinates, if val[0]=1 then it is relative coordinates
				if ( req.flags[i].operation == 0 ) moveTo( vec[0] ); //moves to last saved position
				else if ( req.flags[i].operation == 2 ) clear( vec[0], 0, val[0] );
			}
			return true;

			case 1: //orientation behavior
			if ( val[0] )
			{ //set based on angle
				vec[1] = req.flags[i].vector + val[1] * currentAngle; //if val[0]=0 then it sets absolute orientation, if val[0]=1 then it is relative orientation
				if ( req.flags[i].operation == 0 ) turnTo( vec[1] ); //turns to last saved orienation
				else if ( req.flags[i].operation == 2 ) clear( vec[1], 1, val[1] );
			}
			else
			{ // turn based on last position
				turnTo( angleBetween( vec[0], currentPosition ) );
			}
			return true;

			case 2: //area behavior
			vec[2] = req.flags[i].vector;
			if ( req.flags[i].operation == 1 )
			{
				if ( val[0] ) setArea( vec[2], vec[0] ); //centers at last position
				else setArea( vec[2], currentPosition ); //centers at current position
			}
			else if ( req.flags[i].operation == 2 ) clear( vec[2], 2, val[0] );
			return true;

			default:
			return false; //returns false only if an invalid behavior argument was passed in the message
		}
	}
};
void clear( geometry_msgs::vector3 & vector, const int & behavior, const int & value )
{
	switch ( behavior )
	{
	case 0:
		if ( value ) vector = currentPosition;
		else vector = 0 * vector; //sets vector to 0,0,0
		break;
	case 1:
		if ( value ) vector = currentAngle;
		else vector = 0 * vector; //sets vector to 0,0,0
		break;
	case 2:
		vector = 0 * vector; //sets vector to 0,0,0
		break;
	}
}

#endif /* WAYPOINT_CONTROLLER_H_ */
