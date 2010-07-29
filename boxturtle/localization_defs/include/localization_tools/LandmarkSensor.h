/*******************************************************************************
 *
 *      LandmarkSensor
 * 
 *      Copyright (c) 2010, Edward T. Kaszubski (ekaszubski@gmail.com)
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
 *      * Neither the name of the USC Underwater Robotics Team nor the names of its
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

#include <vector>
#include <localization_tools/LocalizationParticle.h>
#include <landmark_map/LandmarkMap.h>
//#include <sonar_node/SonarScanArray.h>
#include <localization_defs/LandmarkArrayMsg.h>
#include <localization_tools/Util.h> //includes <math.h>

//used to convert sensor data into particle weights
template <typename _T>
class LandmarkSensor
{
public:

	struct SensorType
	{
		const static int buoy_finder = 		0; //buoy
		const static int sonar_sensor = 	1; //pinger
		const static int pipe_finder = 		2; //pipe
	};

	struct Flags
	{
		const static double voteFailed = -1;
	};

	LandmarkSensor(double scale)
	{
		mScale = scale;
		inited = false;
	}

	~LandmarkSensor()
	{
		//delete mVirtualMsg;
	}

	virtual void recordMessage(const _T& realMsg)
	{
		mRealMsg = realMsg;
		calculateInitialOffset();
		inited = true;
	}
	
	void recordVirtualMessage(const _T& virtualMsg)
	{
		mVirtualMsg = virtualMsg;
	}

	virtual void calculateInitialOffset() = 0;

	virtual double getScaledVote() = 0;

	static _T generatetVirtualSensorMessage ( const LocalizationParticle & part, const LandmarkMap & map );

	double mScale;
	_T mRealMsg;
	_T mVirtualMsg;
	double mVote;
	double mScaledVote; //value calculated from last reading
	int mSensorType;

	bool inited;
};

namespace LandmarkSensorTypes
{

/*	class SonarSensor : public LandmarkSensor<sonar_node::SonarScanArray>
	{
	public:
		SonarSensor(double scale) : 
		LandmarkSensor<sonar_node::SonarScanArray>(scale)
		{
			mSensorType = SensorType::sonar_sensor;
		}
		
		virtual void calculateInitialOffset()
		{
			//
		}
		
		virtual double getScaledVote()
		{
			mVote = 0;
			double error = 0.0;
			for(unsigned int i = 0; i < mRealMsg.ScanArray.size(); i ++)
			{
				error += fabs( mRealMsg.ScanArray[i].Distance - mVirtualMsg.ScanArray[i].Distance ) + fabs( LocalizationUtil::angleDistRel(mRealMsg.ScanArray[i].Heading, mVirtualMsg.ScanArray[i].Heading) );
			}
			
			mVote = 1.0 / (error + 1.0); //simple scaling of sensor vote with respect to error
			
			mScaledVote = mVote * mScale;
			return mScaledVote;
		}
		
		static sonar_node::SonarScanArray generateVirtualSensorMessage ( const LocalizationParticle & part, const LandmarkMap & map )
		{
			sonar_node::SonarScanArray result;
			std::vector<Landmark> landmarks = map.fetchLandmarksByType(Landmark::LandmarkType::Pinger);
			for(unsigned int i = 0; i < landmarks.size(); i ++)
			{
				cv::Point2d theVector = LocalizationUtil::vectorTo(part.mState.mCenter, landmarks[i].mCenter);
				sonar_node::SonarScan theScan;
				theScan.Id = static_cast<LandmarkTypes::Pinger*>( &( landmarks[i] ) )->mId;
				
				theScan.Distance = theVector.x;
				theScan.Heading = theVector.y;
				
				result.ScanArray.push_back( theScan );
			}
			return result;
		}
	};*/
	
	class BuoyFinder : public LandmarkSensor<localization_defs::LandmarkArrayMsg>
	{
	public:
		BuoyFinder(double scale) : 
		LandmarkSensor<localization_defs::LandmarkArrayMsg>(scale)
		{
			mSensorType = SensorType::buoy_finder;
		}
		
		virtual void calculateInitialOffset()
		{
			//
		}
		
		virtual double getScaledVote()
		{
			mVote = 0;
			double error = 0.0;
			
			//loop through the "real" buoys and see how they line up with the "virtual" buoys
			/*
			if(mRealMsg.Color != mVirtualMsg.Color)
				error = 1.0;
			else
				error = fabs( mRealMsg.Center.x - mVirtualMsg.Center.x ) + fabs( mRealMsg.Center.y - mVirtualMsg.Center.y ) + fabs( mRealMsg.Center.z - mVirtualMsg.Center.z ) + fabs( LocalizationUtil::angleDistRel(mRealMsg.Ori, mVirtualMsg.Ori) );
						
			mVote = 1.0 / (error + 1.0); //simple scaling of sensor vote with respect to error
			*/
			mScaledVote = mVote * mScale;
			return mScaledVote;
		}
		
		static localization_defs::LandmarkArrayMsg generateVirtualSensorMessage ( const LocalizationParticle & part, const LandmarkMap & map )
		{
			localization_defs::LandmarkArrayMsg result;
			std::vector<Landmark> landmarks = map.fetchLandmarksByType(Landmark::LandmarkType::Buoy);
			for(unsigned int i = 0; i < landmarks.size(); i ++)
			{
				//cv::Point2d theVector = LocalizationUtil::vectorTo(part.mState.mCenter, landmarks[i].mCenter);
				//localization_defs::LandmarkMsg theLandmark = landmarks[i].createMsg()
				//sonar_node::SonarScan theScan;
				//theScan.Id = static_cast<LandmarkTypes::Buoy*>( &( landmarks[i] ) )->mId;
				
				//theScan.Distance = theVector.x;
				//theScan.Heading = theVector.y;
				
				//result.ScanArray.push_back( theScan );
			}
			return result;
		}
	};
	
	/*class BuoyFinder: public LandmarkSensor
	{
	public:
		BuoyFinder() : 
		LandmarkSensor
		{
			mSensorType = LandmarkSensor::SensorType::detector_buoy;
		}
	
		BuoySensor(double scale)
		{
			mSensorType = LandmarkSensor::SensorType::detector_buoy;
			mScale = scale;
		}
	
		virtual void calculateInitialOffset()
		{
			//
		}
	
		virtual double getScaledVote()
		{
			if (mRealMsg->ice_isA("::RobotSimEvents::PipeColorSegmentMessage"))
			{
				RobotSimEvents::BuoyColorSegmentMessagePtr realMsg = RobotSimEvents::BuoyColorSegmentMessagePtr::dynamicCast(mRealMsg);
				VirtualSensorMessage::BuoyColorSegmentMessage * virtualMsg = static_cast<VirtualSensorMessage::BuoyColorSegmentMessage*>(mVirtualMsg);
				mVote = 1.0f;
				mScaledVote = mVote * mScale;
				return mScaledVote;
			}
			return Flags::voteFailed;
		}
	
		static VirtualSensorMessage::VirtualSensorMessage * generatetVirtualSensorMessage(LocalizationParticle & part, LocalizationMap & map)
		{
			VirtualSensorMessage::BuoyColorSegmentMessage * msg = new VirtualSensorMessage::BuoyColorSegmentMessage;
			return msg;
		}
	};*/
	
	/*class CompassSensor: public LandmarkSensor
	{
	public:
		double mInitialOffset;
		CompassSensor()
		{
			mSensorType = LandmarkSensor::SensorType::compass;
			mInitialOffset = 0;
		}
	
		CompassSensor(double scale)
		{
			mSensorType = LandmarkSensor::SensorType::compass;
			mInitialOffset = 0;
			mScale = scale;
		}
	
		virtual void calculateInitialOffset()
		{
			RobotSimEvents::IMUDataServerMessagePtr realMsg = RobotSimEvents::IMUDataServerMessagePtr::dynamicCast(mRealMsg);
			mInitialOffset = realMsg->orientation[2];
		}
	
		virtual double getScaledVote()
		{
			if (mRealMsg->ice_isA("::RobotSimEvents::IMUDataServerMessage"))
			{
				double realOrientation = 0.0f;
				double virtualOrientation = 0.0f;
				RobotSimEvents::IMUDataServerMessagePtr realMsg = RobotSimEvents::IMUDataServerMessagePtr::dynamicCast(mRealMsg);
				VirtualSensorMessage::IMUDataServerMessage * virtualMsg = static_cast<VirtualSensorMessage::IMUDataServerMessage*>(mVirtualMsg);
				realOrientation = realMsg->orientation[2] - mInitialOffset;
				virtualOrientation = virtualMsg->orientation[2];
				mVote = 1.0f - LocalizationUtil::linearAngleDiffRatio(realOrientation, virtualOrientation);
				mScaledVote = mVote * mScale;
				return mScaledVote;
			}
			return Flags::voteFailed;
		}
	
		static VirtualSensorMessage::VirtualSensorMessage * generatetVirtualSensorMessage(LocalizationParticle & part, LocalizationMap & map)
		{
			VirtualSensorMessage::IMUDataServerMessage * msg = new VirtualSensorMessage::IMUDataServerMessage;
			msg->orientation.resize(3);
			msg->orientation[2] = LocalizationUtil::polarToEuler(part.mState.mAngle); // conventional -> euler
			return msg;
		}
	};*/
	
	/*class PipeSensor: public LandmarkSensor
	{
	public:
		PipeSensor()
		{
			mSensorType = LandmarkSensor::SensorType::detector_pipe;
		}
	
		PipeSensor(double scale)
		{
			mSensorType = LandmarkSensor::SensorType::detector_pipe;
			mScale = scale;
		}
	
		virtual void calculateInitialOffset()
		{
			//
		}
	
		virtual double getScaledVote()
		{
			if (mRealMsg->ice_isA("::RobotSimEvents::PipeColorSegmentMessage"))
			{
				mVote = 0.0f;
				RobotSimEvents::PipeColorSegmentMessagePtr realMsg = RobotSimEvents::PipeColorSegmentMessagePtr::dynamicCast(mRealMsg);
				VirtualSensorMessage::PipeColorSegmentMessage * virtualMsg = static_cast<VirtualSensorMessage::PipeColorSegmentMessage*>(mVirtualMsg);
				if(virtualMsg->pipeIsVisible && realMsg->size > 1500) //make sure we see a decent size orange blob
				{
					mVote += realMsg->size / (2.0f * 8000.0f);
					if(mVote > 0.5f)
						mVote = 0.5f;
				}
				mVote += 0.5f;
				mScaledVote = mVote * mScale;
				return mScaledVote;
			}
			return Flags::voteFailed;
		}
	
		static VirtualSensorMessage::VirtualSensorMessage * generatetVirtualSensorMessage(LocalizationParticle & part, LocalizationMap & map)
		{
			VirtualSensorMessage::PipeColorSegmentMessage * msg = new VirtualSensorMessage::PipeColorSegmentMessage;
			msg->pipeIsVisible = false;
			for(unsigned int i = 0; i < map.mMapEntities.size(); i ++)
			{
				if(map.mMapEntities[i].mObjectType == LocalizationMapEntity::ObjectType::pipe)
				{
					double distToPipe = sqrt(pow(part.mState.mPoint.i - map.mMapEntities[i].mCenter.i, 2) + pow(part.mState.mPoint.j - map.mMapEntities[i].mCenter.j, 2));
					double largerLength = map.mMapEntities[i].mDim.i;
					if(map.mMapEntities[i].mDim.j > largerLength)
					{
						largerLength = map.mMapEntities[i].mDim.j;
					}
					if(distToPipe <= largerLength / 2)
					{
						msg->pipeIsVisible = true;
					}
				}
			}
			return msg;
		}
	};*/
	
	/*class RectangleSensor: public LandmarkSensor
	{
	public:
		RectangleSensor()
		{
			mSensorType = LandmarkSensor::SensorType::detector_pipe;
		}
	
		RectangleSensor(double scale)
		{
			mSensorType = LandmarkSensor::SensorType::detector_pipe;
			mScale = scale;
		}
	
		virtual void calculateInitialOffset()
		{
			//
		}
	
		virtual double getScaledVote()
		{
			if (mRealMsg->ice_isA("::RobotSimEvents::VisionRectangleMessage"))
			{
				RobotSimEvents::VisionRectangleMessagePtr realMsg = RobotSimEvents::VisionRectangleMessagePtr::dynamicCast(mRealMsg);
				VirtualSensorMessage::VisionRectangleMessage * virtualMsg = static_cast<VirtualSensorMessage::VisionRectangleMessage*>(mVirtualMsg);
				int realOrientation = 0; //fuck it I'm just gonna average all of them
				for(unsigned int i = 0; i < realMsg->quads.size(); i ++)
				{
					realOrientation += LocalizationUtil::eulerToPolar(realMsg->quads[i].angle);
				}
				realOrientation /= realMsg->quads.size();
				mVote = 0;
				if(realMsg->quads.size() > 0)
				{
					mVote += virtualMsg->rectangleDistance <= 20 ? 0.5 * -(virtualMsg->rectangleDistance - 20) / 20 : 0;
					if(virtualMsg->rectangleIsVisible)
					{
						mVote += 0.5 - 0.5 * LocalizationUtil::linearAngleDiffRatio(realOrientation, virtualMsg->rectangleRelOrientation);
					}
				}
				mScaledVote = mVote * mScale;
				return mScaledVote;
			}
			return Flags::voteFailed;
		}
	
		static VirtualSensorMessage::VirtualSensorMessage * generatetVirtualSensorMessage(LocalizationParticle & part, LocalizationMap & map)
		{
			VirtualSensorMessage::VisionRectangleMessage * msg = new VirtualSensorMessage::VisionRectangleMessage;
			msg->rectangleIsVisible = false;
			int closestRectIndex = -1;
			double closestRectDist = -1;
			for(unsigned int i = 0; i < map.mMapEntities.size(); i ++)
			{
				if((map.mMapEntities[i].mShapeType == LocalizationMapEntity::ShapeType::rect || map.mMapEntities[i].mShapeType == LocalizationMapEntity::ShapeType::square) && map.mMapEntities[i].mInteractionType != LocalizationMapEntity::InteractionType::external)
				{
					double distToRect = sqrt(pow(part.mState.mPoint.i - map.mMapEntities[i].mCenter.i, 2) + pow(part.mState.mPoint.j - map.mMapEntities[i].mCenter.j, 2));
					double largerLength = map.mMapEntities[i].mDim.i;
					if(map.mMapEntities[i].mDim.j > largerLength)
					{
						largerLength = map.mMapEntities[i].mDim.j;
					}
					if(distToRect <= largerLength / 2)
					{
						msg->rectangleIsVisible = true;
					}
					if(distToRect >= closestRectDist)
					{
						closestRectDist = distToRect;
						closestRectIndex = i;
					}
				}
			}
			msg->rectangleDistance = closestRectDist;
			msg->rectangleRelOrientation = map.mMapEntities[closestRectIndex].mOrientation;
			return msg;
		}
	};*/
}
