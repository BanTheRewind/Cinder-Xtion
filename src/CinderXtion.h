/*
* 
* Copyright (c) 2012, Ban the Rewind
* All rights reserved.
* 
* Redistribution and use in source and binary forms, with or 
* without modification, are permitted provided that the following 
* conditions are met:
* 
* Redistributions of source code must retain the above copyright 
* notice, this list of conditions and the following disclaimer.
* Redistributions in binary form must reproduce the above copyright 
* notice, this list of conditions and the following disclaimer in 
* the documentation and/or other materials provided with the 
* distribution.
* 
* Neither the name of the Ban the Rewind nor the names of its 
* contributors may be used to endorse or promote products 
* derived from this software without specific prior written 
* permission.
* 
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS 
* "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT 
* LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS 
* FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE 
* COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, 
* INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
* BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; 
* LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER 
* CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, 
* STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) 
* ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF 
* ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
* 
*/

#pragma once

#include "cinder/Surface.h"
#include "cinder/Thread.h"
#include "cinder/Vector.h"
#include "cinder/Utilities.h"
#include "XnOpenNI.h"
#include "XnCodecIDs.h"
#include "XnCppWrapper.h"
#include <map>

namespace Xtion
{

	typedef std::shared_ptr<class Device>	DeviceRef;
	typedef XnSkeletonJoint					JointName;
	typedef std::map<JointName, class Bone>	Skeleton;

	//////////////////////////////////////////////////////////////////////////////////////////////

	bool								success( XnStatus status );
	void								trace( const std::string &message );
	
	//////////////////////////////////////////////////////////////////////////////////////////////

	class Device;
	class Bone
	{
	public:
		const ci::Vec3f&				getPosition() const;
	private:
		Bone( JointName jointName, const ci::Vec3f &position );
		
		JointName						mJointName;
		ci::Vec3f						mPosition;

		friend class					Device;
	};

	//////////////////////////////////////////////////////////////////////////////////////////////

	class Device
	{
	public:
		static DeviceRef				create();
		~Device();

		bool							isCapturing() const;
		bool							isPaused() const;
		void							pause();
		void							resume();
		void							start( const ci::fs::path &configFilePath );
		void							stop();

		void							enableBinaryMode( bool enable = true, bool invertImage = false );
		bool							isBinaryImageInverted() const;
		bool							isBinaryModeEnabled() const;
		void							enableUserColor( bool enable = true );
		bool							isUserColorEnabled() const;

		uint_fast8_t*					getAudio();
		size_t							getAudioBufferSize() const;
		bool							checkNewAudio();

		bool							checkNewDepthFrame();
		ci::Channel16u					getDepth();
		float							getDepthAt( const ci::Vec2i &position );
		ci::Vec2i						getDepthSize();
		
		bool							checkNewInfraredFrame();
		ci::Channel16u					getInfrared();
		ci::Vec2i						getInfraredSize();

		bool							checkNewUserData();
		std::vector<Skeleton>			getSkeletons();
		ci::Channel16u					getUserImage();
		ci::Vec2i						getUserImageSize();

		bool							checkNewVideoFrame();
		ci::Surface8u					getVideo();
		ci::Vec2i						getVideoSize();

		ci::Vec2i						getSkeletonDepthPos( const ci::Vec3f &position );
		ci::Vec2i						getSkeletonInfraredPos( const ci::Vec3f &position );
		ci::Vec2i						getSkeletonVideoPos( const ci::Vec3f &position );
	private:
		typedef std::shared_ptr<boost::thread>	ThreadRef;

		Device();

		void							init();

		xn::Context						mContext;
		xn::ScriptNode					mScriptNode;
		
		xn::Player						mPlayer;
		bool							mEnabledSkeletonTracking;
		
		XnCallbackHandle				mCallbackCalibration;
		XnCallbackHandle				mCallbackPose;
		XnCallbackHandle				mCallbackUser;

		static void XN_CALLBACK_TYPE	onCalibrationEnd( xn::SkeletonCapability &capability, XnUserID id, XnBool success, void *data );
		static void XN_CALLBACK_TYPE	onNewUser( xn::UserGenerator &generator, XnUserID id, void *data );
		static void XN_CALLBACK_TYPE	onPoseDetected( xn::PoseDetectionCapability &capability, const XnChar *pose, XnUserID id, void *data );
		
		void							onCalibrationEnd( xn::SkeletonCapability &capability, XnUserID id, XnBool success );
		void							onNewUser( xn::UserGenerator &generator, XnUserID id );
		void							onPoseDetected( xn::PoseDetectionCapability &capability, const XnChar *pose, XnUserID id );

		XnChar							mPoseStr[ 20 ];

		bool							mBinary;
		bool							mCalibrationPoseRequired;
		bool							mCapture;
		bool							mGreyScale;
		bool							mInverted;
		bool							mPaused;
		bool							mRemoveBackground;

		XnUInt64						mLastUpdate;

		//////////////////////////////////////////////////////////////////////////////////////////////

		xn::AudioGenerator				mGeneratorAudio;
		xn::DepthGenerator				mGeneratorDepth;
		xn::IRGenerator					mGeneratorInfrared;
		xn::UserGenerator				mGeneratorUser;
		xn::ImageGenerator				mGeneratorVideo;

		xn::AudioMetaData				mMetaDataAudio;
		xn::DepthMetaData				mMetaDataDepth;
		xn::IRMetaData					mMetaDataInfrared;
		xn::SceneMetaData				mMetaDataScene;
		xn::ImageMetaData				mMetaDataVideo;

		bool							mEnabledAudio;
		bool							mEnabledDepth;
		bool							mEnabledInfrared;
		bool							mEnabledUserTracking;
		bool							mEnabledVideo;

		volatile bool					mRunning;
		ThreadRef						mThread;
		void							run();

		boost::mutex					mMutexAudio;
		boost::mutex					mMutexDepth;
		boost::mutex					mMutexInfrared;
		boost::mutex					mMutexUser;
		boost::mutex					mMutexVideo;

		volatile bool					mNewAudio;
		volatile bool					mNewDepthFrame;
		volatile bool					mNewInfraredFrame;
		volatile bool					mNewUserData;
		volatile bool					mNewVideoFrame;

		uint16_t						*mDataDepth;
		uint16_t						*mDataInfrared;
		uint16_t						*mDataUserImage;
		uint8_t							*mDataVideo;

		uint_fast8_t					*mDataAudio;
		volatile size_t					mDataAudioSize;
		ci::Channel16u					mChannelDepth;
		ci::Channel16u					mChannelInfrared;
		ci::Channel16u					mChannelUserImage;
		ci::Surface8u					mSurfaceVideo;
		std::vector<Skeleton>			mSkeletons;

		ci::Vec2i						mSizeDepth;
		ci::Vec2i						mSizeInfrared;
		ci::Vec2i						mSizeUserImage;
		ci::Vec2i						mSizeVideo;
	};

}
