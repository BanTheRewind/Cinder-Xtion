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

	typedef std::shared_ptr<class Device>			DeviceRef;
	typedef std::map<XnSkeletonJoint, class Bone>	Skeleton;

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
		Bone( XnSkeletonJoint jointName, const ci::Vec3f &position );
		
		XnSkeletonJoint					mJointName;
		ci::Vec3f						mPosition;

		friend class					Device;
	};

	//////////////////////////////////////////////////////////////////////////////////////////////

	class DeviceOptions
	{
	public:
		DeviceOptions();

		static ci::Vec2i	getResolutionSize( XnResolution resolution );

		DeviceOptions&		enableAudio( bool enable = true );
		DeviceOptions&		enableDepth( bool enable = true );
		DeviceOptions&		enableInfrared( bool enable = true );
		DeviceOptions&		enableUserTracking( bool enable = true, bool trackSkeletons = true );
		DeviceOptions&		enableVideo( bool enable = true );
		
		XnSampleRate		getAudioSampleRate() const;
		float				getDepthFrameRate() const;
		XnResolution		getDepthResolution() const; 
		const ci::Vec2i&	getDepthSize() const; 
		const std::string&	getDeviceId() const;
		int32_t				getDeviceIndex() const;
		float				getInfraredFrameRate() const;
		XnResolution		getInfraredResolution() const; 
		const ci::Vec2i&	getInfraredSize() const;
		float				getVideoFrameRate() const;
		XnResolution		getVideoResolution() const; 
		const ci::Vec2i&	getVideoSize() const; 

		bool				isAudioEnabled() const;
		bool				isDepthEnabled() const;
		bool				isInfraredEnabled() const; 
		bool				isSkeletonTrackingEnabled() const;
		bool				isUserTrackingEnabled() const;
		bool				isVideoEnabled() const;

		DeviceOptions&		setAudioSampleRate( XnSampleRate sampleRate = XnSampleRate::XN_SAMPLE_RATE_44K ); 
		DeviceOptions&		setDepthFrameRate( float frameRate = 30.0f ); 
		DeviceOptions&		setDepthResolution( const ci::Vec2i &size );
		DeviceOptions&		setDepthResolution( XnResolution resolution = XnResolution::XN_RES_QVGA ); 
		DeviceOptions&		setDeviceId( const std::string &id = "" ); 
		DeviceOptions&		setDeviceIndex( int32_t index = 0 ); 
		DeviceOptions&		setInfraredResolution( const ci::Vec2i &size );
		DeviceOptions&		setInfraredResolution( XnResolution resolution = XnResolution::XN_RES_QVGA ); 
		DeviceOptions&		setInfraredFrameRate( float frameRate = 30.0f ); 
		DeviceOptions&		setVideoResolution( const ci::Vec2i &size );
		DeviceOptions&		setVideoResolution( XnResolution resolution = XnResolution::XN_RES_VGA ); 
		DeviceOptions&		setVideoFrameRate( float frameRate = 30.0f ); 
	private:
		bool				mEnabledAudio;
		bool				mEnabledDepth;
		bool				mEnabledInfrared;
		bool				mEnabledSkeletonTracking;
		bool				mEnabledUserTracking;
		bool				mEnabledVideo;

		float				mFrameRateDepth;
		float				mFrameRateInfrared;
		float				mFrameRateVideo;

		XnSampleRate		mAudioSampleRate;
		XnResolution		mDepthResolution;
		ci::Vec2i			mDepthSize;
		XnResolution		mInfraredResolution;
		ci::Vec2i			mInfraredSize;
		XnResolution		mVideoResolution;
		ci::Vec2i			mVideoSize;

		std::string			mDeviceId;
		int32_t				mDeviceIndex;
	};

	//////////////////////////////////////////////////////////////////////////////////////////////

	class Device
	{
	public:
		static DeviceRef				create();
		static void						release();
		~Device();

		bool							isCapturing() const;
		bool							isPaused() const;
		void							pause();
		void							resume();
		void							start( const DeviceOptions &deviceOptions = DeviceOptions() );
		void							start( const ci::fs::path &configFilePath );
		void							stop();

		const DeviceOptions&			getDeviceOptions() const;

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

		size_t							getDeviceCount() const;
	private:
		typedef std::shared_ptr<boost::thread>	ThreadRef;
		
		Device();

		static const uint32_t MAX_COUNT	= 6;

		void							init();

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

		std::string						mDeviceId;
		size_t							mDeviceCount;

		//////////////////////////////////////////////////////////////////////////////////////////////

		DeviceOptions					mDeviceOptions;

		xn::Context						mContext;
		xn::Device						mDevice;
		xn::Query						mQuery;

		xn::AudioMetaData				mMetaDataAudio;
		xn::DepthMetaData				mMetaDataDepth;
		xn::IRMetaData					mMetaDataInfrared;
		xn::SceneMetaData				mMetaDataScene;
		xn::ImageMetaData				mMetaDataVideo;

		volatile bool					mRunning;
		ThreadRef						mThread;
		void							run();

		boost::mutex					mMutex;
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

		xn::AudioGenerator				mGeneratorAudio;
		xn::DepthGenerator				mGeneratorDepth;
		xn::IRGenerator					mGeneratorInfrared;
		xn::UserGenerator				mGeneratorUser;
		xn::ImageGenerator				mGeneratorVideo;

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
