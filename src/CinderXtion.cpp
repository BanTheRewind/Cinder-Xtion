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

#include "CinderXtion.h"

#include "cinder/app/App.h"
#include <Windows.h>

namespace Xtion
{

	using namespace ci;
	using namespace ci::app;
	using namespace std;
	
	//////////////////////////////////////////////////////////////////////////////////////////////

	bool success( XnStatus status ) 
	{
		if ( status == XN_STATUS_OK ) {
			return true;
		}
		const XnChar* error = xnGetStatusString( status );
		trace( "Error: " + toString( error ) );
		return false;
	}

	void trace( const string &message )
	{
#if defined ( CINDER_MSW )
		OutputDebugStringA( ( message + "\n" ).c_str() );
#else
		console() << message << endl;
#endif
	}

	//////////////////////////////////////////////////////////////////////////////////////////////

	Bone::Bone( XnSkeletonJoint jointName, const Vec3f &position )
		: mJointName( jointName ), mPosition( position )
	{
	}

	const Vec3f& Bone::getPosition() const
	{
		return mPosition;
	}
	//////////////////////////////////////////////////////////////////////////////////////////////

	Vec2i DeviceOptions::getResolutionSize( XnResolution resolution )
	{
		Vec2i size = Vec2i::zero();
		switch ( resolution ) {
		case XnResolution::XN_RES_1080P:
			size = Vec2i( XN_1080P_X_RES, XN_1080P_Y_RES );
			break;
		case XnResolution::XN_RES_240P:
			size = Vec2i( XN_240P_X_RES, XN_240P_Y_RES );
			break;
		case XnResolution::XN_RES_480P:
			size = Vec2i( XN_480P_X_RES, XN_480P_Y_RES );
			break;
		case XnResolution::XN_RES_576P:
			size = Vec2i( XN_576P_X_RES, XN_576P_Y_RES );
			break;
		case XnResolution::XN_RES_720P:
			size = Vec2i( XN_720P_X_RES, XN_720P_Y_RES );
			break;
		case XnResolution::XN_RES_CGA:
			size = Vec2i( XN_CGA_X_RES, XN_CGA_Y_RES );
			break;
		case XnResolution::XN_RES_CIF:
			size = Vec2i( XN_CIF_X_RES, XN_CIF_Y_RES );
			break;
		case XnResolution::XN_RES_DV:
			size = Vec2i( XN_DV_X_RES, XN_DV_Y_RES );
			break;
		case XnResolution::XN_RES_QCIF:
			size = Vec2i( XN_QCIF_X_RES, XN_QCIF_Y_RES );
			break;
		case XnResolution::XN_RES_QQVGA:
			size = Vec2i( XN_QQVGA_X_RES, XN_QQVGA_Y_RES );
			break;
		case XnResolution::XN_RES_QVGA:
			size = Vec2i( XN_QVGA_X_RES, XN_QVGA_Y_RES );
			break;
		case XnResolution::XN_RES_SVGA:
			size = Vec2i( XN_SVGA_X_RES, XN_SVGA_Y_RES );
			break;
		case XnResolution::XN_RES_SXGA:
			size = Vec2i( XN_SXGA_X_RES, XN_SXGA_Y_RES );
			break;
		case XnResolution::XN_RES_UXGA:
			size = Vec2i( XN_UXGA_X_RES, XN_UXGA_Y_RES );
			break;
		case XnResolution::XN_RES_VGA:
			size = Vec2i( XN_VGA_X_RES, XN_VGA_Y_RES );
			break;
		case XnResolution::XN_RES_WVGA:
			size = Vec2i( XN_WVGA_X_RES, XN_WVGA_Y_RES );
			break;
		case XnResolution::XN_RES_XGA:
			size = Vec2i( XN_XGA_X_RES, XN_XGA_Y_RES );
			break;
		default:
			break;
		}
		return size;
	}

	DeviceOptions::DeviceOptions()
	{
		setDeviceId();
		setDeviceIndex();

		enableAudio( false );
		enableDepth( true );
		enableInfrared( false );
		enableUserTracking( false, false );
		enableVideo( true );

		setDepthFrameRate();
		setInfraredFrameRate();
		setVideoFrameRate();

		setAudioSampleRate();
		setDepthResolution();
		setInfraredResolution();
		setVideoResolution();
	}

	DeviceOptions& DeviceOptions::enableAudio( bool enable )
	{
		mEnabledAudio = enable;
		return *this;
	}

	DeviceOptions& DeviceOptions::enableDepth( bool enable )
	{
		mEnabledDepth = enable;
		return *this;
	}

	DeviceOptions& DeviceOptions::enableInfrared( bool enable )
	{
		mEnabledInfrared = enable;
		return *this;
	}

	DeviceOptions& DeviceOptions::enableUserTracking( bool enable, bool trackSkeletons )
	{
		mEnabledSkeletonTracking = trackSkeletons;
		mEnabledUserTracking = enable;
		return *this;
	}

	DeviceOptions& DeviceOptions::enableVideo( bool enable )
	{
		mEnabledVideo = enable;
		return *this;
	}
	
	XnSampleRate DeviceOptions::getAudioSampleRate() const
	{
		return mAudioSampleRate;
	}

	float DeviceOptions::getDepthFrameRate() const
	{
		return mFrameRateDepth;
	}

	XnResolution DeviceOptions::getDepthResolution() const
	{
		return mDepthResolution;
	}
 
	const Vec2i& DeviceOptions::getDepthSize() const
	{
		return mDepthSize;
	}

	const std::string& DeviceOptions::getDeviceId() const
	{
		return mDeviceId;
	}

	int32_t DeviceOptions::getDeviceIndex() const
	{
		return mDeviceIndex;
	}

	float  DeviceOptions::getInfraredFrameRate() const
	{
		return mFrameRateInfrared;
	}

	XnResolution DeviceOptions::getInfraredResolution() const
	{
		return mInfraredResolution;
	}

	const Vec2i& DeviceOptions::getInfraredSize() const
	{
		return mInfraredSize;
	}

	float  DeviceOptions::getVideoFrameRate() const
	{
		return mFrameRateVideo;
	}

	XnResolution DeviceOptions::getVideoResolution() const
	{
		return mVideoResolution;
	}

	const Vec2i& DeviceOptions::getVideoSize() const
	{
		return mVideoSize;
	}

	bool DeviceOptions::isAudioEnabled() const
	{
		return mEnabledAudio;
	}

	bool DeviceOptions::isDepthEnabled() const
	{
		return mEnabledDepth;
	}

	bool DeviceOptions::isInfraredEnabled() const
	{
		return mEnabledInfrared;
	}

	bool DeviceOptions::isSkeletonTrackingEnabled() const
	{
		return mEnabledSkeletonTracking;
	}

	bool DeviceOptions::isUserTrackingEnabled() const
	{
		return mEnabledUserTracking;
	}

	bool DeviceOptions::isVideoEnabled() const
	{
		return mEnabledVideo;
	}

	DeviceOptions& DeviceOptions::setAudioSampleRate( XnSampleRate sampleRate )
	{
		mAudioSampleRate = sampleRate;
		return *this;
	}

	DeviceOptions& DeviceOptions::setDepthFrameRate( float frameRate )
	{
		mFrameRateDepth = frameRate;
		return *this;
	}

	DeviceOptions& DeviceOptions::setDepthResolution( const Vec2i &size )
	{
		mDepthSize = size;
		mDepthResolution = XnResolution::XN_RES_CUSTOM;
		return *this;
	}

	DeviceOptions& DeviceOptions::setDepthResolution( XnResolution resolution )
	{
		mDepthResolution = resolution;
		mDepthSize = getResolutionSize( resolution );
		return *this;
	}

	DeviceOptions& DeviceOptions::setDeviceId( const string &id )
	{
		mDeviceId = id;
		return *this;
	}

	DeviceOptions& DeviceOptions::setDeviceIndex( int32_t index )
	{
		mDeviceIndex = index;
		return *this;
	}

	DeviceOptions& DeviceOptions::setInfraredFrameRate( float frameRate )
	{
		mFrameRateInfrared = frameRate;
		return *this;
	}

	DeviceOptions& DeviceOptions::setInfraredResolution( const Vec2i &size )
	{
		mInfraredSize = size;
		mInfraredResolution = XnResolution::XN_RES_CUSTOM;
		return *this;
	}

	DeviceOptions& DeviceOptions::setInfraredResolution( XnResolution resolution )
	{
		mInfraredResolution = resolution;
		mInfraredSize = getResolutionSize( resolution );
		return *this;
	}
	
	DeviceOptions& DeviceOptions::setVideoFrameRate( float frameRate )
	{
		mFrameRateVideo = frameRate;
		return *this;
	}

	DeviceOptions& DeviceOptions::setVideoResolution( const Vec2i &size )
	{
		mVideoSize = size;
		mVideoResolution = XnResolution::XN_RES_CUSTOM;
		return *this;
	}

	DeviceOptions& DeviceOptions::setVideoResolution( XnResolution resolution )
	{
		mVideoResolution = resolution;
		mVideoSize = getResolutionSize( mVideoResolution );
		return *this;
	}


	//////////////////////////////////////////////////////////////////////////////////////////////

	xn::Context			Device::sContext;
	Device::DeviceInfo	Device::sDevices[ MAX_COUNT ];
	bool				Device::sContextInit	= false;

	DeviceRef Device::create()
	{
		if ( !sContextInit ) {
			initContext();
		}
		return DeviceRef( new Device() );
	}

	void Device::initContext() 
	{
		XnStatus status = sContext.Init();
		if ( !success( status ) ) {
			return;
		}

		xn::NodeInfoList deviceNodes;
		size_t count = 0;
		status = sContext.EnumerateProductionTrees( XN_NODE_TYPE_DEVICE, 0, deviceNodes );

		size_t i = 0;
		for ( xn::NodeInfoList::Iterator iter = deviceNodes.Begin(); iter != deviceNodes.End(); ++iter, ++i ) {
			xn::NodeInfo info = *iter;
			status = sContext.CreateProductionTree( info, sDevices[ i ].mDevice );

			sDevices[ i ].mQuery.AddNeededNode( info.GetInstanceName() );
		}
		sContextInit = true;
	}

	void Device::release()
	{
		for ( size_t i = 0; i < MAX_COUNT; ++i ) {
			sDevices[ i ].mGeneratorAudio.Release();
			sDevices[ i ].mGeneratorDepth.Release();
			sDevices[ i ].mGeneratorInfrared.Release();
			sDevices[ i ].mGeneratorUser.Release();
			sDevices[ i ].mGeneratorVideo.Release();
			sDevices[ i ].mDevice.Release();
		}
		sContext.Release();
	}

	/*void Device::setConfigFile( const fs::path &configFilePath )
	{
		sContextInit = false;

		XnStatus status = XN_STATUS_OK;
		const std::string& filePath = configFilePath.generic_string();
		if ( !fs::exists( configFilePath ) ) {
			trace( "\"" + filePath + "\" does not exist" );
			return;
		}

		xn::ScriptNode scriptNode;
		status = sContext.InitFromXmlFile( filePath.c_str(), scriptNode, 0 );
		if ( status == XN_STATUS_NO_NODE_PRESENT ) {
			trace( "Invalid configuration file" );
			return;
		}
		scriptNode.Release();
		if ( success( status ) ) {
			sContextInit = true;
		}
	}*/
	
	Device::Device()
	{
		init();
	}

	Device::~Device()
	{
		if ( mCapture ) {
			stop();
		}
		
		try {
			mMetaDataAudio.Free();
			mMetaDataDepth.Free();
			mMetaDataInfrared.Free();
			mMetaDataScene.Free();
			mMetaDataVideo.Free();
		} catch ( ... ) {
		}

		if ( mDataAudio != 0 ) {
			delete [] mDataAudio;
		}
		if ( mDataDepth != 0 ) {
			delete [] mDataDepth;
		}
		if ( mDataInfrared != 0 ) {
			delete [] mDataInfrared;
		}
		if ( mDataVideo != 0 ) {
			delete [] mDataVideo;
		}
	}

	bool Device::checkNewAudio()
	{
		bool newAudio = mNewAudio;
		mNewAudio = false;
		return newAudio;
	}

	bool Device::checkNewDepthFrame()
	{
		bool newFrame = mNewDepthFrame;
		mNewDepthFrame = false;
		return newFrame;
	}

	bool Device::checkNewInfraredFrame()
	{
		bool newFrame = mNewInfraredFrame;
		mNewInfraredFrame = false;
		return newFrame;
	}

	bool Device::checkNewUserData()
	{
		bool newUserData = mNewUserData;
		mNewUserData = false;
		return newUserData;
	}

	bool Device::checkNewVideoFrame()
	{
		bool newFrame = mNewVideoFrame;
		mNewVideoFrame = false;
		return newFrame;
	}

	uint_fast8_t* Device::getAudio()
	{
		boost::lock_guard<boost::mutex> lock( mMutexAudio );
		mNewAudio = false;
		return mDataAudio;
	}

	uint32_t Device::getAudioBufferSize() const
	{
		return mDataAudioSize;
	}

	Channel16u Device::getDepth()
	{
		boost::lock_guard<boost::mutex> lock( mMutexDepth );
		mNewDepthFrame = false;
		return mChannelDepth;
	}

	float Device::getDepthAt( const ci::Vec2i &position )
	{
		if ( mChannelDepth ) {
			return 1.0f - (float)( mChannelDepth.getValue( position ) / ( 1.0 * 0x8000 ) ) * 10.0f;
		}
		return 0.0f;
	}

	Vec2i Device::getDepthSize()
	{
		boost::lock_guard<boost::mutex> lock( mMutexDepth );
		return mSizeDepth;
	}

	Channel16u Device::getInfrared()
	{
		boost::lock_guard<boost::mutex> lock( mMutexInfrared );
		mNewInfraredFrame = false;
		return mChannelInfrared;
	}

	Vec2i Device::getInfraredSize()
	{
		boost::lock_guard<boost::mutex> lock( mMutexInfrared );
		return mSizeInfrared;
	}

	vector<Skeleton> Device::getSkeletons()
	{
		boost::lock_guard<boost::mutex> lock( mMutexUser );
		mNewUserData = false;
		return mSkeletons;
	}

	Channel16u Device::getUserImage()
	{
		boost::lock_guard<boost::mutex> lock( mMutexUser );
		mNewUserData = false;
		return mChannelUserImage;
	}

	Vec2i Device::getUserImageSize()
	{
		boost::lock_guard<boost::mutex> lock( mMutexUser );
		return mSizeUserImage;
	}

	Surface8u Device::getVideo()
	{
		boost::lock_guard<boost::mutex> lock( mMutexVideo );
		mNewVideoFrame = false;
		return mSurfaceVideo;
	}

	Vec2i Device::getVideoSize()
	{
		boost::lock_guard<boost::mutex> lock( mMutexVideo );
		return mSizeVideo;
	}

	void Device::init()
	{
		mDataAudio					= 0;
		mDataAudioSize				= 0;
		mDeviceCount				= 0;
		mBinary						= false;
		mCalibrationPoseRequired	= true;
		mCapture					= false;
		mDataDepth					= 0;
		mDataInfrared				= 0;
		mDataUserImage				= 0;
		mDataVideo					= 0;
		mEnabledSkeletonTracking	= false;
		mGreyScale					= false;
		mInverted					= false;
		mNewDepthFrame				= false;
		mNewUserData				= false;
		mNewVideoFrame				= false;
		mPaused						= false;
		mRemoveBackground			= false;
		mSizeDepth					= Vec2i::zero();
		mSizeInfrared				= Vec2i::zero();
		mSizeUserImage				= Vec2i::zero();
		mSizeVideo					= Vec2i::zero();
		mRunning					= false;
		mSkeletons.clear();
	}

	bool Device::isCapturing() const
	{
		return mCapture;
	}

	bool Device::isPaused() const
	{
		return mPaused;
	}

	void XN_CALLBACK_TYPE Device::onCalibrationEnd( xn::SkeletonCapability &capability, XnUserID id, XnBool success, void *data )
	{
		Device* device = static_cast<Device*>( data );
		if ( device != 0 ) {
			device->onCalibrationEnd( capability, id, success );
		}
	}

	void Device::onCalibrationEnd( xn::SkeletonCapability &capability, XnUserID id, XnBool success )
	{
		int32_t index = mDeviceOptions.getDeviceIndex();
		if ( success ) {
			trace( "Calibration successful for user ID: " + toString( id ) );
			sDevices[ index ].mGeneratorUser.GetSkeletonCap().StartTracking( id );
		} else {
			trace( "Calibration failed for user ID: " + toString( id ) );
			if ( mCalibrationPoseRequired ) {
				sDevices[ index ].mGeneratorUser.GetPoseDetectionCap().StartPoseDetection( mPoseStr, id );
			} else {
				sDevices[ index ].mGeneratorUser.GetSkeletonCap().RequestCalibration( id, TRUE );
			}
		}
	}

	void XN_CALLBACK_TYPE Device::onNewUser( xn::UserGenerator &generator, XnUserID id, void *data )
	{
		Device* device = static_cast<Device*>( data );
		if ( device != 0 ) {
			device->onNewUser( generator, id );
		}
	}

	void Device::onNewUser( xn::UserGenerator &generator, XnUserID id )
	{
		trace( "New user ID: " + toString( id ) );
		int32_t index = mDeviceOptions.getDeviceIndex();
		if ( mCalibrationPoseRequired ) {
			sDevices[ index ].mGeneratorUser.GetPoseDetectionCap().StartPoseDetection( mPoseStr, id );
		} else {
			sDevices[ index ].mGeneratorUser.GetSkeletonCap().RequestCalibration( id, TRUE );
		}
	}

	void XN_CALLBACK_TYPE Device::onPoseDetected( xn::PoseDetectionCapability &capability, const XnChar *pose, XnUserID id, void *data )
	{
		Device* device = static_cast<Device*>( data );
		if ( device != 0 ) {
			device->onPoseDetected( capability, pose, id );
		}
	}

	void Device::onPoseDetected( xn::PoseDetectionCapability &capability, const XnChar *pose, XnUserID id )
	{
		trace( "Pose detected for user ID: " + toString( id ) );
		int32_t index = mDeviceOptions.getDeviceIndex();
		sDevices[ index ].mGeneratorUser.GetPoseDetectionCap().StopPoseDetection( id );
		sDevices[ index ].mGeneratorUser.GetSkeletonCap().RequestCalibration( id, TRUE );
	}

	void Device::pause()
	{
		mPaused = true;
	}
	
	void Device::resume()
	{
		mPaused = false;
	}

	void Device::run()
	{
		while ( mRunning ) {
			if ( mCapture && !mPaused ) {

				int32_t index = mDeviceOptions.getDeviceIndex();

				bool enabledAudio = mDeviceOptions.isAudioEnabled();
				bool enabledDepth = mDeviceOptions.isDepthEnabled();
				bool enabledInfrared = mDeviceOptions.isInfraredEnabled();
				bool enabledVideo = mDeviceOptions.isVideoEnabled();

				XnStatus status = XN_STATUS_OK;
				if ( enabledDepth ) {
					XnStatus status = sContext.WaitOneUpdateAll( sDevices[ index ].mGeneratorDepth );
				} else if ( enabledVideo ) {
					XnStatus status = sContext.WaitOneUpdateAll( sDevices[ index ].mGeneratorVideo );
				} else if ( enabledInfrared ) {
					XnStatus status = sContext.WaitOneUpdateAll( sDevices[ index ].mGeneratorInfrared );
				} else if ( enabledAudio ) {
					XnStatus status = sContext.WaitOneUpdateAll( sDevices[ index ].mGeneratorAudio );
				}

				if ( success( status ) ) {
					
					if ( enabledAudio ) {
						sDevices[ index ].mGeneratorAudio.GetMetaData( mMetaDataAudio );
						const uint_fast8_t* buffer = sDevices[ index ].mGeneratorAudio.GetAudioBuffer();
						mDataAudioSize = sDevices[ index ].mGeneratorAudio.GetDataSize();
						if ( mDataAudio == 0 ) {
							mDataAudio = new uint_fast8_t[ mDataAudioSize * 2 ]; // 2 = channels
						}
						memcpy( mDataAudio, buffer, mDataAudioSize );
						mNewAudio = true;
					}

					if ( enabledDepth ) {
						sDevices[ index ].mGeneratorDepth.GetMetaData( mMetaDataDepth );
						mSizeDepth = Vec2i( mMetaDataDepth.XRes(), mMetaDataDepth.YRes() );
						uint32_t count = mSizeDepth.x * mSizeDepth.y;
						mDataDepth = (uint16_t*)mMetaDataDepth.Data();
						if ( !mChannelDepth ) {
							mChannelDepth = Channel16u( mSizeDepth.x, mSizeDepth.y );
						}
						memcpy( mChannelDepth.getData(), mDataDepth, count * mMetaDataDepth.BytesPerPixel() );
						mNewDepthFrame = true;
					}

					if ( enabledInfrared ) {
						sDevices[ index ].mGeneratorInfrared.GetMetaData( mMetaDataInfrared );
						mSizeInfrared = Vec2i( mMetaDataInfrared.XRes(), mMetaDataInfrared.YRes() );
						uint32_t count = mSizeInfrared.x * mSizeInfrared.y;
						mDataInfrared = (uint16_t*)mMetaDataInfrared.Data();
						if ( !mChannelInfrared ) {
							mChannelInfrared = Channel16u( mSizeInfrared.x, mSizeInfrared.y );
						}
						memcpy( mChannelInfrared.getData(), mDataInfrared, count * mMetaDataInfrared.BytesPerPixel() );
						mNewInfraredFrame = true;
					}

					if ( enabledVideo ) {
						sDevices[ index ].mGeneratorVideo.GetMetaData( mMetaDataVideo );
						mSizeVideo = Vec2i( mMetaDataVideo.XRes(), mMetaDataVideo.YRes() );
						uint32_t count = mSizeVideo.x * mSizeVideo.y;
						mDataVideo = (uint8_t*)mMetaDataVideo.Data();
						if ( !mSurfaceVideo ) {
							mSurfaceVideo = Surface8u(  mSizeVideo.x, mSizeVideo.y, false, SurfaceChannelOrder::RGB );
						}
						memcpy( mSurfaceVideo.getData(), mDataVideo, count * mMetaDataVideo.BytesPerPixel() );
						mNewVideoFrame = true;
					}
				}
			}
		}
	}

	void Device::start( const DeviceOptions &deviceOptions )
	{
		mDeviceOptions = deviceOptions;

		if ( mCapture ) {
			stop();
		}

		XnStatus status = XN_STATUS_OK;
		
		if ( !sContextInit ) {
			trace( "Configuration file not set. Please call Device::setConfigFile() before initializing a device." );
			return;
		}

		size_t index = math<size_t>::min( (size_t)mDeviceOptions.getDeviceIndex(), MAX_COUNT );
		mDeviceOptions.setDeviceIndex( (int32_t)index );
		
		if ( sDevices[ index ].mDevice.IsValid() != TRUE ) {
			return;
		}
		
		if ( mDeviceOptions.isAudioEnabled() ) {
			if ( success( sContext.CreateAnyProductionTree( XN_NODE_TYPE_AUDIO,	&sDevices[ index ].mQuery, sDevices[ index ].mGeneratorAudio ) ) && 
				success( sDevices[ index ].mGeneratorAudio.IsValid() ) ) {
				XnWaveOutputMode waveMode;
				// TODO use settings from device options
				waveMode.nSampleRate	= 44100;
				waveMode.nChannels		= 2;
				waveMode.nBitsPerSample	= 16;
				if ( !success( sDevices[ index ].mGeneratorAudio.SetWaveOutputMode( waveMode ) ) || 
					!success( sDevices[ index ].mGeneratorVideo.StartGenerating() ) ) {
						mDeviceOptions.enableAudio( false );
				}
			} else {
				mDeviceOptions.enableAudio( false );
			}
		}

		if ( mDeviceOptions.isDepthEnabled() ) {
			//xn::Query query = sDevices[ index ].mQuery;
			XnMapOutputMode mode;
			mode.nFPS	= (XnUInt32)mDeviceOptions.getDepthFrameRate();
			mode.nXRes  = (XnUInt32)mDeviceOptions.getDepthSize().x;
			mode.nXRes	= (XnUInt32)mDeviceOptions.getDepthSize().x;
			//query.AddSupportedMapOutputMode( mode );
			if ( !success( sContext.CreateAnyProductionTree( XN_NODE_TYPE_DEPTH, &sDevices[ index ].mQuery, sDevices[ index ].mGeneratorDepth ) ) || 
				!success( sDevices[ index ].mGeneratorDepth.IsValid() ) || 
				!success( sDevices[ index ].mGeneratorDepth.StartGenerating() ) ) {
				mDeviceOptions.enableDepth( false );
			}
		}

		if ( mDeviceOptions.isInfraredEnabled() ) {
			//xn::Query query = sDevices[ index ].mQuery;
			XnMapOutputMode mode;
			mode.nFPS	= (XnUInt32)mDeviceOptions.getInfraredFrameRate();
			mode.nXRes  = (XnUInt32)mDeviceOptions.getInfraredSize().x;
			mode.nXRes	= (XnUInt32)mDeviceOptions.getInfraredSize().x;
			//query.AddSupportedMapOutputMode( mode );
			if ( !success( sContext.CreateAnyProductionTree( XN_NODE_TYPE_IR, &sDevices[ index ].mQuery, sDevices[ index ].mGeneratorInfrared ) ) || 
				!success( sDevices[ index ].mGeneratorInfrared.IsValid() ) || 
				!success( sDevices[ index ].mGeneratorInfrared.StartGenerating() ) ) {
				mDeviceOptions.enableInfrared( false );
			}
		}

		if ( mDeviceOptions.isVideoEnabled() ) {
			//xn::Query query = sDevices[ index ].mQuery;
			XnMapOutputMode mode;
			mode.nFPS	= (XnUInt32)mDeviceOptions.getVideoFrameRate();
			mode.nXRes  = (XnUInt32)mDeviceOptions.getVideoSize().x;
			mode.nXRes	= (XnUInt32)mDeviceOptions.getVideoSize().x;
			//query.AddSupportedMapOutputMode( mode );
			if ( !success( sContext.CreateAnyProductionTree( XN_NODE_TYPE_IMAGE, &sDevices[ index ].mQuery, sDevices[ index ].mGeneratorVideo ) ) || 
				!success( sDevices[ index ].mGeneratorVideo.IsValid() ) || 
				!success( sDevices[ index ].mGeneratorVideo.StartGenerating() ) ) {
				mDeviceOptions.enableVideo( false );
			}
		}

		// TODO user nodes, hands, etc

		mCapture	= true;
		mRunning	= true;
		mThread		= ThreadRef( new boost::thread( bind( &Device::run, this ) ) );
	}

	void Device::stop()
	{
		if ( mDeviceOptions.isDepthEnabled() ) {
			sDevices[ mDeviceOptions.getDeviceIndex() ].mGeneratorDepth.StopGenerating();
		}

		mRunning = false;
		if ( mThread ) {
			mThread->join();
		}
		
		mCapture = false;
		mSkeletons.clear();
	}

}
