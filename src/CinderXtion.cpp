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

	Bone::Bone( JointName jointName, const Vec3f &position )
		: mJointName( jointName ), mPosition( position )
	{
	}

	const Vec3f& Bone::getPosition() const
	{
		return mPosition;
	}

	//////////////////////////////////////////////////////////////////////////////////////////////

	xn::Context			Device::sContext;
	xn::Device			Device::sDevice[ MAX_COUNT ];
	xn::AudioGenerator	Device::sGeneratorAudio[ MAX_COUNT ];
	xn::DepthGenerator	Device::sGeneratorDepth[ MAX_COUNT ];
	xn::IRGenerator		Device::sGeneratorInfrared[ MAX_COUNT ];
	xn::UserGenerator	Device::sGeneratorUser[ MAX_COUNT ];
	xn::ImageGenerator	Device::sGeneratorVideo[ MAX_COUNT ];
	xn::Context			Device::sContextFile;
	bool				Device::sContextInit						= false;

	void Device::setConfigFile( const fs::path &configFilePath )
	{
		sContextInit = false;

		XnStatus status = XN_STATUS_OK;
		const std::string& filePath = configFilePath.generic_string();
		if ( !fs::exists( configFilePath ) ) {
			trace( "\"" + filePath + "\" does not exist" );
			return;
		}

		xn::ScriptNode scriptNode;
		status = sContext.Init();//FromXmlFile( filePath.c_str(), scriptNode, 0 );
		if ( status == XN_STATUS_NO_NODE_PRESENT ) {
			trace( "Invalid configuration file" );
			return;
		}
		if ( success( status ) ) {
			
			xn::NodeInfoList deviceNodes;
			size_t count = 0;
			status = sContext.EnumerateProductionTrees( XN_NODE_TYPE_DEVICE, 0, deviceNodes );

			size_t i = 0;
			for ( xn::NodeInfoList::Iterator iter = deviceNodes.Begin(); iter != deviceNodes.End(); ++iter, ++i ) {
			   
				xn::NodeInfo info = *iter;
				status = sContext.CreateProductionTree( info, sDevice[ i ] );

				xn::Query query;
				query.AddNeededNode( info.GetInstanceName() );

				success( sContext.CreateAnyProductionTree( XN_NODE_TYPE_DEPTH, &query, sGeneratorDepth[ i ] ) );
				success( sContext.CreateAnyProductionTree( XN_NODE_TYPE_IR, &query, sGeneratorInfrared[ i ] ) );
				success( sContext.CreateAnyProductionTree( XN_NODE_TYPE_IMAGE, &query, sGeneratorVideo[ i ] ) );
				success( sContext.CreateAnyProductionTree( XN_NODE_TYPE_AUDIO, &query, sGeneratorAudio[ i ] ) );
				
				XnWaveOutputMode waveMode;
				waveMode.nSampleRate	= 44100;
				waveMode.nChannels		= 2;
				waveMode.nBitsPerSample	= 16;
				status = sGeneratorAudio[ i ].SetWaveOutputMode( waveMode );

			}

		   scriptNode.Release();

		}
		sContextInit = true;
	}

	DeviceRef Device::create()
	{
		return DeviceRef( new Device() );
	}

	Device::Device()
	{
		init();
	}

	Device::~Device()
	{
		if ( mCapture ) {
			stop();
		}
		mContext.Release();
		mGeneratorAudio.Release();
		mGeneratorDepth.Release();
		mGeneratorInfrared.Release();
		mGeneratorUser.Release();
		mGeneratorVideo.Release();
		mPlayer.Release();

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
		mEnabledAudio				= false;
		mEnabledDepth				= false;
		mEnabledInfrared			= false;
		mEnabledSkeletonTracking	= false;
		mEnabledUserTracking		= false;
		mEnabledVideo				= false;
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
		if ( success ) {
			trace( "Calibration successful for user ID: " + toString( id ) );
			mGeneratorUser.GetSkeletonCap().StartTracking( id );
		} else {
			trace( "Calibration failed for user ID: " + toString( id ) );
			if ( mCalibrationPoseRequired ) {
				mGeneratorUser.GetPoseDetectionCap().StartPoseDetection( mPoseStr, id );
			} else {
				mGeneratorUser.GetSkeletonCap().RequestCalibration( id, TRUE );
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
		if ( mCalibrationPoseRequired ) {
			mGeneratorUser.GetPoseDetectionCap().StartPoseDetection( mPoseStr, id );
		} else {
			mGeneratorUser.GetSkeletonCap().RequestCalibration( id, TRUE );
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
		mGeneratorUser.GetPoseDetectionCap().StopPoseDetection( id );
		mGeneratorUser.GetSkeletonCap().RequestCalibration( id, TRUE );
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

				XnStatus status = mContext.WaitOneUpdateAll( mGeneratorDepth );
				if ( success( status ) ) {
					
					if ( mEnabledAudio ) {
						mGeneratorAudio.GetMetaData( mMetaDataAudio );
						const uint_fast8_t* buffer = mGeneratorAudio.GetAudioBuffer();
						mDataAudioSize = mGeneratorAudio.GetDataSize();
						if ( mDataAudio == 0 ) {
							mDataAudio = new uint_fast8_t[ mDataAudioSize * 2 ]; // 2 = channels
						}
						memcpy( mDataAudio, buffer, mDataAudioSize );
						mNewAudio = true;
					}

					if ( mEnabledDepth ) {
						mGeneratorDepth.GetMetaData( mMetaDataDepth );
						mSizeDepth = Vec2i( mMetaDataDepth.XRes(), mMetaDataDepth.YRes() );
						uint32_t count = mSizeDepth.x * mSizeDepth.y;
						mDataDepth = (uint16_t*)mMetaDataDepth.Data();
						if ( !mChannelDepth ) {
							mChannelDepth = Channel16u( mSizeDepth.x, mSizeDepth.y );
						}
						memcpy( mChannelDepth.getData(), mDataDepth, count * mMetaDataDepth.BytesPerPixel() );
						mNewDepthFrame = true;
					}

					if ( mEnabledInfrared ) {
						mGeneratorInfrared.GetMetaData( mMetaDataInfrared );
						mSizeInfrared = Vec2i( mMetaDataInfrared.XRes(), mMetaDataInfrared.YRes() );
						uint32_t count = mSizeInfrared.x * mSizeInfrared.y;
						mDataInfrared = (uint16_t*)mMetaDataInfrared.Data();
						if ( !mChannelInfrared ) {
							mChannelInfrared = Channel16u( mSizeInfrared.x, mSizeInfrared.y );
						}
						memcpy( mChannelInfrared.getData(), mDataInfrared, count * mMetaDataInfrared.BytesPerPixel() );
						mNewInfraredFrame = true;
					}

					if ( mEnabledUserTracking && mGeneratorUser.IsNewDataAvailable() ) {
						mGeneratorUser.GetUserPixels( 0, mMetaDataScene );
						mSizeUserImage = Vec2i( mMetaDataScene.XRes(), mMetaDataScene.YRes() );
						uint32_t count = mSizeUserImage.x * mSizeUserImage.y;
						mDataUserImage = (uint16_t*)mMetaDataScene.Data();
						if ( !mChannelUserImage ) {
							mChannelUserImage = Channel16u( mSizeInfrared.x, mSizeInfrared.y );
						}
						memcpy( mChannelUserImage.getData(), mDataUserImage, count * mMetaDataScene.BytesPerPixel() );
						mNewUserData = true;
					}

					if ( mEnabledVideo ) {
						mGeneratorVideo.GetMetaData( mMetaDataVideo );
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

	void Device::start( size_t deviceIndex )
	{
		if ( mCapture ) {
			stop();
		}

		XnStatus status = XN_STATUS_OK;
		
		if ( !sContextInit ) {
			trace( "Configuration file not set. Please call Device::setConfigFile() before initializing a device." );
			return;
		}

		size_t index		= math<size_t>::min( deviceIndex, MAX_COUNT );
		
		mContext			= sContext;
		if ( sDevice[ index ].IsValid() != TRUE ) {
			return;
		}
		mDevice				= sDevice[ index ];
		mGeneratorAudio		= sGeneratorAudio[ index ];
		mGeneratorDepth		= sGeneratorDepth[ index ];
		mGeneratorInfrared	= sGeneratorInfrared[ index ];
		mGeneratorUser		= sGeneratorUser[ index ];
		mGeneratorVideo		= sGeneratorVideo[ index ];

		mEnabledAudio			= mGeneratorAudio.IsValid() == TRUE;
		mEnabledDepth			= mGeneratorDepth.IsValid() == TRUE;
		mEnabledInfrared		= mGeneratorInfrared.IsValid() == TRUE;
		mEnabledUserTracking	= mGeneratorUser.IsValid() == TRUE;
		mEnabledVideo			= mGeneratorVideo.IsValid() == TRUE;

		//mContext.StartGeneratingAll();

		mCapture	= true;
		mRunning	= true;
		mThread		= ThreadRef( new boost::thread( bind( &Device::run, this ) ) );
	}

	void Device::stop()
	{
		mContext.StopGeneratingAll();

		mRunning = false;
		if ( mThread ) {
			mThread->join();
		}
		
		mCapture = false;
		mSkeletons.clear();
	}

}
