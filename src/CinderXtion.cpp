/*
* 
* Copyright (c) 2013, Ban the Rewind
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

	AxisAlignedBox3f toAxisAlignedBox3f( const nite::Point3f& aMin, const nite::Point3f& aMax )
	{
		return AxisAlignedBox3f( toVec3f( aMin ), toVec3f( aMax ) );
	}

	Planef toPlanef( const nite::Point3f& point, const nite::Point3f& normal )
	{
		return Planef( toVec3f( point ), toVec3f( normal ) );
	}

	Quatf toQuatf( const nite::Quaternion& q )
	{
		return Quatf( q.w, q.x, q.y, q.z );
	}

	Vec3f toVec3f( const nite::Point3f& v )
	{
		return Vec3f( v.x, v.y, v.z );
	}

	template<typename T>
	vector<T> toVector( const nite::Array<T>& a )
	{
		vector<T> v;
		v.insert( v.end(), &a[ 0 ], &a[ a.getSize() / sizeof( T ) ]);
		return v;
	}

	Channel8u toChannel8u( const openni::VideoFrameRef& f )
	{
		return Channel8u( f.getWidth(), f.getHeight(), f.getStrideInBytes(), 1, (uint8_t*)f.getData() );
	}
	
	Channel16u toChannel16u( const openni::VideoFrameRef& f )
	{
		return Channel16u( f.getWidth(), f.getHeight(), f.getStrideInBytes(), 1, (uint16_t*)f.getData() );
	}

	Surface8u toSurface8u( const openni::VideoFrameRef& f )
	{
		return Surface8u( (uint8_t*)f.getData(), f.getWidth(), f.getHeight(), f.getStrideInBytes(), SurfaceChannelOrder::RGB );
	}
	
	Surface16u toSurface16u( const openni::VideoFrameRef& f )
	{
		return Surface16u( (uint16_t*)f.getData(), f.getWidth(), f.getHeight(), f.getStrideInBytes(), SurfaceChannelOrder::RGB );
	}

	//////////////////////////////////////////////////////////////////////////////////////////////

	DeviceOptions::DeviceOptions()
	{
		enableColor( true );
		enableDepth( true );
		enableHandTracking( false );
		enableInfrared( false );
		enableUserTracking( false );

		setColorFrameRate( 30.0f );
		setDepthFrameRate( 30.0f );
		setInfraredFrameRate( 30.0f );
		
		setColorSize( Vec2i( 320, 240 ) );
		setDepthSize( Vec2i( 320, 240 ) );
		setInfraredSize( Vec2i( 320, 240 ) );
	}

	bool DeviceOptions::isColorEnabled() const
	{
		return mEnabledColor;
	}

	bool DeviceOptions::isDepthEnabled() const
	{
		return mEnabledDepth;
	}

	bool DeviceOptions::isHandTrackingEnabled() const
	{
		return mEnabledHandTracking;
	}

	bool DeviceOptions::isInfraredEnabled() const
	{
		return mEnabledInfrared;
	}

	bool DeviceOptions::isUserTrackingEnabled() const
	{
		return mEnabledUserTracking;
	}

	float DeviceOptions::getColorFrameRate() const
	{
		return mFrameRateColor;
	}

	float DeviceOptions::getDepthFrameRate() const
	{
		return mFrameRateDepth;
	}
 
	float DeviceOptions::getInfraredFrameRate() const
	{
		return mFrameRateInfrared;
	}

	const Vec2i& DeviceOptions::getColorSize() const
	{
		return mSizeColor;
	}

	const Vec2i& DeviceOptions::getDepthSize() const
	{
		return mSizeDepth;
	}

	const Vec2i& DeviceOptions::getInfraredSize() const
	{
		return mSizeInfrared;
	}

	DeviceOptions& DeviceOptions::enableColor( bool enable )
	{
		mEnabledColor = enable;
		return *this;
	}

	DeviceOptions& DeviceOptions::enableDepth( bool enable )
	{
		mEnabledDepth = enable;
		return *this;
	}

	DeviceOptions& DeviceOptions::enableHandTracking( bool enable )
	{
		mEnabledHandTracking = enable;
		return *this;
	}

	DeviceOptions& DeviceOptions::enableInfrared( bool enable )
	{
		mEnabledInfrared = enable;
		return *this;
	}

	DeviceOptions& DeviceOptions::enableUserTracking( bool enable )
	{
		mEnabledUserTracking = enable;
		return *this;
	}

	DeviceOptions& DeviceOptions::setColorFrameRate( float frameRate )
	{
		mFrameRateColor = frameRate;
		return *this;
	}

	DeviceOptions& DeviceOptions::setDepthFrameRate( float frameRate )
	{
		mFrameRateDepth = frameRate;
		return *this;
	}

	DeviceOptions& DeviceOptions::setInfraredFrameRate( float frameRate )
	{
		mFrameRateInfrared = frameRate;
		return *this;
	}

	DeviceOptions& DeviceOptions::setColorSize( const Vec2i& size )
	{
		mSizeColor = size;
		return *this;
	}

	DeviceOptions& DeviceOptions::setDepthSize( const Vec2i& size )
	{
		mSizeDepth = size;
		return *this;
	}

	DeviceOptions& DeviceOptions::setInfraredSize( const Vec2i& size )
	{
		mSizeInfrared = size;
		return *this;
	}

	//////////////////////////////////////////////////////////////////////////////////////////////
	
	HandTrackerListener::HandTrackerListener( HandTrackerListener::EventHandler eventHandler )
		: nite::HandTracker::NewFrameListener(), mEventHandler( eventHandler )
	{
	}
	
	UserTrackerListener::UserTrackerListener( UserTrackerListener::EventHandler eventHandler )
		: nite::UserTracker::NewFrameListener(), mEventHandler( eventHandler )
	{
	}

	VideoStreamListener::VideoStreamListener( VideoStreamListener::EventHandler eventHandler )
		: openni::VideoStream::NewFrameListener(), mEventHandler( eventHandler )
	{
	}

	void HandTrackerListener::onNewFrame( nite::HandTracker& tracker )
	{
		tracker.readFrame( &mFrame );
		mEventHandler( mFrame );
	}
	
	void UserTrackerListener::onNewFrame( nite::UserTracker& tracker )
	{
		tracker.readFrame( &mFrame );
		mEventHandler( mFrame );
	}

	void VideoStreamListener::onNewFrame( openni::VideoStream& stream ) 
	{
		stream.readFrame( &mFrame );
		mEventHandler( mFrame );
	}

	//////////////////////////////////////////////////////////////////////////////////////////////

	DeviceRef Device::create( const DeviceOptions& deviceOptions )
	{
		return DeviceRef( new Device( deviceOptions ) );
	}

	Device::Device( const DeviceOptions& deviceOptions )
	{
		mDeviceOptions = deviceOptions;
	}

	Device::~Device()
	{
	}

	void Device::start()
	{
		if ( mDeviceOptions.isColorEnabled() ) {
			if ( mStreamColor.create( mDevice, openni::SENSOR_COLOR ) != openni::STATUS_OK ) {
				mDeviceOptions.enableColor( false );
			}
		}

		if ( mDeviceOptions.isDepthEnabled() ) {
			if ( mStreamDepth.create( mDevice, openni::SENSOR_DEPTH ) != openni::STATUS_OK ) {
				mDeviceOptions.enableDepth( false );
			}
		}

		if ( mDeviceOptions.isHandTrackingEnabled() ) {
			if ( mTrackerHand.create( &mDevice ) != nite::STATUS_OK ) {
				mDeviceOptions.enableHandTracking( false );
			}
		}

		if ( mDeviceOptions.isInfraredEnabled() ) {
			if ( mStreamInfrared.create( mDevice, openni::SENSOR_IR ) != openni::STATUS_OK ) {
				mDeviceOptions.enableInfrared( false );
			}
		}

		if ( mDeviceOptions.isUserTrackingEnabled() ) {
			if ( mTrackerUser.create( &mDevice ) != nite::STATUS_OK ) {
				mDeviceOptions.enableUserTracking( false );
			}
		}
	}

	void Device::stop()
	{

	}
	
	const openni::Device& Device::getDevice() const
	{
		return mDevice;
	}

	const openni::DeviceInfo& Device::getDeviceInfo() const
	{
		return mDeviceInfo;
	}

	const DeviceOptions& Device::getDeviceOptions() const
	{
		return mDeviceOptions;
	}

}
