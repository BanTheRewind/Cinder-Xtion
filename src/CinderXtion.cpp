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

	Bone::Bone( const Vec3f& position, const Quatf& rotation )
		: mPosition( position ), mRotation( rotation )
	{
	}

	const Vec3f& Bone::getPosition() const
	{
		return mPosition;
	}

	const Quatf& Bone::getRotation() const
	{
		return mRotation;
	}

	//////////////////////////////////////////////////////////////////////////////////////////////

	DeviceOptions::DeviceOptions()
	{
		setDeviceId( "" );
		setDeviceIndex( 0 );

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

	DeviceOptions& DeviceOptions::enableInfrared( bool enable )
	{
		mEnabledInfrared = enable;
		return *this;
	}

	DeviceOptions& DeviceOptions::enableHandTracking( bool enable )
	{
		mEnabledHandTracking = enable;
		return *this;
	}

	DeviceOptions& DeviceOptions::enableUserTracking( bool enable )
	{
		mEnabledUserTracking		= enable;
		return *this;
	}

	float DeviceOptions::getDepthFrameRate() const
	{
		return mFrameRateDepth;
	}
 
	const Vec2i& DeviceOptions::getDepthSize() const
	{
		return mSizeDepth;
	}

	const std::string& DeviceOptions::getDeviceId() const
	{
		return mDeviceId;
	}

	int32_t DeviceOptions::getDeviceIndex() const
	{
		return mDeviceIndex;
	}

	float DeviceOptions::getInfraredFrameRate() const
	{
		return mFrameRateInfrared;
	}

	const Vec2i& DeviceOptions::getInfraredSize() const
	{
		return mSizeInfrared;
	}

	float  DeviceOptions::getColorFrameRate() const
	{
		return mFrameRateColor;
	}

	const Vec2i& DeviceOptions::getColorSize() const
	{
		return mSizeColor;
	}

	bool DeviceOptions::isColorEnabled() const
	{
		return mEnabledColor;
	}

	bool DeviceOptions::isDepthEnabled() const
	{
		return mEnabledDepth;
	}

	bool DeviceOptions::isInfraredEnabled() const
	{
		return mEnabledInfrared;
	}

	bool DeviceOptions::isUserTrackingEnabled() const
	{
		return mEnabledUserTracking;
	}

	DeviceOptions& DeviceOptions::setDepthFrameRate( float frameRate )
	{
		mFrameRateDepth = frameRate;
		return *this;
	}

	DeviceOptions& DeviceOptions::setDepthSize( const Vec2i& size )
	{
		mSizeDepth = size;
		return *this;
	}

	DeviceOptions& DeviceOptions::setDeviceId( const string& id )
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

	DeviceOptions& DeviceOptions::setInfraredSize( const Vec2i& size )
	{
		mSizeInfrared = size;
		return *this;
	}
	
	DeviceOptions& DeviceOptions::setColorFrameRate( float frameRate )
	{
		mFrameRateColor = frameRate;
		return *this;
	}

	DeviceOptions& DeviceOptions::setColorSize( const Vec2i& size )
	{
		mSizeColor = size;
		return *this;
	}

	//////////////////////////////////////////////////////////////////////////////////////////////

	void ColorListener::onNewFrame( openni::VideoStream& stream ) 
	{
		stream.readFrame( &mFrame );
	}

	void DepthListener::onNewFrame( openni::VideoStream& stream ) 
	{
		stream.readFrame( &mFrame );
	}

	void HandListener::onNewFrame( nite::HandTracker& tracker )
	{
		tracker.readFrame( &mFrame );
		// TODO parse and broadcast hand data
	}
	
	void InfraredListener::onNewFrame( openni::VideoStream& stream ) 
	{
		stream.readFrame( &mFrame );
	}

	void UserListener::onNewFrame( nite::UserTracker& tracker )
	{
		tracker.readFrame( &mFrame );
		// TODO parse and broadcast user data
	}

	//////////////////////////////////////////////////////////////////////////////////////////////

	DeviceRef Device::create()
	{
		return DeviceRef( new Device() );
	}

	Device::Device()
	{
	}

	Device::~Device()
	{
	}

	void Device::start( const DeviceOptions& deviceOptions )
	{
		mDeviceOptions = deviceOptions;
	}

}
