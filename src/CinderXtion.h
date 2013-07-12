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

#pragma once

#include "boost/signals2.hpp"
#include "cinder/AxisAlignedBox.h"
#include "cinder/Channel.h"
#include "cinder/Plane.h"
#include "cinder/Quaternion.h"
#include "cinder/Surface.h"
#include "cinder/Vector.h"
#include "cinder/Utilities.h"
#include "OpenNI.h"
#include "Nite.h"
#include <map>

namespace Xtion
{
	ci::AxisAlignedBox3f	toAxisAlignedBox3f( const nite::Point3f& aMin, const nite::Point3f& aMax );
	ci::Planef				toPlanef( const nite::Point3f& point, const nite::Point3f& normal );
	ci::Quatf				toQuatf( const nite::Quaternion& q );
	ci::Vec3f				toVec3f( const nite::Point3f& v );

	template<typename T>
	std::vector<T>			toVector( const nite::Array<T>& a );

	ci::Channel8u			toChannel8u( const openni::VideoFrameRef& f );
	ci::Channel16u			toChannel16u( const openni::VideoFrameRef& f );
	ci::Surface8u			toSurface8u( const openni::VideoFrameRef& f );
	ci::Surface16u			toSurface16u( const openni::VideoFrameRef& f );
	
	class DeviceOptions
	{
	public:
		DeviceOptions();
		
		bool				isColorEnabled() const;
		bool				isDepthEnabled() const;
		bool				isHandTrackingEnabled() const;
		bool				isInfraredEnabled() const; 
		bool				isUserTrackingEnabled() const;

		float				getColorFrameRate() const;
		float				getDepthFrameRate() const;
		float				getInfraredFrameRate() const;
		
		const ci::Vec2i&	getColorSize() const; 
		const ci::Vec2i&	getDepthSize() const; 
		const ci::Vec2i&	getInfraredSize() const;
		
		DeviceOptions&		enableColor( bool enable = true );
		DeviceOptions&		enableDepth( bool enable = true );
		DeviceOptions&		enableHandTracking( bool enable = true );
		DeviceOptions&		enableInfrared( bool enable = true );
		DeviceOptions&		enableUserTracking( bool enable = true );

		DeviceOptions&		setColorFrameRate( float frameRate ); 
		DeviceOptions&		setDepthFrameRate( float frameRate ); 
		DeviceOptions&		setInfraredFrameRate( float frameRate );

		DeviceOptions&		setColorSize( const ci::Vec2i& size );
		DeviceOptions&		setDepthSize( const ci::Vec2i& size );
		DeviceOptions&		setInfraredSize( const ci::Vec2i& size );
	private:
		bool				mEnabledColor;
		bool				mEnabledDepth;
		bool				mEnabledHandTracking;
		bool				mEnabledInfrared;
		bool				mEnabledUserTracking;

		float				mFrameRateColor;
		float				mFrameRateDepth;
		float				mFrameRateInfrared;

		ci::Vec2i			mSizeColor;
		ci::Vec2i			mSizeDepth;
		ci::Vec2i			mSizeInfrared;
	};

	//////////////////////////////////////////////////////////////////////////////////////////////
	
	class Device;

	class HandTrackerListener : nite::HandTracker::NewFrameListener
	{
	public:
		void						onNewFrame( nite::HandTracker& tracker );
	private:
		typedef std::function<void ( nite::HandTrackerFrameRef )> EventHandler;
		
		HandTrackerListener( EventHandler eventHandler );

		EventHandler				mEventHandler;
		nite::HandTrackerFrameRef	mFrame;
		
		friend class				Device;
	};

	class UserTrackerListener : nite::UserTracker::NewFrameListener
	{
	public:
		void						onNewFrame( nite::UserTracker& tracker );
	private:
		typedef std::function<void ( nite::UserTrackerFrameRef )> EventHandler;
		
		UserTrackerListener( EventHandler eventHandler );

		EventHandler				mEventHandler;
		nite::UserTrackerFrameRef	mFrame;

		friend class				Device;
	};

	class VideoStreamListener : public openni::VideoStream::NewFrameListener
	{
	public:
		void					onNewFrame( openni::VideoStream& stream );
	private:
		typedef std::function<void ( openni::VideoFrameRef )> EventHandler;
		
		VideoStreamListener( EventHandler eventHandler );

		EventHandler			mEventHandler;
		openni::VideoFrameRef	mFrame;

		friend class			Device;
	};

	//////////////////////////////////////////////////////////////////////////////////////////////
	
	typedef std::shared_ptr<class Device>	DeviceRef;
	typedef std::map<int16_t, DeviceRef>	DeviceMap;

	class Device
	{
	public:
		static DeviceRef			create( const DeviceOptions& deviceOptions = DeviceOptions() );
		~Device();

		void						start();
		void						stop();

		const openni::Device&		getDevice() const;
		const openni::DeviceInfo&	getDeviceInfo() const;
		const DeviceOptions&		getDeviceOptions() const;
		
		template<typename T, typename Y>
		inline void					connectColorEventHandler( T callback, Y* callbackObject )
		{
			if ( mDeviceOptions.isColorEnabled() ) {
				mListenerColor = std::shared_ptr<VideoStreamListener>( new VideoStreamListener( std::bind( callback, callbackObject, std::placeholders::_1 ) ) );
				mStreamColor.addNewFrameListener( *mListenerColor );
			}
		}

		template<typename T, typename Y>
		inline void					connectDepthEventHandler( T callback, Y* callbackObject )
		{
			if ( mDeviceOptions.isDepthEnabled() ) {
				mListenerDepth = std::shared_ptr<VideoStreamListener>( new VideoStreamListener( std::bind( callback, callbackObject, std::placeholders::_1 ) ) );
				mStreamDepth.addNewFrameListener( *mListenerDepth );
			}
		}

		template<typename T, typename Y>
		inline void					connectHandEventHandler( T callback, Y* callbackObject )
		{
			if ( mDeviceOptions.isHandTrackingEnabled() ) {
				mListenerHand = std::shared_ptr<HandTrackerListener>( new HandTrackerListener( std::bind( callback, callbackObject, std::placeholders::_1 ) ) );
				mTrackerHand.addNewFrameListener( *mListenerHand );
			}
		}

		template<typename T, typename Y>
		inline void					connectInfraredEventHandler( T callback, Y* callbackObject )
		{
			if ( mDeviceOptions.isInfraredEnabled() ) {
				mListenerInfrared = std::shared_ptr<VideoStreamListener>( new VideoStreamListener( std::bind( callback, callbackObject, std::placeholders::_1 ) ) );
				mStreamInfrared.addNewFrameListener( *mListenerInfrared );
			}
		}

		template<typename T, typename Y>
		inline void					connectUserEventHandler( T callback, Y* callbackObject )
		{
			if ( mDeviceOptions.isUserTrackingEnabled() ) {
				mListenerUser = std::shared_ptr<UserTrackerListener>( new UserTrackerListener( std::bind( callback, callbackObject, std::placeholders::_1 ) ) );
				mTrackerUser.addNewFrameListener( *mListenerUser );
			}
		}
	private:
		Device( const DeviceOptions& deviceOptions );

		openni::Device							mDevice;
		openni::DeviceInfo						mDeviceInfo;

		DeviceOptions							mDeviceOptions;

		openni::VideoStream						mStreamColor;
		openni::VideoStream						mStreamDepth;
		nite::HandTracker						mTrackerHand;
		openni::VideoStream						mStreamInfrared;
		nite::UserTracker						mTrackerUser;

		std::shared_ptr<VideoStreamListener>	mListenerColor;
		std::shared_ptr<VideoStreamListener>	mListenerDepth;
		std::shared_ptr<HandTrackerListener>	mListenerHand;
		std::shared_ptr<VideoStreamListener>	mListenerInfrared;
		std::shared_ptr<UserTrackerListener>	mListenerUser;
	};

}
