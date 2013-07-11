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
#include "cinder/Quaternion.h"
#include "cinder/Surface.h"
#include "cinder/Vector.h"
#include "cinder/Utilities.h"
#include "OpenNI.h"
#include "Nite.h"
#include <map>

namespace Xtion
{

	typedef std::shared_ptr<class Device>			DeviceRef;
	typedef std::map<nite::JointType, class Bone>	Skeleton;
	
	class Device;
	class Listener;

	//////////////////////////////////////////////////////////////////////////////////////////////

	class Hand
	{
	public:
		const ci::Vec3f&	getPosition() const;
	private:
		Hand( const ci::Vec3f& position );

		ci::Vec3f			mPosition;

		friend class		Listener;
	};

	typedef std::map<int16_t, Hand>					HandMap;

	class Gesture
	{
	public:
		const ci::Vec3f&	getPosition() const;
	private:
		Gesture( const ci::Vec3f& position );

		ci::Vec3f			mPosition;

		friend class		Listener;
	};

	typedef std::map<nite::GestureType, Gesture>	GestureMap;

	class HandFrame
	{
	public:
		const GestureMap&	getGestures() const;
		const HandMap&		getHands() const;
	private:
		HandFrame( const HandMap& hands, const GestureMap& gestures );

		GestureMap			mGestures;
		HandMap				mHands;
	};

	//////////////////////////////////////////////////////////////////////////////////////////////

	class Bone
	{
	public:
		const ci::Vec3f&	getPosition() const;
		const ci::Quatf&	getRotation() const;
	private:
		Bone( const ci::Vec3f& position, const ci::Quatf& rotation );
		
		ci::Vec3f			mPosition;
		ci::Quatf			mRotation;

		friend class		Listener;
	};

	//////////////////////////////////////////////////////////////////////////////////////////////

	class User
	{
		const ci::AxisAlignedBox3f&	getBounds() const;
		int16_t						getId() const;
		const ci::Vec3f&			getPosition() const;
		const Skeleton&				getSkeleton() const;
	private:
		ci::AxisAlignedBox3f		mBounds;
		int16_t						mId;
		ci::Vec3f					mPosition;
		Skeleton					mSkeleton;

		friend class				Listener;
	};

	typedef std::map<int16_t, User>	UserMap;

	//////////////////////////////////////////////////////////////////////////////////////////////

	class DeviceOptions
	{
	public:
		DeviceOptions();
		
		bool				isColorEnabled() const;
		bool				isDepthEnabled() const;
		bool				isHandTrackingEnabled() const;
		bool				isInfraredEnabled() const; 
		bool				isUserTrackingEnabled() const;

		const std::string&	getDeviceId() const;
		int32_t				getDeviceIndex() const;

		float				getColorFrameRate() const;
		float				getDepthFrameRate() const;
		float				getInfraredFrameRate() const;
		
		const ci::Vec2i&	getColorSize() const; 
		const ci::Vec2i&	getDepthSize() const; 
		const ci::Vec2i&	getInfraredSize() const;
		
		DeviceOptions&		enableDepth( bool enable = true );
		DeviceOptions&		enableHandTracking( bool enable = true );
		DeviceOptions&		enableInfrared( bool enable = true );
		DeviceOptions&		enableUserTracking( bool enable = true );
		DeviceOptions&		enableColor( bool enable = true );

		DeviceOptions&		setDeviceId( const std::string& id ); 
		DeviceOptions&		setDeviceIndex( int32_t index ); 

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
		
		std::string			mDeviceId;
		int32_t				mDeviceIndex;
	};

	//////////////////////////////////////////////////////////////////////////////////////////////
	
	class ColorListener : public openni::VideoStream::NewFrameListener
	{
	public:
		void						onNewFrame( openni::VideoStream& stream );

		template<typename T, typename Y>
		inline void connectEventHandler( T callback, Y* callbackObject )
		{
			mSignal.connect( std::bind( callback, callbackObject, std::placeholders::_1 ) ) );
		}
	private:
		openni::VideoFrameRef							mFrame;
		boost::signals2::signal<void ( ci::Surface8u )>	mSignal;

		friend class									Device;
	};

	class DepthListener : public openni::VideoStream::NewFrameListener
	{
	public:
		void						onNewFrame( openni::VideoStream& stream );

		template<typename T, typename Y>
		inline void connectEventHandler( T callback, Y* callbackObject )
		{
			mSignal.connect( std::bind( callback, callbackObject, std::placeholders::_1 ) ) );
		}
	private:
		openni::VideoFrameRef							mFrame;
		boost::signals2::signal<void ( ci::Channel8u )>	mSignal;

		friend class									Device;
	};

	class HandListener : nite::HandTracker::NewFrameListener
	{
	public:
		void						onNewFrame( nite::HandTracker& tracker );
	private:
		nite::HandTrackerFrameRef					mFrame;
		boost::signals2::signal<void ( HandFrame )>	mSignal;

		friend class				Device;
	};

	class InfraredListener : public openni::VideoStream::NewFrameListener
	{
	public:
		void						onNewFrame( openni::VideoStream& stream );

		template<typename T, typename Y>
		inline void connectEventHandler( T callback, Y* callbackObject )
		{
			mSignal.connect( std::bind( callback, callbackObject, std::placeholders::_1 ) ) );
		}
	private:
		openni::VideoFrameRef								mFrame;
		boost::signals2::signal<void ( ci::Channel16u )>	mSignal;

		friend class										Device;
	};

	class UserListener : nite::UserTracker::NewFrameListener
	{
	public:
		void						onNewFrame( nite::UserTracker& tracker );

		template<typename T, typename Y>
		inline void connectEventHandler( T callback, Y* callbackObject )
		{
			mSignal.connect( std::bind( callback, callbackObject, std::placeholders::_1 ) ) );
		}
	private:
		nite::UserTrackerFrameRef					mFrame;
		boost::signals2::signal<void ( UserMap )>	mSignal;

		friend class								Device;
	};

	//////////////////////////////////////////////////////////////////////////////////////////////

	class Device
	{
	public:
		
		static DeviceRef				create();
		~Device();

		void							start( const DeviceOptions& deviceOptions = DeviceOptions() );

		const DeviceOptions&			getDeviceOptions() const;

		template<typename T, typename Y>
		inline void connectColorEventHandler( T callback, Y* callbackObject )
		{
			mListenerColor.connectEventHandler( callback, callbackObject );
		}

		template<typename T, typename Y>
		inline void connectDepthEventHandler( T callback, Y* callbackObject )
		{
			mListenerDepth.connectEventHandler( callback, callbackObject );
		}

		template<typename T, typename Y>
		inline void connectHandEventHandler( T callback, Y* callbackObject )
		{
			mListenerHand.connectEventHandler( callback, callbackObject );
		}

		template<typename T, typename Y>
		inline void connectInfraredEventHandler( T callback, Y* callbackObject )
		{
			mListenerInfrared.connectEventHandler( callback, callbackObject );
		}

		template<typename T, typename Y>
		inline void connectUserEventHandler( T callback, Y* callbackObject )
		{
			mListenerUser.connectEventHandler( callback, callbackObject );
		}
	private:
		Device();

		//////////////////////////////////////////////////////////////////////////////////////////////

		DeviceOptions					mDeviceOptions;

		ColorListener					mListenerColor;
		DepthListener					mListenerDepth;
		HandListener					mListenerHand;
		InfraredListener				mListenerInfrared;
		UserListener					mListenerUser;
	};

}
