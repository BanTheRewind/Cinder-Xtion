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

// Includes
#include "cinder/app/AppBasic.h"
#include "cinder/Arcball.h"
#include "cinder/Camera.h"
#include "cinder/gl/gl.h"
#include "cinder/gl/Texture.h"
#include "cinder/ImageIo.h"
#include "cinder/Utilities.h"
#include "CinderXtion.h"

/* 
* This application demonstrates how to represent the 
* Xtion's depth image in 3D space.
*/
class PointCloudApp : public ci::app::AppBasic 
{

public:

	// Cinder callbacks
	void draw();
	void keyDown( ci::app::KeyEvent event );
	void mouseDown( ci::app::MouseEvent event );
	void mouseDrag( ci::app::MouseEvent event );
	void prepareSettings( ci::app::AppBasic::Settings * settings );
	void shutdown();
	void setup();
	void update();

private:

	// Xtion
	Xtion::DeviceRef		mDevice;
	ci::Vec2i				mInputSize;

	// Depth points
	std::vector<ci::Vec3f>	mPoints;

	// Camera
	ci::Arcball				mArcball;
	ci::CameraPersp			mCamera;

	ci::Channel16u			mDepth;
	ci::Channel8u			mInfrared;
	ci::Surface8u			mVideo;

	// Save screen shot
	void					screenShot();

};

// Imports
using namespace ci;
using namespace ci::app;
using namespace Xtion;
using namespace std;

// Render
void PointCloudApp::draw()
{

	// Clear window
	gl::setViewport( getWindowBounds() );
	gl::clear( Colorf::black() );

	gl::setMatricesWindow( getWindowSize() );
	gl::color( ColorAf::white() );
	if ( mDepth ) {
		gl::draw( gl::Texture( mDepth ), mDepth.getBounds(), Rectf( 0.0f, 0.0f, (float)getWindowWidth() * 0.5f, (float)getWindowHeight() * 0.5f ) );
	}
	if ( mInfrared ) {
		gl::draw( gl::Texture( mInfrared ), mInfrared.getBounds(), Rectf( (float)getWindowWidth() * 0.5f, 0.0f, (float)getWindowWidth(), (float)getWindowHeight() * 0.5f ) );
	}
	if ( mVideo ) {
		gl::draw( gl::Texture( mVideo ), mVideo.getBounds(), Rectf( 0.0f, (float)getWindowHeight() * 0.5f, (float)getWindowWidth() * 0.5f, (float)getWindowHeight() ) );
	}
	gl::setMatrices( mCamera );
	gl::rotate( mArcball.getQuat() );

	// Draw point cloud
	glBegin( GL_POINTS );
	for ( vector<Vec3f>::const_iterator pointIt = mPoints.cbegin(); pointIt != mPoints.cend(); ++pointIt ) {
		float depth = 1.0f - pointIt->z / ( mCamera.getEyePoint().z * -2.0f );
		gl::color( ColorAf( 1.0f, depth, 1.0f - depth, depth ) );
		gl::vertex( *pointIt );
	}
	glEnd();

}

// Handles key press
void PointCloudApp::keyDown( KeyEvent event )
{
	switch ( event.getCode() ) {
	case KeyEvent::KEY_ESCAPE:
		quit();
		break;
	case KeyEvent::KEY_f:
		setFullScreen( !isFullScreen() );
		break;
	case KeyEvent::KEY_s:
		screenShot();
		break;
	}
}

void PointCloudApp::mouseDown( ci::app::MouseEvent event )
{
	mArcball.mouseDown( event.getPos() );
}

void PointCloudApp::mouseDrag( ci::app::MouseEvent event )
{
	mArcball.mouseDrag( event.getPos() );
}

// Prepare window
void PointCloudApp::prepareSettings( Settings * settings )
{
	settings->setWindowSize( 800, 600 );
	settings->setFrameRate( 60.0f );
}

// Take screen shot
void PointCloudApp::screenShot()
{
	writeImage( getAppPath() / fs::path( "frame" + toString( getElapsedFrames() ) + ".png" ), copyWindowSurface() );
}

// Set up
void PointCloudApp::setup()
{

	// Set up OpenGL
	gl::enable( GL_DEPTH_TEST );
	glHint( GL_POINT_SMOOTH_HINT, GL_NICEST );
	glEnable( GL_POINT_SMOOTH );
	glPointSize( 0.25f );
	gl::enableAlphaBlending();
	gl::enableAdditiveBlending();
	gl::color( ColorAf::white() );

	mInputSize = Vec2i( 640, 480 );

	// Start Xtion
	mDevice = Device::create();
	mDevice->start( getAppPath() / "data/config.xml" );

	// Set up camera
	mArcball = Arcball( getWindowSize() );
	mArcball.setRadius( (float)getWindowHeight() );
	mCamera.lookAt( Vec3f( 0.0f, 0.0f, 670.0f ), Vec3f::zero() );
	mCamera.setPerspective( 60.0f, getWindowAspectRatio(), 0.01f, 5000.0f );

}

// Called on exit
void PointCloudApp::shutdown()
{
	mDevice->stop();
	mPoints.clear();
}

// Runs update logic
void PointCloudApp::update()
{

	// Device is capturing
	if ( mDevice->isCapturing() ) {

		if ( mDevice->checkNewInfraredFrame() ) {
			mInfrared = mDevice->getInfrared();
		}

		if ( mDevice->checkNewVideoFrame() ) {
			mVideo = mDevice->getVideo();
		}

		// Check for latest depth map
		if ( mDevice->checkNewDepthFrame() ) {
			mDepth = mDevice->getDepth();
			
			return;
			
			// Clear point list
			Vec3f offset( Vec2f( mInputSize ) * Vec2f( -0.5f, 0.5f ) );
			offset.z = mCamera.getEyePoint().z * 0.5f;
			Vec3f position = Vec3f::zero();
			mPoints.clear();

			// Iterate image rows
			for ( int32_t y = 0; y < mInputSize.y; y++ ) {
				for ( int32_t x = 0; x < mInputSize.x; x++ ) {

					// Read depth as 0.0 - 1.0 float
					float depth = mDevice->getDepthAt( Vec2i( x, y ) );

					// Add position to point list
					if ( depth > 0.0f ) {
						position.z = depth * mCamera.getEyePoint().z * 2.0f;
						mPoints.push_back( position * Vec3f( 1.1f, -1.1f, 1.0f ) + offset );
					}

					// Shift point
					position.x++;

				}

				// Update position
				position.x = 0.0f;
				position.y++;

			}

		}

	}

}

// Run application
CINDER_APP_BASIC( PointCloudApp, RendererGl )
