#include "LeapLib/LeapCameraStream.h"

Leap::LeapCameraStream::LeapCameraStream( ) :
    osg::Image()
{
    updateBackgroundImage( nullptr );
}

Leap::LeapCameraStream::~LeapCameraStream() {}

void Leap::LeapCameraStream::updateBackgroundImage( unsigned char* buffer )
{
    setImage(640, 480, 1, GL_RGB, GL_RGB, GL_UNSIGNED_BYTE,
                                  buffer, osg::Image::USE_NEW_DELETE);
//    dirty();
}


