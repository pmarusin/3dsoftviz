#include "LeapLib/LeapCameraStream.h"

Leap::LeapCameraStream::LeapCameraStream( osg::Geometry* geom ) :
    osg::Image(),

    mWidth( 0 ),
    mHeight( 0 ),
    mGeom( geom )
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

void Leap::LeapCameraStream::updateGeometryCoords( int width, int height )
{
    // TODO asi zmazat
    float x;
    x = static_cast<float>( width ) / static_cast<float>( height );

    osg::Vec3Array* coords = static_cast<osg::Vec3Array*>( mGeom->getVertexArray() );
    ( *coords )[0].set( -x, 1.5f, -1.0f );
    ( *coords )[1].set( x, 1.5f, -1.0f );
    ( *coords )[2].set( x, 1.5f,  1.0f );
    ( *coords )[3].set( -x, 1.5f,  1.0f );

    mGeom->dirtyDisplayList();  // update changes

}

