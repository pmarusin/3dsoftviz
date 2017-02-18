#ifndef LEAPCAMERASTREAM_H
#define LEAPCAMERASTREAM_H

#include <osg/Image>
#include <osg/Geometry>

namespace Leap {

class LeapCameraStream : public osg::Image
{

public:

    /**
        * @author Patrik Berger
        * @brief CameraStream constructor
        * @param geom Geometry for updating it's vertexArray according ratio of image
        */
    LeapCameraStream( osg::Geometry* geom = NULL );
    ~LeapCameraStream();

public:
    void updateBackgroundImage( unsigned char* buffer );


private:

    /**
        * @brief updateGeometryCoords Update mGeom's vertexArray according ratio of image, given by width and height
        * @param width image' width
        * @param height image' height
        */
    void updateGeometryCoords( int width, int height );

    int				mWidth; // data about cols cv:Mat
    int				mHeight; // data about rows cv:Mat
    osg::Geometry*	mGeom; // Geometry for vertex array and update
};
}
#endif
