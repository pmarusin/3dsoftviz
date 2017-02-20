#include "LeapLib/LeapCameraStream.h"
#include <easylogging++.h>
#include<string>

Leap::LeapCameraStream::LeapCameraStream( ) :
    osg::Image()
{
//    updateBackgroundImage( nullptr );
    setImage(640, 240, 1, GL_RGB, GL_BGR, GL_UNSIGNED_BYTE,
                                        nullptr,osg::Image::NO_DELETE, 1 );
}

Leap::LeapCameraStream::~LeapCameraStream() {}

void Leap::LeapCameraStream::updateBackgroundImage( unsigned char* buffer )
{
    LOG( INFO) << "Set image pred";


    try{

//            osg::ref_ptr<Leap::LeapCameraStream> osgImage = new Leap::LeapCameraStream;
//            *this = osgImage;
             setData(buffer, osg::Image::NO_DELETE);
//            this->dirty();
//        }

    }catch(const std::exception&){
        LOG (INFO) << "Catch";
    }
//    }

    LOG( INFO) << "Set image za. Pocet: " + std::to_string(getModifiedCount ());

}


