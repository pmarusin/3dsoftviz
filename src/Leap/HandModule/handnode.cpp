/**
 * Created by Patrik Berger on 13.11.2016.
 */
#include <easylogging++.h>
#include "Leap/HandModule/HandNode.h"

Leap::HandNode::HandNode() {}

void Leap::HandNode::initStructure() {}

void Leap::HandNode::generateGeometry( float radius, int colorSwitch ) {}


void Leap::HandNode::setColor( int colorSwitch, osg::ref_ptr<osg::ShapeDrawable> handDrawable )
{
    osg::Vec4f blue = osg::Vec4f( 21.0/255.0,51.0/255.0,252.0/255.0,1.0 );
    osg::Vec4f green = osg::Vec4f( 23.0/255.0,190.0/255.0,40.0/255.0,1.0 );

	// setting color
	switch ( colorSwitch ) {
		case 1:
            handDrawable.get()->setColor( green );
			break;
		case 2:
            handDrawable.get()->setColor( blue );
			break;
		default:
			break;
	}
}
