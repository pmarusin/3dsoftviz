/**
 * Created by Patrik Berger on 19.11.2016.
 */
#ifndef BONE_H
#define BONE_H

#include "Leap/HandModule/HandNode.h"
#include "Leap/HandModule/Joint.h"

namespace Leap {
class HandBone : public HandNode
{
public:
    /**
     * Constructor of HandBone. Automatically adds itself to the given osg::Group, calls
     * generateGeometry().
     *
     * @param type - Value representing bone's type (position in the finger).
     * @param boneGroup - Group of bones representing a single finger.
     */
	HandBone( int type, osg::ref_ptr<osg::Group> boneGroup );


    /**
     * Method used in constructor to generate hierarchy of graph objects to visualize a single bone.
     * Geode -> ShapeDrawable -> Sphere
     *
     * @param radius - Radius of cylinder drawable in geode.
     * @param colorSwitch - Value representing colour of bone - not used in current implementation.
     */
	void generateGeometry( float radius, int colorSwitch );

	Joint* nextJoint;
	Joint* prevJoint;
	osg::ref_ptr<osg::Group> boneGroup;

private:
	//      TYPE_METACARPAL = 0,   /**< Bone connected to the wrist inside the palm */
	//      TYPE_PROXIMAL = 1,     /**< Bone connecting to the palm */
	//      TYPE_INTERMEDIATE = 2, /**< Bone between the tip and the base*/
	//      TYPE_DISTAL = 3,       /**< Bone at the tip of the finger */
	int type;
};
}

#endif // BONE_H
