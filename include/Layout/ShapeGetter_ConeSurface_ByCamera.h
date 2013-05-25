#ifndef Layout_ShapeGetter_ConeSurface_ByCamera_H
#define Layout_ShapeGetter_ConeSurface_ByCamera_H

#include "Layout/ShapeGetter.h"

#include <QSlider>

#include "Data/Node.h"
#include "QOSG/ViewerQT.h"

namespace Layout {

/**
 * \brief Returning a cone surface.
 * Center is defined by the centerNode position.
 * Radius is defined by the surfaceNode position (the radius is computed in a way that
 * surfaceNode will be placed on the surface of the resulting sphere surface).
 */
class ShapeGetter_ConeSurface_ByCamera : public ShapeGetter
{
public:
    ShapeGetter_ConeSurface_ByCamera (const QOSG::ViewerQT *viewerWidget, const QSlider &baseRadiusSlider);

    virtual ~ShapeGetter_ConeSurface_ByCamera() {}

    virtual QSharedPointer<Shape> getShape();

private:
    const QOSG::ViewerQT *mViewerWidget;
    const QSlider &mBaseRadiusSlider;
};

} // namespace

#endif // Layout_ShapeGetter_ConeSurface_ByCamera_H
