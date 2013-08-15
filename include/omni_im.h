#ifndef RVIZ_OMNI_IM_DISPLAY_H
#define RVIZ_OMNI_IM_DISPLAY_H

#include "rviz/display.h"
#include <QObject>
#include <boost/shared_ptr.hpp>
#include "rviz/default_plugin/interactive_markers/interactive_marker_control.h"

namespace rviz
{

class MySceneQueryListener;

class OmniIMDisplay: public Display
{
Q_OBJECT
public:
 OmniIMDisplay();
 virtual ~OmniIMDisplay();

protected:
 virtual void onInitialize();
 void getActiveControl(InteractiveObjectWPtr& ptr, boost::shared_ptr<InteractiveMarkerControl> & control);
};

}
#endif
