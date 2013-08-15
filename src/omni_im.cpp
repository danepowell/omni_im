#include "omni_im.h"
#include <boost/shared_ptr.hpp>
#include "rviz/default_plugin/interactive_markers/interactive_marker_control.h"
#include <OGRE/OgreSceneQuery.h>

namespace rviz
{

class MySceneQueryListener : public Ogre::SceneQueryListener
{
};

OmniIMDisplay::OmniIMDisplay()
{
}

OmniIMDisplay::~OmniIMDisplay()
{
}

void OmniIMDisplay::onInitialize()
{
  //  boost::shared_ptr<InteractiveMarkerControl> control;
  //InteractiveObjectWPtr ptr;
  //getActiveControl(ptr, control);
}

void OmniIMDisplay::getActiveControl(InteractiveObjectWPtr& ptr, boost::shared_ptr<InteractiveMarkerControl>& control)
{
  //  ptr = grabbed_object_;
  //if(!ptr.expired())
  //{
    //control = boost::dynamic_pointer_cast<InteractiveMarkerControl>(ptr.lock());
  //}
}
}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(rviz::OmniIMDisplay, rviz::Display)
