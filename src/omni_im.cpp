#include "omni_im.h"
#include <boost/shared_ptr.hpp>
#include "rviz/default_plugin/interactive_markers/interactive_marker_control.h"
#include "rviz/interactive_object.h"

namespace omni_im
{

OmniIMTool::OmniIMTool()
{
}

OmniIMTool::~OmniIMTool()
{
}

void OmniIMTool::onInitialize()
{
}

void OmniIMTool::activate()
{
}

void OmniIMTool::deactivate()
{
}

int OmniIMTool::processKeyEvent( QKeyEvent* event, rviz::RenderPanel* panel )
{
  rviz::InteractiveObjectPtr my_obj = focused_object_.lock();
  boost::shared_ptr<rviz::InteractiveMarkerControl> control;
  control = boost::dynamic_pointer_cast<rviz::InteractiveMarkerControl>(my_obj);
  ROS_DEBUG("Got control [%s]", control->getName().c_str());
}

}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(omni_im::OmniIMTool, rviz::Tool)
PLUGINLIB_EXPORT_CLASS(omni_im::OmniIMTool, rviz::InteractionTool)
