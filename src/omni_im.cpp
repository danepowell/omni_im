#include "omni_im.h"
#include <boost/shared_ptr.hpp>
#include "rviz/default_plugin/interactive_markers/interactive_marker_control.h"

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

}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(omni_im::OmniIMTool, rviz::Tool)
PLUGINLIB_EXPORT_CLASS(omni_im::OmniIMTool, rviz::InteractionTool)
