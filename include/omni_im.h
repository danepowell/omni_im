#ifndef RVIZ_OMNI_IM_TOOL_H
#define RVIZ_OMNI_IM_TOOL_H

#include "rviz/tool.h"
#include "rviz/default_plugin/tools/interaction_tool.h"
#include <QObject>
#include <boost/shared_ptr.hpp>
#include "rviz/default_plugin/interactive_markers/interactive_marker_control.h"

namespace omni_im
{

class OmniIMTool: public rviz::InteractionTool
{
Q_OBJECT
public:
 OmniIMTool();
 virtual ~OmniIMTool();

 virtual void onInitialize();
 virtual void activate();
 virtual void deactivate();

};

}

#endif
