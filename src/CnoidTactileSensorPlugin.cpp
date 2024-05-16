#include <cnoid/Plugin>
#include <cnoid/MenuManager>
#include <cnoid/MessageView>
#include <cnoid/ItemManager>

#include "TactileSensorItem.h"

using namespace cnoid;

class CnoidTactileSensorPlugin : public Plugin
{
public:

  CnoidTactileSensorPlugin() : Plugin("TactileSensor")
  {
    require("Body");
  }

  virtual bool initialize() override
  {
    TactileSensorItem::initializeClass(this);
    return true;
  }
};

CNOID_IMPLEMENT_PLUGIN_ENTRY(CnoidTactileSensorPlugin)
