#include <cnoid/Plugin>
#include <cnoid/MenuManager>
#include <cnoid/MessageView>
#include <cnoid/ItemManager>

#include "TactileSensorItem.h"

using namespace cnoid;

bool TactileSensorItem::initialize(ControllerIO* io) {
  return true;
}

bool TactileSensorItem::start() {
  return true;
}

bool TactileSensorItem::control() {
  return true;
}

bool TactileSensorItem::store(Archive& archive) {
  return true;
}

bool TactileSensorItem::restore(const Archive& archive) {
  return true;
}

class CnoidTactileSensorPlugin : public Plugin
{
public:

  CnoidTactileSensorPlugin() : Plugin("TactileSensor")
  {
    require("Body");
  }

  virtual bool initialize() override
  {
    itemManager().registerClass<TactileSensorItem>("TactileSensorItem");
    return true;
  }
};

CNOID_IMPLEMENT_PLUGIN_ENTRY(CnoidTactileSensorPlugin)
