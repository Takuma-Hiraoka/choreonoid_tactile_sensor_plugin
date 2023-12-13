#include <cnoid/Plugin>
#include <cnoid/MenuManager>
#include <cnoid/MessageView>
#include <cnoid/ItemManager>
#include <cnoid/Archive>
#include <cnoid/YAMLReader>

#include "TactileSensorItem.h"

using namespace cnoid;

bool TactileSensorItem::initialize(ControllerIO* io) {
  this->io_ = io;
  this->timeStep_ = io->worldTimeStep();

  YAMLReader reader;
  MappingPtr node;
  try {
    node = reader.loadDocument(this->configFileName_)->toMapping();
  } catch(const ValueNode::Exception& ex) {
    this->io_->os() << "\e[0;31m" << "[TactileSensorItem] " << ex.message() <<  "\e[0m" << std::endl;
  }
  if (!loadConfig(node)) this->io_->os() << "\e[0;31m" << "[TactileSensorItem] cannot load config file \e[0m" << std::endl;
  return true;
}

bool TactileSensorItem::start() {
  return true;
}

bool TactileSensorItem::control() {
  return true;
}

bool TactileSensorItem::store(Archive& archive) {
  archive.write("configFileName", this->configFileName_);
  return true;
}

bool TactileSensorItem::restore(const Archive& archive) {
  archive.read("configFileName", this->configFileName_);

  return true;
}

bool TactileSensorItem::loadConfig(Mapping* topNode) {
  auto& tactileSensorList = *topNode->findListing("tactile_sensor");
  if (!tactileSensorList.isValid()) return false;
  for (int i=0; i< tactileSensorList.size(); i++) {
    Mapping* info = tactileSensorList[i].toMapping();
    TactileSensor sensor;
    // link
    std::string linkName;
    info->extract("link", linkName);
    sensor.linkName = linkName;
    // radius
    double radius;
    info->extract("radius", radius);
    sensor.radius = radius;
    // type
    std::string type;
    info->extract("type", type);
    if (type == "rectangle") {
      auto point_ = info->extract("point");
      auto& pointTmp = *point_->toListing();
      Vector3 point = Vector3(pointTmp[0].toDouble(), pointTmp[1].toDouble(), pointTmp[2].toDouble());
      auto direction1_ = info->extract("direction1");
      auto& direction1Tmp = *direction1_->toListing();
      Vector3 direction1 = Vector3(direction1Tmp[0].toDouble(), direction1Tmp[1].toDouble(), direction1Tmp[2].toDouble());
      auto direction2_ = info->extract("direction2");
      auto& direction2Tmp = *direction2_->toListing();
      Vector3 direction2 = Vector3(direction2Tmp[0].toDouble(), direction2Tmp[1].toDouble(), direction2Tmp[2].toDouble());
      int num_dir1 = info->extract("num_dir1")->toInt();
      int num_dir2 = info->extract("num_dir2")->toInt();
      for (int j=0; j < num_dir1; j++) {
	for (int k=0; k < num_dir2; k++) {
	  sensor.positions.push_back(point + direction1 * j / num_dir1 + direction2 * k / num_dir2);
	}
      }
    }
    this->tactileSensorList.push_back(sensor);
    this->io_->os() << "\e[0;31m" << "[TactileSensorItem] " << linkName <<  "\e[0m" << std::endl;
  }
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
