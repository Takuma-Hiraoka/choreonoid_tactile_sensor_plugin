#include <cnoid/Plugin>
#include <cnoid/MenuManager>
#include <cnoid/MessageView>
#include <cnoid/ItemManager>
#include <cnoid/Archive>
#include <cnoid/YAMLReader>
#include <cnoid/BodyItem>

#include "TactileSensorItem.h"

using namespace cnoid;

bool TactileSensorItem::initialize(ControllerIO* io) {
  this->io_ = io;
  this->timeStep_ = io->worldTimeStep();

  int shm_key = 6555;
  initialize_shm(shm_key);

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
  for (int i=0; i<this->tactileSensorList.size(); i++) {
    cnoid::LinkPtr link_ = this->io_->body()->link(this->tactileSensorList[i].linkName);
    if (!link_) {
      this->io_->os() << "\e[0;31m" << "[TactileSensorItem] link [" << this->tactileSensorList[i].linkName << "] not found" << "\e[0m" << std::endl;
    }
  }

  return true;
}

bool TactileSensorItem::control() {
  std::vector<CollisionLinkPairPtr> collisions = this->findOwnerItem<BodyItem>()->collisions();

  for (int tactile_sensor_id=0; tactile_sensor_id<this->tactileSensorList.size(); tactile_sensor_id++) {
    Vector3 depthVector = Vector3::Zero();
    for (int collision_pair_id=0; collision_pair_id<collisions.size(); collision_pair_id++) {
      if (! ((this->tactileSensorList[tactile_sensor_id].linkName == collisions[collision_pair_id]->link[0]->name()) ||
             (this->tactileSensorList[tactile_sensor_id].linkName == collisions[collision_pair_id]->link[1]->name()))) { // センサのあるリンクではない
        continue;
      } else {
        Position positionParent = (this->tactileSensorList[tactile_sensor_id].linkName == collisions[collision_pair_id]->link[0]->name()) ? collisions[collision_pair_id]->link[0]->T() : collisions[collision_pair_id]->link[1]->T();
        Vector3 position = positionParent.translation() + positionParent.rotation() * this->tactileSensorList[tactile_sensor_id].translation;
        double direction = (this->tactileSensorList[tactile_sensor_id].linkName == collisions[collision_pair_id]->link[0]->name()) ? -1.0 : 1.0; // 向きを正に統一
        for (int collision_id=0; collision_id< collisions[collision_pair_id]->collisions.size(); collision_id++) {
          if ((position - collisions[collision_pair_id]->collisions[collision_id].point).norm() < this->tactileSensorList[tactile_sensor_id].radius) {
            Matrix3 worldSensorRot = positionParent.rotation() * this->tactileSensorList[tactile_sensor_id].rotation;
            depthVector = worldSensorRot.inverse() * collisions[collision_pair_id]->collisions[collision_id].normal * collisions[collision_pair_id]->collisions[collision_id].depth * direction * 100000;
            break;
          }
        }
      }
      this->tactileSensorList[tactile_sensor_id].depthVector = depthVector;
    }
  }
  for (int i=0; i<this->tactileSensorList.size(); i++) {
    t_shm->contact_force[i][0] = this->tactileSensorList[i].depthVector[0];
    t_shm->contact_force[i][1] = this->tactileSensorList[i].depthVector[1];
    t_shm->contact_force[i][2] = this->tactileSensorList[i].depthVector[2];
  }
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
    if (linkName.find("LINK") != std::string::npos) {
      replaceOtherStr(linkName, "LINK", "JOINT");
    }
    sensor.linkName = linkName;
    // translation
    auto translation_ = info->extract("translation");
    auto& translationTmp = *translation_->toListing();
    sensor.translation = Vector3(translationTmp[0].toDouble(), translationTmp[1].toDouble(), translationTmp[2].toDouble());
    // rotation
    auto rotation_ = info->extract("rotation");
    auto& rotationTmp = *rotation_->toListing();
    sensor.rotation << rotationTmp[0].toDouble(), rotationTmp[1].toDouble(), rotationTmp[2].toDouble(),
                       rotationTmp[3].toDouble(), rotationTmp[4].toDouble(), rotationTmp[5].toDouble(),
                       rotationTmp[6].toDouble(), rotationTmp[7].toDouble(), rotationTmp[8].toDouble();
    this->tactileSensorList.push_back(sensor);
    this->io_->os() << "[TactileSensorItem] senser attached to " << linkName << std::endl;
  }
  return true;
}

std::string TactileSensorItem::replaceOtherStr(std::string &replacedStr, std::string from, std::string to) {
    const unsigned int pos = replacedStr.find(from);
    const int len = from.length();

    if (pos == std::string::npos || from.empty()) {
        return replacedStr;
    }

    return replacedStr.replace(pos, len, to);
}

void TactileSensorItem::initialize_shm(int shm_key)
{
  t_shm = (struct tactile_shm *) set_shared_memory(shm_key, sizeof(struct tactile_shm));
  if (t_shm == NULL) {
    this->io_->os() << "\e[0;31m" << "[TactileSensorItem] set_shared_memory failed" <<  "\e[0m" << std::endl;
    exit(1);
  }
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
