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
    for (int collision_pair_id=0; collision_pair_id<collisions.size(); collision_pair_id++) {
      if (! ((this->tactileSensorList[tactile_sensor_id].linkName == collisions[collision_pair_id]->link[0]->name()) || (this->tactileSensorList[tactile_sensor_id].linkName == collisions[collision_pair_id]->link[1]->name()))) { // センサのあるリンクではない
	continue;
      } else {
        Position positionParent = (this->tactileSensorList[tactile_sensor_id].linkName == collisions[collision_pair_id]->link[0]->name()) ? collisions[collision_pair_id]->link[0]->T() : collisions[collision_pair_id]->link[1]->T();
        double direction = (this->tactileSensorList[tactile_sensor_id].linkName == collisions[collision_pair_id]->link[0]->name()) ? -1.0 : 1.0; // 向きを正に統一
	for (int in_sensor_id=0; in_sensor_id<this->tactileSensorList[tactile_sensor_id].positions.size(); in_sensor_id++) {
	  Vector3 position = positionParent.translation() + positionParent.rotation() * this->tactileSensorList[tactile_sensor_id].positions[in_sensor_id];
	  for (int collision_id=0; collision_id< collisions[collision_pair_id]->collisions.size(); collision_id++) {
	    if ((position - collisions[collision_pair_id]->collisions[collision_id].point).norm() < this->tactileSensorList[tactile_sensor_id].radius) {
              Matrix3 worldSensorRot = positionParent.rotation() * this->tactileSensorList[tactile_sensor_id].rot[in_sensor_id];
	      this->tactileSensorList[tactile_sensor_id].depthVector[in_sensor_id] = worldSensorRot.inverse() * collisions[collision_pair_id]->collisions[collision_id].normal * collisions[collision_pair_id]->collisions[collision_id].depth * direction;
	      break;
	    }
	  }
	}
      }
    }
  }

  int index = 0;
  for (int i=0; i<this->tactileSensorList.size(); i++) {
    for (int j=0; j<this->tactileSensorList[i].depthVector.size(); j++) {
      t_shm->contact_force[index][0] = this->tactileSensorList[i].depthVector[j][0];
      t_shm->contact_force[index][1] = this->tactileSensorList[i].depthVector[j][1];
      t_shm->contact_force[index][2] = this->tactileSensorList[i].depthVector[j][2];
      index++;
    }
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
      auto rot_ = info->extract("rot");
      auto& rotTmp = *rot_->toListing();
      Matrix3 rot;
      rot << rotTmp[0].toDouble(), rotTmp[1].toDouble(), rotTmp[2].toDouble(),
             rotTmp[3].toDouble(), rotTmp[4].toDouble(), rotTmp[5].toDouble(),
             rotTmp[6].toDouble(), rotTmp[7].toDouble(), rotTmp[8].toDouble();
      for (int j=0; j < num_dir1; j++) {
	for (int k=0; k < num_dir2; k++) {
	  sensor.positions.push_back(point + direction1 * j / num_dir1 + direction2 * k / num_dir2);
          sensor.rot.push_back(rot);
	  sensor.depthVector.push_back(Vector3::Zero());
	}
      }
    }
    this->tactileSensorList.push_back(sensor);
    this->io_->os() << "[TactileSensorItem] senser attached to " << linkName << std::endl;
  }
  return true;
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
