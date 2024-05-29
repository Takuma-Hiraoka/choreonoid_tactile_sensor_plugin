#include "TactileSensorItem.h"

#include <cnoid/ItemManager>
#include <cnoid/SimulationBar>
#include <cnoid/Archive>
#include <cnoid/YAMLReader>
#include <cnoid/BodyItem>
#include <cnoid/CollisionLinkPair>
#include <cnoid/SceneGraph>
#include <iostream>
#include <unordered_map>

namespace cnoid {
  void TactileSensorItem::initializeClass(ExtensionManager* ext)
  {
    ext->itemManager().registerClass<TactileSensorItem>("TactileSensorItem");
  }

  bool TactileSensorItem::initialize(cnoid::ControllerIO* io) {
    this->io_ = io;
    this->timeStep_ = io->worldTimeStep();

    // コンストラクタやcallLaterだとname()やrestore()が未完了
    if(!this->t_shm){
      this->io_->os() << "[TactileSensorItem] shmget " << this->shmKey_ << std::endl;
      int shm_id = shmget(this->shmKey_, sizeof(struct tactile_shm), 0666|IPC_CREAT);
      if(shm_id == -1) {
        this->io_->os() << "\e[0;31m" << "[TactileSensorItem] shmget failed"  << "\e[0m" << std::endl;
        this->t_shm = nullptr;
      }else{
        this->t_shm = (struct tactile_shm *)shmat(shm_id, (void *)0, 0);
        if(this->t_shm == (void*)-1) {
          this->io_->os() << "\e[0;31m" << "[TactileSensorItem] shmat failed"  << "\e[0m" << std::endl;
          this->t_shm = nullptr;
        }
      }
    }

    std::unordered_map<std::string, std::string> URDFToVRMLLinkNameMap;
    for(int l=0;l<this->io_->body()->numLinks() ; l++){
      cnoid::SgGroup* shape = this->io_->body()->link(l)->shape();
      if(shape && shape->numChildObjects() > 0 && shape->child(0)->name().size()!=0){
        URDFToVRMLLinkNameMap[shape->child(0)->name()] = this->io_->body()->link(l)->name();
      }
    }

    cnoid::YAMLReader reader;
    cnoid::MappingPtr node;
    try {
      node = reader.loadDocument(this->configFileName_)->toMapping();
    } catch(const cnoid::ValueNode::Exception& ex) {
      this->io_->os() << "\e[0;31m" << "[TactileSensorItem] " << ex.message() <<  "\e[0m" << std::endl;
    }
    if(node){
      cnoid::Listing* tactileSensorList = node->findListing("tactile_sensor");
      if (!tactileSensorList->isValid()) {
        this->io_->os() << "\e[0;31m" << "[TactileSensorItem] tactile_sensor list is not valid \e[0m" << std::endl;
      }else{
        this->tactileSensorList_.resize(tactileSensorList->size());
        for (int i=0; i< tactileSensorList->size(); i++) {
          cnoid::Mapping* info = tactileSensorList->at(i)->toMapping();
          TactileSensor sensor;
          // linkname
          info->extract("link", sensor.linkName);
          if(sensor.linkName == ""){
            this->io_->os() << "\e[0;31m" << "[TactileSensorItem] link name is not specified" << "\e[0m" << std::endl;
            continue;
          }
          // link
          if(URDFToVRMLLinkNameMap.find(sensor.linkName) != URDFToVRMLLinkNameMap.end()){
            sensor.link = this->io_->body()->link(URDFToVRMLLinkNameMap[sensor.linkName]);
          }else if(this->io_->body()->link(sensor.linkName)){
            sensor.link = this->io_->body()->link(sensor.linkName);
          }
          if (!(sensor.link)) {
            this->io_->os() << "\e[0;31m" << "[TactileSensorItem] link [" << sensor.linkName << "] not found" << "\e[0m" << std::endl;
            continue;
          }
          sensor.link->mergeSensingMode(cnoid::Link::LinkContactState); // enable contact sensing
          // translation
          cnoid::ValueNodePtr translation_ = info->extract("translation");
          if(translation_){
            cnoid::ListingPtr translationTmp = translation_->toListing();
            if(translationTmp->size()==3){
              sensor.translation = Vector3(translationTmp->at(0)->toDouble(), translationTmp->at(1)->toDouble(), translationTmp->at(2)->toDouble());
            }
          }
          // rotation
          cnoid::ValueNodePtr rotation_ = info->extract("rotation");
          if(rotation_){
            cnoid::ListingPtr rotationTmp = rotation_->toListing();
            if(rotationTmp->size() == 4){
              sensor.rotation = cnoid::AngleAxisd(rotationTmp->at(3)->toDouble(),
                                                  cnoid::Vector3{rotationTmp->at(0)->toDouble(), rotationTmp->at(1)->toDouble(), rotationTmp->at(2)->toDouble()}).toRotationMatrix();
            }
          }
          // radius
          cnoid::ValueNodePtr radius_ = info->extract("radius");
          if(radius_ && radius_->isScalar()){
            sensor.radius = radius_->toDouble();
          }
          this->tactileSensorList_[i] = sensor;
        }
      }
    }
    this->io_->os() << "[TactileSensorItem] " << this->tactileSensorList_.size() << " sensers attached from " << this->configFileName_ << std::endl;

    return true;
  }

  void TactileSensorItem::input() {
    for(int i=0;i<this->io_->body()->numLinks();i++){
      this->contactPointsMap_[this->io_->body()->link(i)] = this->io_->body()->link(i)->contactPoints();
    }
  }

  // The body oject given in the initalized function() must not be accessed
  // in this function. The access should be done in input() and output().
  bool TactileSensorItem::control() {
    for (int i=0; i<this->tactileSensorList_.size(); i++) {
      cnoid::Vector3 f = cnoid::Vector3::Zero(); // センサ系. センサが受ける力
      if(this->tactileSensorList_[i].link) {
        std::vector<cnoid::Link::ContactPoint>& contactPoints = this->contactPointsMap_[this->tactileSensorList_[i].link];
        cnoid::Vector3 p = this->tactileSensorList_[i].link->T() * this->tactileSensorList_[i].translation; // world系. センサ位置
        cnoid::Matrix3 R = this->tactileSensorList_[i].link->R() * this->tactileSensorList_[i].rotation; // world系. センサ姿勢
        cnoid::Vector3 normal = R * cnoid::Vector3::UnitZ(); // world系. from another object to this link

        for (int c=0; c<contactPoints.size();) {
          if((p - contactPoints[c].position()).norm() > this->tactileSensorList_[i].radius ||
             std::acos(std::min(1.0,std::max(-1.0,contactPoints[c].normal().dot(normal)))) > this->normalAngle) {
            c++;
            continue;
          }else{
            f += R.transpose() * contactPoints[c].force()/*world系. リンクが受ける力*/;
            contactPoints.erase(contactPoints.begin()+c); // 1つの接触を近くの接触センサで重複カウントすることを防ぐため.
          }
        }
      }
      this->tactileSensorList_[i].f = f;
    }
    for (int i=0; i<this->tactileSensorList_.size(); i++) {
      t_shm->contact_force[i][0] = this->tactileSensorList_[i].f[0];
      t_shm->contact_force[i][1] = this->tactileSensorList_[i].f[1];
      t_shm->contact_force[i][2] = this->tactileSensorList_[i].f[2];
    }
    return true;
  }

  bool TactileSensorItem::store(cnoid::Archive& archive) {
    ControllerItem::store(archive);
    archive.write("configFileName", this->configFileName_);
    archive.write("shmKey", this->shmKey_);
    return true;
  }

  bool TactileSensorItem::restore(const cnoid::Archive& archive) {
    ControllerItem::restore(archive);
    archive.read("configFileName", this->configFileName_);
    archive.read("shmKey", this->shmKey_);
    return true;
  }

};
