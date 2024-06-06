#ifndef TACTILESENSORPLUGIN_H
#define TACTILESENSORPLUGIN_H

#include <cnoid/ControllerItem>
#include "tactile_shm.h"
#include <unordered_map>

namespace cnoid {

  class TactileSensor
  {
  public:
    // from config file
    std::string linkName; // 親リンク名 (!= ジョイント名)
    cnoid::LinkPtr link;
    cnoid::Vector3 translation = cnoid::Vector3::Zero(); // リンク座標系でどこに取り付けられているか
    cnoid::Matrix3 rotation = cnoid::Matrix3::Identity(); // リンク座標系でセンサの姿勢．zがリンク内側方向
    double radius = 0.01; // 接触とみなす半径

    // variable
    cnoid::Vector3 f = cnoid::Vector3::Zero();
  };

  class TactileSensorItem : public ControllerItem
  {
  public:
    static void initializeClass(ExtensionManager* ext);

    virtual bool initialize(ControllerIO* io) override;

    virtual double timeStep() const override { return timeStep_;};
    virtual void input() override;
    virtual bool control() override;

    virtual bool store(Archive& archive) override;
    virtual bool restore(const Archive& archive) override;

  protected:
    cnoid::ControllerIO* io_;
    std::string configFileName_;
    std::vector<TactileSensor> tactileSensorList_;

    double timeStep_;

    struct tactile_shm *t_shm;
    int shmKey_ = 6555;

    double normalAngle = M_PI / 3.0; // 接触とみなす法線ベクトルの角度の範囲. 0なら垂直.

    std::unordered_map<cnoid::LinkPtr, std::vector<cnoid::Link::ContactPoint>> contactPointsMap_; // input()で取得し、control()で使用される
  };
}

#endif
