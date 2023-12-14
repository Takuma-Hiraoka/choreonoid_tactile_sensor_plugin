#ifndef TACTILESENSORPLUGIN_H
#define TACTILESENSORPLUGIN_H

#include <cnoid/ControllerItem>
#include "tactile_shm.h"

namespace cnoid {

class TactileSensor
{
 public:
  std::string linkName;
  std::vector<Vector3> positions; // リンク座標系でどこに取り付けられているか
  std::vector<Matrix3> rot; // リンク座標系でセンサの姿勢．zがリンク内側方向
  std::vector<Vector3> depthVector;
  double radius = 0.01; // 接触とみなす半径
};

  class TactileSensorItem : public ControllerItem
  {
  public:
    virtual bool initialize(ControllerIO* io) override;
    virtual bool start() override;

    virtual double timeStep() const override { return timeStep_;};
    virtual void input() override {}
    virtual bool control() override;
    virtual void output() override {}
    virtual void stop() override {}

    virtual bool store(Archive& archive) override;
    virtual bool restore(const Archive& archive) override;

    bool loadConfig(Mapping* topNode);
    void initialize_shm(int shm_key);

  protected:
    cnoid::ControllerIO* io_;
    std::string configFileName_;
    std::vector<TactileSensor> tactileSensorList;

    double timeStep_;

    struct tactile_shm *t_shm;
  };
}

#endif
