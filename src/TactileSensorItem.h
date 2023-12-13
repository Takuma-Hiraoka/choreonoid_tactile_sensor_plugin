#ifndef TACTILESENSORPLUGIN_H
#define TACTILESENSORPLUGIN_H

#include <cnoid/ControllerItem>

namespace cnoid {
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

  protected:
    double timeStep_;
  };
}

#endif
