// Falcon ground-effect — a custom Gazebo Harmonic (gz-sim8) System plugin that
// applies a near-surface thrust-augmentation "cushion" force, matching the
// SimBackend ground_effect model (thrust ×= 1 + gain·e^(−alt/decay)).
//
// Rationale (v1.32): the stock gz WindEffects plugin over-authors disturbances
// and diverges the falcon mission; the realism arc's effects live in the Rust
// SimBackend, which is the verified path. This plugin is the start of bringing
// the SAME, controlled realism models into the gz SITL so the two agree. It is
// bench-only (gz is not in CI); the gate is a gz headless run that loads it and
// shows the cushion acting.
//
// SDF parameters (all optional):
//   <link_name>        link the cushion acts on        (default "base_link")
//   <gain>             cushion strength (dimensionless) (default 0.4)
//   <decay>            e-folding height of the cushion, m (default 0.25)
//   <reference_force>  N the gain scales (≈ vehicle weight) (default 20.0)

#include <chrono>
#include <cmath>
#include <memory>
#include <string>

#include <gz/plugin/Register.hh>
#include <gz/sim/Link.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/System.hh>
#include <gz/sim/Util.hh>
#include <gz/math/Vector3.hh>
#include <sdf/Element.hh>

namespace falcon {

class GroundEffect : public gz::sim::System,
                     public gz::sim::ISystemConfigure,
                     public gz::sim::ISystemPreUpdate {
 public:
  void Configure(const gz::sim::Entity &entity,
                 const std::shared_ptr<const sdf::Element> &sdf,
                 gz::sim::EntityComponentManager &ecm,
                 gz::sim::EventManager &) override {
    gz::sim::Model model(entity);
    std::string linkName = sdf->Get<std::string>("link_name", "base_link").first;
    this->linkEntity = model.LinkByName(ecm, linkName);
    this->gain = sdf->Get<double>("gain", 0.4).first;
    this->decay = sdf->Get<double>("decay", 0.25).first;
    this->refForce = sdf->Get<double>("reference_force", 20.0).first;
    this->verbose = sdf->Get<bool>("verbose", false).first;
    if (this->decay <= 0.0) this->decay = 0.25;
    gzmsg << "[falcon::GroundEffect] active: gain=" << this->gain
          << " decay=" << this->decay << " ref=" << this->refForce
          << " link=" << linkName << std::endl;
  }

  void PreUpdate(const gz::sim::UpdateInfo &info,
                 gz::sim::EntityComponentManager &ecm) override {
    if (info.paused || this->linkEntity == gz::sim::kNullEntity) return;
    // Altitude above the ground plane = world z of the link.
    double rawAlt = gz::sim::worldPose(this->linkEntity, ecm).Pos().Z();
    double alt = rawAlt < 0.0 ? 0.0 : rawAlt;  // cushion saturates at the surface
    // The cushion: strongest at the surface, decaying with altitude.
    double boost = this->gain * std::exp(-alt / this->decay) * this->refForce;
    gz::sim::Link link(this->linkEntity);
    link.AddWorldForce(ecm, gz::math::Vector3d(0.0, 0.0, boost));

    // Verbose: emit one line per simulated second so a headless bench run can
    // confirm the cushion is acting (altitude held up, boost decaying with it).
    if (this->verbose) {
      double t = std::chrono::duration<double>(info.simTime).count();
      int sec = static_cast<int>(t);
      if (sec != this->lastLoggedSec) {
        this->lastLoggedSec = sec;
        gzmsg << "[falcon::GroundEffect] t=" << sec << "s alt=" << rawAlt
              << "m boost=" << boost << "N" << std::endl;
      }
    }
  }

 private:
  gz::sim::Entity linkEntity{gz::sim::kNullEntity};
  double gain{0.4};
  double decay{0.25};
  double refForce{20.0};
  bool verbose{false};
  int lastLoggedSec{-1};
};

}  // namespace falcon

GZ_ADD_PLUGIN(falcon::GroundEffect, gz::sim::System,
              falcon::GroundEffect::ISystemConfigure,
              falcon::GroundEffect::ISystemPreUpdate)
GZ_ADD_PLUGIN_ALIAS(falcon::GroundEffect, "falcon::GroundEffect")
