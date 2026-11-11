#include "winch.h"

#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <memory>
#include <string>
#include <vector>

#include <mujoco/mjdata.h>
#include <mujoco/mjmodel.h>
#include <mujoco/mjplugin.h>
#include <mujoco/mujoco.h>

namespace physai {
namespace {

double Num(const mjModel* m, int inst, const char* key, double def) {
  const char* v = mj_getPluginConfig(m, inst, key);
  return (v && v[0]) ? std::strtod(v, nullptr) : def;
}
std::string Str(const mjModel* m, int inst, const char* key, const char* def) {
  const char* v = mj_getPluginConfig(m, inst, key);
  return (v && v[0]) ? std::string(v) : std::string(def);
}

class Winch {
 public:
  static std::unique_ptr<Winch> Create(const mjModel* m, int inst) {
    std::unique_ptr<Winch> w(new Winch());
    std::string drum = Str(m, inst, "drum", "");
    w->jnt_ = mj_name2id(m, mjOBJ_JOINT, drum.c_str());
    if (w->jnt_ < 0) {
      mju_warning("physai.winch: drum 관절 '%s' 없음", drum.c_str());
      return nullptr;
    }
    w->radius_ = Num(m, inst, "radius", 0.1);
    w->seglen_ = Num(m, inst, "seglen", 0.02);
    w->nseg_ = (int)Num(m, inst, "nseg", 0);
    w->l0_ = Num(m, inst, "l0", 0);
    w->sign_ = Num(m, inst, "sign", 1) >= 0 ? 1.0 : -1.0;
    std::string ep = Str(m, inst, "eqprefix", "wnd_");
    std::string sp = Str(m, inst, "sensprefix", "f_");
    if (w->nseg_ < 1 || w->radius_ <= 0 || w->seglen_ <= 0) {
      mju_warning("physai.winch: nseg/radius/seglen 오류");
      return nullptr;
    }
    // 마디 k=0..N 의 등식, 변 k=0..N-1 의 텐던
    for (int k = 0; k <= w->nseg_; ++k) {
      int id = mj_name2id(m, mjOBJ_EQUALITY, (ep + std::to_string(k)).c_str());
      if (id < 0) {
        mju_warning("physai.winch: 등식 %s%d 없음", ep.c_str(), k);
        return nullptr;
      }
      w->eq_.push_back(id);
    }
    // 링크 k 의 힘 센서 f_k(site s_k 의 부모 관절 전달력, 3성분)
    for (int k = 0; k <= w->nseg_; ++k) {
      int id = mj_name2id(m, mjOBJ_SENSOR, (sp + std::to_string(k)).c_str());
      if (id < 0 || m->sensor_type[id] != mjSENS_FORCE) {
        mju_warning("physai.winch: 힘 센서 %s%d 없음", sp.c_str(), k);
        return nullptr;
      }
      w->seg_.push_back(id);
    }
    return w;
  }

  double Length(const mjModel* m, const mjData* d) const {
    double th = d->qpos[m->jnt_qposadr[jnt_]];
    double L = l0_ + sign_ * radius_ * th;
    return std::min(std::max(L, 0.0), nseg_ * seglen_);
  }

  // 마디 k 가 감겨 있는가: (N-k)*seglen >= L (경계 마디는 감김으로 취급)
  bool Wound(int k, double L) const { return (nseg_ - k) * seglen_ >= L - 1e-12; }

  void Advance(const mjModel* m, mjData* d) const {
    double L = Length(m, d);
    for (int k = 0; k <= nseg_; ++k) d->eq_active[eq_[k]] = Wound(k, L) ? 1 : 0;
  }

  // 링크 k 의 부모 관절 전달력 크기(힘 센서 f_k, 3성분)
  double LinkForce(const mjModel* m, const mjData* d, int k) const {
    const mjtNum* f = d->sensordata + m->sensor_adr[seg_[k]];
    return std::sqrt(f[0] * f[0] + f[1] * f[1] + f[2] * f[2]);
  }

  void Compute(const mjModel* m, mjData* d, int inst) const {
    double L = Length(m, d);
    // 마지막 감긴 마디 번호 kw -> 첫 풀린 링크는 v_{kw+1}
    int kw = (int)std::floor(nseg_ - L / seglen_ + 1e-9);
    if (kw < 0) kw = 0;
    if (kw > nseg_ - 1) kw = nseg_ - 1;
    for (int s = 0; s < m->nsensor; ++s) {
      if (m->sensor_type[s] == mjSENS_PLUGIN && m->sensor_plugin[s] == inst) {
        mjtNum* out = d->sensordata + m->sensor_adr[s];
        out[0] = L;
        out[1] = nseg_ - kw;  // 풀린 링크 수(v_{kw+1}..v_N)
        out[2] = LinkForce(m, d, kw + 1);
        out[3] = LinkForce(m, d, nseg_);
        out[4] = d->qpos[m->jnt_qposadr[jnt_]];
      }
    }
  }

 private:
  Winch() = default;
  int jnt_ = -1, nseg_ = 0;
  double radius_ = 0.1, seglen_ = 0.02, l0_ = 0, sign_ = 1;
  std::vector<int> eq_, seg_;
};

}  // namespace

void RegisterWinch() {
  mjpPlugin plugin;
  mjp_defaultPlugin(&plugin);
  plugin.name = "physai.winch";
  plugin.capabilityflags |= mjPLUGIN_SENSOR;

  static const char* attrs[] = {"drum", "radius", "seglen", "nseg", "l0", "sign", "eqprefix", "sensprefix"};
  plugin.nattribute = sizeof(attrs) / sizeof(attrs[0]);
  plugin.attributes = attrs;
  plugin.nstate = +[](const mjModel*, int) { return 0; };
  plugin.nsensordata = +[](const mjModel*, int, int) { return 5; };
  plugin.needstage = mjSTAGE_ACC;  // 구속력이 계산된 뒤에 읽는다

  plugin.init = +[](const mjModel* m, mjData* d, int inst) {
    auto w = Winch::Create(m, inst);
    if (!w) return -1;
    d->plugin_data[inst] = reinterpret_cast<uintptr_t>(w.release());
    return 0;
  };
  plugin.destroy = +[](mjData* d, int inst) {
    delete reinterpret_cast<Winch*>(d->plugin_data[inst]);
    d->plugin_data[inst] = 0;
  };
  plugin.reset = +[](const mjModel*, mjtNum*, void*, int) {};
  plugin.compute = +[](const mjModel* m, mjData* d, int inst, int) {
    reinterpret_cast<Winch*>(d->plugin_data[inst])->Compute(m, d, inst);
  };
  plugin.advance = +[](const mjModel* m, mjData* d, int inst) {
    reinterpret_cast<Winch*>(d->plugin_data[inst])->Advance(m, d);
  };
  mjp_registerPlugin(&plugin);
}

}  // namespace physai
