#include "drivetrain.h"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <memory>
#include <vector>

#include <mujoco/mjdata.h>
#include <mujoco/mjmodel.h>
#include <mujoco/mjplugin.h>
#include <mujoco/mujoco.h>

namespace physai {
namespace {

constexpr double kTwoPi = 6.283185307179586;
constexpr char kCurve[] = "curve", kGears[] = "gears", kFinal[] = "final", kEff[] = "eff",
               kIdle[] = "idle", kUp[] = "up_rpm", kDown[] = "down_rpm", kShiftTime[] = "shift_time",
               kGear[] = "gear", kClutchMax[] = "clutch_max", kEngage[] = "engage_rpm",
               kLaunch[] = "launch", kBacklash[] = "backlash", kLashSpeed[] = "lash_speed",
               kEngBrake[] = "engbrake";

// 공백/쉼표/세미콜론 구분 숫자 목록 파싱
std::vector<double> ParseList(const char* s) {
  std::vector<double> out;
  if (!s) return out;
  const char* p = s;
  while (*p) {
    while (*p == ' ' || *p == ',' || *p == ';' || *p == '\t' || *p == '\n') ++p;
    if (!*p) break;
    char* end = nullptr;
    double v = std::strtod(p, &end);
    if (end == p) break;
    out.push_back(v);
    p = end;
  }
  return out;
}

double Num(const mjModel* m, int inst, const char* key, double def) {
  const char* v = mj_getPluginConfig(m, inst, key);
  return (v && v[0]) ? std::strtod(v, nullptr) : def;
}

struct Config {
  std::vector<double> rpm, torque;  // 토크곡선
  std::vector<double> gears;
  double final_ratio = 1, eff = 1, idle = 800, up_rpm = 0, down_rpm = 0, shift_time = 0.3;
  int fixed_gear = 0;
  double clutch_max = 1e30, engage_rpm = 0, launch = 0.5;
  double backlash = 0, lash_speed = 10, engbrake = 0;
  double max_rpm = 0;
};

class Drivetrain {
 public:
  static std::unique_ptr<Drivetrain> Create(const mjModel* m, int inst) {
    Config c;
    auto curve = ParseList(mj_getPluginConfig(m, inst, kCurve));
    if (curve.size() < 4 || curve.size() % 2) {
      mju_warning("physai.drivetrain: curve 는 rpm,토크 쌍 2개 이상이어야 함");
      return nullptr;
    }
    for (size_t i = 0; i < curve.size(); i += 2) {
      c.rpm.push_back(curve[i]);
      c.torque.push_back(curve[i + 1]);
    }
    for (size_t i = 1; i < c.rpm.size(); ++i) {
      if (c.rpm[i] <= c.rpm[i - 1]) {
        mju_warning("physai.drivetrain: curve rpm 은 증가해야 함");
        return nullptr;
      }
    }
    c.gears = ParseList(mj_getPluginConfig(m, inst, kGears));
    if (c.gears.empty()) {
      mju_warning("physai.drivetrain: gears 필요");
      return nullptr;
    }
    c.final_ratio = Num(m, inst, kFinal, 1);
    c.eff = Num(m, inst, kEff, 1);
    c.idle = Num(m, inst, kIdle, 800);
    c.max_rpm = c.rpm.back();
    c.up_rpm = Num(m, inst, kUp, 0.9 * c.max_rpm);
    c.down_rpm = Num(m, inst, kDown, 0.45 * c.up_rpm);
    c.shift_time = Num(m, inst, kShiftTime, 0.3);
    c.fixed_gear = (int)Num(m, inst, kGear, 0);
    c.clutch_max = Num(m, inst, kClutchMax, 1e30);
    c.engage_rpm = Num(m, inst, kEngage, 0);
    c.launch = Num(m, inst, kLaunch, 0.5);
    c.backlash = Num(m, inst, kBacklash, 0);
    c.lash_speed = Num(m, inst, kLashSpeed, 10);
    c.engbrake = Num(m, inst, kEngBrake, 0);
    if (c.fixed_gear < 0 || c.fixed_gear > (int)c.gears.size()) {
      mju_warning("physai.drivetrain: gear 범위 오류");
      return nullptr;
    }
    std::vector<int> acts;
    for (int i = 0; i < m->nactuator; ++i) {
      if (m->actuator_plugin[i] == inst) {
        if (m->actuator_trntype[i] != mjTRN_JOINT) {
          mju_warning("physai.drivetrain: 액추에이터는 joint 전달이어야 함");
          return nullptr;
        }
        acts.push_back(i);
      }
    }
    if (acts.empty()) {
      mju_warning("physai.drivetrain: 인스턴스 %d 에 액추에이터 없음", inst);
      return nullptr;
    }
    return std::unique_ptr<Drivetrain>(new Drivetrain(c, acts));
  }

  static int StateSize(const mjModel*, int) { return 3; }

  void Reset(mjtNum* st) const {
    st[0] = cfg_.fixed_gear > 0 ? cfg_.fixed_gear : 1;
    st[1] = 0;
    st[2] = 0;
  }

  // 평균 바퀴 각속도(부호 포함, rad/s)
  double MeanOmega(const mjModel* m, const mjData* d) const {
    double s = 0;
    for (int a : acts_) {
      int j = m->actuator_trnid[2 * a];
      s += d->qvel[m->jnt_dofadr[j]];
    }
    return s / acts_.size();
  }

  double Curve(double rpm) const {
    const auto& x = cfg_.rpm;
    const auto& y = cfg_.torque;
    if (rpm <= x.front()) return y.front();
    if (rpm >= x.back()) return y.back();
    size_t i = std::upper_bound(x.begin(), x.end(), rpm) - x.begin();
    double t = (rpm - x[i - 1]) / (x[i] - x[i - 1]);
    return y[i - 1] + t * (y[i] - y[i - 1]);
  }

  double Ctrl(const mjModel* m, const mjData* d, int a) const {
    double c = d->ctrl[m->actuator_ctrladr[a]];
    if (m->actuator_ctrllimited[a]) {
      c = std::min(std::max(c, (double)m->actuator_ctrlrange[2 * a]),
                   (double)m->actuator_ctrlrange[2 * a + 1]);
    }
    return c;
  }

  void Compute(const mjModel* m, mjData* d, int inst) const {
    const mjtNum* st = d->plugin_state + m->plugin_stateadr[inst];
    int gear = (int)st[0];
    gear = std::min(std::max(gear, 1), (int)cfg_.gears.size());
    double ratio = cfg_.gears[gear - 1] * cfg_.final_ratio;
    double omega = MeanOmega(m, d);
    double wheel_rpm_eng = std::fabs(omega) * ratio * 60.0 / kTwoPi;  // 바퀴가 강제하는 엔진 rpm
    double eng_rpm = std::max(wheel_rpm_eng, cfg_.idle);
    bool shifting = st[1] > 0;
    double n = (double)acts_.size();
    // 클러치 슬립: 바퀴측 rpm 이 engage 미만이면 전달 비율을 launch..1 로 낮춘다
    double clutch = 1.0;
    if (cfg_.engage_rpm > 0) {
      double r = std::min(std::max(wheel_rpm_eng / cfg_.engage_rpm, 0.0), 1.0);
      clutch = cfg_.launch + (1.0 - cfg_.launch) * r;
    }
    double tcurve = Curve(eng_rpm);
    double lash_pos = cfg_.backlash > 0 ? st[2] : 0;
    double half = 0.5 * cfg_.backlash;
    for (int a : acts_) {
      double thr = Ctrl(m, d, a);
      double f = 0;
      if (!shifting) {
        double dir = thr >= 0 ? 1.0 : -1.0;
        double te = std::fabs(thr) * tcurve * clutch;
        te = std::min(te, cfg_.clutch_max);
        double tw = dir * te * ratio * cfg_.eff / n;
        // 엔진브레이크: 스로틀을 떼면 바퀴 회전 반대 방향 토크
        if (cfg_.engbrake > 0 && std::fabs(thr) < 1e-9 && std::fabs(omega) > 1e-6) {
          double tb = cfg_.engbrake * std::min(eng_rpm / cfg_.max_rpm, 1.0) * ratio / n;
          tw = -(omega > 0 ? 1.0 : -1.0) * tb;
        }
        // 백래시: 구동 방향 쪽 유격 끝에 닿아야 토크가 전달됨
        if (cfg_.backlash > 0 && tw != 0) {
          double s = tw > 0 ? 1.0 : -1.0;
          if (lash_pos * s < half - 1e-9) tw = 0;
        }
        f = tw;
      }
      d->actuator_force[m->actuator_outadr[a]] = f;
    }
  }

  void Advance(const mjModel* m, mjData* d, int inst) const {
    mjtNum* st = d->plugin_state + m->plugin_stateadr[inst];
    double dt = m->opt.timestep;
    // 변속 타이머 / 자동 변속
    if (st[1] > 0) {
      st[1] = std::max(0.0, st[1] - dt);
    } else if (cfg_.fixed_gear > 0) {
      st[0] = cfg_.fixed_gear;
    } else {
      int gear = (int)st[0];
      double rpm = std::fabs(MeanOmega(m, d)) * cfg_.gears[gear - 1] * cfg_.final_ratio * 60.0 / kTwoPi;
      if (gear < (int)cfg_.gears.size() && rpm > cfg_.up_rpm) {
        st[0] = gear + 1;
        st[1] = cfg_.shift_time;
      } else if (gear > 1 && rpm < cfg_.down_rpm) {
        st[0] = gear - 1;
        st[1] = cfg_.shift_time;
      }
    }
    // 유격 위치: 명령 토크 방향으로 lash_speed 로 이동
    if (cfg_.backlash > 0) {
      double sum = 0;
      for (int a : acts_) sum += Ctrl(m, d, a);
      double s = sum > 0 ? 1.0 : (sum < 0 ? -1.0 : 0.0);
      double half = 0.5 * cfg_.backlash;
      st[2] = std::min(std::max(st[2] + s * cfg_.lash_speed * dt, -half), half);
    }
  }

 private:
  Drivetrain(Config c, std::vector<int> a) : cfg_(std::move(c)), acts_(std::move(a)) {}
  Config cfg_;
  std::vector<int> acts_;
};

}  // namespace

void RegisterDrivetrain() {
  mjpPlugin plugin;
  mjp_defaultPlugin(&plugin);
  plugin.name = "physai.drivetrain";
  plugin.capabilityflags |= mjPLUGIN_ACTUATOR;

  static const char* attrs[] = {kCurve,     kGears,  kFinal,   kEff,    kIdle,       kUp,
                                kDown,       kShiftTime, kGear, kClutchMax, kEngage, kLaunch,
                                kBacklash,   kLashSpeed, kEngBrake};
  plugin.nattribute = sizeof(attrs) / sizeof(attrs[0]);
  plugin.attributes = attrs;
  plugin.nstate = Drivetrain::StateSize;

  plugin.init = +[](const mjModel* m, mjData* d, int inst) {
    auto dt = Drivetrain::Create(m, inst);
    if (!dt) return -1;
    d->plugin_data[inst] = reinterpret_cast<uintptr_t>(dt.release());
    return 0;
  };
  plugin.destroy = +[](mjData* d, int inst) {
    delete reinterpret_cast<Drivetrain*>(d->plugin_data[inst]);
    d->plugin_data[inst] = 0;
  };
  plugin.reset = +[](const mjModel*, mjtNum* st, void* data, int) {
    reinterpret_cast<Drivetrain*>(data)->Reset(st);
  };
  plugin.compute = +[](const mjModel* m, mjData* d, int inst, int) {
    reinterpret_cast<Drivetrain*>(d->plugin_data[inst])->Compute(m, d, inst);
  };
  plugin.advance = +[](const mjModel* m, mjData* d, int inst) {
    reinterpret_cast<Drivetrain*>(d->plugin_data[inst])->Advance(m, d, inst);
  };
  mjp_registerPlugin(&plugin);
}

}  // namespace physai
