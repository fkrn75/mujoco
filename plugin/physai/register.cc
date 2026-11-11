// PhysicalAI 보조 플러그인 등록 진입점.
// DLL 로드 시(mj_loadPluginLibrary) 이 라이브러리가 제공하는 모든 플러그인을 등록한다.
#include <mujoco/mjplugin.h>

#include "drivetrain.h"

namespace physai {

mjPLUGIN_LIB_INIT(physai_plugins) {
  RegisterDrivetrain();
}

}  // namespace physai
