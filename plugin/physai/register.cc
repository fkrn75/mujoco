// PhysicalAI 보조 플러그인 등록 진입점.
// DLL 로드 시(mj_loadPluginLibrary) 이 라이브러리가 제공하는 모든 플러그인을 등록한다.
#include <mujoco/mjplugin.h>

namespace physai {

mjPLUGIN_LIB_INIT(physai_plugins) {
  // 0단계: 등록할 플러그인 없음(로딩 경로만 검증). 1단계부터 여기에 Register 호출을 추가한다.
}

}  // namespace physai
