// physai.drivetrain — 엔진 토크곡선·기어·변속·최종감속·효율·백래시·클러치 슬립을 갖는 구동계 액추에이터 플러그인.
//
// 플러그인 인스턴스 1개 = 차량 구동계 1대. 이 인스턴스를 쓰는 액추에이터들이 구동 바퀴(hinge 관절)다.
// 액추에이터 ctrl 은 각 바퀴의 스로틀(-1..1, 음수=후진 방향 토크). 출력은 관절 토크(N·m, gear=1 가정).
//
// 속성(<config key=... value=...>)
//   curve      "rpm,토크,rpm,토크,..."  엔진 토크곡선(N·m, 선형보간, 범위 밖은 끝값). 필수
//   gears      "g1,g2,..."              기어비(1단부터). 필수
//   final      최종감속비(기본 1)
//   eff        구동 효율 0~1(기본 1)
//   idle       공회전 rpm(기본 800)
//   up_rpm     자동 변속 업시프트 엔진 rpm(기본 곡선 최대 rpm의 0.9)
//   down_rpm   다운시프트 rpm(기본 up_rpm 의 0.45)
//   shift_time 변속 시간(s). 이 동안 토크 차단(기본 0.3)
//   gear       0=자동, n>0=n단 고정(기본 0)
//   clutch_max 클러치 전달 토크 상한(엔진측 N·m, 기본 무제한)
//   engage_rpm 클러치 완전 결합 rpm. 그 아래에선 슬립(기본 0=끔)
//   launch     슬립 최저 전달 비율 0~1(기본 0.5)
//   backlash   구동계 유격(rad, 바퀴 측, 기본 0=끔)
//   lash_speed 유격 통과 속도(rad/s, 기본 10)
//   engbrake   스로틀 0 일 때 레드라인 엔진브레이크 토크(엔진측 N·m, 기본 0)
//
// plugin_state: [0]=현재 기어(1부터), [1]=남은 변속 시간, [2]=유격 위치(-backlash/2..+backlash/2)
#ifndef PHYSAI_DRIVETRAIN_H_
#define PHYSAI_DRIVETRAIN_H_

namespace physai {
void RegisterDrivetrain();
}  // namespace physai

#endif  // PHYSAI_DRIVETRAIN_H_
