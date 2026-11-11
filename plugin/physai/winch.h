// physai.winch — 케이블 윈치(마디 이동 방식) 센서/보조 플러그인.
//
// 모델 규약(PhysicalAI_Sim/Tools/gen_winch_xml.py 가 생성):
//  - 케이블 = 드럼의 자식 트리인 강체 링크 v_0..v_N(길이 seglen, 볼 관절, 링크 +z 가 케이블 방향). 늘어나지 않는다.
//  - 드럼 위 "감긴" 링크 k 의 원점은 connect 등식 wnd_k(링크 사이트 s_k <-> 드럼 슬롯 사이트 d_k)로 드럼에 붙고,
//    풀린 링크의 wnd_k 는 비활성이다. 케이블 충돌은 모델 전체에서 끈다.
//  - 링크 k 의 힘 센서 f_k(site s_k, 부모 관절 전달력)가 장력 센서 역할을 한다.
//  - 풀린 길이 L = l0 + sign*radius*(드럼각). 링크 k 는 (N-k)*seglen >= L 이면 감김, 아니면 풀림.
//    advance 콜백(스텝 끝)에서 wnd_k 의 eq_active 를 L 에 맞춰 갱신한다(다음 스텝부터 적용).
//
// 센서 출력(dim 5): [풀린 길이 L(m), 풀린 링크 수, 장력(드럼쪽 첫 풀린 링크, N), 장력(하중쪽 마지막 링크, N), 드럼각(rad)]
// 센서 선언 순서: 플러그인 센서는 힘 센서 f_k 들보다 뒤에 둔다(계산 순서).
//
// 속성: drum(관절 이름), radius(등가 반지름 m), seglen(m), nseg(N), l0(초기 풀린 길이 m),
//       sign(+1: 드럼각 증가 = 풀림), eqprefix(기본 wnd_), sensprefix(기본 f_)
#ifndef PHYSAI_WINCH_H_
#define PHYSAI_WINCH_H_

namespace physai {
void RegisterWinch();
}  // namespace physai

#endif  // PHYSAI_WINCH_H_
