# physai_plugins.dll 단독 빌드(MSVC). mujoco.dll 은 재빌드하지 않고 공식 4.0.0 릴리스에 링크한다.
# 사용: powershell -File plugin/physai/build.ps1 [-MujocoRoot <경로>] [-Install]
param(
  [string]$MujocoRoot = "C:\Users\hong\mujoco_poc\dl_4.0.0\x",
  [switch]$Install   # unity/Runtime/Plugins 로 복사(+ 공식 actuator.dll 도 같이)
)
$ErrorActionPreference = "Stop"
$here = Split-Path -Parent $MyInvocation.MyCommand.Path
$root = (Resolve-Path "$here\..\..").Path
$cmake = "C:\Program Files\Microsoft Visual Studio\2022\Community\Common7\IDE\CommonExtensions\Microsoft\CMake\CMake\bin\cmake.exe"
if (-not (Test-Path $cmake)) { $cmake = "cmake" }
$bld = Join-Path $root "build_physai"
# VS2022(MSVC 14.4x) 우선, 없으면 VS 18
& $cmake -S $here -B $bld -G "Visual Studio 17 2022" -A x64 "-DMUJOCO_ROOT=$MujocoRoot"
if ($LASTEXITCODE -ne 0) { throw "cmake 구성 실패" }
& $cmake --build $bld --config Release
if ($LASTEXITCODE -ne 0) { throw "빌드 실패" }
$dll = Join-Path $bld "physai_plugins.dll"
if (-not (Test-Path $dll)) { throw "dll 없음: $dll" }
Write-Host "빌드 완료: $dll"
if ($Install) {
  $dst = Join-Path $root "unity\Runtime\Plugins"
  New-Item -ItemType Directory -Force $dst | Out-Null
  $target = Join-Path $dst "physai_plugins.dll"
  if (Test-Path $target) {
    # 로드 중(에디터/파이썬)인 dll 은 덮어쓸 수 없지만 이름 변경은 가능하다
    try { Copy-Item $dll $target -Force } catch {
      Move-Item $target ("$target.old." + (Get-Date -Format yyyyMMddHHmmss)) -Force
      Copy-Item $dll $target -Force
    }
  } else { Copy-Item $dll $target -Force }
  Copy-Item "$MujocoRoot\bin\mujoco_plugin\actuator.dll" $dst -Force
  Write-Host "설치: $dst"
}
