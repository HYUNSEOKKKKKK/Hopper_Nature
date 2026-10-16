# Hopper 작업 재개 안내

마지막 정리: **2026-10-16**. 당분간 개발을 쉬는 상태이며, 공개 저장소와 로컬 소스를 맞춰 두었습니다. 다음 작업은 이 문서부터 확인하세요.

## 현재 상태

- 저장소: `HYUNSEOKKKKKK/Hopper_Nature`, 브랜치 `withDHALOneLeg`.
- 공개 준비 기준 소스: `d6c9e47`. `029f4d5`에 마지막 속도 명령 범위·질량 랜덤화·기존 exporter 경로 변경이 포함되어 있습니다.
- 활성 로봇: `rsc/Hop2_Parallel_Nature_20261008/Hop2_Parallel_Nature_20261008.urdf`, 명목 질량 17.100 kg.
- 시뮬레이터: **RaiSim 1.1.8**. 현재 웹 문서의 RaiSim 2와 구분해서 사용합니다.
- Python 3.10, PyTorch 2.5.1 계열, `ruamel.yaml==0.17.21`. 이 컴퓨터의 기존 Conda 환경 이름은 `raisimtorch`입니다.
- 작업 모듈 이름 `hound`와 결과 폴더 이름 `dhal_one_leg`는 호환성을 위해 유지했습니다.
- README에 야외 `IMG_8769.MP4`와 실내 `IMG_8770.MP4`의 GIF 미리보기·전체 MP4를 게시했습니다.

전체 설치·학습 방법은 [문서 목차](docs/README.md), 구현과 관측값 규약은 [method](docs/method.md)를 참고하세요.

## 이 컴퓨터에서 보존한 자료

`.local/`은 Git에서 제외하는 로컬 보관 폴더입니다. 상세 경로와 재개 명령은 그 안의 `README.md`에 있습니다. 새로 clone한 저장소에는 이 폴더가 없으므로 아래 자료는 별도로 보관·전달해야 합니다.

| 자료 | 위치와 상태 |
| --- | --- |
| 최근 호환 체크포인트 묶음 | `.local/checkpoints/2026-10-10-21-43-35/` — `full_10000.pt`, 같은 회차 mean/var, `cfg.yaml`, `Environment.hpp`, 추출 가중치 |
| 원본 촬영 영상 2개 | `.local/media/originals/` — Downloads 원본의 별도 복사본 |
| 실행 환경 기록 | `.local/environment/` — pip 목록, Conda 환경 YAML·explicit 목록 |
| 전체 학습 실행 목록 | `.local/inventory/training-runs.json` — 로컬·기존 형제 저장소의 실행 경로, 체크포인트와 정규화 파일 유무 |
| 보관 자료의 SHA-256 | `.local/inventory/archive-manifest.json` |
| 정리·실행 확인 기록 | `.local/validation/`, `.local/inventory/cleanup-report.json` |

이 저장소의 `raisimGymTorch/data/`에 있던 과거 실행과, 형제 디렉터리 `../hound/raisimGymTorch/data/`의 학습 기록은 원래 위치에 보존했습니다. **코드만 Git에 있다고 체크포인트·원본 영상까지 백업된 것은 아닙니다.** `.local/`의 복사본도 같은 디스크에 있으므로 장기 보관 시 별도 저장장치에 함께 복사하세요.

기존 `export.py`가 가리키던 최근 실행은 `2026-10-10-21-43-35/full_10000.pt`입니다. 해당 실행의 `Environment.hpp`는 정리 시점 소스와 바이트 단위로 일치했습니다. 이 파일을 편의상 보관했으며, 공개 대표 정책이나 두 영상에 사용된 정책으로 확정한 것은 아닙니다.

정리 과정에서 현재 소스를 다시 빌드하고 이 보관본을 22초·2200스텝 재생했습니다. 해당 한 번의 실행에서는 에피소드 종료가 없었고, 문서 점검과 CLI 테스트 4개도 통과했습니다. 상세 범위는 [검증 기록](docs/validation.md)에 남겼습니다.

## 다음에 시작할 때

1. `git status`와 현재 브랜치를 확인하고, 이 컴퓨터에서는 `.local/README.md`를 읽습니다.
2. `conda activate raisimtorch`로 기존 환경을 활성화합니다. 새 컴퓨터라면 [설치 안내](docs/installation.md)를 따릅니다.
3. RaiSim 1.1.8과 개인 활성화 키의 존재를 확인합니다. 키는 보관 폴더나 Git에 복사하지 않았습니다.
4. C++ 환경을 다시 빌드한 후 정적 점검과 짧은 재생부터 시작합니다. 컴파일된 `.so`만 보고 소스 버전이 일치한다고 가정하지 마세요.

원래 작업 공간처럼 Hopper가 `raisimLib/` 안에 있는 경우, 저장소 루트에서:

```bash
conda activate raisimtorch
python setup.py build_ext --inplace --CMAKE_PREFIX_PATH \
  "$PWD/../raisim/linux;$PWD/../thirdParty/Eigen3"

python scripts/check_release.py
python -m unittest discover -s tests -v

python -m raisimGymTorch.env.envs.hound.tester \
  --weight .local/checkpoints/2026-10-10-21-43-35/full_10000.pt \
  --duration 22 --headless --no-realtime
```

`.local/`을 복원하지 않은 새 clone에서는 자신의 체크포인트 경로를 사용하세요. 화면을 보려면 마지막 명령의 `--headless --no-realtime`을 빼고 RaiSim 1.x Unity 클라이언트를 연결합니다. 새로 학습하려면 [training](docs/training.md)을 참고하세요. 현재 runner는 처음부터 학습하며 정확한 학습 재개 기능은 없습니다.

## 남은 일

- 연구실 YouTube 영상이 공개되면 실제 URL을 README 데모 바로 아래와 [영상 기록](docs/demos.md)에 연결합니다. 현재 URL은 아직 전달받지 않았습니다.
- 공개 대표 체크포인트를 선정하고 모델·설정·정규화 CSV·출처 커밋·SHA-256을 묶어 배포합니다.
- 두 하드웨어 촬영 영상과 정책 체크포인트의 대응 관계를 확인합니다.
- 정량 성능, CUDA, 다른 컴퓨터의 새 설치, Unity 영상 녹화, 실제 하드웨어 제어는 각각 별도 검증이 필요합니다. 기존 [검증 기록](docs/validation.md)과 혼동하지 마세요.

## 정리 원칙

학습 결과·로봇 모델 이력·캘리브레이션·원본 영상은 보존했습니다. Python 캐시, 오래된 패키지 메타데이터, 이번 공개 준비에서 만든 임시 빌드·검증 사본을 정리했습니다. Git 이력에 있는 과거 모델과 플랫폼별 파일은 경로 호환성과 출처 보존을 위해 이동하지 않았습니다.

추가 작업 시 `.local/`, `raisimGymTorch/data/`, 외부 학습 결과 폴더를 일괄 삭제하지 말고 먼저 목록을 확인하세요. 특히 `git clean -fdx`는 Git 제외 체크포인트와 원본 보관본까지 지웁니다.
