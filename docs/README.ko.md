# Hopper Nature 한국어 시작 가이드

Hopper Nature는 병렬 발목 기구를 가진 외발 로봇의 서기·호핑 제어를 학습하는 RaiSim/PyTorch 프로젝트입니다. 공개 범위는 로봇 모델, C++ 시뮬레이션 환경, PPO와 상태 추정기, 학습·정책 재생·텍스트 가중치 추출 도구입니다.

[프로젝트 소개](../README.md) · [설치 상세](installation.md) · [한 폴더 작업공간](workspace.md) · [학습 상세](training.md) · [구조와 제어 인터페이스](method.md)

## 실행 환경

- Ubuntu 22.04 x86_64, Python 3.10, GCC 11, CMake 3.22 기준입니다.
- **RaiSim 1.1.8**과 사용자 본인의 활성화 키가 필요합니다. 프로젝트 코드는 MIT이지만 시뮬레이터는 별도 라이선스입니다.
- CPU만으로 짧은 학습 검증이 가능합니다. CUDA 사용은 선택 사항입니다.
- 현재 RaiSim 웹 문서는 RaiSim 2를 설명하므로, 이 프로젝트가 지정한 1.x 버전을 사용하세요.

## 설치

먼저 `build-essential cmake git libeigen3-dev`를 설치하고, RaiSim 1.x 키를 `~/.raisim/activation.raisim`에 준비하세요.

```bash
sudo apt update
sudo apt install -y build-essential cmake git libeigen3-dev

conda create -n hopper_nature python=3.10 pip -y
conda activate hopper_nature

git clone --branch withDHALOneLeg https://github.com/HYUNSEOKKKKKK/Hopper_Nature.git ~/Hopper_Nature
cd ~/Hopper_Nature
python -m pip install torch==2.5.1 --index-url https://download.pytorch.org/whl/cpu
python -m pip install -r requirements.txt

mkdir -p .local/runtime
git clone https://github.com/raisimTech/raisimLib.git .local/runtime/raisimLib
git -C .local/runtime/raisimLib checkout 74b836fba1dbd3227ce61f8abf7bffbd8f6e41ca
python setup.py build_ext --inplace
python -c "from raisimGymTorch.env.bin import hopper; print(hopper.__file__)"
```

코드, RaiSim 1 의존성과 학습 결과는 `~/Hopper_Nature/`에서 관리하고, Python은 이름이 `hopper_nature`인 Conda 환경을 사용합니다. `setup.py`가 내부 SDK를 찾고, `runner.py`/`tester.py`는 자신의 파일 위치를 기준으로 이 저장소를 읽습니다. 작업 폴더와 결과 폴더는 모두 `hopper`이며 기존 방식으로 직접 빌드·실행합니다. `.local/`과 학습 결과는 Git에 포함되지 않으며 별도로 보관해야 합니다.

## 먼저 작은 환경으로 확인

아래 명령은 작업 폴더에서 직접 실행합니다.

```bash
conda activate hopper_nature
cd ~/Hopper_Nature/raisimGymTorch/env/envs/hopper
python runner.py \
  --num-envs 4 --num-threads 2 --iterations 2 \
  --headless --no-tensorboard
```

2회의 PPO 업데이트를 실행하고 `raisimGymTorch/data/hopper/<시간>/`에 설정과 체크포인트를 저장합니다. `full_0.pt`는 첫 업데이트 **전**에 저장되므로 학습된 결과물이 아닙니다.

## 본 학습

```bash
python runner.py \
  --num-envs 400 --num-threads 16 --headless --no-tensorboard
```

CPU 사양에 맞춰 환경 수와 스레드 수를 조절하세요. 원래 설정은 400개 환경과 60개 스레드입니다. 기본 학습은 0–10000 업데이트이며 1000마다 저장합니다. 화면을 보려면 `--headless`를 빼고 지정된 RaiSim 1.x Unity 뷰어를 별도로 연결합니다.

```bash
python -m tensorboard.main \
  --logdir ~/Hopper_Nature/raisimGymTorch/data/hopper --port 6700
```

## 체크포인트 재생과 추출

```bash
python tester.py \
  --weight ../../../data/hopper/RUN_TIMESTAMP/full_10000.pt

python export.py --weight ../../../data/hopper/RUN_TIMESTAMP/full_10000.pt
```

`RUN_TIMESTAMP`는 학습 실행 폴더 이름으로 바꾸고 실제 저장된 회차를 선택하세요. 체크포인트와 같은 폴더에 `cfg.yaml`, `mean10000.csv`, `var10000.csv`가 있어야 합니다. 화면 없이 확인하려면 재생 명령에 `--headless --no-realtime`을 붙이세요. 재생은 22초 동안 서기와 x축 속도 명령을 번갈아 적용합니다. 종료된 에피소드는 자동 초기화되므로 끝까지 실행되었다는 사실만으로 연속 호핑 성공을 의미하지는 않습니다.

`actor.txt`, `estimator.txt` 추출은 하드웨어 제어기 전체를 제공하는 것이 아닙니다. 실제 제어기에 연결하려면 관측값, 필터, 정규화, 동작 스케일링과 병렬 기구 계산을 동일하게 구현해야 합니다. 현재 공개 runner는 처음부터 학습하는 방식이며 정확한 학습 재개 기능은 구현되어 있지 않습니다.

## 결과와 데모

대표 이미지는 저장소의 실제 URDF/메시 렌더입니다. [README의 데모](../README.md#demos)에는 실제 하드웨어 촬영 영상의 GIF 미리보기와 전체 MP4를 넣었습니다. 미리보기는 원래 속도인 1배속이며, 구간과 편집 내용은 [영상 기록](demos.md)에 정리했습니다. 연구실 YouTube 영상은 공개 주소를 전달받으면 데모 아래에 연결합니다. 대표 체크포인트와 정량 평가 결과는 추후 추가합니다. [검증 기록](validation.md).
