# RB-Y1 SDK 실행 가이드

시뮬레이터를 띄우고 `examples/python`의 예제를 실행하기까지의 실전 메모.

- SDK 버전: `rby1-sdk 0.10.0` (Python)
- 시뮬레이터 이미지: `rainbowroboticsofficial/rby1-sim:0.10.6-a_v1.2`
- 기본 접속 주소: `localhost:50051` (gRPC)

---

## 1. Docker 설치 (최초 1회)

```bash
sudo apt update && sudo apt install -y docker.io docker-compose-v2 && sudo usermod -aG docker $USER
```

- `docker.io` : Docker 엔진 (우분투 공식 저장소 패키지)
- `docker-compose-v2` : `docker compose` 서브커맨드 제공 (하이픈 버전 `docker-compose`가 아니다)
- `usermod -aG docker $USER` : `sudo` 없이 docker를 쓰기 위해 현재 사용자를 `docker` 그룹에 추가

### 그룹 반영

`usermod`은 **새 로그인 세션부터** 적용된다. 로그아웃/재로그인 하거나, 지금 쓰는 터미널에서만 임시로 적용하려면:

```bash
newgrp docker
```

### 확인

```bash
docker --version && docker compose version && docker run --rm hello-world
```

`hello-world`가 `permission denied ... /var/run/docker.sock` 로 실패하면 그룹 반영이 안 된 것 — 재로그인 후 다시 시도한다.

서비스가 안 떠 있으면:

```bash
sudo systemctl enable --now docker
```

> 참고: 확인된 환경 기준 `docker.io`는 Docker 29.1.3, `docker-compose-v2`는 Compose 2.40.3 을 설치한다.
> 최신 Docker CE가 필요하면 우분투 패키지 대신 Docker 공식 저장소(`get.docker.com`)를 쓴다.

---

## 2. 시뮬레이터 실행 (Docker)

```bash
xhost +local:docker && docker compose -f ~/rby1-sim/docker-compose.sim.yaml up
```

- `xhost +local:docker` : 컨테이너가 호스트 X 서버에 GUI 창을 띄울 수 있게 허용. **재부팅하면 초기화되므로 매번 필요.**
- 컨테이너는 `network_mode: host` 라서 시뮬레이터 gRPC 서버가 호스트의 `localhost:50051`에 그대로 열린다.
- 포그라운드로 뜨므로 이 터미널은 잡아두고, 예제는 **새 터미널**에서 실행한다.

백그라운드로 띄우려면:

```bash
docker compose -f ~/rby1-sim/docker-compose.sim.yaml up -d
```

종료:

```bash
docker compose -f ~/rby1-sim/docker-compose.sim.yaml down
```

작업이 끝나면 X 접근 권한을 되돌리는 게 안전하다:

```bash
xhost -local:docker
```

### 두 번째부터는 `up` 대신 `start` (권장)

`up`은 **컨테이너를 새로 만드는** 명령이다. 한 번 만들어 두면 그다음부터는 기존 컨테이너를 재사용하는 게 빠르다.

```bash
docker start rby1-sim-rby1-sim-1
```

compose 파일이 없어도 동작한다 (컨테이너 이름 = `<디렉토리명>-<서비스명>-1`). compose로 하려면:

```bash
docker compose -f ~/rby1-sim/docker-compose.sim.yaml start
```

로그를 터미널에 붙여서 보고 싶으면:

```bash
docker start -a rby1-sim-rby1-sim-1
```

```bash
docker logs -f rby1-sim-rby1-sim-1
```

### 정지는 `stop`, `down`이 아니다

| 명령 | 컨테이너 | 다음 실행 |
|---|---|---|
| `docker stop rby1-sim-rby1-sim-1` | 남음 | `docker start` |
| `docker compose ... down` | **삭제됨** | `docker compose ... up` (재생성 필요) |

즉 `down`을 쓰면 매번 `up`을 해야 한다. 평소에는 `stop` / `start`로 돌리는 게 편하다.

### 부팅 시 자동 시작

compose 파일의 서비스에 재시작 정책을 넣고 한 번만 `up -d` 하면, 이후 재부팅해도 자동으로 올라온다.

```yaml
services:
  rby1-sim:
    restart: unless-stopped
```

또는 이미 만들어진 컨테이너에 바로 적용:

```bash
docker update --restart unless-stopped rby1-sim-rby1-sim-1
```

### `xhost`는 여전히 필요하다

`xhost`는 **호스트 X 서버 쪽 설정**이라 컨테이너 재사용과 무관하게 X 세션(로그인/재부팅)마다 초기화된다. 컨테이너가 root로 돌기 때문에 `+local:` 대신 root만 허용하는 게 더 좁고 안전하다:

```bash
xhost +SI:localuser:root
```

매번 치기 싫으면 `~/.profile` 등에 넣어 둔다. 현재 허용 상태는 `xhost` 를 인자 없이 실행해 확인한다.

> 주의: `DISPLAY` 값은 **컨테이너 생성 시점**에 박힌다 (현재 `:0`). 나중에 `DISPLAY`가 바뀌면 `start`로는 반영되지 않으니 컨테이너를 지우고 `up`으로 다시 만들어야 한다.

### compose 파일 내용 (`~/rby1-sim/docker-compose.sim.yaml`)

```yaml
services:
  rby1-sim:
    image: rainbowroboticsofficial/rby1-sim:0.10.6-a_v1.2
    environment:
      - DISPLAY=${DISPLAY}
    volumes:
      - /tmp/.X11-unix:/tmp/.X11-unix
    devices:
      - /dev/dri:/dev/dri
    network_mode: host
```

### 상태 확인

```bash
docker ps
```

```bash
python3 -c "import rby1_sdk as rby; r=rby.create_robot('localhost:50051','a'); print(r.connect()); print(r.get_robot_info())"
```

---

## 3. 예제 실행 방법

### 중요: 반드시 `examples/python` 안에서 실행

예제들이 `importlib.import_module("00_helper")`로 헬퍼를 불러오기 때문에, 다른 디렉토리에서 실행하면 import가 깨진다.

```bash
cd ~/GitHub/rby1-sdk/examples/python
python3 01_hello_rby1.py --address localhost:50051
```

레포 루트에서 돌리고 싶다면 경로를 잡아준다:

```bash
PYTHONPATH=examples/python python3 examples/python/01_hello_rby1.py --address localhost:50051
```

### 공통 인자

| 인자 | 설명 | 기본값 |
|---|---|---|
| `--address` | 로봇/시뮬레이터 gRPC 주소 | 없음 (대부분 필수) |
| `--model` | 로봇 모델: `a`, `m`, `ub` (대소문자 무시) | `a` |
| `--power` | 전원 장치 이름 정규식 | `.*` |
| `--servo` | 서보 이름 정규식 | `.*` |

실기 로봇은 `--address <로봇IP>:50051` 로 바꾸기만 하면 된다.

### 헬퍼가 해주는 것 (`00_helper.py`)

`initialize_robot(address, model, power, servo)` 한 방에:

1. `connect()`
2. `power_on()` (꺼져 있으면)
3. `servo_on()` (꺼져 있으면)
4. Control Manager가 Major/MinorFault면 `reset_fault_control_manager()`
5. `enable_control_manager()`

`movej(robot, torso=, right_arm=, left_arm=, minimum_time=)` 로 관절 위치 이동.

---

## 4. 예제 목록

> ⚠️ 표시는 로봇이 움직이므로 **주변 공간 확보 후** 실행.
> 🚫 표시는 실기 전용 — 시뮬레이터에서는 동작하지 않는다.

| 파일 | 내용 | 비고 |
|---|---|---|
| `01_hello_rby1.py` | 연결 및 로봇 정보 출력 | 첫 동작 확인용 |
| `02_robot_model_setting.py` | 모델 설정 | |
| `03_robot_state.py` | 상태 1회 조회 | |
| `04_robot_state_stream.py` | 상태 스트리밍 | |
| `05_parameter.py` | 파라미터 조회/설정 | |
| `06_state_tool_flange.py` | 툴 플랜지 상태 | |
| `07_power.py` | 전원 제어 | |
| `08_check_firmware_version.py` | 펌웨어 버전 | |
| `09_get_pid_gain.py` | PID 게인 읽기 | 🚫 |
| `10_set_pid_gain.py` | PID 게인 쓰기 | 🚫 주의 |
| `11_factory_default_pid_gain.py` | PID 공장값 복원 | 🚫 |
| `12_led.py` | LED 제어 | |
| `13_set_system_time.py` | 시스템 시간 설정 | |
| `14_wifi.py` | Wi-Fi 설정 | 🚫 |
| `15_log.py` / `16_log_stream.py` / `17_fault_log.py` | 로그 조회/스트리밍/폴트 로그 | |
| `18_dynamics_modeling.py` | 동역학 모델링 (로봇 불필요) | `--address` 없음 |
| `19_dynamics_robot.py` | 실로봇 동역학/FK | |
| `20_leader_arm_state_check.py` | 리더암 상태 | 🚫 |
| `21_record.py` / `22_replay.py` | 모션 기록 / 재생 | |
| `23_zero_pose.py` | 제로 포즈 | |
| `24_demo_motion.py` | 데모 모션 | ⚠️ |
| `25_wiggle_motion.py` | 위글 모션 | ⚠️ |
| `26_cancel_control.py` | 명령 취소 | |
| `27_collisions.py` | 충돌 체크 | |
| `28_cartesian_impedance_control.py` | 카테시안 임피던스 제어 | |
| `29_joint_impedance_control.py` | 관절 임피던스 제어 | |
| `30_joint_group_command.py` | 관절 그룹 명령 | 🚫 |
| `31_multi_controls.py` | 다중 제어 | |
| `32_command_stream.py` | 명령 스트리밍 | ⚠️ |
| `33_cartesian_command_stream.py` | 카테시안 명령 스트리밍 | ⚠️ |
| `34_real_time_control.py` | 실시간 제어 | |
| `35_leader_arm_teleop_with_monitor.py` | 리더암 텔레오퍼레이션 | 🚫 |
| `36_brake_test.py` | 브레이크 테스트 | ⚠️ |
| `37_mobile_test.py` | 모바일 베이스 주행 | ⚠️ |
| `38_rpc_serial_device.py` / `39_rpc_serial_communication.py` | 시리얼 장치/통신 | 🚫 |
| `90_gamepad_teleop.py` | 게임패드 텔레오퍼레이션 (오른팔) | ⚠️ 아래 참고 |

---

## 5. 게임패드 텔레오퍼레이션 (`90_gamepad_teleop.py`)

오른팔 엔드이펙터를 게임패드로 카테시안 델타 제어한다. 시작 시 팔을 ready 자세로 4초간 이동시킨 뒤 20 Hz로 목표 포즈를 스트리밍한다.

### 실행

```bash
cd ~/GitHub/rby1-sdk/examples/python
python3 90_gamepad_teleop.py --address localhost:50051 --model a
```

### 조작

| 입력 | 동작 |
|---|---|
| 왼쪽 스틱 | EE x / y 이동 (위=+x, 왼쪽=+y) |
| 오른쪽 스틱 X | 툴 z축 기준 요(yaw) 회전 |
| ZR / R2 | EE z 상승 |
| ZL / L2 | EE z 하강 |
| Ctrl+C | 정지 |

안전장치: 시작 위치 기준 ±(0.35, 0.35, 0.30) m 박스 안으로 목표가 클램프된다. 속도는 스틱 최대 기준 0.15 m/s, 요 1.0 rad/s.

### 패드 연결 확인

```bash
ls -l /dev/input/js*
```

```bash
python3 -c "import pygame; pygame.init(); pygame.joystick.init(); j=pygame.joystick.Joystick(0); j.init(); print(j.get_name(), j.get_numaxes(), 'axes', j.get_numbuttons(), 'buttons')"
```

매핑을 눈으로 확인하려면 (축/버튼 실시간 출력):

```bash
python3 90_gamepad_teleop.py --probe
```

### 컨트롤러별 트리거 매핑

스크립트가 패드 종류를 자동 감지한다 (`make_trigger_reader()`).

- **Xbox / DualShock 등 (축 6개 이상)** → 트리거는 아날로그 축 4, 5. 누른 정도에 비례해 속도가 나온다.
- **Nintendo Pro Controller (`hid-nintendo`, 축 4개 / 버튼 14개)** → ZL/ZR이 **디지털 버튼 7, 8**. 축이 4개(LX, LY, RX, RY)뿐이라 z축은 on/off 풀스피드로만 움직인다. D-pad는 hat으로 올라온다.

기동 시 로그로 어느 쪽이 잡혔는지 확인할 수 있다:

```
INFO - Gamepad: Nintendo Co., Ltd. Pro Controller (4 axes, 14 buttons)
INFO - Triggers: digital buttons 7/8
```

---

## 6. 트러블슈팅

### `pygame.error: Invalid joystick axis`

패드에 해당 축이 없는 경우. Pro Controller처럼 축이 4개뿐인 패드에 아날로그 트리거(축 4/5)를 읽으려 할 때 발생한다. 현재 스크립트는 자동 분기하므로 뜨지 않아야 하며, 다른 패드에서 나면 `--probe`로 실제 인덱스를 확인해 `_AX_*` / `_BTN_*` 상수를 고친다.

### `No gamepad found`

- `ls -l /dev/input/js*` 로 장치 존재 확인
- VM(VMware 등)이면 USB 패스스루가 게스트로 연결됐는지 확인
- 블루투스 대신 USB 유선으로 연결하면 대체로 인식이 안정적

### 시뮬레이터 창이 안 뜬다

- `xhost +local:docker` 를 실행했는지 확인 (재부팅 시 초기화)
- `echo $DISPLAY` 가 비어 있지 않은지 확인
- GPU 노드 확인: `ls /dev/dri`
- Wayland 세션이면 Xwayland가 있어야 함

### 연결 실패 (`Failed to connect robot`)

- 시뮬레이터 컨테이너가 떠 있는지: `docker ps`
- 포트 확인: `ss -ltnp | grep 50051`
- 주소 오타 확인 — 포트까지 붙여야 한다 (`localhost:50051`)

### Control Manager 폴트

`initialize_robot()`이 Major/MinorFault를 자동 리셋하지만, 계속 폴트가 나면 `17_fault_log.py`로 원인을 확인한다.

```bash
python3 17_fault_log.py --address localhost:50051
```

### 로봇이 움직이지 않는다

전원/서보/Control Manager 순서로 확인. `07_power.py`, `03_robot_state.py`로 상태를 먼저 본다.

---

## 7. SDK 설치 / 재설치

```bash
pip install rby1-sdk
```

이 레포 소스로 설치:

```bash
pip install .
```

C++ 빌드는 루트 [README.md](README.md) 참고 (Conan + CMake).
