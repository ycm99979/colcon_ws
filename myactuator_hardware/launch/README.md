# MyActuator Hardware Launch Files

CAN 인터페이스 테스트를 위한 런치파일입니다.

## test_connection.launch.py

CAN 인터페이스 연결 상태를 확인합니다.

```bash
# 기본 사용
ros2 launch myactuator_hardware test_connection.launch.py

# 특정 인터페이스
ros2 launch myactuator_hardware test_connection.launch.py can_interface:=can0

# 가상 CAN 테스트
ros2 launch myactuator_hardware test_connection.launch.py can_interface:=vcan0
```

| 파라미터 | 기본값 | 설명 |
|----------|--------|------|
| `can_interface` | `can0` | CAN 인터페이스 이름 |

---

## CAN 설정 명령어

```bash
# 실제 CAN
sudo ip link set can0 type can bitrate 1000000
sudo ip link set can0 up

# 가상 CAN (테스트)
sudo modprobe vcan
sudo ip link add dev vcan0 type vcan
sudo ip link set vcan0 up
```

---

## 다음 단계

테스트 성공 후:

```bash
ros2 launch fr_arm_moveit_config hardware.launch.py can_interface:=can0
```
