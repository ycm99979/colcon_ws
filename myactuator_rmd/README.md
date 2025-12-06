# MyActuator RMD

MyActuator RMD-X 시리즈 모터를 위한 CAN 드라이버 라이브러리입니다.

> **원본 저장소**: [2b-t/myactuator_rmd](https://github.com/2b-t/myactuator_rmd)  
> **저자**: Tobit Flatscher

## 📦 패키지 구조

```
myactuator_rmd/
├── include/myactuator_rmd/
│   ├── driver/              # 드라이버 헤더
│   ├── actuator_interface/  # 액추에이터 인터페이스
│   └── protocol/            # CAN 프로토콜 정의
├── src/
│   ├── driver/              # 드라이버 구현
│   └── can/                 # CAN 통신 구현
├── bindings/                # Python 바인딩
├── test/                    # 단위 테스트
└── doc/                     # 문서
```

## ✨ 주요 기능

- ✅ **CAN 통신** - Linux SocketCAN 지원
- ✅ **다양한 모터 지원** - RMD-X 시리즈 전 모델
- ✅ **Python 바인딩** - pybind11 기반 Python API
- ✅ **비동기 통신** - 논블로킹 CAN 통신
- ✅ **완전한 프로토콜 구현** - 모든 RMD 명령 지원

## 🔧 지원 모터

- RMD-X4
- RMD-X6
- RMD-X8
- RMD-X10
- 기타 RMD-X 시리즈

## 🚀 기본 사용법

### C++

```cpp
#include <myactuator_rmd/can/driver.hpp>
#include <myactuator_rmd/actuator_interface/actuator_interface.hpp>

// CAN 드라이버 생성
myactuator_rmd::CanDriver driver("can0");

// 액추에이터 인터페이스 생성
myactuator_rmd::ActuatorInterface actuator(driver, 0x141);  // ID: 1

// 모터 제어
actuator.setPositionAbsolute(180.0f, 100.0f);  // 180도, 100rpm
```

### Python

```python
from myactuator_rmd import CanDriver, ActuatorInterface

driver = CanDriver("can0")
actuator = ActuatorInterface(driver, 0x141)

# 위치 제어
actuator.set_position_absolute(180.0, 100.0)
```

## 📡 CAN 프로토콜

| 명령 | 코드 | 설명 |
|------|------|------|
| Read PID | 0x30 | PID 파라미터 읽기 |
| Write PID RAM | 0x31 | PID 파라미터 쓰기 (RAM) |
| Read Acceleration | 0x33 | 가속도 읽기 |
| Read Encoder | 0x90 | 엔코더 값 읽기 |
| Position Control | 0xA4 | 절대 위치 제어 |
| Speed Control | 0xA2 | 속도 제어 |
| Torque Control | 0xA1 | 토크 제어 |
| Stop Motor | 0x81 | 모터 정지 |

## 🔌 CAN ID 계산

모터 ID → CAN ID 변환:
```
CAN_ID = 0x140 + Motor_ID
```

예: Motor ID 1 → CAN ID 0x141

## 📄 라이선스

MIT License

---

*이 패키지는 MyActuator의 공식 제품이 아닙니다.*
