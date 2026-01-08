# E.S.S.E.N.T.I.A.L  
**ESS Safety System with Environmental & Thermal Intelligent ALert**

> ESS 시설의 화재/가스/환경 이상을 감지하고,  
> Hybrid Patrol Robot(ROS2) + MQTT + Control Tower(서버/DB/UI)로 **즉시 알림/이력/관제**까지 이어지는 통합 안전 시스템

![title](docs/assets/slides/title.png)

---


### Full Demo (All-in-one)
<a href="docs/assets/video/full_demo.gif">
  <img src="docs/assets/video/full_demo.gif" width="900">
</a>

### Video Clips

<table>
  <tr>
    <td align="center">
      <a href="docs/assets/video/aruco.gif">
        <img src="docs/assets/video/aruco.gif" width="260">
      </a><br/>
      <b>ArUco Auto-Docking</b>
    </td>
    <td align="center">
      <a href="docs/assets/video/emergency.gif">
        <img src="docs/assets/video/emergency.gif" width="260">
      </a><br/>
      <b>Emergency</b>
    </td>
    <td align="center">
      <a href="docs/assets/video/critical.gif">
        <img src="docs/assets/video/critical.gif" width="260">
      </a><br/>
      <b>Critical</b>
    </td>
  </tr>
  <tr>
    <td align="center">
      <a href="docs/assets/video/warning.gif">
        <img src="docs/assets/video/warning.gif" width="260">
      </a><br/>
      <b>Warning</b>
    </td>
    <td align="center">
      <a href="docs/assets/video/ventilation.gif">
        <img src="docs/assets/video/ventilation.gif" width="260">
      </a><br/>
      <b>Ventilation</b>
    </td>
    <td align="center">
      <a href="docs/assets/video/rfid.gif">
        <img src="docs/assets/video/rfid.gif" width="260">
      </a><br/>
      <b>RFID</b>
    </td>
  </tr>
</table>

## Background
- ESS 시설의 주요 리스크(화재/가스/환경 변화)에 대해 **“감지 → 대응 → 확인/관제”**까지 연결된 시스템 필요
- 이 프로젝트는 Zone별 센서 데이터 수집과 로봇 순찰을 결합해, 이상 상황을 서버로 전송하고 DB 이력화 및 UI로 관제하는 것을 목표로 함

---

## Goals
- [ ] 각 Zone별 독립적 환경 데이터 수집
- [ ] MQTT 기반 초저지연 데이터 전송
- [ ] 위험 감지 시 이벤트/이력 저장 + 관제 UI 표시
- [ ] 로봇 순찰 + ArUco 기반 홈 복귀

---

## Hardware Architecture

<p>
  <img src="docs/assets/slides/hw-arch.png"  width="900">
</p>
<p>
  <img src="docs/assets/slides/hw-arch2.png" width="900">
</p>
<p>
  <img src="docs/assets/slides/hw-arch3.png" width="900">
</p>

---

## Software Architecture
![sw-arch](docs/assets/slides/sw-arch.png)

---

## Data Flow / Sequence
![sequence](docs/assets/slides/sequence.png)

---

### ROS Nodes / Graph
![ros-nodes-1](docs/assets/diagrams/ros-nodes-1.png)
![ros-nodes-2](docs/assets/diagrams/ros-nodes-2.png)

## Key Features
### 1) Access Authentication & Alarm
- RFID 기반 관리자 인증
- 가스 이상 감지 시 시청각 알림 + 서버 즉각 전송(MQTT)

### 2) Environmental Monitoring & HVAC Control
- 온/습도 수집 및 임계 조건 기반 공조 제어
- 데이터는 DB 적재 + UI 조회/그래프 제공


### 3) Thermal Safety (Infrared)
- 열화상 ROI 기반 이상 픽셀 감지
- 최고온도/좌표 기반 이벤트 생성 → MQTT 전송 → DB 기록

### 4) Hybrid Patrol Robot (ROS2)
- NAV2 기반 순찰 + 구역/층 전환 로직
- ArUco 마커 기반 홈 위치 정렬(복귀 보정)

---

## MQTT Protocol (Topics)
> 서버 구독(예): `ess/env`, `ess/alert`, `ess/access/request`

### Environment
Topic: `ess/env`
```json
{"t": 23.10, "h": 55.30, "fan": "ON", "reason": "TEMP"}
```

### Alert (Gas / Thermal)
Topic: ess/alert
```text
{
  "event_type": "gas",
  "level": "warning",
  "value": 650,
  "location": "zone_1",
  "message": "Gas level elevated"
}
```

### Access Request/Response
Req Topic: ess/access/request
```text
{"admin_id":"RFID_123456","access_point":"main"}
```

Resp Topic: ess/access/response
```text
{"result":"success"}
```

### Project Structure
```text
.
├── deploy/                 # systemd/udev/scripts (로봇/라즈베리파이 자동실행)
├── ess_map/                # NAV2 map
└── src/
    ├── ess_server/         # MariaDB + MQTT subscriber + Qt UI
    ├── ess_control_pkg/    # ROS2 control node (NAV2/상태머신 등)
    ├── ess_mqtt_bridge_pkg/# ROS↔MQTT 브릿지(초안/실험 포함)
    ├── esp8266/            # ESP8266(D1 mini) MQTT bridge
    └── SGP30_3*/           # STM32 펌웨어(센서/제어)
```

### Deployment (systemd / udev)

deploy/ 폴더는 로봇(또는 Pi)에서 부팅 시 자동으로 서비스가 올라오도록 구성되어 있음.

udev: 카메라 심볼릭 링크(/dev/cam_rgb) 등

systemd:

ess-usb-camera.service : ROS2 카메라 퍼블리셔

ess-aruco-move.service : ArUco 기반 정렬/복귀 노드

ess-thermal-*.service : 열화상 체크/수집/게이트(환경에 따라 경로 조정 필요)

환경별로 /opt/ess-guardian/current/... 같은 경로는 수정이 필요할 수 있음.

### Troubleshooting (Short)

STM32 하드웨어 배선 이슈: 접점/전원/그라운드 재정리로 안정화

소프트 I2C Bit-banging 이슈: 타이밍 마진 조정 + 풀업/노이즈 대응

ROS NAV2 이슈: TF/파라미터 튜닝으로 주행 안정화

UI Update 이슈: 알림 해제 후 상태 전이 로직 보강 필요

<details>
<summary><b>Troubleshooting Screenshots</b></summary>

![hw-trouble](docs/assets/troubleshooting/hw-troubleshooting.png)
![ros-trouble-1](docs/assets/troubleshooting/ros-troubleshooting-1.png)
![ros-trouble-2](docs/assets/troubleshooting/ros-troubleshooting-2.png)
![ui-trouble](docs/assets/troubleshooting/ui-troubleshooting.png)

</details>

### Roadmap

이벤트 정합성(중복/쿨다운) 정책 고도화

Thermal ROI / 임계값 튜닝 자동화

Control Tower 기능 강화(필터/리포트/통계)

## 👥 Team & Roles

| Name | Role | Main Contribution (1-line) |
| :---: | :---: | :--- |
| 김찬미 (Team Leader) | STM32 / HW | 센서 수집 + 제어 로직 + 하드웨어 안정화 및 일정 리드 |
| 이두현 | ROS2 / Navigation | Nav2 기반 순찰/상태머신 및 주행 파라미터 튜닝 |
| 김민성 | Vision / Deploy | ArUco Auto-Docking + 카메라 파이프라인 + systemd/udev 자동 실행 |
| 류균봉 | Server / UI | Qt 관제 UI + MariaDB 적재 + MQTT 프로토콜/정합성 |

<details>
<summary><b>Details</b></summary>

### 김찬미 (STM32 Firmware & Hardware)
- **Firmware**: 센서(ADC/I2C) 수집 + 액추에이터(PWM) 제어 로직 구현
- **Hardware**: 배선/전원/GND/신호 품질 점검 및 통신 안정화
- 관련 코드: `src/SGP30_3*/`

### 이두현 (ROS2 Control & Navigation)
- **ROS2 Control**: 순찰 동작/상태 머신 구성
- **Navigation**: Nav2 파라미터 튜닝으로 주행 안정화
- 관련 코드: `src/ess_control_pkg/`, `ess_map/`

### 김민성 (Computer Vision & System Deploy)
- **Auto-Docking**: ArUco 기반 정렬/Homing 알고리즘(오차/hold/스케일/각도 보정) 구현
- **Vision Pipeline**: RGB/열화상 스트림 수집→이벤트 생성→전송 흐름 구성
- **Deploy**: systemd/udev로 부팅 자동 실행(카메라 심링크, 서비스 의존성, 재시작 정책)
- 관련 코드: `deploy/`, `src/ess_aruco_move/`, `src/...thermal...`

### 류균봉 (Central Server & Control UI)
- **Control Tower**: Qt(C++) 관제 대시보드/모니터링 UI 구현
- **Backend**: MariaDB 스키마/적재 및 로그 관리
- **Protocol**: MQTT 토픽/메시지 포맷 설계 및 정합성 관리
- 관련 코드: `src/ess_server/`

</details>
