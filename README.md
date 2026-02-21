# ROKEY Projects Overview

이 저장소는 로봇/AI 융합 프로젝트 4개를 모아둔 통합 포트폴리오입니다.
각 폴더는 독립 프로젝트이며, 세부 실행법과 아키텍처는 폴더별 `README.md`에 정리되어 있습니다.

## 프로젝트 목록

### 1. 다중 AMR 주차대행 자동화 시스템 제어
- 요약: ROS2 기반으로 다중 AMR(TurtleBot 계열)의 입차/출차 미션을 자동화하는 시스템
- 핵심: 슬롯 할당, 미션 생성/분배, Nav2 주행, Dock/Undock, 상태 동기화
- 주요 기술: ROS2, Nav2, Supabase, OpenCV
- 경로: `다중 AMR 주차대행 자동화 시스템 제어/README.md`

### 2. AI 비전 기반 스마트 제조 로봇 솔루션
- 요약: YOLO 비전 인식 + 음성 명령 해석 + 협동로봇 제어를 통합한 스마트 제조 셀
- 핵심: 객체 인식/좌표화, LLM 명령 파싱, 픽앤플레이스/폐기 자동화
- 주요 기술: ROS2, Ultralytics YOLO, OpenCV, Speech Recognition, LangChain/OpenAI, Doosan Robot
- 경로: `AI 비전 기반 스마트 제조 로봇 솔루션/README.md`

### 3. 디지털 트윈 기반 수산양식장 청소로봇
- 요약: Isaac Sim에서 수조 환경과 UR10을 이용해 청소 동작을 검증하는 디지털 트윈 프로젝트
- 핵심: 월드/에셋 구성, IK 경로 계획, 스캔/추적 청소, 물리 피드백
- 주요 기술: NVIDIA Isaac Sim, USD, Python, IK
- 경로: `디지털 트윈 기반 수산양식장 청소로봇/README.md`

### 4. 협동로봇 기반 핸드드립 자동화 로봇
- 요약: 웹 주문 UI와 협동로봇을 연동해 핸드드립 제조를 자동화한 시스템
- 핵심: Firebase 기반 주문/상태 동기화, 공정 시퀀스 실행, STOP/RECOVER 제어
- 주요 기술: ROS2, Doosan Robot, Firebase Realtime DB, HTML/CSS/JS
- 경로: `협동로봇 기반 핸드드립 자동화 로봇/README.md`

## 저장소 구조
```text
ROKEY_Projects/
|- AI 비전 기반 스마트 제조 로봇 솔루션/
|- 다중 AMR 주차대행 자동화 시스템 제어/
|- 디지털 트윈 기반 수산양식장 청소로봇/
'- 협동로봇 기반 핸드드립 자동화 로봇/
```

## 빠른 시작
1. 관심 프로젝트 폴더로 이동
2. 해당 폴더의 `README.md` 확인
3. 환경 의존성(ROS2/Isaac Sim/Firebase 등) 설치 후 실행

## 참고
- 프로젝트별 코드 품질/실험 버전 파일이 혼재할 수 있으므로, 각 README의 엔트리 파일 기준으로 실행하는 것을 권장합니다.
