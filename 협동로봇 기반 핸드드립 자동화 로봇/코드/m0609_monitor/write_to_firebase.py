import time
import firebase_admin
from firebase_admin import credentials
from firebase_admin import db

# 1. Firebase Admin SDK 초기화
# 1단계 5번에서 다운로드한 json 키 파일 경로
SERVICE_ACCOUNT_KEY_PATH = "/home/wook/cobot1_ws/src/m0609_monitor/config/serviceAccountKey.json" 
# 1단계 4번의 databaseURL 값 (Firebase 콘솔 -> Realtime Database에서 확인)
DATABASE_URL = "https://rokey-baristar-robot-default-rtdb.asia-southeast1.firebasedatabase.app" 

try:
    cred = credentials.Certificate(SERVICE_ACCOUNT_KEY_PATH)
    firebase_admin.initialize_app(cred, {
        'databaseURL': DATABASE_URL
    })
except ValueError:
    print("Firebase 앱이 이미 초기화되었습니다.")

# 데이터를 쓸 DB 경로 참조 (예: '/robot_status')
ref = db.reference('/robot_status')

print("Firebase DB에 데이터 쓰기를 시작합니다. (Ctrl+C로 종료)")

job_count = 0

# 2. 1초마다 데이터 업데이트 (시뮬레이션)
try:
    while True:
        job_count += 1 # 작업 완료 횟수 1 증가

        # 3. Firebase DB에 데이터 쓰기 (업데이트)
        ref.update({
            'completed_jobs': job_count,
            'last_update_timestamp': time.time() # 현재 시간을 타임스탬프로 저장
        })

        print(f"데이터 업데이트: {job_count}개 완료")

        # 1초 대기
        time.sleep(1)

except KeyboardInterrupt:
    print("\n프로그램을 종료합니다.")