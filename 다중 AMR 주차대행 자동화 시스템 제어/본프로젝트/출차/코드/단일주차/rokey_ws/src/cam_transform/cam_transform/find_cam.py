import cv2

def test_camera(index):
    cap = cv2.VideoCapture(index)
    if not cap.isOpened():
        print(f"❌ {index}번 카메라: 연결 불가")
        return False
    
    ret, frame = cap.read()
    if not ret:
        print(f"⚠️ {index}번 카메라: 연결은 됐으나 화면이 안 나옴")
        return False
    
    print(f"✅ {index}번 카메라: 작동 중! (화면을 띄웁니다. 아무 키나 누르면 닫힙니다.)")
    
    while True:
        ret, frame = cap.read()
        if not ret: break
        
        # 화면에 번호 쓰기
        cv2.putText(frame, f"Camera ID: {index}", (50, 50), 
                   cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
        cv2.imshow(f'Camera Test ID: {index}', frame)
        
        # 키 입력 시 종료
        if cv2.waitKey(1) != -1:
            break
            
    cap.release()
    cv2.destroyAllWindows()
    return True

print("=== 카메라 전수 조사 시작 ===")
# 0번부터 9번까지 다 테스트해봅니다.
for i in range(10):
    test_camera(i)
