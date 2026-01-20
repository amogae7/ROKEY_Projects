import cv2
import numpy as np
import os

# ==========================================
# [사용자 설정]
# ==========================================
MAP_FILE = "teleop_map.pgm"
RESOLUTION = 0.05         # yaml에서 확인한 값 (0.05)
ORIGIN_X = -3.47          # yaml에서 확인한 값
ORIGIN_Y = -1.23          # yaml에서 확인한 값
VIEW_SCALE = 5.0          # ⭐ 5배 확대해서 보기 (원하는 배율로 수정 가능)
# ==========================================

def main():
    if not os.path.exists(MAP_FILE):
        print(f"❌ '{MAP_FILE}' 파일을 찾을 수 없습니다.")
        return

    # 1. 원본 이미지 로드
    original_img = cv2.imread(MAP_FILE, cv2.IMREAD_COLOR)
    if original_img is None:
        print("❌ 이미지 로드 실패!")
        return
        
    orig_h, orig_w, _ = original_img.shape
    print(f"✅ 원본 크기: {orig_w}x{orig_h} | 확대 배율: {VIEW_SCALE}배")

    # 2. 보기용 확대 이미지 생성 (픽셀이 깨지지 않게 nearest 옵션 사용)
    display_img = cv2.resize(original_img, None, fx=VIEW_SCALE, fy=VIEW_SCALE, interpolation=cv2.INTER_NEAREST)

    def click_event(event, x, y, flags, param):
        if event == cv2.EVENT_LBUTTONDOWN:
            # 3. 확대된 좌표(x, y)를 원본 좌표(real_x, real_y)로 복구
            orig_pixel_x = int(x / VIEW_SCALE)
            orig_pixel_y = int(y / VIEW_SCALE)

            # 4. SLAM 맵 좌표 변환 공식 (오리지널 픽셀 기준)
            # Map_X = Origin_X + (Pixel_X * Resolution)
            # Map_Y = Origin_Y + ((Image_Height - Pixel_Y) * Resolution)
            
            map_real_x = ORIGIN_X + (orig_pixel_x * RESOLUTION)
            map_real_y = ORIGIN_Y + ((orig_h - orig_pixel_y) * RESOLUTION)
            
            print(f"🎯 클릭: 화면({x},{y}) -> 원본({orig_pixel_x},{orig_pixel_y}) -> 🗺️ 맵좌표 [{map_real_x:.3f}, {map_real_y:.3f}]")
            
            # 시각화 (확대된 이미지에 그리기)
            cv2.circle(display_img, (x, y), 5, (0, 0, 255), -1)
            text = f"{map_real_x:.2f}, {map_real_y:.2f}"
            cv2.putText(display_img, text, (x+10, y), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 0, 0), 2)
            cv2.imshow("Zoomed Map Picker", display_img)

    cv2.imshow("Zoomed Map Picker", display_img)
    cv2.setMouseCallback("Zoomed Map Picker", click_event)

    print("\n🔍 맵이 확대되었습니다. 중앙 사각형의 4개 모서리를 클릭하세요.")
    print("   [순서: 뒤 -> 왼 -> 앞 -> 오]")
    
    cv2.waitKey(0)
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()