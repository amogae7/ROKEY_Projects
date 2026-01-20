import os
import glob
import json

# =========================================================
# [사용자 설정 구역]
# 라벨링할 때 쓴 "클래스 이름"과 "번호"를 매칭해주세요.
# YOLO는 0번부터 시작합니다.
# =========================================================
class_map = {
    "part_1_good": 0,   # 예시: 정상품
    # "part_1_bad": 1,  # 예시: 불량품 (없으면 지워도 됨)
    # "class_name": 2, # 추가할 경우 이런 식으로 작성
}
# =========================================================

def convert_and_delete():
    # 현재 폴더의 모든 json 파일 찾기
    json_files = glob.glob("*.json")
    
    if not json_files:
        print("변환할 JSON 파일이 없습니다.")
        return

    print(f"총 {len(json_files)}개의 JSON 파일을 발견했습니다.")
    print("변환 후 원본 JSON 파일은 삭제됩니다.")
    
    # 안전장치: 사용자 확인
    confirm = input(">>> 정말로 진행하시겠습니까? (y/n): ")
    if confirm.lower() != 'y':
        print("작업을 취소합니다.")
        return

    converted_count = 0
    deleted_count = 0

    for json_file in json_files:
        try:
            # 1. JSON 파일 읽기
            with open(json_file, "r", encoding="utf-8") as f:
                data = json.load(f)

            # 이미지 크기 정보 가져오기
            img_w = data.get("imageWidth")
            img_h = data.get("imageHeight")

            # 저장할 TXT 파일 이름 (확장자만 변경)
            txt_filename = os.path.splitext(json_file)[0] + ".txt"
            
            yolo_lines = []

            # 2. Shapes(라벨 정보) 변환
            # shapes가 비어있으면(빈 배경) 루프를 돌지 않고 빈 리스트가 됨 -> 빈 txt 생성 (정상)
            for shape in data.get("shapes", []):
                label = shape["label"]
                
                # 클래스 맵에 없는 라벨이면 경고하고 건너뜀
                if label not in class_map:
                    print(f"[경고] '{json_file}'에서 알 수 없는 라벨 발견: {label} (건너뜀)")
                    continue
                
                class_id = class_map[label]
                points = shape["points"]

                # 좌표 추출 (Polygon이든 Rectangle이든 점들의 Min/Max를 구하면 Box가 됨)
                x_coords = [p[0] for p in points]
                y_coords = [p[1] for p in points]
                
                min_x = min(x_coords)
                max_x = max(x_coords)
                min_y = min(y_coords)
                max_y = max(y_coords)

                # YOLO 형식 (Center X, Center Y, Width, Height) 계산 - 0~1 정규화
                # 중심점
                x_center = (min_x + max_x) / 2.0 / img_w
                y_center = (min_y + max_y) / 2.0 / img_h
                # 너비, 높이
                w = (max_x - min_x) / img_w
                h = (max_y - min_y) / img_h

                # 소수점 6자리까지 저장
                line = f"{class_id} {x_center:.6f} {y_center:.6f} {w:.6f} {h:.6f}\n"
                yolo_lines.append(line)

            # 3. TXT 파일 쓰기
            with open(txt_filename, "w", encoding="utf-8") as f:
                f.writelines(yolo_lines)
            
            converted_count += 1

            # 4. 원본 JSON 삭제 (여기가 삭제하는 부분!)
            os.remove(json_file)
            deleted_count += 1

        except Exception as e:
            print(f"[에러] {json_file} 변환 중 문제 발생: {e}")

    print("\n" + "="*30)
    print(f"작업 완료!")
    print(f"- 생성된 txt 파일: {converted_count}개")
    print(f"- 삭제된 json 파일: {deleted_count}개")
    print("="*30)

if __name__ == "__main__":
    convert_and_delete()