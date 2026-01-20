import os
import glob
import random

# ==========================================
# 사용자 설정
# ==========================================
# 남길 이미지 개수
TARGET_COUNT = 100 
# 이미지 확장자 목록 (필요하면 추가)
IMG_EXTENSIONS = ['.jpg', '.png', '.jpeg', '.bmp']
# ==========================================

def reduce_dataset():
    current_dir = os.getcwd()
    print(f"작업 폴더: {current_dir}")

    # 1. 모든 이미지 파일 찾기
    all_images = []
    for ext in IMG_EXTENSIONS:
        # 대소문자 구분 없이 찾기 (Linux 환경 고려)
        all_images.extend(glob.glob(os.path.join(current_dir, f"*{ext}")))
        all_images.extend(glob.glob(os.path.join(current_dir, f"*{ext.upper()}")))
    
    # 중복 제거 (확장자 대소문자 이슈 방지)
    all_images = list(set(all_images))
    total_files = len(all_images)

    if total_files <= TARGET_COUNT:
        print(f"\n[알림] 현재 이미지 개수({total_files}장)가 목표({TARGET_COUNT}장)보다 적거나 같습니다.")
        print("삭제할 파일이 없습니다.")
        return

    print(f"\n총 {total_files}장의 이미지를 발견했습니다.")
    print(f"무작위로 {TARGET_COUNT}장만 남기고, 나머지 {total_files - TARGET_COUNT}장은 삭제합니다.")

    # 2. 사용자 확인 (안전장치)
    check = input(">>> 정말로 삭제하시겠습니까? (y/n): ")
    if check.lower() != 'y':
        print("작업을 취소합니다.")
        return

    # 3. 셔플 및 리스트 분리
    random.shuffle(all_images)
    
    files_to_keep = all_images[:TARGET_COUNT]  # 앞의 100개 (남길 것)
    files_to_delete = all_images[TARGET_COUNT:] # 나머지 (지울 것)

    # 4. 삭제 실행
    deleted_count = 0
    print("\n삭제 시작...")
    
    for img_path in files_to_delete:
        try:
            # 4-1. 이미지 삭제
            os.remove(img_path)
            
            # 4-2. 짝꿍 라벨 파일(.txt) 찾아서 삭제
            # 확장자를 제거하고 .txt 붙이기
            file_root, _ = os.path.splitext(img_path)
            txt_path = file_root + ".txt"
            
            if os.path.exists(txt_path):
                os.remove(txt_path)
            
            deleted_count += 1
            
        except Exception as e:
            print(f"삭제 실패: {img_path} ({e})")

    print(f"\n완료! 총 {deleted_count}세트(이미지+라벨)를 삭제했습니다.")
    print(f"남은 이미지: {len(files_to_keep)}장")

if __name__ == "__main__":
    reduce_dataset()