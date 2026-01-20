import os
import glob
import random
import shutil

# ==========================================
# [사용자 설정] 비율 설정 (합이 1.0이 되게 하세요)
# ==========================================
TRAIN_RATIO = 0.7  # 학습용 70%
VALID_RATIO = 0.2  # 검증용 20%
TEST_RATIO  = 0.1  # 테스트용 10%

IMAGE_EXTENSIONS = ['.jpg', '.png', '.jpeg', '.bmp']
# ==========================================

def split_dataset_3way():
    current_dir = os.getcwd()
    
    # 1. 이미지 파일 모으기
    image_files = []
    for ext in IMAGE_EXTENSIONS:
        image_files.extend(glob.glob(os.path.join(current_dir, f'*{ext}')))
        image_files.extend(glob.glob(os.path.join(current_dir, f'*{ext.upper()}')))
    
    # 중복 제거 및 정렬
    image_files = list(set(image_files))
    total_count = len(image_files)
    
    if total_count == 0:
        print("이미지 파일이 없습니다.")
        return

    print(f"총 {total_count}개의 이미지를 발견했습니다.")

    # 2. 섞기
    random.shuffle(image_files)

    # 3. 개수 계산
    train_count = int(total_count * TRAIN_RATIO)
    valid_count = int(total_count * VALID_RATIO)
    # 남은 건 전부 테스트로 (그래야 1장이 남거나 모자라지 않음)
    test_count = total_count - train_count - valid_count

    # 4. 리스트 자르기
    train_files = image_files[:train_count]
    valid_files = image_files[train_count : train_count + valid_count]
    test_files  = image_files[train_count + valid_count :]

    print(f"비율 설정 -> Train: {TRAIN_RATIO*100}% / Valid: {VALID_RATIO*100}% / Test: {TEST_RATIO*100}%")
    print(f"파일 개수 -> Train: {len(train_files)}개 / Valid: {len(valid_files)}개 / Test: {len(test_files)}개")

    # 5. 폴더 3개 만들기
    folders = [
        'train/images', 'train/labels',
        'valid/images', 'valid/labels',
        'test/images',  'test/labels'
    ]
    
    for folder in folders:
        os.makedirs(os.path.join(current_dir, folder), exist_ok=True)

    # 6. 파일 이동 함수
    def move_files(files, split_type):
        for img_path in files:
            filename = os.path.basename(img_path)
            file_root, _ = os.path.splitext(filename)
            
            # 라벨 파일 경로 (.txt)
            label_filename = f"{file_root}.txt"
            label_path = os.path.join(current_dir, label_filename)

            # 이동할 경로
            dest_img = os.path.join(current_dir, split_type, 'images', filename)
            dest_label = os.path.join(current_dir, split_type, 'labels', label_filename)

            # 이동 실행
            shutil.move(img_path, dest_img)
            
            if os.path.exists(label_path):
                shutil.move(label_path, dest_label)

    # 7. 실제 이동
    print("\n파일 이동 시작...")
    move_files(train_files, 'train')
    move_files(valid_files, 'valid')
    move_files(test_files,  'test')
    
    print("작업 완료! 폴더가 train, valid, test로 나뉘었습니다.")

if __name__ == '__main__':
    split_dataset_3way()