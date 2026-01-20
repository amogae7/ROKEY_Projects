import os
import glob
from collections import Counter

def check_label_distribution():
    # 현재 폴더 안에 train/labels 경로가 있다고 가정
    label_dir = os.path.join(os.getcwd(), 'train', 'labels')
    
    txt_files = glob.glob(os.path.join(label_dir, "*.txt"))
    
    if not txt_files:
        print(f"❌ 오류: '{label_dir}' 경로에 .txt 파일이 하나도 없습니다!")
        return

    print(f"총 {len(txt_files)}개의 라벨 파일을 검사합니다...")
    
    class_counts = Counter()
    
    for txt_file in txt_files:
        with open(txt_file, 'r') as f:
            lines = f.readlines()
            for line in lines:
                parts = line.strip().split()
                if len(parts) > 0:
                    class_id = int(parts[0]) # 맨 앞 숫자 가져오기
                    class_counts[class_id] += 1

    print("\n" + "="*30)
    print("📊 클래스별 데이터 개수 현황")
    print("="*30)
    
    # 0번부터 5번까지 순서대로 출력
    for i in range(6):
        count = class_counts.get(i, 0)
        print(f"👉 클래스 {i}번: {count}개")
    
    print("="*30)
    
    # 진단 결과
    if len(class_counts) == 1 and 0 in class_counts:
        print("🚨 [문제 발견] 모든 데이터가 '0번 클래스'로 되어 있습니다!")
        print("   -> json2yolo 변환 코드를 수정해서 다시 변환해야 합니다.")
    elif len(class_counts) > 1:
        print("✅ 데이터가 정상적으로 분포되어 있습니다. (학습 설정 문제일 수 있음)")

if __name__ == "__main__":
    check_label_distribution()