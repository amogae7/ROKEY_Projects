#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import speech_recognition as sr
from langchain_openai import ChatOpenAI
from langchain_core.prompts import ChatPromptTemplate
from langchain_core.output_parsers import StrOutputParser
from dotenv import load_dotenv
import os
import sys
import re 
import json
import threading
import time
from gtts import gTTS

env_path = os.path.expanduser('~/ros2_ws/.env')
load_dotenv(dotenv_path=env_path)

if not os.getenv("OPENAI_API_KEY"):
    print(f"❌ Error: .env 파일을 찾을 수 없거나 OPENAI_API_KEY가 없습니다.")
    sys.exit(1)

class SmartManagerNode(Node):
    def __init__(self):
        super().__init__('smart_manager_node')
        
        # ---------------------------------------------------------
        # [Publishers]
        # ---------------------------------------------------------
        self.publisher_dispose = self.create_publisher(String, '/part_n_bad_dispose', 10)
        self.publisher_show = self.create_publisher(String, '/part_n_bad_show', 10)
        self.publisher_stop = self.create_publisher(String, '/robot_stop', 10)
        self.publisher_gripper = self.create_publisher(String, '/gripper_control', 10)
        self.publisher_target_update = self.create_publisher(String, '/target_update', 10)
        self.publisher_resume = self.create_publisher(String, '/robot_resume', 10)

        # ---------------------------------------------------------
        # [Subscribers] YOLO 데이터 수신
        # ---------------------------------------------------------
        self.create_subscription(String, '/yolo_all_detect', self.yolo_callback, 10)

        # ---------------------------------------------------------
        # [AI & Logic Setup]
        # ---------------------------------------------------------
        self.llm = ChatOpenAI(model="gpt-4o", temperature=0)
        self.prompt = ChatPromptTemplate.from_messages([
            ("system", """
            당신은 로봇 제어 관리자입니다. 사용자의 음성 명령을 분석하여 [행동 태그]와 [대상 번호]를 출력하세요.
            [행동 규칙]
            1. 처리/버리기/치우기/폐기 -> [ACTION_DISPOSE]
            2. 보여줘/찾아줘/어딨어/확인/검사 -> [ACTION_SHOW]
            3. 멈춰/정지/스탑/잠깐 -> [ACTION_STOP]
            4. 진행해/재개해/계속해/고/다시시작 -> [ACTION_RESUME]
            5. 집/홈/복귀 -> [ACTION_HOME]
            6. 전환해/바꿔줘 -> [ACTION_CHANGE]
            7. 그리퍼 열어 -> [ACTION_GRIPPER_OPEN]
            8. 그리퍼 닫아 -> [ACTION_GRIPPER_CLOSE]
            [번호 매핑 규칙]
            - 1번: 1, 2번: 2, 3번: 3, 없음: 0
            [출력 형식] 예: "[ACTION_DISPOSE] 2"
            """),
            ("user", "{input}")
        ])
        self.output_parser = StrOutputParser()
        self.chain = self.prompt | self.llm | self.output_parser

        # ---------------------------------------------------------
        # [TTS & Count Logic Variables]
        # ---------------------------------------------------------
        self.current_obj_count = 0  # 초기값 0으로 설정
        
        # 이전 코드에 있던 안정화 로직 변수들은 이제 불필요하지만, 
        # 혹시 모를 노이즈 제거를 위해 카운팅 로직은 남겨둡니다.
        self.stability_counter = 0       
        self.stable_count_threshold = 10 

        # [추가] 시작하자마자 안내 멘트를 했는지 체크하는 변수
        self.startup_announced = False 
        self.stable_frame_count = 0  # 안정화 카운트

        self.get_logger().info("🤖 Smart Manager Ready (Bad Parts Only Count)")
        
        self.voice_thread = threading.Thread(target=self.run_voice_recognition)
        self.voice_thread.daemon = True
        self.voice_thread.start()

    # -------------------------------------------------------------
    # [수정됨] 이제 여기서 말을 하지 않고, 개수만 셉니다.
    # -------------------------------------------------------------
    def yolo_callback(self, msg):
        try:
            data = json.loads(msg.data)
            
            # 이름에 'bad'가 포함된 것만 리스트로 다시 만듦
            bad_parts = [obj for obj in data if "bad" in obj['name']]
            
            count = len(bad_parts) # 현재 프레임의 개수
            
            # [수정] 단순히 현재 개수를 최신 상태로 업데이트합니다.
            # (원한다면 노이즈 제거를 위해 stability 로직을 써도 되지만, 
            # 여기서는 즉각적인 반영을 위해 바로 대입합니다.)
            self.current_obj_count = count
            
            # 🚨 중요: 여기서 self.speak_status()를 호출하던 코드를 삭제했습니다.

            # [추가된 로직] 프로그램 시작 후 첫 자동 안내
            if not self.startup_announced:
                # 데이터가 들어오고 있는지 확인 (노이즈 방지용 10프레임 대기)
                self.stable_frame_count += 1
                
                if self.stable_frame_count > 10:
                    self.get_logger().info(f"🚀 초기 스캔 완료: 불량품 {count}개 감지됨")
                    self.speak_status(count) # 안내 방송 실행
                    self.startup_announced = True # 이제 더 이상 자동 안내 안 함

        except Exception as e:
            pass

    # -------------------------------------------------------------
    # TTS 방송 함수
    # -------------------------------------------------------------
    def speak_status(self, count):
        text = ""
        num_map = {1: "한", 2: "두", 3: "세", 4: "네", 5: "다섯"}
        
        if count > 0:
            korean_num = num_map.get(count, str(count))
            text = f"현재 불량품이 총 {korean_num}개 감지됐습니다. 번호를 말씀해주시면 처리해 드리겠습니다."
        else:
            text = "불량품이 더 이상 없습니다. 조립 공정을 시작할 수 있습니다."

        self.get_logger().info(f"📢 TTS 방송: {text}")
        
        try:
            tts = gTTS(text=text, lang='ko')
            filename = 'voice_guide.mp3'
            tts.save(filename)
            os.system(f'mpg321 {filename} --quiet')
        except Exception as e:
            self.get_logger().error(f"TTS Error: {e}")

    # -------------------------------------------------------------
    # 음성 인식 루프
    # -------------------------------------------------------------
    def run_voice_recognition(self):
        recognizer = sr.Recognizer()
        mic = sr.Microphone()
        
        with mic as source:
            recognizer.adjust_for_ambient_noise(source, duration=1.0)
        
        self.get_logger().info("👂 음성 명령 대기 중...")

        while rclpy.ok():
            try:
                with mic as source:
                    audio = recognizer.listen(source, timeout=None, phrase_time_limit=5)
                
                text = recognizer.recognize_google(audio, language='ko-KR')
                self.get_logger().info(f"🗣️ 인식된 단어: \"{text}\"")
                
                intent = self.chain.invoke({"input": text}).strip()
                self.get_logger().info(f"🧠 AI 해석: {intent}")
                
                self.execute_robot_action(intent)
                
            except sr.WaitTimeoutError: pass
            except sr.UnknownValueError: pass 
            except Exception as e: self.get_logger().error(f"Audio Error: {e}")

    # -------------------------------------------------------------
    # [수정됨] 홈 복귀 시에만 멘트 실행
    # -------------------------------------------------------------
    def execute_robot_action(self, intent):
        msg = String()
        target_num = "1"
        match = re.search(r'\d+', intent)
        if match: target_num = match.group()
        target_object = f"part_{target_num}_bad"

        if "[ACTION_DISPOSE]" in intent:
            msg.data = target_object
            self.publisher_dispose.publish(msg)
        
        elif "[ACTION_SHOW]" in intent:
            msg.data = target_object
            self.publisher_show.publish(msg)
        
        elif "[ACTION_STOP]" in intent:
            msg.data = "stop"
            self.publisher_stop.publish(msg)
        
        elif "[ACTION_RESUME]" in intent:
            msg.data = "resume"
            self.publisher_resume.publish(msg)
        
        # ★★★ 여기가 수정된 부분입니다 ★★★
        elif "[ACTION_HOME]" in intent:
            # 1. 로봇에게 홈으로 가라고 명령
            msg.data = "home"
            self.publisher_show.publish(msg)
            self.get_logger().info("🏠 홈 복귀 명령 전송됨")
            
            # 2. 1초 대기
            time.sleep(3.0)
            
            # 3. yolo_callback에서 세어둔 개수로 방송
            self.speak_status(self.current_obj_count)
        # ★★★★★★★★★★★★★★★★★★★★★★

        elif "[ACTION_CHANGE]" in intent:
            msg.data = target_object
            self.publisher_target_update.publish(msg)
        
        elif "[ACTION_GRIPPER_OPEN]" in intent:
            msg.data = "open"
            self.publisher_gripper.publish(msg)
        
        elif "[ACTION_GRIPPER_CLOSE]" in intent:
            msg.data = "close"
            self.publisher_gripper.publish(msg)
            
def main(args=None):
    rclpy.init(args=args)
    node = SmartManagerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt: pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()