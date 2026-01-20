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
        
        # Publishers
        self.publisher_dispose = self.create_publisher(String, '/part_n_bad_dispose', 10)
        self.publisher_show = self.create_publisher(String, '/part_n_bad_show', 10)
        self.publisher_stop = self.create_publisher(String, '/robot_stop', 10)
        self.publisher_gripper = self.create_publisher(String, '/gripper_control', 10)
        self.publisher_target_update = self.create_publisher(String, '/target_update', 10)
        self.publisher_resume = self.create_publisher(String, '/robot_resume', 10)

        # Subscribers
        self.create_subscription(String, '/yolo_all_detect', self.yolo_callback, 10)
        
        # [NEW] 로봇 상태 수신 (위치 확인용)
        self.create_subscription(String, '/robot_status', self.robot_status_callback, 10)

        # AI Setup
        self.llm = ChatOpenAI(model="gpt-4o", temperature=0)
        self.prompt = ChatPromptTemplate.from_messages([
            ("system", """
            당신은 로봇 제어 관리자입니다. 사용자의 음성 명령을 분석하여 [행동 태그]와 [대상]을 출력하세요.
            
            [대상 매핑 규칙]
            - 합격, 패스, 완성품, 조립품 -> pass
            - 불량, 실패 -> non_pass
            - 1번: part_1_bad, 2번: part_2_bad, 3번: part_3_bad

            [행동 규칙]
            1. 처리/버리기/치우기/폐기 -> [ACTION_DISPOSE]
            2. 보여줘/찾아줘/어딨어/확인/검사 -> [ACTION_SHOW]
            3. 멈춰/정지/스탑/잠깐 -> [ACTION_STOP]
            4. 진행해/재개해/계속해/고/다시시작 -> [ACTION_RESUME]
            5. 집/홈/복귀 -> [ACTION_HOME]
            6. 전환해/바꿔줘 -> [ACTION_CHANGE]
            7. 그리퍼 열어 -> [ACTION_GRIPPER_OPEN]
            8. 그리퍼 닫아 -> [ACTION_GRIPPER_CLOSE]
            
            [출력 형식] 예: "[ACTION_SHOW] pass", "[ACTION_DISPOSE] part_1_bad"
            """),
            ("user", "{input}")
        ])
        self.output_parser = StrOutputParser()
        self.chain = self.prompt | self.llm | self.output_parser

        # Variables
        self.current_obj_count = 0
        self.startup_announced = False 
        self.stable_frame_count = 0 

        # 조립 결과 판독용 변수
        self.last_assembly_state = "none" 
        self.assembly_state_start_time = 0
        self.assembly_announced = False
        
        # [NEW] 로봇 위치 플래그 (기본값 True)
        self.is_robot_at_home = True

        self.get_logger().info("🤖 Smart Manager Ready (Safe Assembly Check)")
        
        self.voice_thread = threading.Thread(target=self.run_voice_recognition)
        self.voice_thread.daemon = True
        self.voice_thread.start()

    # -------------------------------------------------------------
    # [NEW] 로봇 상태 콜백 (이동 중일 때 판독 금지)
    # -------------------------------------------------------------
    def robot_status_callback(self, msg):
        if msg.data == "AT_HOME":
            self.is_robot_at_home = True
            # 집에 왔을 때 즉시 방송하지 않고, YOLO 콜백에서 5초 카운트 후 방송함
        else:
            self.is_robot_at_home = False
            # 이동 중에는 상태를 초기화하여, 다시 집에 왔을 때 방송할 준비
            self.last_assembly_state = "none"
            self.assembly_announced = False

    def yolo_callback(self, msg):
        try:
            data = json.loads(msg.data)
            
            # 1. 불량품 개수 카운트
            bad_parts = [obj for obj in data if "bad" in obj['name'] and "part" in obj['name']]
            self.current_obj_count = len(bad_parts)
            
            # 초기 안내 방송 (집에 있을 때만)
            if self.is_robot_at_home and not self.startup_announced:
                self.stable_frame_count += 1
                if self.stable_frame_count > 10:
                    self.get_logger().info(f"🚀 초기 스캔 완료")
                    self.speak_status(self.current_obj_count) 
                    self.startup_announced = True

            # 2. 조립 결과 판독 (집에 있을 때만 실행!)
            self.check_assembly_result(data)

        except Exception as e:
            pass

    def check_assembly_result(self, data):
        # [핵심] 로봇이 집에 없으면 절대 판독하지 않음
        if not self.is_robot_at_home:
            return

        has_pass = any(obj['name'] == 'pass' for obj in data)
        has_non_pass = any(obj['name'] == 'non_pass' for obj in data)

        current_state = "none"
        if has_pass and has_non_pass:
            current_state = "both"
        elif has_pass:
            current_state = "pass"
        elif has_non_pass:
            current_state = "non_pass"

        # 상태 변화 감지
        if current_state != self.last_assembly_state:
            self.last_assembly_state = current_state
            self.assembly_state_start_time = time.time()
            self.assembly_announced = False 
        
        # 상태 유지 확인 (5초)
        if current_state != "none" and not self.assembly_announced:
            if time.time() - self.assembly_state_start_time > 5.0:
                self.speak_assembly_result(current_state)
                self.assembly_announced = True

    def speak_assembly_result(self, state):
        text = ""
        if state == "pass":
            text = "조립이 아주 잘되었으니 합격입니다."
        elif state == "non_pass":
            text = "아이고, 이런. 조립이 잘못된 상태입니다. 다시 조립하셔야 합니다."
        elif state == "both":
            text = "조립품은 한 개만 제시해주셔야 합니다."
        
        if text:
            self.get_logger().info(f"📢 조립 판독 방송: {text}")
            self.generate_and_play_tts(text)

    def speak_status(self, count):
        text = ""
        num_map = {1: "한", 2: "두", 3: "세", 4: "네", 5: "다섯"}
        if count > 0:
            korean_num = num_map.get(count, str(count))
            text = f"현재 불량품이 총 {korean_num}개 감지됐습니다. 번호를 말씀해주시면 처리해 드리겠습니다."
        else:
            text = "불량품이 더 이상 없습니다. 이제 조립 공정을 시작할 수 있습니다."
        
        self.get_logger().info(f"📢 상태 방송: {text}")
        self.generate_and_play_tts(text)

    def generate_and_play_tts(self, text):
        try:
            tts = gTTS(text=text, lang='ko')
            filename = 'voice_guide.mp3'
            tts.save(filename)
            os.system(f'mpg321 {filename} --quiet')
        except Exception as e:
            self.get_logger().error(f"TTS Error: {e}")

    def run_voice_recognition(self):
        recognizer = sr.Recognizer()
        mic = sr.Microphone()
        with mic as source: recognizer.adjust_for_ambient_noise(source, duration=1.0)
        self.get_logger().info("👂 음성 명령 대기 중...")

        while rclpy.ok():
            try:
                with mic as source:
                    audio = recognizer.listen(source, timeout=None, phrase_time_limit=5)
                text = recognizer.recognize_google(audio, language='ko-KR')
                self.get_logger().info(f"🗣️ 인식: \"{text}\"")
                
                intent = self.chain.invoke({"input": text}).strip()
                self.get_logger().info(f"🧠 해석: {intent}")
                self.execute_robot_action(intent)
            except: pass

    def execute_robot_action(self, intent):
        msg = String()
        target = "none"
        if "pass" in intent: target = "pass"
        elif "non_pass" in intent: target = "non_pass"
        else:
            match = re.search(r'part_\d+_bad', intent)
            if match: target = match.group()
            else:
                num_match = re.search(r'\d+', intent)
                if num_match: target = f"part_{num_match.group()}_bad"

        if "[ACTION_DISPOSE]" in intent:
            if target != "none":
                msg.data = target
                self.publisher_dispose.publish(msg)
        
        elif "[ACTION_SHOW]" in intent:
            if target != "none":
                msg.data = target
                self.publisher_show.publish(msg)
        
        elif "[ACTION_STOP]" in intent:
            self.publisher_stop.publish(String(data="stop"))
        
        elif "[ACTION_RESUME]" in intent:
            self.publisher_resume.publish(String(data="resume"))
        
        elif "[ACTION_HOME]" in intent:
            msg.data = "home"
            self.publisher_show.publish(msg)
            # 홈으로 가면 robot_status가 AT_HOME으로 바뀌면서 다시 판독 가능해짐
            self.get_logger().info("🏠 홈 복귀 명령")

        elif "[ACTION_GRIPPER_OPEN]" in intent:
            self.publisher_gripper.publish(String(data="open"))
        
        elif "[ACTION_GRIPPER_CLOSE]" in intent:
            self.publisher_gripper.publish(String(data="close"))
            
def main(args=None):
    rclpy.init(args=args)
    node = SmartManagerNode()
    try: rclpy.spin(node)
    except KeyboardInterrupt: pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()