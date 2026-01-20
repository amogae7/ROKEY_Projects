#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float32MultiArray
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

env_path = os.path.expanduser('/home/wook/ros2_ws/src/DoosanBootcampCol2/dsr_rokey/pick_and_place_voice/resource/.env')
load_dotenv(dotenv_path=env_path)

if not os.getenv("OPENAI_API_KEY"):
    print(f"❌ Error: .env 파일을 찾을 수 없거나 OPENAI_API_KEY가 없습니다.")
    sys.exit(1)

class SmartManagerNode(Node):
    def __init__(self):
        super().__init__('smart_manager_node')
        
        # =========================================================
        # 1. Publishers
        # =========================================================
        self.publisher_dispose = self.create_publisher(String, '/part_n_bad_dispose', 10)
        self.publisher_show = self.create_publisher(String, '/part_n_bad_show', 10)
        self.publisher_stop = self.create_publisher(String, '/robot_stop', 10)
        self.publisher_gripper = self.create_publisher(String, '/gripper_control', 10)
        self.publisher_target_update = self.create_publisher(String, '/target_update', 10)
        self.publisher_resume = self.create_publisher(String, '/robot_resume', 10)

        # [Fin 전용] 좌표 중계 퍼블리셔 (로봇 제어용)
        self.coord_pubs = {
            'part_1_bad': self.create_publisher(Float32MultiArray, '/part_1_bad_coord', 10),
            'part_2_bad': self.create_publisher(Float32MultiArray, '/part_2_bad_coord', 10),
            'part_3_bad': self.create_publisher(Float32MultiArray, '/part_3_bad_coord', 10),
        }
        # [Fin 전용] 개별 Dispose 명령 퍼블리셔
        self.dispose_pubs = {
            'part_1_bad': self.create_publisher(String, '/part_1_bad_dispose', 10),
            'part_2_bad': self.create_publisher(String, '/part_2_bad_dispose', 10),
            'part_3_bad': self.create_publisher(String, '/part_3_bad_dispose', 10),
        }

        # =========================================================
        # 2. Subscribers (소스별로 별개 수신)
        # =========================================================
        # (1) JSON 데이터 수신 (Integrated 기능용: 판독, 카운트)
        self.create_subscription(String, '/yolo_all_detect', self.yolo_callback, 10)
        
        # (2) 개별 좌표 데이터 수신 (Fin 기능용: 로봇 정밀 제어)
        self.create_subscription(Float32MultiArray, '/part_1_bad', lambda msg: self.bad_callback(msg, 'part_1_bad'), 10)
        self.create_subscription(Float32MultiArray, '/part_2_bad', lambda msg: self.bad_callback(msg, 'part_2_bad'), 10)
        self.create_subscription(Float32MultiArray, '/part_3_bad', lambda msg: self.bad_callback(msg, 'part_3_bad'), 10)

        # (3) 로봇 상태 수신
        self.create_subscription(String, '/robot_status', self.robot_status_callback, 10)

        # =========================================================
        # 3. Variables & Storage (저장소 분리)
        # =========================================================
        # [중요] 데이터를 섞지 않고 따로 저장합니다.
        self.detected_bad_parts_fin = {}   # from 개별 토픽 (로봇용)
        self.detected_bad_parts_json = {}  # from JSON (판독용)

        self.current_obj_count = 0
        self.startup_announced = False 
        self.stable_frame_count = 0 
        self.is_robot_at_home = True

        self.last_assembly_state = "none" 
        self.assembly_state_start_time = 0
        self.assembly_announced = False

        self.llm = ChatOpenAI(model="gpt-4o", temperature=0)
        self.prompt = ChatPromptTemplate.from_messages([
            ("system", """
            당신은 로봇 제어 관리자입니다. 사용자의 음성 명령을 분석하여 [행동 태그]와 [대상]을 출력하세요.
            
            [대상 매핑 규칙]
            - 합격, 패스, 완성품, 조립품 -> pass
            - 불량, 실패 -> non_pass
            - 1번: part_1_bad, 2번: part_2_bad, 3번: part_3_bad

            [행동 규칙]
            1. (개별) 처리/버리기/치우기 -> [ACTION_DISPOSE]
            2. (개별) 보여줘/확인/검사 -> [ACTION_SHOW]
            3. (전체) 모두 처리/전부 버리기/싹 다 치워 -> [ACTION_DISPOSE_ALL]
            4. (전체) 모두 확인/전부 보여줘/전체 검사 -> [ACTION_SHOW_ALL]
            5. 멈춰/정지/스탑 -> [ACTION_STOP]
            6. 진행해/재개해/계속해 -> [ACTION_RESUME]
            7. 집/홈/복귀 -> [ACTION_HOME]
            8. 전환해/바꿔줘 -> [ACTION_CHANGE]
            9. 그리퍼 열어 -> [ACTION_GRIPPER_OPEN]
            10. 그리퍼 닫아 -> [ACTION_GRIPPER_CLOSE]
            
            [출력 예시]
            - "1번 버려" -> "[ACTION_DISPOSE] part_1_bad"
            - "모두 처리해" -> "[ACTION_DISPOSE_ALL] 0"
            """),
            ("user", "{input}")
        ])
        self.output_parser = StrOutputParser()
        self.chain = self.prompt | self.llm | self.output_parser

        self.get_logger().info("🤖 Smart Manager Integrated Ready (Separated Data Sources)")
        
        self.voice_thread = threading.Thread(target=self.run_voice_recognition)
        self.voice_thread.daemon = True
        self.voice_thread.start()

    # [Source 1] 개별 토픽 데이터 -> fin 저장소에만 저장
    def bad_callback(self, msg, part_name):
        self.detected_bad_parts_fin[part_name] = msg.data

    def robot_status_callback(self, msg):
        if msg.data == "AT_HOME":
            self.is_robot_at_home = True
        else:
            self.is_robot_at_home = False
            self.last_assembly_state = "none"
            self.assembly_announced = False

    # [Source 2] JSON 데이터 -> json 저장소에만 저장 (좌표 섞지 않음)
    def yolo_callback(self, msg):
        try:
            data = json.loads(msg.data)
            
            # JSON 데이터 저장 (판독 로직용)
            self.detected_bad_parts_json = {} 
            for obj in data:
                if "bad" in obj['name']:
                    self.detected_bad_parts_json[obj['name']] = obj['coords']

            bad_parts = [obj for obj in data if "bad" in obj['name'] and "part" in obj['name']]
            self.current_obj_count = len(bad_parts)
            
            if self.is_robot_at_home and not self.startup_announced:
                self.stable_frame_count += 1
                if self.stable_frame_count > 10:
                    self.get_logger().info(f"🚀 초기 스캔 완료")
                    self.speak_status(self.current_obj_count) 
                    self.startup_announced = True

            self.check_assembly_result(data)

        except Exception as e:
            pass

    def check_assembly_result(self, data):
        if not self.is_robot_at_home:
            return

        has_pass = any(obj['name'] == 'pass' for obj in data)
        has_non_pass = any(obj['name'] == 'non_pass' for obj in data)

        current_state = "none"
        if has_pass and has_non_pass: current_state = "both"
        elif has_pass: current_state = "pass"
        elif has_non_pass: current_state = "non_pass"

        if current_state != self.last_assembly_state:
            self.last_assembly_state = current_state
            self.assembly_state_start_time = time.time()
            self.assembly_announced = False 
        
        if current_state != "none" and not self.assembly_announced:
            if time.time() - self.assembly_state_start_time > 5.0:
                self.speak_assembly_result(current_state)
                self.assembly_announced = True

    def speak_assembly_result(self, state):
        text = ""
        if state == "pass": text = "조립이 아주 잘되었으니 합격입니다."
        elif state == "non_pass": text = "아이고, 이런. 조립이 잘못된 상태입니다. 다시 조립하셔야 합니다."
        elif state == "both": text = "조립품은 한 개만 제시해주셔야 합니다."
        
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
            match = re.search(r'part_(\d+)_bad', intent)
            if match: target = match.group()
            else:
                num_match = re.search(r'\d+', intent)
                if num_match: target = f"part_{num_match.group()}_bad"

        # -------------------------------------------------------------
        # [Extended] 일괄 처리
        # -------------------------------------------------------------
        if "[ACTION_DISPOSE_ALL]" in intent:
            if not self.detected_bad_parts_fin:
                self.generate_and_play_tts("처리할 부품이 없습니다.")
                return
            
            self.generate_and_play_tts("모두 처리하겠습니다.")
            
            # (1) [Fin 저장소 사용] 모든 좌표 중계
            for part_name, coords in self.detected_bad_parts_fin.items():
                if part_name in self.coord_pubs:
                    c_msg = Float32MultiArray()
                    c_msg.data = coords
                    self.coord_pubs[part_name].publish(c_msg)
            
            # (2) [Fin 퍼블리셔 사용] 모든 Dispose 명령 전송
            cmd_msg = String(data="dispose_all")
            for part_name in self.detected_bad_parts_fin.keys():
                if part_name in self.dispose_pubs:
                    self.dispose_pubs[part_name].publish(cmd_msg)

        elif "[ACTION_SHOW_ALL]" in intent:
             self.generate_and_play_tts("전체 부품을 확인합니다.")
             # (1) [Fin 저장소 사용] 좌표 일괄 전송
             for part_name, coords in self.detected_bad_parts_fin.items():
                 if part_name in self.coord_pubs:
                    c_msg = Float32MultiArray()
                    c_msg.data = coords
                    self.coord_pubs[part_name].publish(c_msg)
             
             # (2) Show 명령
             msg.data = "show_all"
             self.publisher_show.publish(msg)

        # -------------------------------------------------------------
        # [Integrated] 개별 처리
        # -------------------------------------------------------------
        elif "[ACTION_DISPOSE]" in intent:
            if target != "none":
                # [Fin 저장소 사용] 좌표 먼저 중계
                if target in self.detected_bad_parts_fin and target in self.coord_pubs:
                    c_msg = Float32MultiArray()
                    c_msg.data = self.detected_bad_parts_fin[target]
                    self.coord_pubs[target].publish(c_msg)
                
                # 기존 통합 토픽
                msg.data = target
                self.publisher_dispose.publish(msg)
                
                # [Fin 퍼블리셔 사용] 개별 Dispose 토픽도 함께 발행
                if target in self.dispose_pubs:
                    cmd_msg = String(data="dispose")
                    self.dispose_pubs[target].publish(cmd_msg)

        elif "[ACTION_SHOW]" in intent:
            if target != "none":
                # [Fin 저장소 사용] 좌표 중계
                if target in self.detected_bad_parts_fin and target in self.coord_pubs:
                    c_msg = Float32MultiArray()
                    c_msg.data = self.detected_bad_parts_fin[target]
                    self.coord_pubs[target].publish(c_msg)
                    
                msg.data = target
                self.publisher_show.publish(msg)

        elif "[ACTION_STOP]" in intent:
            self.publisher_stop.publish(String(data="stop"))
            self.generate_and_play_tts("정지합니다.")
        
        elif "[ACTION_RESUME]" in intent:
            self.publisher_resume.publish(String(data="resume"))
            self.generate_and_play_tts("재개합니다.")
        
        elif "[ACTION_HOME]" in intent:
            msg.data = "home"
            self.publisher_show.publish(msg)
            self.generate_and_play_tts("홈으로 갑니다.")

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