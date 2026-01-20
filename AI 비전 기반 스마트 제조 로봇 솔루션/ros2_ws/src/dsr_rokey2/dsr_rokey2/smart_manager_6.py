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

# .env 파일 로드
env_path = os.path.expanduser('~/ros2_ws/.env')
load_dotenv(dotenv_path=env_path)

if not os.getenv("OPENAI_API_KEY"):
    print(f"❌ Error: .env 파일을 찾을 수 없거나 OPENAI_API_KEY가 없습니다.")
    sys.exit(1)

class SmartManagerNode(Node):
    def __init__(self):
        super().__init__('smart_manager_node')
        
        # 퍼블리셔 설정
        self.publisher_dispose = self.create_publisher(String, '/part_n_bad_dispose', 10)
        self.publisher_show = self.create_publisher(String, '/part_n_bad_show', 10)
        self.publisher_stop = self.create_publisher(String, '/robot_stop', 10)
        
        # [NEW] 그리퍼 제어 및 타겟 변경 전용 퍼블리셔
        self.publisher_gripper = self.create_publisher(String, '/gripper_control', 10)
        self.publisher_target_update = self.create_publisher(String, '/target_update', 10)

        # LangChain 설정
        self.llm = ChatOpenAI(model="gpt-4o", temperature=0)
        
        # [핵심 수정] 프롬프트에 그리퍼 제어 및 타겟 전환 규칙 추가
        self.prompt = ChatPromptTemplate.from_messages([
            ("system", """
            당신은 로봇 제어 관리자입니다. 사용자의 음성 명령을 분석하여 [행동 태그]와 [대상 번호]를 출력하세요.
            
            [행동 규칙]
            1. 처리/버리기/치우기/폐기 -> [ACTION_DISPOSE]
            2. 보여줘/어딨어/확인/검사 -> [ACTION_SHOW] (로봇 이동 함)
            3. 멈춰/정지/스탑 -> [ACTION_STOP]
            4. 집/홈/복귀/원위치 -> [ACTION_HOME]
            5. 전환해/바꿔줘/변경해 -> [ACTION_CHANGE] (로봇 이동 안함, 타겟만 변경)
            6. 그리퍼 열어/펴/놔 -> [ACTION_GRIPPER_OPEN]
            7. 그리퍼 닫아/잡아 -> [ACTION_GRIPPER_CLOSE]

            [번호 매핑 규칙]
            - 1번: '1번', '일번', '하나', '원', '파트원' -> 1
            - 2번: '2번', '이번', '둘', '투', '파트투' -> 2
            - 3번: '3번', '삼번', '셋', '쓰리', '파트쓰리' -> 3
            - 번호가 없으면 0으로 출력 (그리퍼 명령 등)
              
            [출력 형식]
            반드시 "[TAG] 번호" 형태로 출력하세요.
            예시: "[ACTION_DISPOSE] 2", "[ACTION_CHANGE] 1", "[ACTION_GRIPPER_OPEN] 0"
            """),
            ("user", "{input}")
        ])
        self.output_parser = StrOutputParser()
        self.chain = self.prompt | self.llm | self.output_parser

        self.get_logger().info("🤖 Smart Manager Ready! (Gripper & Target Switch Added)")
        self.run_voice_recognition()

    def run_voice_recognition(self):
        recognizer = sr.Recognizer()
        mic = sr.Microphone()
        
        self.get_logger().info("🎤 마이크 보정 중...")
        with mic as source:
            recognizer.adjust_for_ambient_noise(source, duration=1.0)
        
        self.get_logger().info("👂 명령 대기 중...")

        while rclpy.ok():
            try:
                with mic as source:
                    audio = recognizer.listen(source, timeout=None, phrase_time_limit=5)
                
                text = recognizer.recognize_google(audio, language='ko-KR')
                self.get_logger().info(f"🗣️ 인식된 단어: \"{text}\"")
                
                intent = self.chain.invoke({"input": text}).strip()
                self.get_logger().info(f"🧠 AI 해석결과: {intent}")
                
                self.execute_robot_action(intent)
                
            except sr.WaitTimeoutError: pass
            except sr.UnknownValueError: pass 
            except Exception as e: self.get_logger().error(f"에러: {e}")

    def execute_robot_action(self, intent):
        msg = String()
        
        target_num = "1"
        match = re.search(r'\d+', intent)
        if match: target_num = match.group()
        
        target_object = f"part_{target_num}_bad"

        if "[ACTION_DISPOSE]" in intent:
            msg.data = target_object
            self.publisher_dispose.publish(msg)
            self.get_logger().info(f"🚀 [명령] {target_num}번 처리")
            
        elif "[ACTION_SHOW]" in intent:
            msg.data = target_object
            self.publisher_show.publish(msg)
            self.get_logger().info(f"👀 [명령] {target_num}번 확인 (이동)")
            
        elif "[ACTION_STOP]" in intent:
            msg.data = "stop"
            self.publisher_stop.publish(msg)
            self.get_logger().warn("🚨 [명령] 정지")
            
        elif "[ACTION_HOME]" in intent:
            msg.data = "home"
            self.publisher_show.publish(msg)
            self.get_logger().info("🏠 [명령] 홈 복귀")

        # [NEW] 타겟만 변경 (로봇은 가만히)
        elif "[ACTION_CHANGE]" in intent:
            msg.data = target_object
            self.publisher_target_update.publish(msg)
            self.get_logger().info(f"🔄 [명령] 타겟만 {target_num}번으로 변경 (로봇 부동)")

        # [NEW] 그리퍼 제어
        elif "[ACTION_GRIPPER_OPEN]" in intent:
            msg.data = "open"
            self.publisher_gripper.publish(msg)
            self.get_logger().info("🖐 [명령] 그리퍼 열기")
            
        elif "[ACTION_GRIPPER_CLOSE]" in intent:
            msg.data = "close"
            self.publisher_gripper.publish(msg)
            self.get_logger().info("✊ [명령] 그리퍼 닫기")
            
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