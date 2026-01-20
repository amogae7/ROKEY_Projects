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

env_path = os.path.expanduser('~/ros2_ws/.env')
load_dotenv(dotenv_path=env_path)

if not os.getenv("OPENAI_API_KEY"):
    print(f"❌ Error: .env 파일을 찾을 수 없거나 OPENAI_API_KEY가 없습니다.")
    sys.exit(1)

class SmartManagerNode(Node):
    def __init__(self):
        super().__init__('smart_manager_node')
        
        # 퍼블리셔
        self.publisher_dispose = self.create_publisher(String, '/part_n_bad_dispose', 10)
        self.publisher_show = self.create_publisher(String, '/part_n_bad_show', 10)
        self.publisher_stop = self.create_publisher(String, '/robot_stop', 10)
        self.publisher_gripper = self.create_publisher(String, '/gripper_control', 10)
        self.publisher_target_update = self.create_publisher(String, '/target_update', 10)
        
        # [NEW] 재개 명령 퍼블리셔 추가
        self.publisher_resume = self.create_publisher(String, '/robot_resume', 10)

        self.llm = ChatOpenAI(model="gpt-4o", temperature=0)
        
        # [핵심] '재개' 관련 프롬프트 추가
        self.prompt = ChatPromptTemplate.from_messages([
            ("system", """
            당신은 로봇 제어 관리자입니다. 사용자의 음성 명령을 분석하여 [행동 태그]와 [대상 번호]를 출력하세요.
            
            [행동 규칙]
            1. 처리/버리기/치우기/폐기 -> [ACTION_DISPOSE]
            2. 보여줘/찾아줘/어딨어/확인/검사 -> [ACTION_SHOW]
            3. 멈춰/정지/스탑/잠깐 -> [ACTION_STOP] (일시 정지)
            4. 진행해/재개해/계속해/고/다시시작 -> [ACTION_RESUME] (다시 시작)
            5. 집/홈/복귀 -> [ACTION_HOME]
            6. 전환해/바꿔줘 -> [ACTION_CHANGE]
            7. 그리퍼 열어 -> [ACTION_GRIPPER_OPEN]
            8. 그리퍼 닫아 -> [ACTION_GRIPPER_CLOSE]

            [번호 매핑 규칙]
            - 1번: '1번', '하나', '원', '파트원' -> 1
            - 2번: '2번', '둘', '투', '파트투' -> 2
            - 3번: '3번', '셋', '쓰리', '파트쓰리' -> 3
            - 번호 없음 -> 0
              
            [출력 형식]
            예시: "[ACTION_DISPOSE] 2", "[ACTION_RESUME] 0"
            """),
            ("user", "{input}")
        ])
        self.output_parser = StrOutputParser()
        self.chain = self.prompt | self.llm | self.output_parser

        self.get_logger().info("🤖 Smart Manager v7 Ready (Pause & Resume Supported)")
        self.run_voice_recognition()

    def run_voice_recognition(self):
        recognizer = sr.Recognizer()
        mic = sr.Microphone()
        
        with mic as source:
            recognizer.adjust_for_ambient_noise(source, duration=1.0)
        
        self.get_logger().info("👂 명령 대기 중... ('멈춰', '재개해' 등)")

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
            self.get_logger().info(f"👀 [명령] {target_num}번 확인")
            
        elif "[ACTION_STOP]" in intent:
            msg.data = "stop"
            self.publisher_stop.publish(msg)
            self.get_logger().warn("⏸️ [명령] 일시 정지")

        # [NEW] 재개 명령
        elif "[ACTION_RESUME]" in intent:
            msg.data = "resume"
            self.publisher_resume.publish(msg)
            self.get_logger().info("▶️ [명령] 작업 재개")
            
        elif "[ACTION_HOME]" in intent:
            msg.data = "home"
            self.publisher_show.publish(msg)
            self.get_logger().info("🏠 [명령] 홈 복귀")

        elif "[ACTION_CHANGE]" in intent:
            msg.data = target_object
            self.publisher_target_update.publish(msg)
            self.get_logger().info(f"🔄 [명령] 타겟 변경 ({target_num}번)")

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
