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
import re # 숫자를 찾기 위한 정규식 라이브러리

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

        # LangChain 설정
        self.llm = ChatOpenAI(model="gpt-4o", temperature=0)
        
        # [핵심 수정 1] 프롬프트 강화: 번호 정보를 함께 출력하도록 지시
        self.prompt = ChatPromptTemplate.from_messages([
            ("system", """
            당신은 로봇 제어 관리자입니다. 사용자의 명령을 분석하여 [행동 태그]와 [대상 번호]를 출력하세요.
            
            [행동 규칙]
            1. 처리/버리기/치우기 -> [ACTION_DISPOSE]
            2. 보여줘/어딨어/확인 -> [ACTION_SHOW]
            3. 멈춰/정지 -> [ACTION_STOP]
            4. 집/홈/복귀/원위치 -> [ACTION_HOME]  <-- [NEW] 추가됨

            [번호 규칙]
            - 번호 언급이 있으면 해당 번호 (1, 2, 3...)
            - 없으면 기본값 1
            
            [출력 형식]
            반드시 "[TAG] 번호" 형태로 출력하세요.
            예시: "[ACTION_DISPOSE] 2", "[ACTION_SHOW] 1", "[ACTION_STOP] 0"
            """),
            ("user", "{input}")
        ])
        self.output_parser = StrOutputParser()
        self.chain = self.prompt | self.llm | self.output_parser

        self.get_logger().info("🤖 Smart Manager Ready! (Number Recognition Enabled)")
        self.run_voice_recognition()

    def run_voice_recognition(self):
        recognizer = sr.Recognizer()
        mic = sr.Microphone()
        
        self.get_logger().info("🎤 마이크 보정 중...")
        with mic as source:
            recognizer.adjust_for_ambient_noise(source, duration=1.0)
        
        self.get_logger().info("👂 명령을 기다립니다 (예: '2번 불량품 처리해')")

        while rclpy.ok():
            try:
                with mic as source:
                    audio = recognizer.listen(source, timeout=None, phrase_time_limit=5)
                
                self.get_logger().info("Processing...")
                text = recognizer.recognize_google(audio, language='ko-KR')
                self.get_logger().info(f"🗣️ 인식된 문장: \"{text}\"")
                
                # GPT 판단
                intent = self.chain.invoke({"input": text}).strip()
                self.get_logger().info(f"🧠 AI 판단: {intent}")
                
                self.execute_robot_action(intent)
                
            except sr.WaitTimeoutError: pass
            except sr.UnknownValueError: self.get_logger().warn("음성 인식 실패")
            except Exception as e: self.get_logger().error(f"에러: {e}")

    # [핵심 수정 2] 번호를 파싱해서 동적으로 타겟 이름 만들기
    def execute_robot_action(self, intent):
        msg = String()
        
        # 번호 추출
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
            self.get_logger().warn("🚨 [명령] 정지!")
            
        # [NEW] 홈 명령 처리
        elif "[ACTION_HOME]" in intent:
            msg.data = "home"  # 로봇에게 "home"이라는 특별한 신호를 보냄
            self.publisher_show.publish(msg) # show 토픽을 재활용해서 보냄
            self.get_logger().info("🏠 [명령] 홈 위치로 복귀")

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