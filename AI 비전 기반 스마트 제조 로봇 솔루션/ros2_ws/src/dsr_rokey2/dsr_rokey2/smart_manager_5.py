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
env_path = os.path.expanduser(r'/home/rokey/ros2_ws/.env')
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
        
        # [핵심 수정] 프롬프트에 '영어 발음'과 '자주 틀리는 발음' 규칙 추가
        self.prompt = ChatPromptTemplate.from_messages([
            ("system", """
            당신은 로봇 제어 관리자입니다. 사용자의 음성 명령을 분석하여 [행동 태그]와 [대상 번호]를 출력하세요.
            음성 인식 결과가 부정확할 수 있으므로, 아래 규칙에 따라 유연하게 해석하세요.
            
            [행동 규칙]
            1. 처리/버리기/치우기/폐기 -> [ACTION_DISPOSE]
            2. 보여줘/어딨어/확인/검사 -> [ACTION_SHOW]
            3. 멈춰/정지/스탑 -> [ACTION_STOP]
            4. 집/홈/복귀/원위치 -> [ACTION_HOME]

            [번호 매핑 규칙] (유사 발음 주의)
            - 1번: '1번', '일번', '하나', '첫번째', '원', '파트원', 'Part 1'
              (오인식 케이스: '일본', '일반', '파전', '하트원') -> 1로 해석
            
            - 2번: '2번', '이번', '둘', '두번째', '투', '파트투', 'Part 2'
              (오인식 케이스: '이반', '투투', '타투') -> 2로 해석
            
            - 3번: '3번', '삼번', '셋', '세번째', '쓰리', '파트쓰리', 'Part 3'
              (오인식 케이스: '산본', '삼분', '쓰리', '파슬리') -> 3으로 해석
              
            [출력 형식]
            반드시 "[TAG] 번호" 형태로 출력하세요. (번호는 아라비아 숫자만)
            예시: "[ACTION_DISPOSE] 2", "[ACTION_SHOW] 1", "[ACTION_HOME] 0"
            """),
            ("user", "{input}")
        ])
        self.output_parser = StrOutputParser()
        self.chain = self.prompt | self.llm | self.output_parser

        self.get_logger().info("🤖 Smart Manager Ready! (Enhanced Speech Recognition)")
        self.run_voice_recognition()

    def run_voice_recognition(self):
        recognizer = sr.Recognizer()
        mic = sr.Microphone()
        
        self.get_logger().info("🎤 마이크 보정 중...")
        with mic as source:
            recognizer.adjust_for_ambient_noise(source, duration=1.0)
        
        self.get_logger().info("👂 명령 대기 중 ('파트투 처리해', '산본 보여줘' 등)")

        while rclpy.ok():
            try:
                with mic as source:
                    audio = recognizer.listen(source, timeout=None, phrase_time_limit=5)
                
                # 구글 음성 인식 (한국어)
                text = recognizer.recognize_google(audio, language='ko-KR')
                self.get_logger().info(f"🗣️ 인식된 단어: \"{text}\"")
                
                # GPT에게 해석 요청
                intent = self.chain.invoke({"input": text}).strip()
                self.get_logger().info(f"🧠 AI 해석결과: {intent}")
                
                self.execute_robot_action(intent)
                
            except sr.WaitTimeoutError: pass
            except sr.UnknownValueError: pass # 잡음 무시
            except Exception as e: self.get_logger().error(f"에러: {e}")

    def execute_robot_action(self, intent):
        msg = String()
        
        # 숫자 추출 (GPT가 이미 매핑해서 '1', '2', '3'으로 줌)
        target_num = "1"
        match = re.search(r'\d+', intent)
        if match: target_num = match.group()
        
        target_object = f"part_{target_num}_bad"

        if "[ACTION_DISPOSE]" in intent:
            msg.data = target_object
            self.publisher_dispose.publish(msg)
            self.get_logger().info(f"🚀 [명령 전송] {target_num}번 불량품 처리")
            
        elif "[ACTION_SHOW]" in intent:
            msg.data = target_object
            self.publisher_show.publish(msg)
            self.get_logger().info(f"👀 [명령 전송] {target_num}번 불량품 확인")
            
        elif "[ACTION_STOP]" in intent:
            msg.data = "stop"
            self.publisher_stop.publish(msg)
            self.get_logger().warn("🚨 [명령 전송] 로봇 정지")
            
        elif "[ACTION_HOME]" in intent:
            msg.data = "home"
            self.publisher_show.publish(msg)
            self.get_logger().info("🏠 [명령 전송] 홈 위치 복귀")
            
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