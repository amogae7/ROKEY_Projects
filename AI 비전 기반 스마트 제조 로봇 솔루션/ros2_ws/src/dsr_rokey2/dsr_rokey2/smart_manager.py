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

# [중요] .env 파일 로드 (ros2 run 실행 시 경로 문제 방지)
# 홈 디렉토리의 ros2_ws 폴더에 .env가 있다고 가정합니다.
env_path = os.path.expanduser('~/ros2_ws/.env')
load_dotenv(dotenv_path=env_path)

# API KEY 확인
if not os.getenv("OPENAI_API_KEY"):
    print(f"❌ Error: .env 파일을 찾을 수 없거나 OPENAI_API_KEY가 없습니다.")
    print(f"참조한 경로: {env_path}")
    sys.exit(1)

class SmartManagerNode(Node):
    def __init__(self):
        super().__init__('smart_manager_node')
        
        # --- [1] 퍼블리셔 설정 (Robot Control 3와 연결) ---
        # 로봇에게 "불량품 처리해(집어서 버려)" 명령 전달
        self.publisher_dispose = self.create_publisher(String, '/part_n_bad_dispose', 10)
        
        # 로봇에게 "불량품 보여줘(가리켜)" 명령 전달
        self.publisher_show = self.create_publisher(String, '/part_n_bad_show', 10)
        
        # 로봇에게 "멈춰" 명령 전달
        self.publisher_stop = self.create_publisher(String, '/robot_stop', 10)

        # --- [2] LangChain(GPT) 설정 ---
        self.llm = ChatOpenAI(model="gpt-4o", temperature=0)
        
        # GPT에게 역할을 부여하는 프롬프트
        # 사용자의 말을 듣고 [ACTION_XXX] 형태의 정해진 키워드만 뱉도록 훈련시킵니다.
        self.prompt = ChatPromptTemplate.from_messages([
            ("system", """
            당신은 로봇 제어 관리자입니다. 사용자의 음성 명령을 분석하여 다음 중 하나의 행동 태그만 출력하세요.
            다른 말은 하지 말고 오직 태그만 출력해야 합니다.
            
            [규칙]
            1. 사용자가 '불량품 치워', '버려', '폐기해', '처리해' 등 물건을 집어서 옮기라고 하면:
               -> [ACTION_DISPOSE]
               
            2. 사용자가 '불량품 어디 있어?', '위치 보여줘', '가리켜' 등 확인을 요청하면:
               -> [ACTION_SHOW]
               
            3. 사용자가 '멈춰', '정지', '그만' 이라고 하면:
               -> [ACTION_STOP]
               
            4. 그 외 로봇과 관련 없는 말이면:
               -> [ACTION_NONE]
            """),
            ("user", "{input}")
        ])
        self.output_parser = StrOutputParser()
        self.chain = self.prompt | self.llm | self.output_parser

        self.get_logger().info("🤖 Smart Manager Ready! (Voice -> Robot Control 3)")
        
        # 음성 인식 루프 실행
        self.run_voice_recognition()

    def run_voice_recognition(self):
        recognizer = sr.Recognizer()
        
        # [수정] 기본 마이크 자동 사용 (device_index 제거)
        mic = sr.Microphone() 
        
        self.get_logger().info("🎤 마이크 초기화 중... (주변 소음 보정)")
        with mic as source:
            recognizer.adjust_for_ambient_noise(source, duration=1.0)
        
        self.get_logger().info("👂 말씀하세요! (예: '불량품 좀 치워줘')")

        while rclpy.ok():
            try:
                with mic as source:
                    # 음성 듣기 (타임아웃 설정)
                    audio = recognizer.listen(source, timeout=None, phrase_time_limit=5)
                
                self.get_logger().info("Processing...")
                
                # 1. STT (Speech to Text)
                text = recognizer.recognize_google(audio, language='ko-KR')
                self.get_logger().info(f"🗣️ 인식된 문장: \"{text}\"")
                
                # 2. LLM 판단 (Intent Classification)
                intent = self.chain.invoke({"input": text}).strip()
                self.get_logger().info(f"🧠 AI 판단: {intent}")
                
                # 3. 로봇에게 명령 전송 (Action)
                self.execute_robot_action(intent)
                
            except sr.WaitTimeoutError:
                pass # 말이 없으면 무시하고 다시 대기
            except sr.UnknownValueError:
                self.get_logger().warn("음성을 이해하지 못했습니다.")
            except Exception as e:
                self.get_logger().error(f"에러 발생: {e}")

    def execute_robot_action(self, intent):
        msg = String()
        
        # 1. GPT 응답(intent)에서 번호 추출 (기본값은 1)
        # 예: "[ACTION_DISPOSE] 2" -> target_num은 "2"
        target_num = "1"
        if "2" in intent:
            target_num = "2"
        elif "3" in intent:
            target_num = "3"
            
        # 2. 번호에 맞는 대상 이름 생성 (YOLO 클래스 명과 일치)
        target_object = f"part_{target_num}_bad" 

        # 3. 판단된 행동에 따라 해당 타겟 전송
        if "[ACTION_DISPOSE]" in intent:
            msg.data = target_object
            self.publisher_dispose.publish(msg)
            self.get_logger().info(f"🚀 명령 전송: /part_n_bad_dispose -> '{target_object}'")
            self.speak(f"{target_num}번 불량품을 처리할게요.")
            
        elif "[ACTION_SHOW]" in intent:
            msg.data = target_object
            self.publisher_show.publish(msg)
            self.get_logger().info(f"👀 명령 전송: /part_n_bad_show -> '{target_object}'")
            self.speak(f"{target_num}번 불량품의 위치를 가리킬게요.")
            
        elif "[ACTION_STOP]" in intent:
            msg.data = "stop"
            self.publisher_stop.publish(msg)
            self.get_logger().warn("🚨 명령 전송: 즉시 정지!")
            self.speak("로봇을 멈춥니다.")
            
        else:
            self.get_logger().info("음.. 로봇 명령이 아니네요.")

def main(args=None):
    rclpy.init(args=args)
    node = SmartManagerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()