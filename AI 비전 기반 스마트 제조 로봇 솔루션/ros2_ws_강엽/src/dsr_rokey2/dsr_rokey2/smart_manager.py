import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import threading
import time
import os
import speech_recognition as sr
from gtts import gTTS

class SmartManagerNode(Node):
    def __init__(self):
        super().__init__('smart_manager_node')
        
        # --- Publishers ---
        self.pub_search = self.create_publisher(String, '/search_for_part_n_bad', 10)
        self.pub_bad_detected = self.create_publisher(String, '/part_n_bad', 10)
        self.pub_show_bad = self.create_publisher(String, '/part_n_bad_show', 10)
        self.pub_dispose = self.create_publisher(String, '/part_n_bad_dispose', 10)

        # --- Subscribers ---
        self.create_subscription(String, '/yolo_results', self.vision_callback, 10)

        # --- Variables ---
        self.current_detections = []
        self.good_part_timer = 0
        self.bad_part_timer = 0  # [추가] 불량 알림 반복용 타이머
        self.is_bad_detected = False
        self.waiting_for_voice = False 

        # --- Start STT Thread ---
        self.stt_thread = threading.Thread(target=self.stt_listener_loop)
        self.stt_thread.daemon = True
        self.stt_thread.start()

        # --- Start Logic Timer ---
        self.timer = self.create_timer(1.0, self.logic_loop) # 1초마다 로직 수행

        self.get_logger().info("System Started. Sending Robot to Home...")
        msg = String()
        msg.data = "init"
        self.pub_search.publish(msg)

    # --- 1. Vision Callback ---
    def vision_callback(self, msg):
        self.current_detections = msg.data.split(',')

    # --- 2. Main Logic Loop (1초마다 실행) ---
    def logic_loop(self):
        if not self.current_detections:
            return

        has_bad = any("bad" in item for item in self.current_detections)
        has_good = any("good" in item for item in self.current_detections)

        # [상황 A] 불량 발생!
        if has_bad:
            # 1. 처음 발견했을 때만 실행 (로봇 정지 명령 등)
            if not self.is_bad_detected:
                self.get_logger().warn("⚠️ 불량 발생! (Defect Detected)")
                self.pub_bad_detected.publish(String(data="bad_detected"))
                
                self.is_bad_detected = True
                self.waiting_for_voice = True
                self.good_part_timer = 0
                self.bad_part_timer = 3 # 바로 소리가 나도록 타이머 설정

            # 2. 불량 상태가 지속되는 동안 계속 실행 (소리 반복)
            self.bad_part_timer += 1
            if self.bad_part_timer >= 6: # 3초 간격으로 말하기
                self.speak("불량이 발생했습니다.")
                self.bad_part_timer = 0

        # # [상황 B] 양품만 있음 (3초 체크)
        # elif has_good and not has_bad:
        #     self.is_bad_detected = False
        #     self.waiting_for_voice = False
        #     self.bad_part_timer = 0 # 불량 타이머 초기화
            
        #     self.good_part_timer += 1
        #     self.get_logger().info(f"양품 유지 중... {self.good_part_timer}초")

        #     if self.good_part_timer >= 3:
        #         self.get_logger().info("✅ 불량이 없음 (All Clear)")
        #         self.speak("불량이 없습니다.")
        #         self.good_part_timer = 0 

    # --- 3. STT Listener (Background Thread) ---
    def stt_listener_loop(self):
        recognizer = sr.Recognizer()
        mic = sr.Microphone()
        
        self.get_logger().info("👂 STT Thread Started. Listening...")
        
        while rclpy.ok():
            if self.waiting_for_voice: 
                try:
                    with mic as source:
                        recognizer.adjust_for_ambient_noise(source)
                        # 타임아웃을 짧게 주어 루프가 빨리 돌게 함
                        audio = recognizer.listen(source, timeout=3, phrase_time_limit=3)
                    
                    text = recognizer.recognize_google(audio, language='ko-KR')
                    self.get_logger().info(f"들린 말: {text}")

                    if "불량 부품" in text or "알려 줘" in text:
                        self.get_logger().info("Command: Show Bad Part")
                        self.speak("불량 부품을 표시합니다.")
                        self.pub_show_bad.publish(String(data="show"))
                        print('불량 부품 알림 토픽 발행')

                    elif "불량 처리" in text or "바빠" in text:
                        self.get_logger().info("Command: Dispose Bad Part")
                        self.speak("제가 처리하겠습니다.")
                        self.pub_dispose.publish(String(data="dispose"))
                        self.waiting_for_voice = False 
                        self.is_bad_detected = False 
                        self.bad_part_timer = 0 # 처리 완료 후 타이머 리셋
                        print('불량 부품 처리 토픽 발행')

                except sr.WaitTimeoutError:
                    pass
                except sr.UnknownValueError:
                    pass
                except Exception as e:
                    self.get_logger().error(f"STT Error: {e}")
            else:
                time.sleep(0.5)

    # --- Helper: TTS ---
    def speak(self, text):
        try:
            # 기존 파일 삭제 (충돌 방지)
            if os.path.exists('voice.mp3'):
                os.remove('voice.mp3')
                
            tts = gTTS(text=text, lang='ko')
            filename = 'voice.mp3'
            tts.save(filename)
            os.system(f"mpg321 -q {filename} > /dev/null 2>&1 &")
        except Exception as e:
            self.get_logger().error(f"TTS Error: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = SmartManagerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
