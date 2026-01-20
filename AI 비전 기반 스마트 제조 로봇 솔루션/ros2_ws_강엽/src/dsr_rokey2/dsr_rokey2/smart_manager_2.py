import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float32MultiArray
import threading
import time
import os
import speech_recognition as sr
from gtts import gTTS

class SmartManagerNode(Node):
    def __init__(self):
        super().__init__('smart_manager_node')
        
        # =========================================================
        # 1. Publishers
        # =========================================================
        
        # [Coord] 좌표 중계용
        self.coord_pubs = {
            'part_1_bad': self.create_publisher(Float32MultiArray, '/part_1_bad_coord', 10),
            'part_2_bad': self.create_publisher(Float32MultiArray, '/part_2_bad_coord', 10),
            'part_3_bad': self.create_publisher(Float32MultiArray, '/part_3_bad_coord', 10),
        }
        # [Show] 명령용 (단일 토픽 사용)
        self.pub_show_bad = self.create_publisher(String, '/part_bad_show', 10)
        # [Dispose] 명령용 (부품별 개별 토픽 생성)  
        self.dispose_pubs = {
            'part_1_bad': self.create_publisher(String, '/part_1_bad_dispose', 10),
            'part_2_bad': self.create_publisher(String, '/part_2_bad_dispose', 10),
            'part_3_bad': self.create_publisher(String, '/part_3_bad_dispose', 10),
        }
        # =========================================================
        # 2. Subscribers
        # =========================================================
        self.create_subscription(Float32MultiArray, '/part_1_bad', lambda msg: self.bad_callback(msg, 'part_1_bad'), 10)
        self.create_subscription(Float32MultiArray, '/part_2_bad', lambda msg: self.bad_callback(msg, 'part_2_bad'), 10)
        self.create_subscription(Float32MultiArray, '/part_3_bad', lambda msg: self.bad_callback(msg, 'part_3_bad'), 10)

        # =========================================================
        # 3. Variables
        # =========================================================
        self.detected_bad_parts = {}
        self.is_bad_detected = False
        self.waiting_for_voice = False 
        self.bad_part_timer = 0
        self.last_detection_time = 0

        # STT & Logic
        self.stt_thread = threading.Thread(target=self.stt_listener_loop)
        self.stt_thread.daemon = True
        self.stt_thread.start()

        self.timer = self.create_timer(1.0, self.logic_loop)
        self.get_logger().info("Smart Manager Started (Specific Dispose Topic Mode)")

    def bad_callback(self, msg, part_name):
        self.detected_bad_parts[part_name] = msg.data
        self.last_detection_time = time.time()

    def logic_loop(self):
        current_time = time.time()
        if current_time - self.last_detection_time > 2.0:
            self.detected_bad_parts.clear()
            self.is_bad_detected = False
            return

        if len(self.detected_bad_parts) > 0:
            if not self.is_bad_detected:
                self.get_logger().warn(f"⚠️ 불량 발생! 목록: {list(self.detected_bad_parts.keys())}")
                self.is_bad_detected = True
                self.waiting_for_voice = True
                self.bad_part_timer = 3 

            self.bad_part_timer += 1
            if self.bad_part_timer >= 4: 
                self.speak("불량이 발생했습니다.")
                self.bad_part_timer = 0
        else:
            self.is_bad_detected = False
            self.waiting_for_voice = False

    def stt_listener_loop(self):
        recognizer = sr.Recognizer()
        mic = sr.Microphone()
        
        while rclpy.ok():
            if self.waiting_for_voice: 
                try:
                    with mic as source:
                        recognizer.adjust_for_ambient_noise(source)
                        audio = recognizer.listen(source, timeout=3, phrase_time_limit=3)
                    
                    text = recognizer.recognize_google(audio, language='ko-KR')
                    self.get_logger().info(f"🎤 들린 말: {text}")

                    if "불량 부품" in text or "알려 줘" in text:
                        self.process_command(action="show")
                        self.speak("불량 부품을 표시합니다.")

                    elif "불량 처리" in text or "바빠" in text or "처리해" in text:
                        self.process_command(action="dispose")
                        self.speak("제가 처리하겠습니다.")
                        self.waiting_for_voice = False 
                        self.is_bad_detected = False 
                        self.detected_bad_parts.clear()

                except (sr.WaitTimeoutError, sr.UnknownValueError):
                    pass
                except Exception as e:
                    self.get_logger().error(f"STT Error: {e}")
            else:
                time.sleep(0.5)

    def process_command(self, action):
        if not self.detected_bad_parts:
            self.get_logger().warn("❌ 저장된 불량 좌표가 없습니다.")
            return

        # 1. 좌표 중계 (먼저 보냄)
        for part_name, coords in self.detected_bad_parts.items():
            if part_name in self.coord_pubs:
                msg = Float32MultiArray()
                msg.data = coords
                self.coord_pubs[part_name].publish(msg)
                self.get_logger().info(f"📡 show 좌표 중계: {part_name} -> {part_name}_coord")

        # 2. 명령 발행
        cmd_msg = String(data=action)
        
        if action == "show":
            self.pub_show_bad.publish(cmd_msg)
            
        if action == "dispose":
            # [수정] 감지된 각 부품에 맞는 Dispose 토픽 발행
            for part_name in self.detected_bad_parts.keys():
                if part_name in self.dispose_pubs:
                    self.dispose_pubs[part_name].publish(cmd_msg)
                    self.get_logger().info(f"🚀 Dispose 명령 발행: {part_name}_dispose")

    def speak(self, text):
        try:
            if os.path.exists('voice.mp3'):
                os.remove('voice.mp3')
            tts = gTTS(text=text, lang='ko')
            tts.save('voice.mp3')
            os.system(f"mpg321 -q voice.mp3 > /dev/null 2>&1 &")
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