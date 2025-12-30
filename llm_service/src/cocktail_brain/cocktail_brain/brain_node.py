import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import speech_recognition as sr
import google.generativeai as genai
import json
import os

# ==========================================
# 1. 설정 구역 (Configuration)
# ==========================================
GOOGLE_API_KEY = os.getenv('GOOGLE_API_KEY') # <-- 환경 변수에서 가져오기
genai.configure(api_key=GOOGLE_API_KEY)

# 시스템 프롬프트 (로봇의 본능/성격 설정)
# 생명과학 비유: DNA에 각인된 본능적 행동 양식입니다.
SYSTEM_PROMPT = """
당신은 전문 바텐더 로봇입니다. 
손님의 말을 듣고 기분이나 상황을 파악하여 적절한 칵테일을 추천하세요.
답변은 반드시 아래의 JSON 형식으로만 출력해야 합니다. 다른 말은 하지 마세요.

가능한 칵테일 목록: [Gin Tonic, Martini, Whiskey Sour, Orange Juice]

출력 형식 예시:
{
  "reason": "손님이 우울해 보여서 달콤한 것을 추천함",
  "cocktail": "Gin Tonic",
  "action_code": "make_gin_tonic"
}
"""

# 제미나이 모델 설정 (뇌 세팅)
model = genai.GenerativeModel(
    model_name='gemini-2.5-flash', 
    system_instruction=SYSTEM_PROMPT
)




class CocktailBrain(Node):
    def __init__(self):
        super().__init__('cocktail_brain_node')
        
        # 2. 퍼블리셔 생성 (신경 전달 물질 발사대)
        # 'robot_order'라는 토픽으로 명령을 보냅니다.
        self.publisher_ = self.create_publisher(String, 'robot_order', 10)
        
        # STT 객체 생성 (귀)
        self.recognizer = sr.Recognizer()
        self.microphone = sr.Microphone(None)
        
        self.get_logger().info('Brain Node가 깨어났습니다. (준비 완료)')
        
        # 메인 루프 시작 (주기적으로 듣고 판단함)
        self.timer = self.create_timer(1.0, self.listen_and_think)

    def listen_and_think(self):
        # 3. 듣기 (Listening)
        try:
            with self.microphone as source:
                self.get_logger().info('손님의 말을 듣고 있습니다... (말씀하세요)')
                # 주변 소음 적응 (0.5초)
                self.recognizer.adjust_for_ambient_noise(source, duration=0.5)
                # 음성 수집 (최대 5초간)
                audio = self.recognizer.listen(source, timeout=5, phrase_time_limit=5)
            
            # 4. 인식 (STT)
            text = self.recognizer.recognize_google(audio, language='ko-KR')
            self.get_logger().info(f'손님(User): "{text}"')
            
            # 5. 판단 (Gemini LLM Inference)
            response = model.generate_content(f"{SYSTEM_PROMPT}\n손님: {text}")
            ai_json = response.text.strip()
            
            # JSON 포맷 정리 (가끔 ```json ... ``` 이렇게 줄 때가 있어서 제거)
            ai_json = ai_json.replace("```json", "").replace("```", "").strip()
            
            self.get_logger().info(f'제미나이(Brain): {ai_json}')

            # 6. 명령 전달 (Publish to Spinal Cord)
            msg = String()
            msg.data = ai_json
            self.publisher_.publish(msg)
            self.get_logger().info('>> 로봇 팔에게 명령을 전송했습니다.')

        except sr.WaitTimeoutError:
            pass # 아무 말도 안 하면 그냥 넘어감
        except sr.UnknownValueError:
            self.get_logger().warn("잘 못 알아들었습니다.")
        except Exception as e:
            self.get_logger().error(f"에러 발생: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = CocktailBrain()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()