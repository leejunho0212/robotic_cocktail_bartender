import google.generativeai as genai

# ==========================================
# 1. 설정 구역
# ==========================================
# 여기에 발급받은 제미나이 API 키를 입력하세요 (따옴표 필수!)
GOOGLE_API_KEY = "YAIzaSyDuv5V-7k9-sq_K3NMV12jvg0MApkiU2KE"

genai.configure(api_key=GOOGLE_API_KEY)

# 시스템 프롬프트 (로봇의 성격/본능 설정)
# GPT 코드에 있던 "너는 한국어로 친절하게 답하는 AI 비서야"를 여기에 설정합니다.
SYSTEM_INSTRUCTION = "너는 한국어로 친절하게 답하는 AI 비서야."

# 모델 초기화 (gemini-1.5-flash가 속도가 빨라 로봇 제어에 유리합니다. 성능형은 pro)
model = genai.GenerativeModel(
    model_name='gemini-2.5-flash', 
    system_instruction=SYSTEM_INSTRUCTION
)

def ask_gemini(user_text: str) -> str:
    """
    사용자의 텍스트를 받아 Gemini의 답변을 반환하는 함수
    """
    try:
        # Gemini에게 질문 던지기
        response = model.generate_content(user_text)
        
        # 답변 텍스트 추출 및 공백 제거
        return response.text.strip()
        
    except Exception as e:
        print(f"Gemini 호출 중 에러 발생: {e}")
        return "죄송해요, 지금은 생각할 수가 없어요."
