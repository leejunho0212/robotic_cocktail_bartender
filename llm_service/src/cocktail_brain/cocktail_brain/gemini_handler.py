import google.generativeai as genai
import os


# 1. 설정 구역 (Gemini API)
GOOGLE_API_KEY = os.getenv('GOOGLE_API_KEY')

genai.configure(api_key=GOOGLE_API_KEY)

# 시스템 프롬프트 (로봇의 성격/본능 설정)
SYSTEM_INSTRUCTION = "너는 한국어로 친절하게 답하는 AI 비서야."

# 모델 선택
model = genai.GenerativeModel(
    model_name='gemini-2.5-flash-lite', 
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
