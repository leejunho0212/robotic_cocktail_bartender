import sounddevice as sd
import numpy as np
import whisper
import tempfile
import scipy.io.wavfile as wav

model = whisper.load_model("base")

def speech_to_text(duration=5):
    device_id = 1      # 마이크(Realtek Audio), MME
    fs = 48000
    channels = 2

    print("🎤 말하세요...")

    audio = sd.rec(
        int(duration * fs),
        samplerate=fs,
        channels=channels,
        dtype="float32",
        device=device_id
    )
    sd.wait()

    max_volume = np.max(np.abs(audio))
    print(f"🔊 max volume: {max_volume}")

    if max_volume < 0.001:
        print("⚠️ 마이크 입력이 거의 없습니다.")
        return ""

    audio_mono = np.mean(audio, axis=1)

    with tempfile.NamedTemporaryFile(suffix=".wav", delete=False) as f:
        wav.write(f.name, fs, audio_mono)
        result = model.transcribe(f.name, language="ko")

    text = result["text"].strip()
    print("📝 인식된 텍스트:", text)
    return text
