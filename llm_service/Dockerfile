# 1. 베이스 이미지: ROS2 Humble이 이미 설치된 우분투
FROM osrf/ros:humble-desktop

SHELL ["/bin/bash", "-c"]
# 2. 시스템 패키지 설치 
# portaudio19-dev: 마이크 사용을 위한 필수 드라이버
# alsa-utils: 리눅스 사운드 유틸리티
RUN apt-get update && apt-get install -y \
    python3-pip \
    python3-venv \
    portaudio19-dev \
    alsa-utils \
    && rm -rf /var/lib/apt/lists/*

# 3. 파이썬 라이브러리 설치 
RUN pip3 install \
    google-generativeai \
    SpeechRecognition \
    pyaudio \
    setuptools==58.2.0

# 4. 워크스페이스 설정 
WORKDIR /ros2_ws

# 5. 소스 코드 복사 
# 호스트의 src 폴더를 컨테이너 안의 src로 복사
COPY src/ src/

# 6. 빌드 
# bash 쉘을 열어서 colcon build를 실행
RUN /bin/bash -c "source /opt/ros/humble/setup.bash && colcon build"

# 7. 환경 설정 자동화 (깨어날 때마다 자동으로 활성화)
RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
RUN echo "source /ros2_ws/install/setup.bash" >> ~/.bashrc
# [추가] 오디오 설정 파일 생성 (영구 박제)
# 파이썬이 호스트의 PulseAudio를 사용하도록 강제함
RUN echo 'pcm.!default { type pulse hint.description "Default Audio Device" } ctl.!default { type pulse }' > /etc/asound.conf

# 8. 실행 명령 (기본 행동)
CMD ["bash"]
