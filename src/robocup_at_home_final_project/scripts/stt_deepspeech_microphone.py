import deepspeech
import numpy as np
import pyaudio
import wave

# 模型参数
MODEL_FILE = '/home/jin/ros/tiago_ws/src/robocup_at_home_final_project/config/deepspeech-0.9.3-models.pbmm'
SCORER_FILE = '/home/jin/ros/tiago_ws/src/robocup_at_home_final_project/config/deepspeech-0.9.3-models.scorer'

# 音频参数
FORMAT = pyaudio.paInt16
CHANNELS = 1
RATE = 16000
CHUNK = 1024
RECORD_SECONDS = 5

# 初始化模型
model = deepspeech.Model(MODEL_FILE)
model.enableExternalScorer(SCORER_FILE)

# PyAudio初始化
p = pyaudio.PyAudio()

# 打开流
stream = p.open(format=FORMAT,
                channels=CHANNELS,
                rate=RATE,
                input=True,
                frames_per_buffer=CHUNK)

print("now recording ,please talk...")

frames = []

# 录制5秒钟
for i in range(0, int(RATE / CHUNK * RECORD_SECONDS)):
    data = stream.read(CHUNK)
    frames.append(data)

print("finish recording ...")

# 停止和关闭流
stream.stop_stream()
stream.close()
p.terminate()

# 连接帧并转换为numpy数组
audio_frame = b''.join(frames)
audio_np = np.frombuffer(audio_frame, dtype=np.int16)

# 使用DeepSpeech进行STT
text = model.stt(audio_np)
print("TEXT：", text)
