import deepspeech
import numpy as np
import pyaudio

class DeepSpeechRecognizer:
    def __init__(self, model_file, scorer_file, rate=16000, chunk=1024, record_seconds=5):
        # 初始化模型
        self.model = deepspeech.Model(model_file)
        self.model.enableExternalScorer(scorer_file)

        # 音频参数
        self.rate = rate
        self.chunk = chunk
        self.record_seconds = record_seconds

        # PyAudio初始化
        self.p = pyaudio.PyAudio()

        # 打开流
        self.stream = self.p.open(format=pyaudio.paInt16,
                                  channels=1,
                                  rate=self.rate,
                                  input=True,
                                  frames_per_buffer=self.chunk)

    def record_and_recognize(self):
        print("now recording, please talk...")

        frames = []

        # 录制音频
        for i in range(0, int(self.rate / self.chunk * self.record_seconds)):
            data = self.stream.read(self.chunk)
            frames.append(data)

        print("finish recording...")

        # 连接帧并转换为numpy数组
        audio_frame = b''.join(frames)
        audio_np = np.frombuffer(audio_frame, dtype=np.int16)

        # 使用DeepSpeech进行STT
        text = self.model.stt(audio_np)
        return text

    def close(self):
        # 停止和关闭流
        self.stream.stop_stream()
        self.stream.close()
        self.p.terminate()
