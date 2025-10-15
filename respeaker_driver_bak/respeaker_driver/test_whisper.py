# # import whisper

# # model = whisper.load_model("base")

# # result = model.transcribe("/home/pi/rec_main_ru.wav", fp16=False)
# # print(result["text"])



# from faster_whisper import WhisperModel


# # Run on GPU with FP16
# # model = WhisperModel("base", device="cpu", compute_type="float16")

# # # or run on GPU with INT8
# # # model = WhisperModel(model_size, device="cuda", compute_type="int8_float16")
# # # or run on CPU with INT8
# # # model = WhisperModel(model_size, device="cpu", compute_type="int8")

# # segments, info = model.transcribe("audio.mp3", beam_size=5)

# # print("Detected language '%s' with probability %f" % (info.language, info.language_probability))

# # for segment in segments:
# #     print("[%.2fs -> %.2fs] %s" % (segment.start, segment.end, segment.text))


# import whisper
# import sounddevice as sd
# import numpy as np
# import queue

import pyaudio
import json
from vosk import Model, KaldiRecognizer


import rclpy
from rclpy import node
from robohead_msgs.msg import AudioData
# import threading

# # model = whisper.load_model("base")
# model = WhisperModel("base", device="cpu", compute_type="int8")

# q = queue.Queue()


model = Model("vosk-model-small-ru-0.22")
recognizer = KaldiRecognizer(model, 16000)

def m_callback(msg:AudioData):
    if recognizer.AcceptWaveform(msg.data.tobytes()):
        result = json.loads(recognizer.Result())
        print("You:", result["text"])


# # def callback(indata, frames, time, status):
# #     q.put(indata.copy())

# def transcribe_stream():
#     DURATION = 5
#     SAMPLE_RATE = 16000
#     NUM_FRAMES = DURATION * SAMPLE_RATE

#     buffer = np.empty((1), dtype=np.float32)

#     while True:
#         while buffer.shape[0] < NUM_FRAMES:
#             buffer = np.append(buffer, q.get(), axis=0)

#         audio_input = buffer[:NUM_FRAMES].flatten()
#         buffer = buffer[NUM_FRAMES:]

#         segments, info = model.transcribe(audio_input, beam_size=5)
#         # result = model.transcribe(audio_input, language="ru", fp16=False)
#         # print("recognized: ", result["text"])

#         print("Detected language '%s' with probability %f" % (info.language, info.language_probability))

#         for segment in segments:
#             print("[%.2fs -> %.2fs] %s" % (segment.start, segment.end, segment.text))


# # stream = sd.InputStream(callback=callback, samplerate=16000, channels=1)

rclpy.init(args=None)
mnode = node.Node("example_record")
mnode.create_subscription(AudioData, "/respeaker_driver/audio/main", m_callback, 10)
print("start")

# th = threading.Thread(target=transcribe_stream)
# th.start()
rclpy.spin(mnode)

# rclpy.shutdown()



# print("Detected language '%s' with probability %f" % (info.language, info.language_probability))

# for segment in segments:
#     print("[%.2fs -> %.2fs] %s" % (segment.start, segment.end, segment.text))









# p = pyaudio.PyAudio()
# stream = p.open(format=pyaudio.paInt16, channels=1, rate=16000, input=True, frames_per_buffer=8192)
# stream.start_stream()

# print("listening...")

# while True:
#     data = stream.read(4096, exception_on_overflow=False)
#     if recognizer.AcceptWaveform(data):
#         result = json.loads(recognizer.Result())
#         print("You:", result["text"])