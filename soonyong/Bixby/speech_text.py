# transcribe_streaming_mic.py

import re # 정규표현식 모듈
import sys

from google.cloud import speech
import pyaudio  # 파이썬에서 오디오 입력 사용
import queue

from data import update_word

client = None
streaming_config = None

# Audio recording parameters
RATE = 16000
CHUNK = int(RATE / 10)  

class MicrophoneStream(object):
    def __init__(self, rate, chunk):
        self._rate = rate
        self._chunk = chunk

        # Create a thread-safe buffer of audio data
        self._buff = queue.Queue()  # pyaudio가 전달해주는 데이터를 담을 큐 
        self.closed = True

    # 파이썬 context manager사용. 여기에서는 실행중 문제가 발생해도 오디오장치를 제대로 닫도록 할 수 있기 위함.
    def __enter__(self):
        self._audio_interface = pyaudio.PyAudio()   # 시작할 때 pyaudio 데이터 스트림 열림
        self._audio_stream = self._audio_interface.open( # pyaudio.open()은 pyaudio.Stream object를 리턴.
            format=pyaudio.paInt16, # 16bit 다이나믹 레인지
            channels=1,
            rate=self._rate,
            input=True,     
            frames_per_buffer=self._chunk,
            stream_callback=self._fill_buffer,  # pyaudio에서 한 블록의 데이터가 들어올 때 호출되는 콜백
        )

        self.closed = False

        return self

    def __exit__(self, type, value, traceback):
        self._audio_stream.stop_stream()
        self._audio_stream.close()
        self.closed = True
        self._buff.put(None)
        self._audio_interface.terminate()   # 끝날 때 반드시 pyaudio 스트림 닫도록 한다.

    def _fill_buffer(self, in_data, frame_count, time_info, status_flags):  # pyaudio.Stream에서 호출되는 콜백은 4개 매개변수 갖고, 2개값 리턴한다. pyaudio문서 참고.
        self._buff.put(in_data) # 큐에 데이터 추가
        return None, pyaudio.paContinue

    # 한 라운드의 루프마다 현재 버퍼의 내용을 모아서 byte-stream을 yield함.
    def generator(self):
        while not self.closed:
            # Use a blocking get() to ensure there's at least one chunk of
            # data, and stop iteration if the chunk is None, indicating the
            # end of the audio stream.
            chunk = self._buff.get()
            if chunk is None:
                return
            data = [chunk]

            # Now consume whatever other data's still buffered.
            while True:
                try:
                    chunk = self._buff.get(block=False)  # 가장 오래된 데이터부터 순차적으로 data[]에 추가함.
                    if chunk is None:
                        return
                    data.append(chunk)
                except queue.Empty: # 큐에 더이상 데이터가 없을 때까지
                    break

            yield b''.join(data) # byte-stream


def listening(stop_event):
    global client, streaming_config
    while not stop_event.is_set():
        with MicrophoneStream(RATE, CHUNK) as stream:   # 사운드 스트림 오브젝트 생성. 
                                                        # pyaudio가 terminate()되는 것을 보장하기 위해 python
                                                        # context manager  사용.
            if(stop_event.is_set()):
                break
            
            audio_generator = stream.generator()
            requests = (speech.StreamingRecognizeRequest(audio_content=content)
                        for content in audio_generator) # generator expression. 요청 생성

            responses = client.streaming_recognize(streaming_config, requests)  # 요청 전달 & 응답 가져옴
            
            old_transcript = ""
            
            for response in responses:
                if(stop_event.is_set()):
                    stream.closed = True
                    break
                
                if not response.results:
                    continue

                # The `results` list is consecutive. For streaming, we only care about
                # the first result being considered, since once it's `is_final`, it
                # moves on to considering the next utterance.
                # 최종적인 결과값은 언제나 results[0]에 반영되므로 result[0]만 고려.
                result = response.results[0]
                if not result.alternatives:
                    continue

                # 확실성 가장 높은 alternative의 해석
                transcript = result.alternatives[0].transcript
    
                # 새로운 부분만 업데이트
                new_transcript = transcript[len(old_transcript) : ]
                old_transcript = transcript
                if new_transcript != "":
                    update_word(new_transcript)
                    
                # 하지만 final 이면 걍 싹다 업데이트.
                # 중복 입력 가능성 때문에 일단 보류.
                # if result.is_final:
                #     update_sentence(transcript)

def init():
    global client, streaming_config
    # 한국말 사용
    language_code = 'ko-KR'  # a BCP-47 language tag

    client = speech.SpeechClient()
    config = speech.RecognitionConfig(
        encoding='LINEAR16', # enums.RecognitionConfig.AudioEncoding.LINEAR16
        sample_rate_hertz=RATE,
        max_alternatives=1, # 가장 가능성 높은 1개 alternative만 받음.
        language_code=language_code,
        speech_contexts=[speech.SpeechContext(phrases=["빅스비", "따라", "돌아", "멈춰", "목록", "리스트", "뭐할수있", "보고", "상태", "요약", "로그"], boost=20)]
        )
    streaming_config = speech.StreamingRecognitionConfig(
        config=config,
        interim_results=True) # 중도값 사용 여부