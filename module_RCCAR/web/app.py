# web/app.py
from flask import Flask, render_template, Response
from multiprocessing import Queue
import cv2
app = Flask(__name__)
frame_queue = Queue(maxsize=1)

@app.route('/')
def index():
    return render_template('index.html')

@app.route('/video_feed')
def video_feed():
    return Response(streaming_frame(),
                   mimetype='multipart/x-mixed-replace; boundary=frame')
def streaming_frame():
    while True:
        if not frame_queue.empty():
            frame = frame_queue.get()
            ret, jpeg = cv2.imencode('.jpg', frame)
            yield (b'--frame\r\n'
                   b'Content-Type: image/jpeg\r\n\r\n' + jpeg.tobytes() + b'\r\n\r\n')

def run_flask():
    app.run(host='0.0.0.0', port=5000, debug=False)