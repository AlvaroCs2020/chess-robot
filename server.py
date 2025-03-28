import cv2
import numpy as np
from flask import Flask, Response, request, jsonify
from flask import Flask, render_template
from flask_socketio import SocketIO # type: ignore
import serial # type: ignore
import time 
global frame
global success
global commands_to_execute
commands_to_execute = []
app = Flask(__name__)
#arduino = serial.Serial(port='COM4', baudrate=9600, timeout=.1)
time.sleep(1)#Esperamos que se conecte el arduino
def set_frame(video, suc):
    global frame
    global success
    success = suc
    frame = video
socketio = SocketIO(app, cors_allowed_origins="*")
def generate_frames():
    global frame
    global success
    while True:
        if not success:
            break
        _, buffer = cv2.imencode('.jpg', frame)
        frame_bytes = buffer.tobytes()
        yield (b'--frame\r\n'
               b'Content-Type: image/jpeg\r\n\r\n' + frame_bytes + b'\r\n')
def generate_frames_resized():
    global frame
    global success
    while True:
        if not success:
            break
        img_resized = cv2.resize(frame, (400,300), interpolation=cv2.INTER_AREA)
        _, buffer = cv2.imencode('.jpg', img_resized)
        frame_bytes = buffer.tobytes()
        yield (b'--frame\r\n'
               b'Content-Type: image/jpeg\r\n\r\n' + frame_bytes + b'\r\n')
        
@app.route('/video_feed')
def video_feed():
    return Response(generate_frames(), mimetype='multipart/x-mixed-replace; boundary=frame')
@app.route('/video_feed_android')
def video_android_android():
    return Response(generate_frames_resized(), mimetype='multipart/x-mixed-replace; boundary=frame')

# Nuevo endpoint para recibir datos desde MAUI
@app.route('/receive_data', methods=['POST'])
def receive_data():
    global commands_to_execute
    data = request.json  # Recibe JSON desde la app
    commands_to_execute.append(data.get("message"))
    #print(data.get("message"))
    #print(f"Datos recibidos: {data}")  # Muestra en consola
    return jsonify({"status": "success", "message": "Datos recibidos correctamente"})

def get_command():
    global commands_to_execute
    
    if(len(commands_to_execute) != 0):
        out = commands_to_execute.pop()
    else:
        out = 0
    return out
def run_server():
    app.run(host='0.0.0.0', port=8081, debug=False, threaded=True)
#if __name__ == '__main__':
#    app.run(host='0.0.0.0', port=5000, debug=False)