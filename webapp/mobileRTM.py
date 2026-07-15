import json
import socket
from flask import Flask, render_template, request
from flask_socketio import SocketIO
from threading import Lock
import os.path
import config
from werkzeug.middleware.proxy_fix import ProxyFix

thread = None
thread_lock = Lock()

global HOST, PORT, bufferSize 
HOST = "127.0.0.1"  
PORT = 65431
bufferSize = 65507

# full Path is neccessary for the Systemtest
app = Flask("Runtimemodel", template_folder=os.path.dirname(__file__) + "/templates/", static_folder=os.path.dirname(__file__) + "/static/")
app.wsgi_app = ProxyFix(
    app.wsgi_app,
    x_for=1,
    x_proto=1,
    x_host=1,
    x_prefix=1
)
app.config.from_object(config.Config)
socketio = SocketIO(app, cors_allowed_origins='*')

global udpClientSocket
udpClientSocket=socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
udpClientSocket.sendto(str.encode("hello"), (HOST, PORT))


"""
Background Thread
"""
#receive System status information from Runtimemodel
def background_thread():

    global udpClientSocket, bufferSize    
    while(True):
        if udpClientSocket is None:
            continue  # Wait until the socket is initialized
        msg, addr = udpClientSocket.recvfrom(bufferSize) # BLOCKS
        #print(msg.decode())
        socketio.emit('updateSensorData', msg.decode())

"""
Serve root index file
"""
@app.route('/')
def index():
    return render_template('model.html', active_tab="dashboard", round_decimals=app.config["ROUND_DECIMALS"])

"""
Serve tables view
"""
@app.route("/tables")
def tables():
    return render_template("tables.html", active_tab="tables", round_decimals=app.config["ROUND_DECIMALS"])

"""
Serve actions view
"""
@app.route("/actions")
def actions():
    return render_template("actions.html", active_tab="actions", round_decimals=app.config["ROUND_DECIMALS"])

"""
Serve behavior view
"""
@app.route("/behavior")
def behavior():
    return render_template("behavior.html", active_tab="behavior", round_decimals=app.config["ROUND_DECIMALS"])


"""
Decorator for connect
"""
@socketio.on('connect')
def connect():
    global thread
    print('Client connected')

    udpClientSocket.sendto(str.encode("hello"), (HOST, PORT))

    global thread
    with thread_lock:
        if thread is None:
            thread = socketio.start_background_task(background_thread)

"""
Decorator for disconnect
"""
@socketio.on('disconnect')
def disconnect():
    print('Client disconnected',  request.sid)
    global updClientSocket

"""
Decorator for Receiving formData
"""
# receive goal from webapp and send it to runtimemodel
@socketio.on('runAction')
def getInput(args):
    global updClientSocket
    print(args)
    print(udpClientSocket)
    if(udpClientSocket != None):
        udpClientSocket.sendto(str.encode(json.dumps(args)), (HOST, PORT))
        msg , addr = udpClientSocket.recvfrom(bufferSize) # BLOCKS
        socketio.emit('updateSensorData', msg.decode())

@socketio.on('sendCommand')
def getInput(args):
    global updClientSocket
    print(args)
    print(udpClientSocket)
    if(udpClientSocket != None):
        udpClientSocket.sendto(str.encode(args['type']), (HOST, PORT))
        msg , addr = udpClientSocket.recvfrom(bufferSize) # BLOCKS
        socketio.emit('updateSensorData', msg.decode())

if __name__ == '__main__':
    socketio.run(app, allow_unsafe_werkzeug=True, use_reloader=False)
