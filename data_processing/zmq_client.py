import numpy as np
import zmq
import pickle
import time
context = zmq.Context()

#  Socket to talk to server
print("Connecting to hello world server…")
socket = context.socket(zmq.REQ)
socket.connect("tcp://localhost:5555")

#  Do 10 requests, waiting each time for a response
for request in range(10):
    print("Sending request %s …" % request)
    array = np.random.rand(10, 10)
    msg = {"a": array, "b": 123}
    socket.send(pickle.dumps(msg))
    time.sleep(1)

    #  Get the reply.
    #message = socket.recv()
    #print("Received reply %s [ %s ]" % (request, message))