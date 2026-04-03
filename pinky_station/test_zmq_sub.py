import zmq
import time

print("Testing ZMQ subscription to 192.168.0.40:9200")
ctx = zmq.Context()
sub = ctx.socket(zmq.SUB)
sub.connect("tcp://192.168.0.40:9200")
sub.setsockopt_string(zmq.SUBSCRIBE, "")

poller = zmq.Poller()
poller.register(sub, zmq.POLLIN)

timeout = time.time() + 5
while time.time() < timeout:
    events = dict(poller.poll(1000))
    if sub in events:
        parts = sub.recv_multipart()
        print(f"Received {len(parts)} parts. Topic: {parts[0]}")
        break
else:
    print("Timeout: No data received in 5 seconds.")
