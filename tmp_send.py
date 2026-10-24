import sys
from palmixer.zmq_transport import ZMQClient

cmd = " ".join(sys.argv[1:])
c = ZMQClient(host="127.0.0.1", port=9880, timeout_ms=60000)
print(c.send(cmd))
