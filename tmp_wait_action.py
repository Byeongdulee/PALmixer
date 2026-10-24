import sys, time, json
from palmixer.zmq_transport import ZMQClient

want = sys.argv[1]
c = ZMQClient(host="127.0.0.1", port=9880, timeout_ms=15000)
while True:
    try:
        d = json.loads(c.send("get_state"))
    except Exception as e:
        print("poll error:", e); time.sleep(2); continue
    lr = d.get("last_result") or {}
    if lr.get("action_id") == want:
        print("DONE action_id=%s ok=%s detail=%s" % (want, lr.get("ok"), lr.get("detail")))
        break
    if not d.get("busy"):
        # not busy and our action isn't the last_result yet -> give it a moment, then report whatever is latest
        time.sleep(1)
        d = json.loads(c.send("get_state"))
        lr = d.get("last_result") or {}
        if lr.get("action_id") == want:
            print("DONE action_id=%s ok=%s detail=%s" % (want, lr.get("ok"), lr.get("detail")))
            break
    time.sleep(2)
