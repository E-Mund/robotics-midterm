"""Record Gazebo contact pairs without retaining high-volume raw messages."""
import collections
import json
import os
from pathlib import Path
import selectors
import signal
import subprocess
import sys
import time

out = Path(sys.argv[1])
seconds = float(sys.argv[2])
pairs = collections.Counter()
p = subprocess.Popen(['gz', 'topic', '-e', '/gazebo/default/physics/contacts'], stdout=subprocess.PIPE, stderr=subprocess.PIPE, start_new_session=True, bufsize=0)
selector = selectors.DefaultSelector()
selector.register(p.stdout, selectors.EVENT_READ)
start = time.monotonic()
buffer = b''
first = None
last_write = start
def checkpoint(final=False, stderr=''):
    result = {'wall_seconds':time.monotonic()-start, 'final':final,
              'contact_pairs':[{'first':a,'second':b,'samples':n} for (a,b),n in pairs.items()],
              'stderr':stderr[-2000:]}
    out.write_text(json.dumps(result,indent=2)+'\n')
    return result
try:
    while time.monotonic() - start < seconds:
        for key, _ in selector.select(timeout=.2):
            data = os.read(key.fd, 65536)
            if not data:
                continue
            buffer += data
            lines = buffer.split(b'\n')
            buffer = lines.pop()
            for raw in lines:
                line = raw.decode(errors='replace').strip()
                if line.startswith('collision1:'):
                    first = line.split('"')[1]
                elif line.startswith('collision2:') and first:
                    pairs[(first, line.split('"')[1])] += 1
                    first = None
        if time.monotonic() - last_write >= 10:
            checkpoint()
            last_write = time.monotonic()
        if p.poll() is not None:
            break
except KeyboardInterrupt:
    pass
finally:
    if p.poll() is None:
        os.killpg(p.pid, signal.SIGINT)
    try:
        p.wait(timeout=5)
    except subprocess.TimeoutExpired:
        os.killpg(p.pid, signal.SIGKILL)
        p.wait()
    err = p.stderr.read().decode(errors='replace')
    result = checkpoint(final=True, stderr=err)
    print(json.dumps(result),flush=True)
