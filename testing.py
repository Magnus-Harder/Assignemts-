import subprocess
from time import time
from time import sleep


start_time = time()
proc = subprocess.Popen('java -jar server.jar -g -s 300 -t 180 -c "python searchclient/searchclient.py -decentralised" -l levels/MAsimple3.lvl', shell = True, text=True)

while False and time() < start_time + 1:
    try:
        outs, errs = proc.communicate(timeout=0.1)
    except subprocess.TimeoutExpired as excpt:
        print("W: ", excpt.stderr, excpt.output, excpt.cmd)
    print(time())

try:
    outs, errs = proc.communicate(timeout=1)
except subprocess.TimeoutExpired as excpt:
    print("W: ", excpt.stderr, excpt.output, excpt.cmd)
    proc.kill() #Try send ctrl c instead?
    outs, errs = proc.communicate()
    print("TEST",outs, errs)