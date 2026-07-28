import glob
import os
import time
import datetime
import wave
import logging
from logging.handlers import RotatingFileHandler


# log init
ROOT_DIR = os.path.dirname(os.path.abspath(__file__))
PATH_LOG = ROOT_DIR + "/logs/"
if not os.path.isdir(PATH_LOG):
    os.makedirs(PATH_LOG, exist_ok=True)
logger = logging.getLogger("MyLogger")
logger.setLevel(logging.DEBUG)
handler = RotatingFileHandler(PATH_LOG + "/log.log", maxBytes=1 * 1024 * 1024, backupCount=5)
formatter = logging.Formatter('%(asctime)s - %(levelname)s - %(message)s')
handler.setFormatter(formatter)
logger.addHandler(handler)


def log_record(*inp):
    tmp_str = " ".join([str(ll) for ll in inp])
    return logger.debug(tmp_str)


def save_wave(des_p, wav_data):
    with wave.open(des_p, 'w') as wf:
        wf.setnchannels(1)
        wf.setsampwidth(2)
        wf.setframerate(16000)
        wf.writeframes(b''.join(wav_data))
    wf.close()


# make new dir
def mk_dir(path_dir):
    if not os.path.isdir(path_dir):
        os.makedirs(path_dir, exist_ok=True)
        return True
    else:
        return False


def wait_period(delay_time, start_t) -> None:
    delta_time_ = delay_time/1000
    start, end = 0, 0
    start = time.time()
    if (start - start_t) < delta_time_:
        t = (delta_time_ - (start-start_t))
        while end - start < t:
            end = time.time()


# time_print
def time_print(*str_):
    print(time.strftime("%Y-%m-%d %H-%M-%S:", time.localtime()), *str_)


if __name__ =="__main__":
    # LOG FLUSH
    log_record(111, 222)