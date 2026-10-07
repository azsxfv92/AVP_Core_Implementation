#!/usr/bin/env python3

import mmap
import os
import struct
import time

SHM_NAME = "avp_target_accel"          
SHM_PATH = "/dev/shm/" + SHM_NAME
SLOT_SIZE = 24                          

_FMT_SEQ = "<Q"
_FMT_ALL = "<QQf"


def open_channel():
    if not os.path.exists(SHM_PATH):
        return None
    fd = os.open(SHM_PATH, os.O_RDONLY)
    try:
        mm = mmap.mmap(fd, SLOT_SIZE, mmap.MAP_SHARED, mmap.PROT_READ)
    finally:
        os.close(fd)                  
    return mm


def read_accel(mm):
    """ read seqlock
    """
    seq1 = struct.unpack_from(_FMT_SEQ, mm, 0)[0]
    if (seq1%2) == 1 :
        return None
    
    seq_xm, stamp_ns, accel = struct.unpack_from(_FMT_ALL, mm, 0)
    seq2 = struct.unpack_from(_FMT_SEQ, mm, 0)[0]
    if seq1 != seq2 :
        return None
    else :
        return (seq1, stamp_ns, accel)


def age_ns(stamp_ns):
    """ measure the time after the data was written
    """
    return time.monotonic_ns() - stamp_ns


if __name__ == "__main__":
    mm = open_channel()
    while mm is None:
        print(f"waiting for {SHM_PATH} ...", flush=True)
        time.sleep(0.2)
        mm = open_channel()

    last_seq = 0
    while True:
        got = read_accel(mm)
        if got is not None:
            seq, stamp_ns, accel = got
            if seq != 0 and seq != last_seq:
                last_seq = seq
                print(f"seq={seq} accel={accel:.3f} age_us={age_ns(stamp_ns)/1000:.1f}",
                      flush=True)
        time.sleep(0.005)
