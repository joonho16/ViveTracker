# test_vive_ut.py
import time
from vive_ut_openvr import ViveUTrackerOpenVR

def main():
    ctx = ViveUTrackerOpenVR()
    ctx.thread_start()
    try:
        while True:
            poses = ctx.get_tracker_poses(["right_wrist","left_wrist"])
            print("poses:", poses)
            time.sleep(0.5)
    except KeyboardInterrupt:
        ctx.thread_stop()

if __name__ == "__main__":
    main()
