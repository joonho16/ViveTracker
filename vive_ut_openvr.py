# vive_ut_openvr.py
import time, math, threading
import numpy as np
import openvr

def _mat34_to_pos_quat(m):
    r00,r01,r02,px = m[0][0], m[0][1], m[0][2], m[0][3]
    r10,r11,r12,py = m[1][0], m[1][1], m[1][2], m[1][3]
    r20,r21,r22,pz = m[2][0], m[2][1], m[2][2], m[2][3]
    tr = r00 + r11 + r22
    if tr > 0:
        S  = math.sqrt(tr + 1.0) * 2.0
        qw = 0.25 * S; qx = (r21 - r12) / S; qy = (r02 - r20) / S; qz = (r10 - r01) / S
    elif r00 > r11 and r00 > r22:
        S  = math.sqrt(1.0 + r00 - r11 - r22) * 2.0
        qw = (r21 - r12) / S; qx = 0.25 * S; q
        y = (r01 + r10) / S; qz = (r02 + r20) / S
    elif r11 > r22:
        S  = math.sqrt(1.0 + r11 - r00 - r22) * 2.0
        qw = (r02 - r20) / S; qx = (r01 + r10) / S; qy = 0.25 * S; qz = (r12 + r21) / S
    else:
        S  = math.sqrt(1.0 + r22 - r00 - r11) * 2.0
        qw = (r10 - r01) / S; qx = (r02 + r20) / S; qy = (r12 + r21) / S; qz = 0.25 * S
    return px,py,pz,qx,qy,qz,qw

class ViveUTrackerOpenVR:
    """
    OpenVR 기반 대체 구현:
    - thread_start()/thread_stop()
    - get_tracker_poses(["right_wrist","left_wrist","chest"]) -> np.array(7) 딕셔너리
    역할명 매핑은 시리얼/타입으로 지정(아래 MAP_ROLE_SERIALS 참고).
    """
    HZ = 60
    # 필요에 맞게 시리얼 맵핑해줘
    MAP_ROLE_SERIALS = {
        "right_wrist": "53-A33W04371",
        "left_wrist":  "53-A33W04329",
        # "chest":     "53-XXXXXXX",
    }

    def __init__(self, visualize=False):
        self.visualize = visualize
        self.stop_flag = False
        self.thread = threading.Thread(target=self._loop, daemon=True)
        self.rhand_pose_np = np.zeros(7, dtype=float)
        self.lhand_pose_np = np.zeros(7, dtype=float)
        self.chest_pose_np = np.zeros(7, dtype=float)
        self.serial_to_role = {v:k for k,v in self.MAP_ROLE_SERIALS.items()}
        openvr.init(openvr.VRApplication_Other)
        self.vr = openvr.VRSystem()

    def __del__(self):
        try:
            openvr.shutdown()
        except Exception:
            pass

    def thread_start(self):
        self.stop_flag = False
        self.thread.start()

    def thread_stop(self):
        self.stop_flag = True
        self.thread.join()

    def _loop(self):
        dt = 1.0/self.HZ
        while not self.stop_flag:
            poses = self.vr.getDeviceToAbsoluteTrackingPose(
                openvr.TrackingUniverseStanding, 0, openvr.k_unMaxTrackedDeviceCount
            )
            for i,p in enumerate(poses):
                if not self.vr.isTrackedDeviceConnected(i):
                    continue
                if self.vr.getTrackedDeviceClass(i) != openvr.TrackedDeviceClass_GenericTracker:
                    continue
                if not p.bPoseIsValid:
                    continue
                try:
                    serial = self.vr.getStringTrackedDeviceProperty(i, openvr.Prop_SerialNumber_String)
                except:
                    continue
                role = self.serial_to_role.get(serial)
                if not role:
                    continue
                x,y,z,qx,qy,qz,qw = _mat34_to_pos_quat(p.mDeviceToAbsoluteTracking.m)
                if role == "right_wrist":
                    self.rhand_pose_np[:] = [x,y,z,qx,qy,qz,qw]
                elif role == "left_wrist":
                    self.lhand_pose_np[:] = [x,y,z,qx,qy,qz,qw]
                elif role == "chest":
                    self.chest_pose_np[:] = [x,y,z,qx,qy,qz,qw]
            time.sleep(dt)

    def get_tracker_poses(self, tracker_id_list):
        out = {}
        for tid in tracker_id_list:
            if tid == "right_wrist":
                out[tid] = self.rhand_pose_np.copy()
            elif tid == "left_wrist":
                out[tid] = self.lhand_pose_np.copy()
            elif tid == "chest":
                out[tid] = self.chest_pose_np.copy()
            else:
                print(f"[WARN] unknown tracker id: {tid}")
        return out
