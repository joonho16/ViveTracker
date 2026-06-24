# send_udp_openvr.py
import argparse
import json
import socket
import time
import math
import signal
import sys

import openvr

def mat34_to_pos_quat(m):
    # m: openvr.HmdMatrix34_t.m => ((c_float*4)*3)
    r00,r01,r02,px = m[0][0], m[0][1], m[0][2], m[0][3]
    r10,r11,r12,py = m[1][0], m[1][1], m[1][2], m[1][3]
    r20,r21,r22,pz = m[2][0], m[2][1], m[2][2], m[2][3]
    tr = r00 + r11 + r22
    if tr > 0:
        S  = math.sqrt(tr + 1.0) * 2.0
        qw = 0.25 * S
        qx = (r21 - r12) / S
        qy = (r02 - r20) / S
        qz = (r10 - r01) / S
    elif r00 > r11 and r00 > r22:
        S  = math.sqrt(1.0 + r00 - r11 - r22) * 2.0
        qw = (r21 - r12) / S
        qx = 0.25 * S
        qy = (r01 + r10) / S
        qz = (r02 + r20) / S
    elif r11 > r22:
        S  = math.sqrt(1.0 + r11 - r00 - r22) * 2.0
        qw = (r02 - r20) / S
        qx = (r01 + r10) / S
        qy = 0.25 * S
        qz = (r12 + r21) / S
    else:
        S  = math.sqrt(1.0 + r22 - r00 - r11) * 2.0
        qw = (r10 - r01) / S
        qx = (r02 + r20) / S
        qy = (r12 + r21) / S
        qz = 0.25 * S
    return (px, py, pz, qx, qy, qz, qw)

def get_string_prop(vr, i, prop):
    try:
        return vr.getStringTrackedDeviceProperty(i, prop)
    except Exception:
        return ""

def main():
    ap = argparse.ArgumentParser(description="Send Vive/SteamVR GenericTracker poses over UDP(JSON)")
    ap.add_argument("--ip", default="127.0.0.1", help="Destination IP (receiver)")
    ap.add_argument("--port", type=int, default=9000, help="Destination UDP port")
    ap.add_argument("--hz", type=float, default=60.0, help="Send rate (Hz)")
    # ap.add_argument("--frame", default="steamvr_world", help="frame_id value in JSON")
    ap.add_argument("--frame", default="base_link", help="frame_id value in JSON")
    ap.add_argument("--serial", action="append", default=[],
                    help="Tracker serial to send (repeatable). If omitted, send ALL GenericTrackers.")
    ap.add_argument("--rolemap", action="append", default=[],
                    help="Optional ROLE=SERIAL mapping (e.g., right_wrist=53-A33W04371). Repeatable.")
    ap.add_argument("--unscaled", action="store_true",
                    help="Send raw pose only (ignore role mapping keys).")
    args = ap.parse_args()

    # Parse role mapping
    rolemap = {}
    for entry in args.rolemap:
        if "=" in entry:
            k, v = entry.split("=", 1)
            rolemap[k.strip()] = v.strip()

    # Build reverse map: serial -> role
    serial_to_role = {v: k for k, v in rolemap.items()}

    # UDP socket
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
    
    dest = (args.ip, args.port)

    # Graceful shutdown
    stop = {"flag": False}
    def _sig(*_):
        stop["flag"] = True
    signal.signal(signal.SIGINT, _sig)
    if hasattr(signal, "SIGTERM"):
        signal.signal(signal.SIGTERM, _sig)

    # OpenVR init (SteamVR running & null HMD enabled recommended)
    openvr.init(openvr.VRApplication_Other)
    vr = openvr.VRSystem()

    dt = 1.0 / max(args.hz, 1e-3)
    print(f"[INFO] Sending UDP to {dest} at {args.hz} Hz")
    if args.serial:
        print(f"[INFO] Filtering serials: {args.serial}")
    if rolemap:
        print(f"[INFO] Role mapping: {rolemap}")

    try:
        while not stop["flag"]:
            poses = vr.getDeviceToAbsoluteTrackingPose(
                openvr.TrackingUniverseStanding, 0, openvr.k_unMaxTrackedDeviceCount
            )
            tnow = time.time()
            for i, p in enumerate(poses):
                if not vr.isTrackedDeviceConnected(i):
                    continue
                if vr.getTrackedDeviceClass(i) != openvr.TrackedDeviceClass_GenericTracker:
                    continue
                if not p.bPoseIsValid:
                    print(f"[DEBUG] Device {i} pose not valid")
                    continue

                serial = get_string_prop(vr, i, openvr.Prop_SerialNumber_String)
                if args.serial and (serial not in args.serial):
                    continue

                x,y,z,qx,qy,qz,qw = mat34_to_pos_quat(p.mDeviceToAbsoluteTracking.m)

                msg = {
                    "serial": serial,
                    "pos": [x, y, z],
                    "quat": [qx, qy, qz, qw],
                    "frame": args.frame,
                    "t": tnow,
                }

                # Optional role tag
                role = serial_to_role.get(serial)
                if role and not args.unscaled:
                    msg["role"] = role

                if rolemap:
                    print(f"[{role}] x={x:.3f} y={y:.3f} z={z:.3f}")

                sock.sendto(json.dumps(msg).encode("utf-8"), dest)

            time.sleep(dt)
    finally:
        try:
            openvr.shutdown()
        except Exception:
            pass
        sock.close()
        print("\n[INFO] Stopped.")

if __name__ == "__main__":
    main()
