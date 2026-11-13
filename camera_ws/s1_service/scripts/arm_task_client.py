#!/usr/bin/env python3
"""Local JSON socket client for the persistent arm integration service."""
import argparse
import json
import socket
import time
from pathlib import Path

DEFAULT_SOCKET = "/tmp/s1-arm-integration.sock"

def request(command, socket_path=DEFAULT_SOCKET):
    with socket.socket(socket.AF_UNIX, socket.SOCK_STREAM) as client:
        client.settimeout(3.0)
        client.connect(socket_path)
        client.sendall((json.dumps({"command": command})+"\n").encode())
        with client.makefile("rb") as stream:
            return json.loads(stream.readline())

def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("command", choices=["status", "grasp", "zero"])
    parser.add_argument("--socket", default=DEFAULT_SOCKET)
    parser.add_argument('--wait-ready', action='store_true')
    parser.add_argument('--timeout', type=float, default=45.0)
    parser.add_argument('--status-file')
    args = parser.parse_args()
    if args.wait_ready and (args.command != 'status' or args.timeout <= 0):
        parser.error('--wait-ready requires status and a positive timeout')
    deadline=time.monotonic()+args.timeout
    while True:
        try:
            result=request(args.command,args.socket)
        except OSError as exc:
            if not args.wait_ready:
                raise
            p=Path(args.status_file) if args.status_file else None
            result=json.loads(p.read_text()) if p and p.exists() else dict(state='STARTING',error=str(exc))
        if not args.wait_ready:
            break
        state=result.get('readiness_state',result.get('state',''))
        if (state=='ZERO_HOLD' and result.get('at_zero') is True and
                result.get('navigation_allowed') is True and result.get('joint_positions_rad')):
            break
        if (state in ('COMMUNICATION_ERROR','FAULT') or state.startswith('FAULT') or
                result.get('return_zero_allowed') is False):
            break
        if time.monotonic()>=deadline:
            result=dict(state='FAULT',error='S1 readiness timeout',last_status=result)
            break
        time.sleep(.2)
    print(json.dumps(result, ensure_ascii=False, indent=2))

if __name__ == "__main__":
    main()
