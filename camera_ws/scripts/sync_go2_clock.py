#!/usr/bin/env python3
"""Run interactively on PC, before sensors: authenticate, then send a fresh clock value."""
import getpass
import shlex
import subprocess
import time


def main():
    password = getpass.getpass('GO2 sudo password (local input, not saved): ')
    # Set only the camera companion clock. Avoid spawning date after the PC
    # sampled its time: that process startup becomes a clock-setting error.
    root_script = ("import sys,time; print('CLOCK_SYNC_READY',flush=True); "
                   "value=float(sys.stdin.readline()); "
                   "time.clock_settime(time.CLOCK_REALTIME,value); "
                   "print('COMPANION_CLOCK_SET',time.time(),flush=True)")
    command = shlex.join(['sudo', '-k', '-S', '-p', '', '/usr/bin/python3', '-u', '-c', root_script])
    proc = subprocess.Popen(
        ['ssh', '-i', '/home/czx666/.ssh/id_ed25519_go2', '-o', 'ConnectTimeout=5',
         'unitree@192.168.123.18', command],
        stdin=subprocess.PIPE, stdout=subprocess.PIPE, text=True)
    try:
        proc.stdin.write(password + '\n')
        proc.stdin.flush()
        password = None
        for line in proc.stdout:
            if line.strip() == 'CLOCK_SYNC_READY':
                proc.stdin.write(f'{time.time():.9f}\n')
                proc.stdin.flush()
                proc.stdin.close()
            else:
                print(line.rstrip())
        raise SystemExit(proc.wait())
    finally:
        if proc.poll() is None:
            proc.terminate()


if __name__ == '__main__':
    main()
