#!/usr/bin/env python3
"""Run interactively on PC, before sensors: authenticate, then send a fresh clock value."""
import getpass
import shlex
import subprocess
import time


def main():
    password = getpass.getpass('GO2 sudo password (local input, not saved): ')
    root_script = 'printf "CLOCK_SYNC_READY\\n"; read -r clock_epoch; date -s "@$clock_epoch"'
    command = shlex.join(['sudo', '-k', '-S', '-p', '', 'sh', '-c', root_script])
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
