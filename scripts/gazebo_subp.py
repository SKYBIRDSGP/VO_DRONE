#!/usr/bin/env python3

import subprocess
import re
import time

# Regex to capture x, y, z floats
pattern = re.compile(r"position\s*{\s*.*?x:\s*([0-9eE\.\-]+)\s*y:\s*([0-9eE\.\-]+)\s*z:\s*([0-9eE\.\-]+)", re.DOTALL)

def main():
    cmd = ["gz", "topic", "-e", "-t", "/model/iris/pose"]
    proc = subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.DEVNULL, text=True)

    print(" Listening to gz topic output...")
    buffer = ""
    try:
        for line in proc.stdout:
            buffer += line
            # Once we detect the closing brace of 'position', attempt parsing
            if "}" in line:
                match = pattern.search(buffer)
                if match:
                    x, y, z = map(float, match.groups())
                    # print("\033c", end="")
                    print(" Drone Position (ENU):")
                    print(f"  ➤ X (East):  {x:.3f} m")
                    print(f"  ➤ Y (North): {y:.3f} m")
                    print(f"  ➤ Z (Up):    {z:.3f} m")
                buffer = ""  # reset buffer after parsing
    except KeyboardInterrupt:
        print("\n Stopped listening")
        proc.terminate()
        proc.wait()

if __name__ == "__main__":
    main()
