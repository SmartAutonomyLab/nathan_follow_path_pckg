#!/usr/bin/env python3
import subprocess
import time
import random

def run_child_script(script_name, timeout=30):
    print(f"Running {script_name}")
    try: 
        subprocess.run(["python3", script_name],timeout=timeout)
    except subprocess.TimeoutExpired:
        print(f"Terminated {script_name} due to timeout")
    print(f"Finished {script_name}")
    

def switch_processes():
    while True:
        bool = random.choice( [ True, False ] )
        if bool:
            run_child_script("child_test1.py")
        else:
            run_child_script("child_test2.py")
        time.sleep(2)
if __name__ == "__main__":
    switch_processes()
