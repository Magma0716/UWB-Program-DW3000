import sys
import time

try:
    while True:
        print("Loop is running... Press Ctrl+C to stop.")
        time.sleep(2) # Simulates a long-running task
except KeyboardInterrupt:
    print("\nProgram terminated by user (Ctrl+C).")
    
print(1)