import time
import xrobotoolkit_sdk as xrt

xrt.init()

try:
    while True:
        head = xrt.get_headset_pose()
        left = xrt.get_left_controller_pose()
        right = xrt.get_right_controller_pose()

        timestamp = xrt.get_time_stamp_ns()

        print("\n========== Quest 3 ==========")
        print("Timestamp:", timestamp)
        print("Head:", head)
        print("Left Controller:", left)
        print("Right Controller:", right)

        print("A Button:", xrt.get_A_button())
        print("Right Trigger:", xrt.get_right_trigger())

        time.sleep(0.2)

except KeyboardInterrupt:
    print("Stopping...")

finally:
    xrt.close()