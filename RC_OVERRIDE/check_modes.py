from pymavlink import mavutil
import time

master = mavutil.mavlink_connection('udp:127.0.0.1:14550')

master.wait_heartbeat()
print("Heartbeat получен!!!")

for i in range (29):
    print(f"Setting mode {i}")
    master.set_mode(i)
    time.sleep(2)

'''
0
Mode STABILIZE

1
Mode ACRO

2
Mode ALT_HOLD

3
Mode AUTO

4
Mode GUIDED

5
Mode LOITER

6
Mode RTL

7
Mode CIRCLE

8
AP: No such mode 8

9
Mode LAND

10
AP: No such mode 10

11
Mode DRIFT

12
AP: No such mode 12

13
AP: No such mode 13

14
AP: Mode change to FLIP failed: init failed

15
AP: Mode change to AUTOTUNE failed: init failed

16
Mode POSHOLD

17
Mode BRAKE

18
Mode THROW

19
Mode AVOID_ADSB

20
Mode GUIDED_NOGPS

21
Mode SMART_RTL

22
AP: Mode change to FLOWHOLD failed: init failed

23
AP: Set FOLL_ENABLE = 1
AP: Mode change to FOLLOW failed: init failed

24
Mode ZIGZAG

25
AP: No axis selected, SID_AXIS = 0
AP: Mode change to SYSTEMID failed: init failed

26
AP: No such mode 26

27
AP: Unable to start landing sequence
AP: Mode change to AUTO RTL failed: No return path or landing sequence found

28
AP: Mode change to TURTLE failed: init failed
'''