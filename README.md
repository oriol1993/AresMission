# Ares Mission

state: 0 standby, 1 ready, 2 ignited, 3 10s, 4 parachute

- ```read_buttn()```..
...long press: erase flash, short press: state 0->1, or (2,3,4)->0..
- ```updt_accel()```
state = 0, nothing
state > 0, f=200hz to flash
only sampling vertical component
- ```updt_barom()```
state = 0, f=max
state > 0, f=max to flash
- ```updt_gps()```
state = any, f=1Hz to XBee
- ```igni_stage()```
when t>t_burn AND h>h_min AND state 2
- ```igni_parac()```
when h_max>1000m AND h<h_parachute  AND state 3
- ```rset_flash()```
- ```writ_flash()```
write only values with flag=1
time_stamp[13 bits]+code[3 bits]
code:
0 accel data
1 barom data
2 launch
3 ignition 2nd stage
4 ignition parachute
5 2nd ignition failed h<hmin
- ```send_xbee()```
always send GPS when flag=1
- ```send_info()```
when serial input = info
- ```updt_leds()```
- ```dump_flash()```
when serial input = dump
