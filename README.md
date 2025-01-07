# A modbus rtu - rtu bridge

My setup consists of OVUM heat pump reading regulated depending on readings from smartmeter.
But as I have solar inverter with batteries, it is not so straightforward. 

e.g. you may want to ensure that heatpump gets priority instead of batteries when sun is shining. or vice a versa.
or when heatpump is on batteries are discharging so household balance is set at 0, it send a signal to heatpump that there is excess of energy
(heatpump runs and balance is still zero), so it increases the power.

So, to get good control of what signal we want to send to heatpump, I have this bridge.

Besides, I was relying on the readings of the household consumption balance from IMEON inverter, but it updates values only every 10 secs or so.

# Design
esp32-wroom controller, because it has 3 hw serials and wifi
TTL-RS485 two converters, autoflow control

# Logic
mbMeter is modbusRTU instance which reads via modbusRTU registers from smartmeter
in my instance I have PRO380 Mod
https://cdn.prod.website-files.com/6617f74190bf0a53bfd776e8/667e880abf83adebc352e6b9_PRO380%20User%20manual%20S%20Mb%20Mod.pdf

registers read from Smartmeter are stored in mbTcp - modbusTCP server to be accessible for home automation readings.
mbTCP 51 writable register: it is a manual control vehicle. What you write to hreg51, will override hreg50, so if hreg51 is not zero this value will be tramsited to heat pump.

Smartmeter readings of active power are recalculated to the format required by OVUM heat pump (Ovum reads from 50 holding register, value of activepower in kwh * -100).
I do some calculations to offset battery activities. And is stored in in hreg50

When heatpump tries to read hreg50, logic checks if hreg51 is not zero and returns hreg50 if zero, or hreg51 if nonzero.

# Contributions
Relying on:
https://github.com/emelianov/modbus-esp8266/
