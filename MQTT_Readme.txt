======================================================
MQTT COMMAND GUIDE
======================================================

The controller uses the MQTT protocol for remote control and status publishing.
All topics use a root prefix of "stormbee/".

------------------------------------------------------
1. CONTROL TOPICS (INPUT)
------------------------------------------------------

These topics are subscribed to by the ESP32 and allow you to send remote
commands to manage the charging process.

A. SET SOC CHARGE LIMIT
   - TOPIC: stormbee/charge/set_limit
   - PAYLOAD: Integer (50 to 100)
   - DESCRIPTION: Sets the maximum State of Charge (SOC) percentage allowed.
                  The relay opens (stops charging) if the SOC is reached 
                  (+1% hysteresis).

B. FORCE CHARGING STATE (Master Override)
   - TOPIC: stormbee/charge/set_power
   - PAYLOAD: "OFF" or any other string (e.g., "ON", "RESUME")
   - DESCRIPTION: If payload is "OFF", charging is stopped immediately. 
                  Any other payload clears the force-off flag and reverts 
                  control to the SOC limit logic.

--- EXAMPLE USAGE ---
- To set limit to 85%:
  Publish "85" to stormbee/charge/set_limit
- To stop charging immediately:
  Publish "OFF" to stormbee/charge/set_power
- To resume SOC-based control:
  Publish "ON" to stormbee/charge/set_power

------------------------------------------------------
2. STATUS TOPICS (OUTPUT)
------------------------------------------------------

These topics are published to by the ESP32 periodically (every 10 seconds).

A. MAIN STATUS DATA

   - TOPIC: stormbee/bms/status
   - FORMAT: JSON object

   - KEYS:
     - v    : Total battery voltage (V)
     - i    : Current (A, positive = charging)
     - soc  : State of Charge (%)
     - soh  : State of Health (%)
     - cap  : Full capacity (Wh)
     - cyc  : Total cycle count
     - t1, t2, tm: Temperature readings (°C)
     - rly  : Current relay state (1=Active/Charging, 0=Off)
     - lim  : Current target charge limit (%)

   - EXAMPLE:
     {"v":58.10,"i":-1.50,"soc":82,"soh":98,"cap":1000.0,"cyc":150,"t1":25,"t2":26,"tm":28,"rly":0,"lim":85}

B. CELL VOLTAGES DATA

   - TOPIC: stormbee/bms/cells
   - FORMAT: JSON object

   - KEYS:
     - cells: Array containing 28 individual cell voltages (V).

   - EXAMPLE (simplified):
     {"cells":[4.100,4.101,4.100,...,4.102]}
