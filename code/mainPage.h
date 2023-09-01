/*  Copyright (C) 2016 Buxtronix and Alexander Pruss

This program is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, version 3 of the License.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program.  If not, see <http://www.gnu.org/licenses/>. */

const char MAIN_page[] PROGMEM = R"=====(
<html>
<head>
 <meta name="viewport" content="initial-scale=1">
 <style>
 body {font-family: helvetica,arial,sans-serif;}
 table {border-collapse: collapse; border: 1px solid black;}
 td {padding: 0.25em;}
 .title { font-size: 2em; font-weight: bold;}
 .name {padding: 0.5em;}
 .heading {font-weight: bold; background: #c0c0c0; padding: 0.5em;}
 .update {color: #dd3333; font-size: 0.75em;}
 </style>
</head>
<div class=title>ESP8266 Air Quality sensor</div>
<div class=version>Firmware: @@VERSION@@</div>
<div class=name>Location: @@CLOCKNAME@@</div>
<form method="post" action="/form">
<table>
<tr><td colspan=2 class=heading>Current Status</td></tr>
<tr><td>PM10:   </td><td>@@PM10@@ &mu;g/m<sup>3</sup></td></tr>
<tr><td>PM2.5:  </td><td>@@PM25@@ &mu;g/m<sup>3</sup></td></tr>
<tr><td>Temp:  </td><td>@@TEMPERATURE@@ &#186C</td></tr>
<tr><td>Humid:  </td><td>@@HUMIDITY@@ %</td></tr>
<tr><td>Battery:    </td><td>@@BATT@@ %</td></tr>

<tr><td colspan=2 class=heading>WiFi Setup</td></tr>
<tr><td>SSID:</td><td><input type=text name="ssid" value="@@SSID@@"></td></tr>
<tr><td>PSK:</td><td><input type=password name="psk" value="@@PSK@@"></td></tr>
<tr><td>Name:</td><td><input type=text name="clockname" value="@@CLOCKNAME@@"></td></tr>
<tr><td colspan=2>Update Wifi config:<input type=checkbox name=update_wifi value=1></td></tr>

<tr><td colspan=2 class=heading>Parameters</td></tr>
<tr><td>Pooling Interval:</td><td><input type=text name="poolint" value="@@POOLINT@@"> sec (10 - 3600)</td></tr>
<tr><td colspan=2>ECO Mode:<input type=checkbox name=eco_mode value=@@ECOMODE@@></td></tr>

<tr><td colspan=2 class=heading>MQTT Setup</td></tr>
<tr><td>MQTT broker:</td><td><input type=text size=35 name="mqttbroker" value="@@MQTTBROKER@@"></td></tr>
<tr><td>MQTT port:</td><td><input type=text size=35 name="mqttport" value="@@MQTTPORT@@"></td></tr>
<tr><td>MQTT user:</td><td><input type=text size=35 name="mqttuser" value="@@MQTTUSER@@"></td></tr>
<tr><td>MQTT passwd:</td><td><input type=text size=35 name="mqttpasswd" value="@@MQTTPASSWD@@"></td></tr>
<tr><td>MQTT topic:</td><td><input type=text size=35 name="mqtttopic" value="@@MQTTTOPIC@@"></td></tr>

<tr><td colspan=2 class=heading>Wunderground access</td></tr>
<tr><td>Station ID:</td><td><input type=text size=35 name="stationid" value="@@STATIONID@@"></td></tr>
<tr><td>Station Key:</td><td><input type=text size=35 name="stationkey" value="@@STATIONKEY@@"></td></tr>

</table>
<p/>
<input type="submit" value="Update">
</form>
<div class="update">@@UPDATERESPONSE@@</div>
</html>
)=====";
