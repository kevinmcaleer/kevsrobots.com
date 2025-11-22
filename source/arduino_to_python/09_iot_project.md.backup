---
title: IoT Project - WiFi Temperature Monitor
description: Build a complete IoT project that connects to WiFi, reads sensor data, and serves a web page showing real-time temperature readings
layout: lesson
type: page
cover: /learn/arduino_to_python/assets/cover.jpg
---

## Building Your First IoT Project

Let's put everything together and build a real IoT project: a WiFi-enabled temperature monitor that serves a web page showing real-time sensor readings. This project demonstrates why MicroPython excels at IoT development.

**What you'll build:**
- WiFi connection to your network
- Temperature sensor reading (simulated or real)
- Web server serving a simple HTML page
- Real-time data updates
- Status LED indicator

**Hardware needed:**
- Arduino Nano ESP32, Raspberry Pi Pico W, or Pico 2W
- (Optional) Temperature sensor like DHT22 or BME280
- LED for status indication

## Project Overview

The project will:
1. Connect to your WiFi network
2. Read temperature data every second
3. Serve a web page at the board's IP address
4. Display current temperature and uptime
5. Blink an LED when serving requests

## Step 1: WiFi Connection

First, let's connect to WiFi. This is much simpler in MicroPython than Arduino:

**Arduino C++ (ESP32):**
```cpp
#include <WiFi.h>

const char* ssid = "YourWiFi";
const char* password = "YourPassword";

void setup() {
    Serial.begin(115200);
    WiFi.begin(ssid, password);

    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.print(".");
    }

    Serial.println("Connected!");
    Serial.println(WiFi.localIP());
}
```

**MicroPython:**
```python
import network
import time

SSID = "YourWiFi"
PASSWORD = "YourPassword"

def connect_wifi():
    wlan = network.WLAN(network.STA_IF)
    wlan.active(True)

    if not wlan.isconnected():
        print(f"Connecting to {SSID}...")
        wlan.connect(SSID, PASSWORD)

        # Wait for connection
        timeout = 10
        while not wlan.isconnected() and timeout > 0:
            print(".", end="")
            time.sleep(1)
            timeout -= 1

    if wlan.isconnected():
        print("\nConnected!")
        print(f"IP address: {wlan.ifconfig()[0]}")
        return True
    else:
        print("\nConnection failed!")
        return False

# Connect to WiFi
connect_wifi()
```

Much cleaner! No includes, just import `network` and call methods.

## Step 2: Reading Temperature

We'll simulate temperature for this example, but you can easily add a real sensor:

**Simulated Temperature:**
```python
import random

def read_temperature():
    """Simulate temperature reading (20-30°C)"""
    return 20 + random.random() * 10
```

**Real DHT22 Sensor (if you have one):**
```python
from machine import Pin
import dht

sensor = dht.DHT22(Pin(15))  # DHT22 on pin 15

def read_temperature():
    """Read temperature from DHT22 sensor"""
    try:
        sensor.measure()
        return sensor.temperature()
    except:
        return None
```

**Real BME280 Sensor (if you have one):**
```python
from machine import Pin, I2C
import bme280

i2c = I2C(0, scl=Pin(22), sda=Pin(21))
sensor = bme280.BME280(i2c=i2c)

def read_temperature():
    """Read temperature from BME280 sensor"""
    temp, pressure, humidity = sensor.read_compensated_data()
    return temp / 100  # Convert to Celsius
```

## Step 3: Web Server

Now let's create a simple web server:

**MicroPython:**
```python
import socket
import time

def create_web_server():
    """Create and return a socket web server"""
    addr = socket.getaddrinfo('0.0.0.0', 80)[0][-1]
    s = socket.socket()
    s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    s.bind(addr)
    s.listen(1)
    print(f"Web server listening on port 80")
    return s

def generate_html(temperature, uptime):
    """Generate HTML page with current data"""
    html = f"""<!DOCTYPE html>
<html>
<head>
    <title>Temperature Monitor</title>
    <meta http-equiv="refresh" content="5">
    <style>
        body {{
            font-family: Arial, sans-serif;
            text-align: center;
            background: #f0f0f0;
            padding: 50px;
        }}
        .container {{
            background: white;
            border-radius: 10px;
            padding: 30px;
            box-shadow: 0 4px 6px rgba(0,0,0,0.1);
            max-width: 500px;
            margin: 0 auto;
        }}
        .temperature {{
            font-size: 72px;
            color: #ff6b6b;
            font-weight: bold;
        }}
        .label {{
            font-size: 24px;
            color: #666;
            margin-top: 10px;
        }}
        .uptime {{
            margin-top: 30px;
            color: #999;
            font-size: 16px;
        }}
    </style>
</head>
<body>
    <div class="container">
        <h1>IoT Temperature Monitor</h1>
        <div class="temperature">{temperature:.1f}°C</div>
        <div class="label">Current Temperature</div>
        <div class="uptime">Uptime: {uptime:.0f} seconds</div>
        <p>Page auto-refreshes every 5 seconds</p>
    </div>
</body>
</html>"""
    return html
```

## Step 4: Complete IoT Project

Now let's combine everything into a complete project:

**complete_iot_project.py:**
```python
from machine import Pin
import network
import socket
import time
import random

# Configuration
SSID = "YourWiFi"
PASSWORD = "YourPassword"
LED_PIN = 25  # Built-in LED (Pico) or 13 (Arduino Nano ESP32)

# Global variables
start_time = time.time()
led = Pin(LED_PIN, Pin.OUT)

def connect_wifi():
    """Connect to WiFi network"""
    wlan = network.WLAN(network.STA_IF)
    wlan.active(True)

    if not wlan.isconnected():
        print(f"Connecting to {SSID}...")
        wlan.connect(SSID, PASSWORD)

        timeout = 10
        while not wlan.isconnected() and timeout > 0:
            print(".", end="")
            led.toggle()
            time.sleep(0.5)
            timeout -= 0.5

    if wlan.isconnected():
        led.on()  # Solid LED when connected
        print("\nConnected!")
        ip = wlan.ifconfig()[0]
        print(f"IP address: {ip}")
        print(f"Visit http://{ip} in your browser")
        return True
    else:
        print("\nConnection failed!")
        return False

def read_temperature():
    """Simulate temperature reading (20-30°C)"""
    # Replace with real sensor code
    return 20 + random.random() * 10

def generate_html(temperature, uptime):
    """Generate HTML page with current data"""
    html = f"""<!DOCTYPE html>
<html>
<head>
    <title>Temperature Monitor</title>
    <meta http-equiv="refresh" content="5">
    <style>
        body {{
            font-family: Arial, sans-serif;
            text-align: center;
            background: linear-gradient(135deg, #667eea 0%, #764ba2 100%);
            padding: 50px;
            margin: 0;
            min-height: 100vh;
        }}
        .container {{
            background: white;
            border-radius: 20px;
            padding: 40px;
            box-shadow: 0 10px 25px rgba(0,0,0,0.2);
            max-width: 500px;
            margin: 0 auto;
        }}
        .temperature {{
            font-size: 72px;
            color: #ff6b6b;
            font-weight: bold;
            margin: 20px 0;
        }}
        .label {{
            font-size: 24px;
            color: #666;
            margin-top: 10px;
        }}
        .uptime {{
            margin-top: 30px;
            color: #999;
            font-size: 16px;
            padding: 15px;
            background: #f8f9fa;
            border-radius: 10px;
        }}
        h1 {{
            color: #333;
            margin-top: 0;
        }}
    </style>
</head>
<body>
    <div class="container">
        <h1>🌡️ IoT Temperature Monitor</h1>
        <div class="temperature">{temperature:.1f}°C</div>
        <div class="label">Current Temperature</div>
        <div class="uptime">
            ⏱️ Uptime: {uptime:.0f} seconds<br>
            🔄 Auto-refresh every 5 seconds
        </div>
    </div>
</body>
</html>"""
    return html

def handle_request(client):
    """Handle incoming HTTP request"""
    try:
        request = client.recv(1024).decode()

        # Read current data
        temperature = read_temperature()
        uptime = time.time() - start_time

        # Generate and send response
        html = generate_html(temperature, uptime)
        response = f"HTTP/1.1 200 OK\r\nContent-Type: text/html\r\n\r\n{html}"
        client.send(response.encode())

        # Blink LED to show activity
        led.off()
        time.sleep(0.05)
        led.on()

    except Exception as e:
        print(f"Error handling request: {e}")
    finally:
        client.close()

def main():
    """Main program"""
    print("=" * 50)
    print("IoT Temperature Monitor")
    print("=" * 50)

    # Connect to WiFi
    if not connect_wifi():
        print("Cannot start without WiFi")
        return

    # Create web server
    server = socket.socket()
    addr = socket.getaddrinfo('0.0.0.0', 80)[0][-1]
    server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    server.bind(addr)
    server.listen(1)

    print("\nServer ready! Waiting for connections...")

    # Main server loop
    while True:
        try:
            client, addr = server.accept()
            print(f"Connection from {addr}")
            handle_request(client)
        except KeyboardInterrupt:
            print("\nShutting down...")
            break
        except Exception as e:
            print(f"Error: {e}")

    server.close()

# Run the program
if __name__ == '__main__':
    main()
```

## Step 5: Running the Project

1. **Edit WiFi credentials:**
   ```python
   SSID = "YourWiFiName"
   PASSWORD = "YourWiFiPassword"
   ```

2. **Upload to your board:**
   - Save as `main.py` on your board
   - MicroPython runs `main.py` automatically on boot

3. **Connect to the REPL to see output:**
   - Watch for the IP address
   - Example: `IP address: 192.168.1.100`

4. **Visit the web page:**
   - Open a browser on the same network
   - Go to `http://192.168.1.100` (use your board's IP)
   - See your temperature monitor!

## Adding Real Sensor Support

To use a real DHT22 temperature sensor:

1. **Wire the sensor:**
   - VCC → 3.3V
   - GND → GND
   - DATA → GPIO15 (or any GPIO pin)

2. **Update the code:**
   ```python
   from machine import Pin
   import dht

   sensor = dht.DHT22(Pin(15))

   def read_temperature():
       try:
           sensor.measure()
           return sensor.temperature()
       except Exception as e:
           print(f"Sensor error: {e}")
           return None
   ```

3. **Handle errors in the HTML:**
   ```python
   def generate_html(temperature, uptime):
       if temperature is None:
           temp_display = "ERROR"
           temp_color = "#999"
       else:
           temp_display = f"{temperature:.1f}°C"
           temp_color = "#ff6b6b"

       html = f"""<!DOCTYPE html>
   <html>
   <head>
       <title>Temperature Monitor</title>
       <meta http-equiv="refresh" content="5">
       <style>
           /* styles here */
       </style>
   </head>
   <body>
       <div class="container">
           <h1>🌡️ IoT Temperature Monitor</h1>
           <div class="temperature" style="color: {temp_color};">{temp_display}</div>
           <div class="label">Current Temperature</div>
           <div class="uptime">⏱️ Uptime: {uptime:.0f} seconds</div>
       </div>
   </body>
   </html>"""
       return html
   ```

## Enhancements and Ideas

**Add more data:**
```python
def read_sensors():
    return {
        'temperature': read_temperature(),
        'humidity': read_humidity(),
        'pressure': read_pressure()
    }
```

**Add API endpoint:**
```python
def handle_request(client):
    request = client.recv(1024).decode()

    if "/api/data" in request:
        # Return JSON
        data = {'temperature': read_temperature(), 'uptime': time.time() - start_time}
        response = f"HTTP/1.1 200 OK\r\nContent-Type: application/json\r\n\r\n{data}"
    else:
        # Return HTML
        html = generate_html(...)
        response = f"HTTP/1.1 200 OK\r\nContent-Type: text/html\r\n\r\n{html}"

    client.send(response.encode())
    client.close()
```

**Add data logging:**
```python
readings = []

def log_reading(temp):
    timestamp = time.time() - start_time
    readings.append((timestamp, temp))
    if len(readings) > 100:  # Keep last 100
        readings.pop(0)
```

## Comparing to Arduino

Building this in Arduino C++ would require:
- WiFi library (different for each board)
- String manipulation for HTML
- More complex socket handling
- Separate web server library

MicroPython's advantages:
- Built-in networking modules
- Easy string formatting with f-strings
- Simple socket API
- Rapid development and testing via REPL

## Try It Yourself

**Exercise 1: Add Humidity Display**

Modify the project to display humidity alongside temperature:
- Add a simulated humidity reading (40-80%)
- Update the HTML to show both values
- Use different colors for each

**Answer:**

```python
def read_sensors():
    temperature = 20 + random.random() * 10
    humidity = 40 + random.random() * 40
    return temperature, humidity

def generate_html(temperature, humidity, uptime):
    html = f"""<!DOCTYPE html>
<html>
<head>
    <title>Environmental Monitor</title>
    <meta http-equiv="refresh" content="5">
    <style>
        body {{
            font-family: Arial, sans-serif;
            text-align: center;
            background: linear-gradient(135deg, #667eea 0%, #764ba2 100%);
            padding: 50px;
        }}
        .container {{
            background: white;
            border-radius: 20px;
            padding: 40px;
            max-width: 600px;
            margin: 0 auto;
        }}
        .reading {{
            display: inline-block;
            margin: 20px;
            padding: 20px;
            border-radius: 10px;
        }}
        .temperature {{
            font-size: 48px;
            color: #ff6b6b;
            font-weight: bold;
        }}
        .humidity {{
            font-size: 48px;
            color: #4dabf7;
            font-weight: bold;
        }}
    </style>
</head>
<body>
    <div class="container">
        <h1>🌡️ Environmental Monitor</h1>
        <div class="reading">
            <div class="temperature">{temperature:.1f}°C</div>
            <div>Temperature</div>
        </div>
        <div class="reading">
            <div class="humidity">{humidity:.0f}%</div>
            <div>Humidity</div>
        </div>
        <div>⏱️ Uptime: {uptime:.0f}s</div>
    </div>
</body>
</html>"""
    return html
```

**Exercise 2: Add LED Control**

Add a button to the web page that turns an LED on/off:
- Add a form with ON/OFF buttons
- Parse the request URL for commands
- Control an LED based on the request

**Answer:**

```python
led_state = False
control_led = Pin(14, Pin.OUT)

def generate_html(temperature, uptime, led_state):
    led_status = "ON" if led_state else "OFF"
    led_color = "#4caf50" if led_state else "#f44336"

    html = f"""<!DOCTYPE html>
<html>
<head>
    <title>Temperature Monitor with Control</title>
    <style>
        /* Previous styles plus: */
        .controls {{
            margin-top: 30px;
        }}
        button {{
            padding: 15px 30px;
            margin: 10px;
            font-size: 18px;
            border: none;
            border-radius: 5px;
            cursor: pointer;
        }}
        .on-btn {{ background: #4caf50; color: white; }}
        .off-btn {{ background: #f44336; color: white; }}
    </style>
</head>
<body>
    <div class="container">
        <h1>🌡️ Temperature Monitor</h1>
        <div class="temperature">{temperature:.1f}°C</div>
        <div class="controls">
            <p>LED Status: <span style="color: {led_color};">{led_status}</span></p>
            <a href="/?led=on"><button class="on-btn">Turn ON</button></a>
            <a href="/?led=off"><button class="off-btn">Turn OFF</button></a>
        </div>
    </div>
</body>
</html>"""
    return html

def handle_request(client):
    global led_state
    request = client.recv(1024).decode()

    # Parse LED control
    if "GET /?led=on" in request:
        led_state = True
        control_led.on()
    elif "GET /?led=off" in request:
        led_state = False
        control_led.off()

    temperature = read_temperature()
    uptime = time.time() - start_time
    html = generate_html(temperature, uptime, led_state)
    response = f"HTTP/1.1 200 OK\r\nContent-Type: text/html\r\n\r\n{html}"
    client.send(response.encode())
    client.close()
```

**Exercise 3: Create a Data Logger**

Store the last 10 temperature readings and display them as a list on the web page with timestamps.

**Answer:**

```python
readings_log = []

def log_reading(temp):
    timestamp = time.time() - start_time
    readings_log.append((timestamp, temp))
    if len(readings_log) > 10:
        readings_log.pop(0)

def generate_html(temperature, uptime):
    # Generate readings table
    readings_html = ""
    for timestamp, temp in reversed(readings_log):
        readings_html += f"<tr><td>{timestamp:.0f}s</td><td>{temp:.1f}°C</td></tr>"

    html = f"""<!DOCTYPE html>
<html>
<head>
    <title>Temperature Logger</title>
    <meta http-equiv="refresh" content="5">
    <style>
        table {{ margin: 20px auto; border-collapse: collapse; }}
        th, td {{ padding: 10px; border: 1px solid #ddd; }}
        th {{ background: #667eea; color: white; }}
    </style>
</head>
<body>
    <div class="container">
        <h1>Temperature Logger</h1>
        <div class="temperature">{temperature:.1f}°C</div>
        <h2>Recent Readings</h2>
        <table>
            <tr><th>Time</th><th>Temperature</th></tr>
            {readings_html}
        </table>
    </div>
</body>
</html>"""
    return html

# In main loop, log every reading
temperature = read_temperature()
log_reading(temperature)
```
