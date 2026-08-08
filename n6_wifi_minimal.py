"""Minimal OpenMV N6 firmware 5.0 WiFi stability test."""

import network
import socket
import time
import csi


SSID = "AndersonHub"
PASSWORD = "momrules"
CONNECT_TIMEOUT_MS = 15000


print("TEST 1: script started")
camera = csi.CSI()
camera.reset()
camera.pixformat(csi.RGB565)
camera.framesize(csi.QVGA)
camera.snapshot(time=2000)
print("TEST 1A: camera ready")

wlan = network.WLAN(network.WLAN.IF_STA)
print("TEST 2: WLAN created")
wlan.active(True)
print("TEST 3: WLAN active")
wlan.connect(SSID, PASSWORD)
print("TEST 4: connect requested")

started = time.ticks_ms()
while not wlan.isconnected():
    if time.ticks_diff(time.ticks_ms(), started) > CONNECT_TIMEOUT_MS:
        raise RuntimeError("WiFi connection timed out")
    time.sleep_ms(250)

print("TEST 5: connected")
ip = wlan.ipconfig("addr4")[0]
print("IP:", ip)

server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
server.bind(("0.0.0.0", 80))
server.listen(1)
server.settimeout(0)
print("TEST 6: HTTP listening at http://" + ip + "/")

page = (
    "<!doctype html><html><head><meta name='viewport' "
    "content='width=device-width,initial-scale=1'><title>N6 test</title>"
    "<script>setInterval(function(){document.getElementById('camera').src="
    "'/frame.jpg?t='+Date.now()},500)</script></head>"
    "<body><h1>OpenMV N6 is alive</h1><p>Camera, WiFi, HTTP and JPEG test.</p>"
    "<img id='camera' src='/frame.jpg' style='width:100%;max-width:640px'>"
    "</body></html>"
)
page_response = (
    "HTTP/1.1 200 OK\r\n"
    "Content-Type: text/html\r\n"
    "Cache-Control: no-store\r\n"
    "Connection: close\r\n"
    "Content-Length: " + str(len(page)) + "\r\n\r\n" + page
).encode()

count = 0
while True:
    image = camera.snapshot()

    try:
        client, address = server.accept()
    except OSError:
        client = None

    if client is not None:
        try:
            client.settimeout(2)
            request = client.recv(512)
            request_line = request.split(b"\r\n", 1)[0]
            if b" /frame.jpg" in request_line:
                jpeg = image.to_jpeg(quality=60, copy=True)
                header = (
                    "HTTP/1.1 200 OK\r\n"
                    "Content-Type: image/jpeg\r\n"
                    "Cache-Control: no-store\r\n"
                    "Connection: close\r\n"
                    "Content-Length: " + str(len(jpeg)) + "\r\n\r\n"
                ).encode()
                client.sendall(header)
                client.sendall(jpeg)
                print("JPEG served:", len(jpeg), "bytes")
            else:
                client.sendall(page_response)
                print("HTML served:", address)
        except Exception as error:
            print("HTTP error:", error)
        finally:
            client.close()

    count += 1
    if count % 100 == 0:
        print(
            "ALIVE", count // 100,
            "connected=", wlan.isconnected(),
            "frame=", image.width(), "x", image.height(),
        )
    time.sleep_ms(10)
