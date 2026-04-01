import serial

ser = serial.Serial('/dev/ttyACM0', 115200, timeout=1)

while True:
    line = ser.readline().decode('utf-8').strip()
    if line.startswith("SENSOR:"):
        values = line.replace("SENSOR:", "").split(",")
        x, y, z = float(values[0]), float(values[1]), float(values[2])
        print(f"X:{x} Y:{y} Z:{z}")
