import serial

# XBee setting
serdev = '/dev/ttyUSB0'                # '/dev/ttyACM0' or '/dev/ttyUSB0'
s = serial.Serial(serdev, 9600)
s.write("+++".encode())
char = s.read(3)
print("Enter AT mode.")
print(char.decode())

s.write("ATMY 0x120\r".encode())
char = s.read(3)
print("Set MY 0x120.")
print(char.decode())

s.write("ATDL 0x220\r".encode())
char = s.read(3)
print("Set DL 0x220.")
print(char.decode())

s.write("ATID 0x1\r".encode())
char = s.read(3)
print("Set PAN ID 0x1.")
print(char.decode())

s.write("ATWR\r".encode())
char = s.read(3)
print("Write config.")
print(char.decode())

s.write("ATMY\r".encode())
char = s.read(4)
print("MY :")
print(char.decode())

s.write("ATDL\r".encode())
char = s.read(4)
print("DL : ")
print(char.decode())

s.write("ATCN\r".encode())
char = s.read(3)
print("Exit AT mode.")
print(char.decode())
s.close()