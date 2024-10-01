


import socket, struct, time


sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
server_address = ('192.168.29.42', 12002)


msg = 'ok'
while True:
	data = raw_input("Enter: ")
	sent = sock.sendto(str(data), server_address)
	print ("msg111", msg)
	time.sleep(0.01)
