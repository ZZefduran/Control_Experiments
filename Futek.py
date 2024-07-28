import socket
import numpy as np
import time
# Set the IP address and port of the server
# SERVER_IP = "192.168.33.70"
# SERVER_IP = "192.168.33.43"
# SERVER_IP = "192.168.33.89"

# SERVER_IP = "192.168.30.33"
# SERVER_IP = "192.168.55.100"
SERVER_IP = "192.168.31.50"
PORT = 1220

class FutekClient():
    def __init__(self,tester=False):
        self.client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        try:
            # Connect to the server
            self.client_socket.connect((SERVER_IP, PORT))
            print("Connected to the server.")
            self.zero = 0
            if not tester:
                
                print("setting torque meter zero.")

                self.set_zero()
                print("zero is set:",self.zero)
            
                
           
        except Exception as e:
            print("Error:", e)

    def test(self):
        val = self.get_torque()
        print('Futek torque read: ',val)   
        if val:
            return True 
        else:
            return False

    def set_zero(self):
        self.zero = 0
        reads = []
        for i in range(100):
            reads.append(self.get_torque())
            
        self.zero = np.mean(reads)
        print('Futek is set to zero')
        # print(f"the calced Futek bias is: {self.zero}")
        # self.zero = 0 #omris removed 19.12.23

    def send_request(self,request):
        # Send the request to the server
        self.client_socket.sendall(request.encode())

    def receive_response(self):
        # Receive the server's response
        response = self.client_socket.recv(1024).decode()
        # print("Response from server:", response)
        return response


    def get_torque(self):
        # Send requests and receive responses
        
        try:
            self.send_request('request')
            response = self.receive_response()
            torque = np.float64(response)-self.zero
            
            return torque
        
        except Exception as e:
                print("Error:", e)
    
    def close(self):
        # Close the socket
        self.client_socket.close()

# futek = FutekClient()
# while True:
#     print(futek.get_torque())
# # print(f.zero)

# for i in range(1000):  
    
#     print (f.test())
#     time.sleep(0.05) 
    



