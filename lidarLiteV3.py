#Date:-20/01/2020
#This code considers distance value fro 0 to 360 degrees
#This code scans Lidar sensor values and feeds into the neural network(network.py)

import network
import numpy as np
from time import sleep
#import smbus #import SMBus module of I2C
import logging
import sys
import time
import codecs
import serial
import struct

SYNC_BYTE = b'\xA5'
SYNC_BYTE2 = b'\x5A'

GET_INFO_BYTE = b'\x50'
GET_HEALTH_BYTE = b'\x52'

STOP_BYTE = b'\x25'
RESET_BYTE = b'\x40'

SCAN_BYTE = b'\x20'
FORCE_SCAN_BYTE = b'\x21'

DESCRIPTOR_LEN = 7
INFO_LEN = 20
HEALTH_LEN = 3

INFO_TYPE = 4
HEALTH_TYPE = 6
SCAN_TYPE = 129

#Constants & Command to start A2 motor
MAX_MOTOR_PWM = 1023
DEFAULT_MOTOR_PWM = 660
SET_PWM_BYTE = b'\xF0'

_HEALTH_STATUSES = {
    0: 'Good',
    1: 'Warning',
    2: 'Error',
}
# Lidar sensor setup
class RPLidarException(Exception):
    '''Basic exception class for RPLidar'''


def _b2i(byte):
	return byte if int(sys.version[0]) == 3 else ord(byte)

def _process_scan(raw):
	new_scan = bool(_b2i(raw[0]) & 0b1)
   	inversed_new_scan = bool((_b2i(raw[0]) >> 1) & 0b1)
   	quality = _b2i(raw[0]) >> 2
	if new_scan == inversed_new_scan:
   		raise RPLidarException('New scan flags mismatch')
	check_bit = _b2i(raw[1]) & 0b1
	if check_bit != 1:
	        raise RPLidarException('Check bit not equal to 1')
	angle = ((_b2i(raw[1]) >> 1) + (_b2i(raw[2]) << 7)) / 64.
	distance = (_b2i(raw[3]) + (_b2i(raw[4]) << 8)) / 4.
	return new_scan, quality, angle, distance

class Lidar_Lite():
	_serial_port = None  #: serial port connection
	port = '/dev/ttyUSB0'  #: Serial port name, e.g. /dev/ttyUSB0 //wrote the port here
	timeout = 1  #: Serial port timeout
	motor = False  #: Is motor running?
	baudrate = 256000  #: Baudrate for serial port

	def __init__(self, port = '/dev/ttyUSB0', baudrate=256000, timeout=1, logger=None):
		self._serial_port = None
        	self.port = port
        	print("port is:",self.port)#added to check if port is assigned or not
        	self.baudrate = baudrate
        	self.timeout = timeout
        	self.motor_running = None
        	if logger is None:
        		logger = logging.getLogger('rplidar')
        	self.logger = logger
        	print("came here!")
        	self.connect()
        	self.start_motor()

	def connect(self):
	        if self._serial_port is not None:
			self.disconnect()
        	try:
            		self._serial_port = serial.Serial(self.port, self.baudrate,parity=serial.PARITY_NONE, stopbits=serial.STOPBITS_ONE,timeout=self.timeout)
            		print("checking serial por:",self._serial_port)    #added a check here too
        	except serial.SerialException as err:
            		raise RPLidarException('Failed to connect to the sensor ''due to: %s' % err)

	def disconnect(self):
        
        	if self._serial_port is None:
                	return
        	self._serial_port.close()

    	def set_pwm(self, pwm):
        	assert(0 <= pwm <= MAX_MOTOR_PWM)
        	payload = struct.pack("<H", pwm)
        	self._send_payload_cmd(SET_PWM_BYTE, payload)

    	def start_motor(self):

        	self.logger.info('Starting motor')
        # For A1
        	self._serial_port.setDTR(False)

        # For A2
        	self.set_pwm(DEFAULT_MOTOR_PWM)
        	self.motor_running = True

    	def stop_motor(self):
        
        	self.logger.info('Stoping motor')
        # For A2
        	self.set_pwm(0)
        	time.sleep(.001)
        # For A1
        	self._serial_port.setDTR(True)
        	self.motor_running = False

	def _send_payload_cmd(self, cmd, payload):
        
        	size = struct.pack('B', len(payload))
        	req = SYNC_BYTE + cmd + size + payload
        	checksum = 0
        	for v in struct.unpack('B'*len(req), req):
        		checksum ^= v
        	req += struct.pack('B', checksum)
        	self._serial_port.write(req)
        	self.logger.debug('Command sent: %s' % req)

    	def _send_cmd(self, cmd):
        
        	req = SYNC_BYTE + cmd
        	self._serial_port.write(req)
        	self.logger.debug('Command sent: %s' % req)

    	def _read_descriptor(self):
        
        	descriptor = self._serial_port.read(DESCRIPTOR_LEN) #all problem is here.
        #added this myself
        	print(len(descriptor)) #changed
        	self.logger.debug('Recieved descriptor: %s', descriptor)
        	if len(descriptor) !=DESCRIPTOR_LEN:
            		raise RPLidarException('Descriptor length mismatch')
        	elif not descriptor.startswith(SYNC_BYTE + SYNC_BYTE2):
            		raise RPLidarException('Incorrect descriptor starting bytes')
        	is_single = _b2i(descriptor[-2]) == 0
        	return _b2i(descriptor[2]), is_single, _b2i(descriptor[-1])

    	def _read_response(self, dsize):
        
        	self.logger.debug('Trying to read response: %d bytes', dsize)
        	data = self._serial_port.read(dsize)
        	self.logger.debug('Recieved data: %s', data)
		#print("dsize=%d",dsize,"/","len(data)=%d",len(data)) #changed
        	if len(data) != dsize:
            		raise RPLidarException('Wrong body size')
        	return data

    	def get_info(self):
        	print("got info")
        	self._send_cmd(GET_INFO_BYTE)
        	dsize, is_single, dtype = self._read_descriptor()
        	if dsize != INFO_LEN:
            		raise RPLidarException('Wrong get_info reply length')
        	if not is_single:
            		raise RPLidarException('Not a single response mode')
        	if dtype != INFO_TYPE:
            		raise RPLidarException('Wrong response data type')
        	raw = self._read_response(dsize)
        	serialnumber = codecs.encode(raw[4:], 'hex').upper()
        	serialnumber = codecs.decode(serialnumber, 'ascii')
        	data = {'model': _b2i(raw[0]),'firmware': (_b2i(raw[2]), _b2i(raw[1])),'hardware': _b2i(raw[3]),'serialnumber': serialnumber,}
        	return data

	def get_health(self):
		print("got health")
        	self._send_cmd(GET_HEALTH_BYTE)
        	dsize, is_single, dtype = self._read_descriptor()
        	if dsize != HEALTH_LEN:
            		raise RPLidarException('Wrong get_info reply length')
        	if not is_single:
            		raise RPLidarException('Not a single response mode')
        	if dtype != HEALTH_TYPE:
            		raise RPLidarException('Wrong response data type')
        	raw = self._read_response(dsize)
        	status = _HEALTH_STATUSES[_b2i(raw[0])]
        	error_code = (_b2i(raw[1]) << 8) + _b2i(raw[2])
        	return status, error_code

    	def clear_input(self):
        
        	self._serial_port.read_all()

    	def stop(self):
        	self.logger.info('Stoping scanning')
        	self._send_cmd(STOP_BYTE)
        	time.sleep(.001)
        	self.clear_input()

    	def reset(self):
        	self.logger.info('Reseting the sensor')
        	self._send_cmd(RESET_BYTE)
        	time.sleep(.002)


    	def iter_measurments(self, max_buf_meas=500):
        	self.start_motor()
        	status, error_code = self.get_health()
        	self.logger.debug('Health status: %s [%d]', status, error_code)
        	if status == _HEALTH_STATUSES[2]:
            		self.logger.warning('Trying to reset sensor due to the error. ''Error code: %d', error_code)
            		self.reset()
            		status, error_code = self.get_health()
            		if status == _HEALTH_STATUSES[2]:
                		raise RPLidarException('RPLidar hardware failure. ''Error code: %d' % error_code)
        	elif status == _HEALTH_STATUSES[1]:
            		self.logger.warning('Warning sensor status detected! ''Error code: %d', error_code)
        	cmd = SCAN_BYTE
        	self._send_cmd(cmd)
        	dsize, is_single, dtype = self._read_descriptor()
        	if dsize != 5:
            		raise RPLidarException('Wrong get_info reply length')
        	if is_single:
            		raise RPLidarException('Not a multiple response mode')
        	if dtype != SCAN_TYPE:
            		raise RPLidarException('Wrong response data type')
        	while True:
            		raw = self._read_response(dsize)
            		self.logger.debug('Recieved scan response: %s' % raw)
            		if max_buf_meas:
                		data_in_buf = self._serial_port.in_waiting
                		if data_in_buf > max_buf_meas*dsize:
                    			self.logger.warning('Too many measurments in the input buffer: %d/%d. ''Clearing buffer...',data_in_buf//dsize, max_buf_meas)
                    			self._serial_port.read(data_in_buf//dsize*dsize)
            		yield _process_scan(raw)
	
	#global sweepData
	'''
	def getDistance(self, max_buf_meas=500, min_len=5):
		print("in getDistance")
        	
		a={}
		
        	iterator = self.iter_measurments(max_buf_meas)
        	for new_scan, quality, angle, distance in iterator:
            		if new_scan:
                		if len(a) > min_len:
					#print(sweepData)
                    			return a
                		
            		if quality > 0 and distance > 0:
                		a[angle]=distance
				#print(sweepData)
              			return a
	'''
	global sweepData
	def iter_scans(self, max_buf_meas=5000, min_len=0):
        	sweepData = {}
        	iterator = self.iter_measurments(max_buf_meas)
        	for new_scan, quality, angle, distance in iterator:
            		if new_scan:
                		if len(sweepData) > min_len:
                    			yield sweepData
                		sweepData = {}
            		#if quality > 0 and distance > 0:
                	sweepData[angle]=distance
			#print("data:-")                	
			#print(sweepData)


# ======================================================================================================
# generating training data set. The output of this module is given to training neural network module.

def generateTrainData():
	train_data = [];
	train_labels = [];

	#set the below parameters during changes in angles,safe distance etc.
	global threshold,safeDistance,leftAngle,rightAngle,normalizeAngle,normalizeDistance,points,totalDistance;

	dataSetSize = 20000 # bunches of data taken  to train for each condition

	#forward contidion sets..i.e index 0
	result = [1,0,0,0]
	result = np.reshape(result,(4,1));
	for i in range(0,dataSetSize):
		distance = np.random.randint(safeDistance,totalDistance,points)
		distance = distance / normalizeDistance; # distance upto 1000
		print "Distance:"
		print distance;
		angle = np.array([k for k in range(leftAngle,rightAngle+1,1)]) # angle leftAngle(-) to rightAngle(+)
		angle = angle / normalizeAngle;	
		print "Angle:"
		print angle;
		sweepPoints = np.array([angle,distance]);#will construct 2x100 matrix
		sweepPoints = sweepPoints.T #transpose it to make 100x2;
		train_data.append(sweepPoints);
		train_labels.append(result) # forward condition for above points

	#stop contidion sets...i.e index 3
	result = [0,0,0,1]
	result = np.reshape(result,(4,1));
	for i in range(0,dataSetSize):
		distance = np.random.randint(0,threshold,points);
		distance = distance / normalizeDistance;
		angle = np.array([k for k in range(leftAngle,rightAngle+1,1)]) # angle leftAngle(-) to rightAngle(+)
		angle = angle / normalizeAngle;
		sweepPoints = np.array([angle,distance]);
		sweepPoints = sweepPoints.T;
		train_data.append(sweepPoints);
		train_labels.append(result) # append the results for the corresponding record

	#condition for right..i.e..index 2. The left side of the UAV is taken as negative angle, and right side of FOV is positive angle
	result = [0,0,1,0]
	result = np.reshape(result,(4,1));
	for i in range(0,dataSetSize):
		unsafePoints = np.random.randint(0,threshold,rightAngle)
		safePoints = np.random.randint(safeDistance,totalDistance,rightAngle+1)
		distance = np.append(unsafePoints,safePoints);
		angle = np.array([k for k in range(leftAngle,rightAngle+1,1)]) # all angles in sequence
		distance = distance / normalizeDistance;
		angle = angle / normalizeAngle;
		sweepPoints = np.array([angle,distance]);
		sweepPoints = sweepPoints.T;
		train_data.append(sweepPoints);
		train_labels.append(result)

	#condition for left..i.e index 1. The right side of UAV is not safe, distances with positive angles are no safe.
	result = [0,1,0,0]
	result = np.reshape(result,(4,1));
	for i in range(0,dataSetSize):
		unsafePoints = np.random.randint(0,threshold,rightAngle+1);
		safePoints = np.random.randint(safeDistance,totalDistance,rightAngle);
		distance = np.append(safePoints,unsafePoints);
		angle = np.array([k for k in range(leftAngle,rightAngle+1,1)]);
		distance = distance / normalizeDistance;
		angle = angle / normalizeAngle;
		sweepPoints = np.array([angle,distance]); #will construct 2 x no. of points  matrix
		sweepPoints = sweepPoints.T; #transpose it to make no. of points x 2;
		train_data.append(sweepPoints);
		train_labels.append(result)

	train_input = [np.reshape(data,(2*points,1)) for data in train_data]
	#print train_input[0].shape;
	#print train_input[0];
	#print type(train_label[0])
	training_data = zip(train_input, train_labels)
	return training_data;

# ======================================================================================================
# This module trains the neural network. Training data sets are needed to train it. The trained model is saved in a numpy byte file.


def trainNeuralNetwork():
	global points;
	training_data = generateTrainData();	
	print "Training the network"
	net = network.Network([2*points, 80, 4])
	net.SGD(training_data, 10, 10, 3.0)
	saveNeuralNet(net);
	testNeuralNetwork(net);


# ======================================================================================================
#Testing of the neural network with random generated points.

def testNeuralNetwork(net):
	global threshold,safeDistance,totalDistance,points,leftAngle,rightAngle,normalizeAngle,normalizeDistance;
	class_names = ['FORWARD', 'LEFT', 'RIGHT','STOP'];
	angle = np.array([k for k in range(leftAngle,rightAngle+1,1)])
	distance = np.random.randint(safeDistance,totalDistance,points);
	angle = angle / normalizeAngle;
	distance = distance / normalizeDistance;
	sweepPoints = np.array([angle,distance]);
	sweepPoints = sweepPoints.T;
	data = np.reshape(sweepPoints,(2*points,1));
	res = net.feedforward(data)
	print "Predictions:"
	print res
	print class_names[np.argmax(res)];

# ======================================================================================================
# read neural networks matrix values from files 'weights.npy' and 'biases.npy' and test it.

def testSavedNetwork():
	global points;
	weights = np.load("weights.npy");
	biases = np.load("biases.npy");
	net = network.Network([2*points,80,4]);
	net.weights = list(weights);
	net.biases = list(biases);
	net.weights = [net.weights[i][0] for i in range(0,len(net.weights))]; # saving in file changes the structure, so we bring it back to form.
	net.biases = [net.biases[i][0] for i in range(0,len(net.biases))];
	testNeuralNetwork(net);


# ======================================================================================================
# This program generates random distance using numpy random module. This replaces lidars getDistance method. This thread is used only for testing. 
#thread.start_new_thread(sweepLidarRandomDistance, ("Lidar Sweep Thread generating random distance",)); "sytax to start the below thread"
'''

def sweepLidarRandomDistance():
	angle = -90;
	flag = True;
	global sweepData,threshold,safeDistance,leftAngle,rightAngle,normalizeAngle,normalizeDistance,points,totalDistance;
	while True:
		if flag:
                        for angle in range(leftAngle,rightAngle+1,1):
                                sweepData[angle] = int(np.random.randint(0,threshold,1));
                                time.sleep(0.01);
                        flag = False;
                else:
                        for angle in range(rightAngle,leftAngle-1,-1):
                                sweepData[angle] = int(np.random.randint(0,threshold,1));
                                time.sleep(0.01);
                                flag = True;
	print(sweepData)
'''
