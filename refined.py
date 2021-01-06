#Date:-12/08/2020
# @ Autonomous Vehicles REsearch Lab 
# This code is the main driver code which loads the data and fits it in the neural network
#This code considers distance value fro 0 to 360 degrees

import network
import numpy as np
import thread
import time
from time import sleep
import copy
import operator



leftAngle = -60;
rightAngle = 60;
normalizeAngle = 60.0;
threshold = 10000 
safeDistance = threshold + 500
totalDistance = 10001 
normalizeDistance = 10000.0

global sweepData
sweepData = {};

# This segment generates random distance and fills the sweepData dictionary
# This segment can be activated in case of Lidar-Sensor failure

def sweepLidarRandomDistance():
	print("sweep lidar block")
	angle = -90;
	flag = True;
	c=10;	
	global sweepData,threshold,safeDistance,leftAngle,rightAngle,normalizeAngle,normalizeDistance,points,totalDistance;
	while c>0 :		
		if flag:
                        for angle in range(leftAngle,rightAngle+1,1):
                                sweepData[angle] = int(np.random.randint(0,threshold,1));
                                # sweepData[angle] = 5; # If you enable this line, 'STOP' decision will be taken by the network. 
                                time.sleep(0.01);
                        	flag = False;
                else:
                        for angle in range(rightAngle,leftAngle-1,-1):
                                sweepData[angle] = int(np.random.randint(0,threshold,1));
                                #sweepData[angle] = 7; # If you enable this line, 'STOP' decision will be taken by the network.
                                time.sleep(0.01);
                                flag = True;
		c=c-1
	print(sweepData)


# This segmenet checks for critical distance threat between the robot and the obstacles in the environment
 
def checkPoints():
	global sweepData,threshold,safeDistance,points,totalDistance;
	localSweepData = {};
	try:
                localSweepData = copy.deepcopy(sweepData);
                for angle in localSweepData:
                        if(localSweepData[angle] < threshold):
                                print "******************************"
                                print "Angle: ",angle;
                                print "Distance: ",localSweepData[angle];
                                return True;
        except RuntimeError:
                print "Dictionary Error..."
        
	return False;

# weights and biases (dependency .npy files) have to placed in same folder.(13/02/2020)
def saveNeuralNet(net):
	weights = np.reshape(net.weights,(len(net.weights),1))
	biases = np.reshape(net.biases,(len(net.biases),1))
	np.save("weights.npy",weights);
	np.save("biases.npy",biases);


def callNeuralNetwork(): 
	#commented the arduino related part(13/02/2020)
	print("block entry!!")
	global sweepData,threshold,safeDistance,leftAngle,rightAngle,normalizeAngle,normalizeDistance,points,totalDistance;
        class_names = ['FORWARD', 'LEFT', 'RIGHT','STOP'];
        weights = np.load("weights.npy");
	biases = np.load("biases.npy");
	net = network.Network([2*points,80,4]);
	net.weights = list(weights);
	net.biases = list(biases);
	net.weights = [net.weights[i][0] for i in range(0,len(net.weights))]; # saving in file changes the structure, so we bring it back to form.
	net.biases = [net.biases[i][0] for i in range(0,len(net.biases))];
	localData = copy.deepcopy(sweepData); # The global sweep data copy is taken to provide guidance
        print "Sweep Data Dictionary:"
	#changed
	#locaData=sorted(localData.items(),key=operator.itemgetter(0))
	
        for x in sorted(localData.items()):
		print(x[0],":",x[1])

	angle =np.array(sorted(localData.iterkeys()));
        distance = np.array([localData[ang] for ang in angle ]);
        angle = angle / normalizeAngle;
	distance = distance / normalizeDistance;
	sweepPoints = np.array([angle,distance]);
	sweepPoints = sweepPoints.T;
	data = np.reshape(sweepPoints,(2*points,1));
	res = net.feedforward(data)
	print "Predictions:"
	print res
	print class_names[np.argmax(res)];
	guidance = np.argmax(res);
	if guidance == 0:
                global forwardstate
                #forwardstate = not forwardstate
                print "FORWARD!!"
                
        elif guidance == 1:
                global leftstate
                #leftstate = not leftstate
                print "LEFT!!"
                
        elif guidance == 2:
                global rightstate
                #rightstate = not rightstate
		print "RIGHT!!"
                
        else:
                global stopstate
                #stopstate = not stopstate
		print "STOP!!"
        



# Allow time for initial sweep..i.e the bot is started and takes a initial sweep to provide guidance
#time.sleep(5);

#called the random distance generator to generate the random distances PS:- should be replaced with real time data (13/02/2020)
sweepLidarRandomDistance()


# Keep checking for unsafe points. Call neural network module if unsafe point is found.
'''
i=0
while i<10:
	sweepData.update(lidar.getDistance())
	i+=1
'''
j=0

'''
for i, sweepData in enumerate(lidar.iter_scans()):
    print('%d: Got %d measurments' % (i, len(sweepData)))
    if i == 4:
        break
'''

points = len(sweepData); 
#print(points)

checkPoints()
callNeuralNetwork()


'''
print(sweepData)
while j<10:
	print("while loop")
	if checkPoints():
		print('inside if')
      		callNeuralNetwork()
	else:	
		print('not in if')
	j+=1	
'''


