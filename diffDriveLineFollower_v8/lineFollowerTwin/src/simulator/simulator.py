#!/usr/bin/env python3
from __future__ import print_function
import struct
import sys
import argparse
import math

PythonGateways = 'pythonGateways/'
sys.path.append(PythonGateways)

import VsiCommonPythonApi as vsiCommonPythonApi
import VsiTcpUdpPythonGateway as vsiEthernetPythonGateway


class MySignals:
	def __init__(self):
		# Inputs
		self.vR_cmd = 0
		self.vL_cmd = 0

		# Outputs
		self.x = 0
		self.y = 0
		self.theta = 0
		self.ref_x = 0
		self.ref_y = 0



srcMacAddress = [0x12, 0x34, 0x56, 0x78, 0x9A, 0x01]
controllerMacAddress = [0x12, 0x34, 0x56, 0x78, 0x9A, 0x02]
visualizerMacAddress = [0x12, 0x34, 0x56, 0x78, 0x9A, 0x03]
srcIpAddress = [192, 168, 1, 1]
controllerIpAddress = [192, 168, 1, 2]
visualizerIpAddress = [192, 168, 1, 3]

SimulatorSocketPortNumber0 = 8070
SimulatorSocketPortNumber1 = 8071

Controller0 = 0
Visualizer1 = 1


# Start of user custom code region. Please apply edits only within these regions:  Global Variables & Definitions

import random
import numpy as np

# Robot physical constants
WHEEL_BASE  = 0.3        # [m]     wheelbase of the differential-drive robot
MAX_V       = 2.0        # [m/s]   linear-velocity saturation guard
MAX_OMEGA   = 3.14159    # [rad/s] angular-rate saturation guard
MAX_V_WHEEL = 2.0        # [m/s]   per-wheel velocity saturation guard

# Reference-path builders
def _cubic_bezier(p0, p1, p2, p3, t):
	"""Evaluate a cubic Bezier curve at parameter t ∈ [0, 1]."""
	mt = 1.0 - t
	x = mt**3*p0[0] + 3*mt**2*t*p1[0] + 3*mt*t**2*p2[0] + t**3*p3[0]
	y = mt**3*p0[1] + 3*mt**2*t*p1[1] + 3*mt*t**2*p2[1] + t**3*p3[1]
	return (x, y)

def build_straight_path(total_length=30.0, resolution=0.01):
	"""Straight reference line along the x-axis, y = 0."""
	n = int(total_length / resolution) + 1
	y_path = np.linspace(0.0, total_length, n)
	return [(i, 0.0) for i in y_path.tolist()]

def build_curved_path(resolution=0.01):
	"""
	Piecewise cubic Bezier chain — C1-continuous S-curve.
	"""
	segments = [
		((0,0),  (0.67,0), (1.334,0), (2.0,0)),
		((2.0,0),  (5.0,0),    (8.0,2),   (10.0,2)),
		((10.0,2), (12.0,2),   (15.0,-2),  (18.0,-2)),
		((18.0,-2),(20.0,-2),  (23.0,0),   (25.0,0)),
		((25.0,0), (27.0,0),   (29.0,0),   (31.0,0)),
	]
	path = []
	n = int(1.0 / resolution)
	for seg in segments:
		for i in range(n + 1):
			path.append(_cubic_bezier(seg[0], seg[1], seg[2], seg[3], i / n))
	return path

def closest_point_on_path(path, x, y):
	"""Return (ref_x, ref_y, ref_theta) on *path* closest to robot position (x, y).
	ref_theta is the path tangent angle [rad] at that point."""
	best_idx  = 0
	best_dist = float('inf')
	for i, (px, py) in enumerate(path):
		d = (px - x)**2 + (py - y)**2
		if d < best_dist:
			best_dist = d
			best_idx  = i
	best_idx = best_idx + 1 if best_idx < len(path) - 1 else best_idx
	return path[best_idx][0], path[best_idx][1]

# End of user custom code region. Please don't edit beyond this point.
class Simulator:

	def __init__(self, args):
		self.componentId = 0
		self.localHost = args.server_url
		self.domain = args.domain
		self.portNum = 50101
        
		self.simulationStep = 0
		self.stopRequested = False
		self.totalSimulationTime = 0
        
		self.receivedNumberOfBytes = 0
		self.receivedPayload = []

		self.numberOfPorts = 2
		self.clientPortNum = [0] * self.numberOfPorts
		self.receivedDestPortNumber = 0
		self.receivedSrcPortNumber = 0
		self.expectedNumberOfBytes = 0
		self.mySignals = MySignals()

		# Start of user custom code region. Please apply edits only within these regions:  Constructor

		# Robot initial pose
		# self.mySignals.x     = random.uniform(0.0, 20.0)
		self.mySignals.x     = 0.0
		self.mySignals.y     = 0.0
		# self.mySignals.theta = random.uniform(-np.pi/3.0, np.pi/3.0)    # [rad] pointing along +x
		self.mySignals.theta = 0.0

		# Reference path
		# Swap to build_curved_path() when running Experiment E2.
		# self.ref_path = build_straight_path()
		self.ref_path = build_curved_path()

		# Sensor-noise std-dev
		# Set > 0 to enable Gaussian noise on the published pose (E3 / E4).
		self.noise_std = 0.01
		random.seed(0)

		# End of user custom code region. Please don't edit beyond this point.



	def mainThread(self):
		dSession = vsiCommonPythonApi.connectToServer(self.localHost, self.domain, self.portNum, self.componentId)
		vsiEthernetPythonGateway.initialize(dSession, self.componentId, bytes(srcMacAddress), bytes(srcIpAddress))
		try:
			vsiCommonPythonApi.waitForReset()

			# Start of user custom code region. Please apply edits only within these regions:  After Reset

			self.simulationStep      = vsiCommonPythonApi.getSimulationStep()
			self.totalSimulationTime = vsiCommonPythonApi.getTotalSimulationTime()

			# End of user custom code region. Please don't edit beyond this point.
			self.updateInternalVariables()

			if(vsiCommonPythonApi.isStopRequested()):
				raise Exception("stopRequested")
			self.establishTcpUdpConnection()
			nextExpectedTime = vsiCommonPythonApi.getSimulationTimeInNs()
			while(vsiCommonPythonApi.getSimulationTimeInNs() < self.totalSimulationTime):

				# Start of user custom code region. Please apply edits only within these regions:  Inside the while loop

				dt = self.simulationStep * 1e-9
 
				# 1. Read wheel velocities from PID controller
				vR = max(-MAX_V_WHEEL, min(MAX_V_WHEEL, self.mySignals.vR_cmd))
				vL = max(-MAX_V_WHEEL, min(MAX_V_WHEEL, self.mySignals.vL_cmd))
 
				# 2. Differential-drive kinematics
				# Derive linear and angular velocity from individual wheel speeds:
				#   v     = (vR + vL) / 2          [m/s]
				#   omega = (vR - vL) / WHEEL_BASE  [rad/s]
				v     = (vR + vL) / 2.0
				omega = (vR - vL) / WHEEL_BASE
 
				# Clamp derived quantities as a safety guard
				v     = max(-MAX_V,     min(MAX_V,     v))
				omega = max(-MAX_OMEGA, min(MAX_OMEGA, omega))
 
				# 3. Integrate unicycle pose
				#   ẋ = v·cos(θ)
				#   ẏ = v·sin(θ)
				#   θ̇ = ω
				self.mySignals.x     += v * np.cos(self.mySignals.theta) * dt
				self.mySignals.y     += v * np.sin(self.mySignals.theta) * dt
				self.mySignals.theta += omega * dt
 
				# Wrap heading to (−π, π]
				self.mySignals.theta = (self.mySignals.theta + np.pi) % (2.0 * np.pi) - np.pi
 
				# 4. Reference waypoint: closest point on path
				self.mySignals.ref_x, self.mySignals.ref_y = closest_point_on_path(self.ref_path, self.mySignals.x, self.mySignals.y)				

				# 5. Optional sensor noise (experiments E3 / E4)
				if self.noise_std > 0.0:
					self.mySignals.x     += random.gauss(0.0, self.noise_std)
					self.mySignals.y     += random.gauss(0.0, self.noise_std)
					self.mySignals.theta += random.gauss(0.0, self.noise_std * 0.1)

				# End of user custom code region. Please don't edit beyond this point.

				self.updateInternalVariables()

				if(vsiCommonPythonApi.isStopRequested()):
					raise Exception("stopRequested")

				if(vsiEthernetPythonGateway.isTerminationOnGoing()):
					print("Termination is on going")
					break

				if(vsiEthernetPythonGateway.isTerminated()):
					print("Application terminated")
					break

				receivedData = vsiEthernetPythonGateway.recvEthernetPacket(self.clientPortNum[Controller0])
				if(receivedData[3] != 0):
					self.decapsulateReceivedData(receivedData)

				receivedData = vsiEthernetPythonGateway.recvEthernetPacket(self.clientPortNum[Visualizer1])
				if(receivedData[3] != 0):
					self.decapsulateReceivedData(receivedData)

				# Start of user custom code region. Please apply edits only within these regions:  Before sending the packet

				# End of user custom code region. Please don't edit beyond this point.

				#Send ethernet packet to controller
				self.sendEthernetPacketTocontroller()

				#Send ethernet packet to visualizer
				self.sendEthernetPacketTovisualizer()

				# Start of user custom code region. Please apply edits only within these regions:  After sending the packet

				# End of user custom code region. Please don't edit beyond this point.

				print("\n+=simulator+=")
				print("  VSI time:", end = " ")
				print(vsiCommonPythonApi.getSimulationTimeInNs(), end = " ")
				print("ns")
				print("  Inputs:")
				print("\tvR_cmd =", end = " ")
				print(self.mySignals.vR_cmd)
				print("\tvL_cmd =", end = " ")
				print(self.mySignals.vL_cmd)
				print("  Outputs:")
				print("\tx =", end = " ")
				print(self.mySignals.x)
				print("\ty =", end = " ")
				print(self.mySignals.y)
				print("\ttheta =", end = " ")
				print(self.mySignals.theta)
				print("\tref_x =", end = " ")
				print(self.mySignals.ref_x)
				print("\tref_y =", end = " ")
				print(self.mySignals.ref_y)
				print("\n\n")

				self.updateInternalVariables()

				if(vsiCommonPythonApi.isStopRequested()):
					raise Exception("stopRequested")
				nextExpectedTime += self.simulationStep

				if(vsiCommonPythonApi.getSimulationTimeInNs() >= nextExpectedTime):
					continue

				if(nextExpectedTime > self.totalSimulationTime):
					remainingTime = self.totalSimulationTime - vsiCommonPythonApi.getSimulationTimeInNs()
					vsiCommonPythonApi.advanceSimulation(remainingTime)
					break

				vsiCommonPythonApi.advanceSimulation(nextExpectedTime - vsiCommonPythonApi.getSimulationTimeInNs())

			if(vsiCommonPythonApi.getSimulationTimeInNs() < self.totalSimulationTime):
				vsiEthernetPythonGateway.terminate()
		except Exception as e:
			if str(e) == "stopRequested":
				print("Terminate signal has been received from one of the VSI clients")
				# Advance time with a step that is equal to "simulationStep + 1" so that all other clients
				# receive the terminate packet before terminating this client
				vsiCommonPythonApi.advanceSimulation(self.simulationStep + 1)
			else:
				print(f"An error occurred: {str(e)}")
		except:
			# Advance time with a step that is equal to "simulationStep + 1" so that all other clients
			# receive the terminate packet before terminating this client
			vsiCommonPythonApi.advanceSimulation(self.simulationStep + 1)



	def establishTcpUdpConnection(self):
		if(self.clientPortNum[Controller0] == 0):
			self.clientPortNum[Controller0] = vsiEthernetPythonGateway.tcpListen(SimulatorSocketPortNumber0)

		if(self.clientPortNum[Visualizer1] == 0):
			self.clientPortNum[Visualizer1] = vsiEthernetPythonGateway.tcpListen(SimulatorSocketPortNumber1)

		if(self.clientPortNum[Visualizer1] == 0):
			print("Error: Failed to connect to port: Simulator on TCP port: ") 
			print(SimulatorSocketPortNumber0)
			exit()

		if(self.clientPortNum[Visualizer1] == 0):
			print("Error: Failed to connect to port: Simulator on TCP port: ") 
			print(SimulatorSocketPortNumber1)
			exit()



	def decapsulateReceivedData(self, receivedData):
		self.receivedDestPortNumber = receivedData[0]
		self.receivedSrcPortNumber = receivedData[1]
		self.receivedNumberOfBytes = receivedData[3]
		self.receivedPayload = [0] * (self.receivedNumberOfBytes)

		for i in range(self.receivedNumberOfBytes):
			self.receivedPayload[i] = receivedData[2][i]

		if(self.receivedSrcPortNumber == self.clientPortNum[Controller0]):
			print("Received packet from controller")
			receivedPayload = bytes(self.receivedPayload)
			self.mySignals.vR_cmd, receivedPayload = self.unpackBytes('d', receivedPayload)

			self.mySignals.vL_cmd, receivedPayload = self.unpackBytes('d', receivedPayload)


	def sendEthernetPacketTocontroller(self):
		bytesToSend = bytes()

		bytesToSend += self.packBytes('d', self.mySignals.x)

		bytesToSend += self.packBytes('d', self.mySignals.y)

		bytesToSend += self.packBytes('d', self.mySignals.theta)

		bytesToSend += self.packBytes('d', self.mySignals.ref_x)

		bytesToSend += self.packBytes('d', self.mySignals.ref_y)

		#Send ethernet packet to controller
		vsiEthernetPythonGateway.sendEthernetPacket(self.clientPortNum[Controller0], bytes(bytesToSend))

	def sendEthernetPacketTovisualizer(self):
		bytesToSend = bytes()

		bytesToSend += self.packBytes('d', self.mySignals.x)

		bytesToSend += self.packBytes('d', self.mySignals.y)

		bytesToSend += self.packBytes('d', self.mySignals.theta)

		bytesToSend += self.packBytes('d', self.mySignals.ref_x)

		bytesToSend += self.packBytes('d', self.mySignals.ref_y)

		#Send ethernet packet to visualizer
		vsiEthernetPythonGateway.sendEthernetPacket(self.clientPortNum[Visualizer1], bytes(bytesToSend))

		# Start of user custom code region. Please apply edits only within these regions:  Protocol's callback function

		# End of user custom code region. Please don't edit beyond this point.



	def packBytes(self, signalType, signal):
		if isinstance(signal, list):
			if signalType == 's':
				packedData = b''
				for str in signal:
					str += '\0'
					str = str.encode('utf-8')
					packedData += struct.pack(f'={len(str)}s', str)
				return packedData
			else:
				return struct.pack(f'={len(signal)}{signalType}', *signal)
		else:
			if signalType == 's':
				signal += '\0'
				signal = signal.encode('utf-8')
				return struct.pack(f'={len(signal)}s', signal)
			else:
				return struct.pack(f'={signalType}', signal)



	def unpackBytes(self, signalType, packedBytes, signal = ""):
		if isinstance(signal, list):
			if signalType == 's':
				unpackedStrings = [''] * len(signal)
				for i in range(len(signal)):
					nullCharacterIndex = packedBytes.find(b'\0')
					if nullCharacterIndex == -1:
						break
					unpackedString = struct.unpack(f'={nullCharacterIndex}s', packedBytes[:nullCharacterIndex])[0].decode('utf-8')
					unpackedStrings[i] = unpackedString
					packedBytes = packedBytes[nullCharacterIndex + 1:]
				return unpackedStrings, packedBytes
			else:
				unpackedVariable = struct.unpack(f'={len(signal)}{signalType}', packedBytes[:len(signal)*struct.calcsize(f'={signalType}')])
				packedBytes = packedBytes[len(unpackedVariable)*struct.calcsize(f'={signalType}'):]
				return list(unpackedVariable), packedBytes
		elif signalType == 's':
			nullCharacterIndex = packedBytes.find(b'\0')
			unpackedVariable = struct.unpack(f'={nullCharacterIndex}s', packedBytes[:nullCharacterIndex])[0].decode('utf-8')
			packedBytes = packedBytes[nullCharacterIndex + 1:]
			return unpackedVariable, packedBytes
		else:
			numBytes = 0
			if signalType in ['?', 'b', 'B']:
				numBytes = 1
			elif signalType in ['h', 'H']:
				numBytes = 2
			elif signalType in ['f', 'i', 'I', 'L', 'l']:
				numBytes = 4
			elif signalType in ['q', 'Q', 'd']:
				numBytes = 8
			else:
				raise Exception('received an invalid signal type in unpackBytes()')
			unpackedVariable = struct.unpack(f'={signalType}', packedBytes[0:numBytes])[0]
			packedBytes = packedBytes[numBytes:]
			return unpackedVariable, packedBytes

	def updateInternalVariables(self):
		self.totalSimulationTime = vsiCommonPythonApi.getTotalSimulationTime()
		self.stopRequested = vsiCommonPythonApi.isStopRequested()
		self.simulationStep = vsiCommonPythonApi.getSimulationStep()



def main():
	inputArgs = argparse.ArgumentParser(" ")
	inputArgs.add_argument('--domain', metavar='D', default='AF_UNIX', help='Socket domain for connection with the VSI TLM fabric server')
	inputArgs.add_argument('--server-url', metavar='CO', default='localhost', help='server URL of the VSI TLM Fabric Server')

	# Start of user custom code region. Please apply edits only within these regions:  Main method

	# End of user custom code region. Please don't edit beyond this point.

	args = inputArgs.parse_args()
                      
	simulator = Simulator(args)
	simulator.mainThread()



if __name__ == '__main__':
    main()
