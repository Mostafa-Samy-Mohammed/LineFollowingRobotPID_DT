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
		self.x = 0
		self.y = 0
		self.theta = 0
		self.ref_x = 0
		self.ref_y = 0

		# Outputs
		self.vR_cmd = 0
		self.vL_cmd = 0



srcMacAddress = [0x12, 0x34, 0x56, 0x78, 0x9A, 0x02]
simulatorMacAddress = [0x12, 0x34, 0x56, 0x78, 0x9A, 0x01]
srcIpAddress = [192, 168, 1, 2]
simulatorIpAddress = [192, 168, 1, 1]

SimulatorSocketPortNumber0 = 8070

Controller0 = 0


# Start of user custom code region. Please apply edits only within these regions:  Global Variables & Definitions

import numpy as np

WHEEL_BASE = 0.3        # L  — distance between wheels [m]
V_NOMINAL  = 1.5       # constant forward speed [m/s]
V_MAX_WHEEL = 2.0       # wheel velocity saturation [m/s]

# --- PID gains ---
# Lateral error  → drives angular correction
KP = 6.1		#5.2	2.5		3.8		6.1		6.1
KI = 0.12		#0.08	0.02	0.14	0.01	0.12
KD = 0.05		#0.12	0.08	0.02	0.05	0.05

# End of user custom code region. Please don't edit beyond this point.
class Controller:

	def __init__(self, args):
		self.componentId = 1
		self.localHost = args.server_url
		self.domain = args.domain
		self.portNum = 50102
        
		self.simulationStep = 0
		self.stopRequested = False
		self.totalSimulationTime = 0
        
		self.receivedNumberOfBytes = 0
		self.receivedPayload = []

		self.numberOfPorts = 1
		self.clientPortNum = [0] * self.numberOfPorts
		self.receivedDestPortNumber = 0
		self.receivedSrcPortNumber = 0
		self.expectedNumberOfBytes = 0
		self.mySignals = MySignals()

		# Start of user custom code region. Please apply edits only within these regions:  Constructor

		self.prevRef_x = 0.0
		self.prevRef_y = 0.0

		# --- PID state: lateral ---
		self.int_lat  = 0.0   # integral accumulator
		self.prev_lat = 0.0   # previous error for derivative

		# End of user custom code region. Please don't edit beyond this point.



	def mainThread(self):
		dSession = vsiCommonPythonApi.connectToServer(self.localHost, self.domain, self.portNum, self.componentId)
		vsiEthernetPythonGateway.initialize(dSession, self.componentId, bytes(srcMacAddress), bytes(srcIpAddress))
		try:
			vsiCommonPythonApi.waitForReset()

			# Start of user custom code region. Please apply edits only within these regions:  After Reset

			self.simulationStep = vsiCommonPythonApi.getSimulationStep()
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

				# -----------------------------------------------------------------
				# Error computation
				# -----------------------------------------------------------------
				# Lateral error: signed perpendicular distance from reference line
				ref_theta = np.atan2(self.mySignals.ref_y - self.prevRef_y, self.mySignals.ref_x - self.prevRef_x)
				e_lat = (self.mySignals.x - self.mySignals.ref_x) * np.sin(ref_theta) - (self.mySignals.y - self.mySignals.ref_y) * np.cos(ref_theta) - (self.mySignals.theta - ref_theta)

				# -----------------------------------------------------------------
				# PID — lateral
				# -----------------------------------------------------------------
				self.int_lat  += e_lat * dt
				d_lat          = (e_lat - self.prev_lat) / dt if dt > 0 else 0.0
				self.prev_lat  = e_lat
				omega_lat      = (KP * e_lat + KI * self.int_lat + KD * d_lat)

				# -----------------------------------------------------------------
				# Combined angular velocity command
				# -----------------------------------------------------------------
				omega = omega_lat

				# -----------------------------------------------------------------
				# Differential drive inverse kinematics
				# v = V_NOMINAL (constant forward speed)
				# vR = v + omega * L/2
				# vL = v - omega * L/2
				# -----------------------------------------------------------------
				v  = V_NOMINAL
				vR = v + omega * (WHEEL_BASE / 2.0)
				vL = v - omega * (WHEEL_BASE / 2.0)

				# Saturate wheel velocities
				vR = max(-V_MAX_WHEEL, min(V_MAX_WHEEL, vR))
				vL = max(-V_MAX_WHEEL, min(V_MAX_WHEEL, vL))

				self.mySignals.vR_cmd = vR
				self.mySignals.vL_cmd = vL

				self.prevRef_x = self.mySignals.ref_x
				self.prevRef_y = self.mySignals.ref_y

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

				receivedData = vsiEthernetPythonGateway.recvEthernetPacket(SimulatorSocketPortNumber0)
				if(receivedData[3] != 0):
					self.decapsulateReceivedData(receivedData)

				# Start of user custom code region. Please apply edits only within these regions:  Before sending the packet

				# End of user custom code region. Please don't edit beyond this point.

				#Send ethernet packet to simulator
				self.sendEthernetPacketTosimulator()

				# Start of user custom code region. Please apply edits only within these regions:  After sending the packet

				# End of user custom code region. Please don't edit beyond this point.

				print("\n+=controller+=")
				print("  VSI time:", end = " ")
				print(vsiCommonPythonApi.getSimulationTimeInNs(), end = " ")
				print("ns")
				print("  Inputs:")
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
				print("  Outputs:")
				print("\tvR_cmd =", end = " ")
				print(self.mySignals.vR_cmd)
				print("\tvL_cmd =", end = " ")
				print(self.mySignals.vL_cmd)
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
			self.clientPortNum[Controller0] = vsiEthernetPythonGateway.tcpConnect(bytes(simulatorIpAddress), SimulatorSocketPortNumber0)

		if(self.clientPortNum[Controller0] == 0):
			print("Error: Failed to connect to port: Simulator on TCP port: ") 
			print(SimulatorSocketPortNumber0)
			exit()



	def decapsulateReceivedData(self, receivedData):
		self.receivedDestPortNumber = receivedData[0]
		self.receivedSrcPortNumber = receivedData[1]
		self.receivedNumberOfBytes = receivedData[3]
		self.receivedPayload = [0] * (self.receivedNumberOfBytes)

		for i in range(self.receivedNumberOfBytes):
			self.receivedPayload[i] = receivedData[2][i]

		if(self.receivedSrcPortNumber == SimulatorSocketPortNumber0):
			print("Received packet from simulator")
			receivedPayload = bytes(self.receivedPayload)
			self.mySignals.x, receivedPayload = self.unpackBytes('d', receivedPayload)

			self.mySignals.y, receivedPayload = self.unpackBytes('d', receivedPayload)

			self.mySignals.theta, receivedPayload = self.unpackBytes('d', receivedPayload)

			self.mySignals.ref_x, receivedPayload = self.unpackBytes('d', receivedPayload)

			self.mySignals.ref_y, receivedPayload = self.unpackBytes('d', receivedPayload)


	def sendEthernetPacketTosimulator(self):
		bytesToSend = bytes()

		bytesToSend += self.packBytes('d', self.mySignals.vR_cmd)

		bytesToSend += self.packBytes('d', self.mySignals.vL_cmd)

		#Send ethernet packet to simulator
		vsiEthernetPythonGateway.sendEthernetPacket(SimulatorSocketPortNumber0, bytes(bytesToSend))

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
                      
	controller = Controller(args)
	controller.mainThread()



if __name__ == '__main__':
    main()
