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



srcMacAddress = [0x12, 0x34, 0x56, 0x78, 0x9A, 0x03]
simulatorMacAddress = [0x12, 0x34, 0x56, 0x78, 0x9A, 0x01]
srcIpAddress = [192, 168, 1, 3]
simulatorIpAddress = [192, 168, 1, 1]

SimulatorSocketPortNumber0 = 8071

Visualizer0 = 0


# Start of user custom code region. Please apply edits only within these regions:  Global Variables & Definitions

import matplotlib.pyplot as plt
import csv
import time
import numpy as np

# How many steps between plot redraws (1 = every step, higher = faster sim)
PLOT_INTERVAL = 10

# End of user custom code region. Please don't edit beyond this point.
class Visualizer:

	def __init__(self, args):
		self.componentId = 2
		self.localHost = args.server_url
		self.domain = args.domain
		self.portNum = 50103
        
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

		# Trajectory / reference-path data stores
		self.trajectory_x   = []
		self.trajectory_y   = []
		self.ref_path_x     = []
		self.ref_path_y     = []
		self.lateral_errors = []
		self.times_s        = []
		self._step_count    = 0
		
		self._plot_running = True

		# End of user custom code region. Please don't edit beyond this point.



	def mainThread(self):
		dSession = vsiCommonPythonApi.connectToServer(self.localHost, self.domain, self.portNum, self.componentId)
		vsiEthernetPythonGateway.initialize(dSession, self.componentId, bytes(srcMacAddress), bytes(srcIpAddress))
		try:
			vsiCommonPythonApi.waitForReset()

			# Start of user custom code region. Please apply edits only within these regions:  After Reset

			self.simulationStep      = vsiCommonPythonApi.getSimulationStep()
			self.totalSimulationTime = vsiCommonPythonApi.getTotalSimulationTime()

			# Live Matplotlib figure (two sub-plots)
			plt.ion()
			self.fig, (self.ax_traj, self.ax_err) = plt.subplots(2, 1, figsize=(10, 6))
			self.fig.suptitle("Differential Drive Line Follower / Real-Time Simulation Visualizer", fontsize=10)

			plt.tight_layout()

			self.ref_path_x.append(0.0)
			self.ref_path_y.append(0.0)

			# End of user custom code region. Please don't edit beyond this point.
			self.updateInternalVariables()

			if(vsiCommonPythonApi.isStopRequested()):
				raise Exception("stopRequested")
			self.establishTcpUdpConnection()
			nextExpectedTime = vsiCommonPythonApi.getSimulationTimeInNs()
			while(vsiCommonPythonApi.getSimulationTimeInNs() < self.totalSimulationTime):

				# Start of user custom code region. Please apply edits only within these regions:  Inside the while loop

				# 1. Compute lateral error
				lat_err = (self.mySignals.x - self.mySignals.ref_x) * np.sin(self.mySignals.theta) - (self.mySignals.y - self.mySignals.ref_y) * np.cos(self.mySignals.theta)

				# 2. Store data
				t_s  = vsiCommonPythonApi.getSimulationTimeInNs() * 1e-9

				if self.mySignals.x != 0.0 or self.mySignals.y != 0.0:
					self.trajectory_x.append(self.mySignals.x)
					self.trajectory_y.append(self.mySignals.y)
				self.ref_path_x.append(self.mySignals.ref_x)
				self.ref_path_y.append(self.mySignals.ref_y)
				self.lateral_errors.append(np.abs(lat_err))
				self.times_s.append(t_s)
				self._step_count += 1

				# 3. Refresh live plot every PLOT_INTERVAL steps
				if self._step_count % PLOT_INTERVAL == 0:
					# Trajectory sub-plot
					self.ax_traj.clear()
					self.ax_traj.set_title("Simulation Plot")
					self.ax_traj.set_xlabel("x [m]")
					self.ax_traj.set_ylabel("y [m]")
					self.ax_traj.grid(True, linestyle='--', alpha=0.5)
					self.ax_traj.plot(self.ref_path_x, self.ref_path_y, 'r--', lw=2, label='Path Ref.')
					self.ax_traj.plot(self.trajectory_x, self.trajectory_y, 'b-',  lw=2,   label='Trajectory')
					self.ax_traj.quiver(self.trajectory_x[-1], self.trajectory_y[-1], 0.05*np.cos(self.mySignals.theta), 0.05*np.sin(self.mySignals.theta), angles='xy', scale_units='xy', scale=1, color='green', width=0.005, headwidth=4, headlength=5, label=f'CurrentPos@t={t_s:.1f}s')
					self.ax_traj.plot(self.trajectory_x[0],  self.trajectory_y[0], 'ks', ms=7, label='Start')
					self.ax_traj.legend(fontsize=8, loc='upper left')
					l_traj, h_traj = self.ax_traj.get_ylim()
					bound_traj = max(abs(l_traj), abs(h_traj))
					self.ax_traj.set_ylim(ymin=-(bound_traj+0.2*bound_traj), ymax=(bound_traj+0.2*bound_traj))

					# Error sub-plot
					self.ax_err.clear()
					self.ax_err.set_title("Error Plot")
					self.ax_err.set_xlabel("Time [s]")
					self.ax_err.set_ylabel("abs(error) [m]")
					self.ax_err.grid(True, linestyle='--', alpha=0.5)
					self.ax_err.plot(self.times_s, self.lateral_errors, 'm-', lw=1.5, label='abs(error)')
					l_err, h_err = self.ax_err.get_ylim()
					bound_err = max(abs(l_err), abs(h_err))
					self.ax_err.set_ylim(ymin=-(bound_err), ymax=(bound_err+0.4*bound_err))

					n_e = len(self.lateral_errors)
					if n_e > 1:
						# --- Overshoot: mark the peak error so far ---
						peak_val = max(self.lateral_errors)
						peak_idx = self.lateral_errors.index(peak_val)
						peak_t   = self.times_s[peak_idx]
						self.ax_err.plot(peak_t, peak_val, 'rv', ms=8)
						self.ax_err.annotate(f'Overshoot\n{peak_val:.3f} m', xy=(peak_t, peak_val), xytext=(peak_t, peak_val + 0.015), fontsize=7, color='red')
 
						# --- Settling time: first index after which all errors < 0.05 m ---
						settle_thresh = 0.5*peak_val
						self.ax_err.axhline(y=settle_thresh, color='orange', linestyle=':', lw=1.2, label='Settling thresh')
						settling_t = None
						for i in range(n_e):
							if all(self.lateral_errors[j] < settle_thresh for j in range(i, n_e)):
									settling_t = self.times_s[i]
									break
						if settling_t is not None:
							self.ax_err.axvline(x=settling_t, color='green', linestyle='--', lw=1.4, label=f'Settling@t={settling_t:.2f} s')
							self.ax_err.annotate(f'Settled\n{settling_t:.2f} s', xy=(settling_t, 0.05), fontsize=7, color='green')
 
						# --- Steady-state error: mean of last 20% of samples so far ---
						ss_start = int(n_e * 0.80)
						if ss_start < n_e:
							ss_err = (sum(self.lateral_errors[ss_start:]) / max(1, n_e - ss_start))
							ss_t0  = self.times_s[ss_start]
							ss_t1  = self.times_s[-1]
							self.ax_err.hlines(ss_err, ss_t0, ss_t1, colors='blue', linestyles='-.', lw=1.4, label=f'SS error = {ss_err:.4f} m')
							self.ax_err.annotate(f'SS err\n{ss_err:.4f} m', xy=(ss_t1, ss_err), xytext=(ss_t1, ss_err + 0.008), fontsize=7, color='blue')	

					self.ax_err.legend(fontsize=8, loc='lower left')

					plt.tight_layout()
					plt.pause(0.01)

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

				# Start of user custom code region. Please apply edits only within these regions:  After sending the packet

				# End of user custom code region. Please don't edit beyond this point.

				print("\n+=visualizer+=")
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
		if(self.clientPortNum[Visualizer0] == 0):
			self.clientPortNum[Visualizer0] = vsiEthernetPythonGateway.tcpConnect(bytes(simulatorIpAddress), SimulatorSocketPortNumber0)

		if(self.clientPortNum[Visualizer0] == 0):
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
                      
	visualizer = Visualizer(args)
	visualizer.mainThread()



if __name__ == '__main__':
    main()
