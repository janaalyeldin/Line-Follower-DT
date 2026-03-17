#!/usr/bin/env python3
from __future__ import print_function
import struct
import sys
import argparse
import math

PythonGateways = 'pythonGateways/'
sys.path.append(PythonGateways)

import VsiCommonPythonApi as vsiCommonPythonApi
import VsiCanPythonGateway as vsiCanPythonGateway


class MySignals:
	def __init__(self):
		# Inputs
		self.v_left = 0
		self.v_right = 0

		# Outputs
		self.x_pos = 0
		self.y_pos = 0
		self.theta = 0
		self.lateral_error = 0
		self.heading_error = 0
		self.x_ref = 0
		self.y_ref = 0




# Start of user custom code region. Please apply edits only within these regions:  Global Variables & Definitions
import os
# Navigate up to src/ where lineRobot/ and PIDController/ folders live
src_dir = os.path.abspath(os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', '..', '..'))
sys.path.insert(0, src_dir)
from lineRobot.robot_kinematics import RobotState, StraightPath, kinematics_step, compute_errors
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
		self.mySignals = MySignals()

		# Start of user custom code region. Please apply edits only within these regions:  Constructor
# Initialize path and robot state
		# Spawn robot with slight offset to demonstrate PID correction
		self.path  = StraightPath(x_end=10.0, y_end=0.0)
		self.state = RobotState(x=0.0, y=0.8, theta=0.3)
 
		# Compute and publish initial errors before first controller step
		lat_err, head_err, x_ref, y_ref = compute_errors(self.state, self.path)
		self.mySignals.x_pos         = self.state.x
		self.mySignals.y_pos         = self.state.y
		self.mySignals.theta         = self.state.theta
		self.mySignals.lateral_error = lat_err
		self.mySignals.heading_error = head_err
		self.mySignals.x_ref         = x_ref
		self.mySignals.y_ref         = y_ref
		# End of user custom code region. Please don't edit beyond this point.



	def mainThread(self):
		dSession = vsiCommonPythonApi.connectToServer(self.localHost, self.domain, self.portNum, self.componentId)
		vsiCanPythonGateway.initialize(dSession, self.componentId)
		try:
			vsiCommonPythonApi.waitForReset()

			# Start of user custom code region. Please apply edits only within these regions:  After Reset

			# End of user custom code region. Please don't edit beyond this point.
			self.updateInternalVariables()

			if(vsiCommonPythonApi.isStopRequested()):
				raise Exception("stopRequested")
			nextExpectedTime = vsiCommonPythonApi.getSimulationTimeInNs()
			while(vsiCommonPythonApi.getSimulationTimeInNs() < self.totalSimulationTime):

				# Start of user custom code region. Please apply edits only within these regions:  Inside the while loop
# Step 1: Advance robot state using wheel commands from controller
				self.state = kinematics_step(
					self.state,
					self.mySignals.v_left,
					self.mySignals.v_right
				)
 
				# Step 2: Compute new errors against path
				lat_err, head_err, x_ref, y_ref = compute_errors(self.state, self.path)
 
				# Step 3: Update output signals — VSI will send these to controller + plotter
				self.mySignals.x_pos         = self.state.x
				self.mySignals.y_pos         = self.state.y
				self.mySignals.theta         = self.state.theta
				self.mySignals.lateral_error = lat_err
				self.mySignals.heading_error = head_err
				self.mySignals.x_ref         = x_ref
				self.mySignals.y_ref         = y_ref
				# End of user custom code region. Please don't edit beyond this point.

				self.updateInternalVariables()

				if(vsiCommonPythonApi.isStopRequested()):
					raise Exception("stopRequested")

				signalNumBytes = 8
				receivedData = vsiCanPythonGateway.recvVariableFromCanPacket(signalNumBytes, 0, 64, 10)
				self.mySignals.v_left, receivedData = self.unpackBytes('d', receivedData, self.mySignals.v_left)

				signalNumBytes = 8
				receivedData = vsiCanPythonGateway.recvVariableFromCanPacket(signalNumBytes, 0, 64, 11)
				self.mySignals.v_right, receivedData = self.unpackBytes('d', receivedData, self.mySignals.v_right)

				# Start of user custom code region. Please apply edits only within these regions:  Before sending the packet

				# End of user custom code region. Please don't edit beyond this point.

				vsiCanPythonGateway.setCanId(12)
				vsiCanPythonGateway.setCanPayloadBits(self.packBytes('d', self.mySignals.lateral_error), 0, 64)
				vsiCanPythonGateway.setDataLengthInBits(64)
				vsiCanPythonGateway.sendCanPacket()

				vsiCanPythonGateway.setCanId(13)
				vsiCanPythonGateway.setCanPayloadBits(self.packBytes('d', self.mySignals.heading_error), 0, 64)
				vsiCanPythonGateway.setDataLengthInBits(64)
				vsiCanPythonGateway.sendCanPacket()

				vsiCanPythonGateway.setCanId(14)
				vsiCanPythonGateway.setCanPayloadBits(self.packBytes('d', self.mySignals.x_pos), 0, 64)
				vsiCanPythonGateway.setDataLengthInBits(64)
				vsiCanPythonGateway.sendCanPacket()

				vsiCanPythonGateway.setCanId(15)
				vsiCanPythonGateway.setCanPayloadBits(self.packBytes('d', self.mySignals.y_pos), 0, 64)
				vsiCanPythonGateway.setDataLengthInBits(64)
				vsiCanPythonGateway.sendCanPacket()

				vsiCanPythonGateway.setCanId(16)
				vsiCanPythonGateway.setCanPayloadBits(self.packBytes('d', self.mySignals.theta), 0, 64)
				vsiCanPythonGateway.setDataLengthInBits(64)
				vsiCanPythonGateway.sendCanPacket()

				vsiCanPythonGateway.setCanId(17)
				vsiCanPythonGateway.setCanPayloadBits(self.packBytes('d', self.mySignals.x_ref), 0, 64)
				vsiCanPythonGateway.setDataLengthInBits(64)
				vsiCanPythonGateway.sendCanPacket()

				vsiCanPythonGateway.setCanId(18)
				vsiCanPythonGateway.setCanPayloadBits(self.packBytes('d', self.mySignals.y_ref), 0, 64)
				vsiCanPythonGateway.setDataLengthInBits(64)
				vsiCanPythonGateway.sendCanPacket()

				# Start of user custom code region. Please apply edits only within these regions:  After sending the packet

				# End of user custom code region. Please don't edit beyond this point.

				print("\n+=simulator+=")
				print("  VSI time:", end = " ")
				print(vsiCommonPythonApi.getSimulationTimeInNs(), end = " ")
				print("ns")
				print("  Inputs:")
				print("\tv_left =", end = " ")
				print(self.mySignals.v_left)
				print("\tv_right =", end = " ")
				print(self.mySignals.v_right)
				print("  Outputs:")
				print("\tx_pos =", end = " ")
				print(self.mySignals.x_pos)
				print("\ty_pos =", end = " ")
				print(self.mySignals.y_pos)
				print("\ttheta =", end = " ")
				print(self.mySignals.theta)
				print("\tlateral_error =", end = " ")
				print(self.mySignals.lateral_error)
				print("\theading_error =", end = " ")
				print(self.mySignals.heading_error)
				print("\tx_ref =", end = " ")
				print(self.mySignals.x_ref)
				print("\ty_ref =", end = " ")
				print(self.mySignals.y_ref)
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
