import numpy as numpy
from abr_control.interfaces import PROSTHETIC_HAND
from abr_control.controllers import NEURAL_PD	
import time
import numpy as np
import matplotlib.pyplot as plt
from datetime import datetime
import csv

test_dur = 1.5*60

#TODO: init adaption part
interface = PROSTHETIC_HAND()
interface.connect()

#Open-close 2 times a sec; doesn't work well.
sine_func = (lambda t: abs(180*np.sin(0.5*np.pi*t)))

def normalize_val(val):
	#Expected input: 4D
	#Angle ranges from approx. 0 to 180 degrees.
	#dAngle values are sometimes neg. Range from -180 to 180 
	#Clip btwn -180 to 180
	val = np.clip(val,a_min= -180,a_max= 180) 
	return val/180

#step function; 1 times a sec. 
def step_func(t):
	#open-close every second. 
	#Alternate btwn 0 and 180 every half-second. 
	cur_second = np.fix(t)
	millis = t-cur_second
	if millis < 0.500: #close open. 
		return 180
	else:
		return 0

#Period = 2 sec. Give time for PID to adjust. 
def step_func_2(t):
	period = 3
	cur_second = np.fix(t)
	if (cur_second//period)%2 == 0:
		return 180
	else:
		return 0

#Stable val
const_func = (lambda t: 100)

#Repeated
def target_func(my_func, t):
	return np.array([my_func(t)]*interface.num_motors)


cur_target = sine_func

#For tracking
actual_path = []
target_path = []
times = []

#Initiate controller
ctrlr = NEURAL_PD(kp = 5, kd = 1.5, neural=True, adapt=True, neuron_model="LIF",pes_learning_rate=1e-1) 
#if kp = 5, everything
#up to 180/5 = 37 degrees will result in max (255) force applied.  


try:

	#For calculating the desired position. 
	start_time = time.time()
	target = target_func(cur_target, 0)
	prev_target = target #First time step will do nothing. 
	# prev_time = time.time()

	while time.time() - start_time < test_dur:
		feedback = interface.get_feedback()

		# print(time.time()-prev_time)
		# prev_time = time.time()
		prev_target = target
		target = target_func(cur_target, time.time()-start_time)


		if feedback != False:
			u = 180*ctrlr.generate_neural(
				q=normalize_val(feedback["q"]), 
				dq=normalize_val(feedback["dq"]), 
				target=normalize_val(target), 
				d_target=normalize_val(target-prev_target)
				)
			#Motor cannot move if u <50. Because of this, give some debounce space.
			for i in range(len(u)):
				if u[i] > 0.5 and abs(u[i]) < 50:
					u[i] = 50
				elif u[i] < 0.5 and abs(u[i]) < 50:
					u[i] = -50
					
			# u = ctrlr.generate_simple(
			# q=feedback["q"], 
			# dq=feedback["dq"], 
			# target=target, 
			# d_target=target-prev_target
			# )

			# print(u)
			# print(feedback["q"])
			# print(target)

			#Build adaption into PID. 
			# u += adapt.generate(input_signal = feedback["q"]/180, 
			# 	training_signal=ctrlr.training_signal)

			#New Target
			# prev_target = target

			# target = target_func(cur_target, time.time()-start_time)
			# target_time = time.time()

			#Send new forces
			interface.send_forces(u)
			actual_path.append(np.copy(feedback["q"]))

		else:
			print("Read Failed.")
			interface.send_forces([0,0,0,0])
			actual_path.append(actual_path[-1]) #if read fails, plot previous position. 
		
		target_path.append(np.copy(target))
		times.append(time.time()-start_time)

finally:
	#Log results
	cur_date = datetime.now()
	cur_date = cur_date.timetuple()

	folder_name = "../../test_results/PESRates/"
	test_name = "e1-1" 
	test_time = "-{}-{}:{}".format(cur_date[2],cur_date[3],cur_date[4])

	file_name1 = folder_name + test_name + test_time
	# file_name2 = folder_name + "target" + test_name + test_time

	np.save(file_name1,np.array([actual_path, target_path, times]))

	interface.get_feedback()
	interface.send_forces([0,0,0,0])

	interface.disconnect()
	print("Arm control stopped!")
