from live_reader import LiveReader
from trace_parser import TraceParser
import frequency_sender
import calculator
from indicators import Indicators, REGULATION_TYPE

import time
import sys

url = sys.argv[1]
BASELINE = sys.argv[2]
if BASELINE == 'True':
    print("==========================================================================")
    print("================================ Baseline ================================")
    BASELINE = True
else:
    BASELINE = False
    print("==========================================================================")
    print("================================== ATER ==================================")

outfile1 = sys.argv[3]
file_saving_drops = open(outfile1, 'w')

outfile2 = sys.argv[4]
file_saving_cputime = open(outfile2, 'w')

outfile3 = sys.argv[5]
file_saving_throughput = open(outfile3, 'w')

arch_file = './arch/architecture.yaml'

##############################
##### Parameters to be tuned
##############################

Delta_t = 8 # sampling period
varphi = 1   # expected sampling periods for regulation
sigma = 1
imdi_theta = 0.05 # theta
pmdi_w = 3        # w for PMDI, not used
ieri_lambda = 0.8 # lambda
buffer_sizes = [1,1,1,1]
##############################

live_reader = LiveReader(url)
live_reader.start_consume()

trace_data = []
epoch = 0
pre_position = None
pre_reduced_frequency = 0
pre_increased_frequency = float('inf')
pre_regulation_type = REGULATION_TYPE.NOACT

time.sleep(Delta_t) # the initial sleep time shold not be too small to prepare the trace data
trace_data = live_reader.get_trace_data()

if len(trace_data):
    print("Warning: Trace data set is not empty (epoch: 1)!\n")
else:
    print("Warning: Trace data set is empty (epoch: 1)!\n")

print("\n\nEpoch {}: analyzing ...".format(epoch+1))
trace_parser = TraceParser(arch_file, trace_data, buffer_sizes)
trace_parser.analyze_indicators()
Nd, Np, Ns, min_sub_period, avg_sub_period, Nb, throughput = trace_parser.get_message_indicators()
current_sub_period, pw_time, max_sub_period = trace_parser.get_callback_indicators()
print("==================================")
print("The number of drop messages: ", Nd)
print("The number of pub messages: ", Np)
print("The number of sub messages: ", Ns)
print("Number of messages in buffer: ", Nb)
print("Minimum subscription period: ", min_sub_period)
print("Maximum subscription period: ", max_sub_period)
print("Average subscription period: ", avg_sub_period)
print("Current subscription period: ", current_sub_period)
print("Executor PW workload: ", pw_time)

file_saving_drops.write("%d\n" % sum(Nd))
file_saving_throughput.write("%d\n" % Ns[3])

cputimes = [a * b for a, b in zip(Nd, pw_time)]
file_saving_cputime.write("%f\n" % sum(cputimes))

indicators = Indicators(Nd, Np, Ns, Nb, Delta_t, imdi_theta, ieri_lambda, current_sub_period, pw_time, pmdi_w)
regulation_type, regulation_position = indicators.check()
print("Regulation position: ", regulation_position)
regulation_flag = any(item == 1 for item in regulation_position)
if regulation_flag:
    if regulation_type == REGULATION_TYPE.REDUCE:
        modified_period, pre_position = calculator.calculate_reduced_frequency(max_sub_period, varphi, Delta_t, Nb, regulation_position)
        print(f"--- Reduced timer frequency: {1000/modified_period} -> {modified_period}ms")
        if BASELINE == False:
            frequency_sender.publish_modified_cycle_time(modified_period)
            pre_reduced_frequency = modified_period
            pre_regulation_type = REGULATION_TYPE.REDUCE
    elif regulation_type == REGULATION_TYPE.INCREASE:
        modified_period = calculator.calculate_increased_frequency(avg_sub_period, sigma, Delta_t, Nb, regulation_position, pre_position)
        
        if BASELINE == False:
            if modified_period > 0.0:
                print(f"+++ Increased timer frequency: {1000/modified_period} -> {modified_period}ms")
                frequency_sender.publish_modified_cycle_time(modified_period)
                pre_increased_frequency = modified_period
                pre_regulation_type = REGULATION_TYPE.INCREASE

elapsed_time = 0

while True:
    print("\n\nEpoch {}: analyzing ...".format(epoch+2))
    time.sleep(max(0, Delta_t - elapsed_time))
    start_time = time.time()
    trace_data = live_reader.get_trace_data()
    if len(trace_data):
        trace_parser.add_new_trace_data(trace_data)
        trace_parser.analyze_indicators()
        Nd, Np, Ns, min_sub_period, avg_sub_period, Nb, throughput = trace_parser.get_message_indicators()
        current_sub_period, pw_time, max_sub_period = trace_parser.get_callback_indicators()
        print("==================================")
        print("The number of drop messages: ", Nd)
        print("The number of pub messages: ", Np)
        print("The number of sub messages: ", Ns)
        print("Number of messages in buffer: ", Nb)
        print("Minimum subscription period: ", min_sub_period)
        print("Maximum subscription period: ", max_sub_period)
        print("Average subscription period: ", avg_sub_period)
        print("Current subscription period: ", current_sub_period)
        print("Executor PW workload: ", pw_time)

        file_saving_drops.write("%d\n" % sum(Nd))
        file_saving_throughput.write("%d\n" % Ns[3])

        cputimes = [a * b for a, b in zip(Nd, pw_time)]
        file_saving_cputime.write("%f\n" % sum(cputimes))

        indicators.update(Nd, Np, Ns, Nb, current_sub_period, pw_time)
        regulation_type, regulation_position = indicators.check()
        print("Regulation position: ", regulation_position)
        regulation_flag = any(item == 1 for item in regulation_position)
        if regulation_flag:
            if regulation_type == REGULATION_TYPE.REDUCE:
                modified_period, pre_position = calculator.calculate_reduced_frequency(max_sub_period, varphi, Delta_t, Nb, regulation_position)
                print(f"--- Reduced timer frequency: {1000/modified_period} -> {modified_period}ms")
                if BASELINE == False:
                    if pre_regulation_type == REGULATION_TYPE.REDUCE or pre_regulation_type == REGULATION_TYPE.NOACT:
                        if modified_period > pre_reduced_frequency:
                            frequency_sender.publish_modified_cycle_time(modified_period)
                            pre_reduced_frequency = modified_period
                            pre_regulation_type = REGULATION_TYPE.REDUCE
                    elif pre_regulation_type == REGULATION_TYPE.INCREASE:
                        if modified_period > pre_increased_frequency:
                            frequency_sender.publish_modified_cycle_time(modified_period)
                            pre_reduced_frequency = modified_period
                            pre_regulation_type = REGULATION_TYPE.REDUCE
            elif regulation_type == REGULATION_TYPE.INCREASE:
                modified_period = calculator.calculate_increased_frequency(avg_sub_period, sigma, Delta_t, Nb, regulation_position, pre_position)
                
                if BASELINE == False:
                    if pre_regulation_type == REGULATION_TYPE.INCREASE or pre_regulation_type == REGULATION_TYPE.NOACT:
                        if 0.0 < modified_period < pre_increased_frequency:
                            print(f"+++ Increased timer frequency: {1000/modified_period} -> {modified_period}ms")
                            # if modified_period < 80:
                            #     modified_period = 80.0
                            frequency_sender.publish_modified_cycle_time(modified_period)
                            pre_increased_frequency = modified_period
                            pre_regulation_type = REGULATION_TYPE.INCREASE
                    elif pre_regulation_type == REGULATION_TYPE.REDUCE:
                        if 0.0 < modified_period < pre_reduced_frequency:
                            print(f"+++ Increased timer frequency: {1000/modified_period} -> {modified_period}ms")
                            frequency_sender.publish_modified_cycle_time(modified_period)
                            pre_increased_frequency = modified_period
                            pre_regulation_type = REGULATION_TYPE.INCREASE
    else:
        print("Warning: Trace data set is empty (epoch: {})!\n".format(epoch+2))
        continue
    
    end_time = time.time()
    elapsed_time = end_time - start_time
    if epoch == 30:
        live_reader.stop_consume()
        exit()
    epoch+=1