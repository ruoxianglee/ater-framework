from typing import Optional
from latency_analyzer import latency_statistics
import pandas as pd
from collections.abc import Sequence
from caret_analyze import Architecture, Application, Lttng
from caret_analyze.runtime.publisher import Publisher
from caret_analyze.runtime.subscription import Subscription
from caret_analyze.record.interface import RecordsInterface
from enum import Enum
from typing import Dict, List, Sequence
import yaml
from collections.abc import Collection
from typing import Any

TargetObjectTypes = Publisher | Subscription
class RoleType(Enum):
    PUB = 1
    SUB = 2

import numpy as np

class TraceParser:
    def __init__(
        self, 
        arch: str, 
        trace_data: list[dict],
        buffer_sizes: list[int]
    ):
        self._arch_file = arch
        self._arch = Architecture('yaml', self._arch_file)
        self._lttng = Lttng(trace_data)
        self._app = Application(self._arch, self._lttng)
        
        ##### the number of messages produced/consumed within one sampling period
        ##### only _num_of_subs is used for sub period calculation
        self._num_of_pubs = [] 
        self._num_of_subs = []
        self.num_of_drops = []

        ##### sub period indicators, min and avg period may not be used
        self._min_sub_period = []
        self._avg_sub_period = []
        self._max_sub_period = [] # float
        self._num_of_sampling = 1

        ##### message buffer size for each pub-sub pair
        self._buffer_sizes = buffer_sizes[:]
        
        ##### total number of messages that are dropped
        self._sum_of_drops = []

        ##### number of messages not stored in buffer for each pub-sub pair
        self._num_of_buffer_msgs = []

        #### number of messages published by the last task
        self._num_of_commands = 0
        self._pre_sum_of_commands = 0

        ##### real sampling period, should be very close to sampling_period, but may be not exactly same
        self._sub_durations = [] #ns

        ##### may not used
        self._pre_sum_of_pubs = []

        ##### record the total number of subscribed messages before current sampling period
        self._pre_sum_of_subs = []

        ##### used for calculation of callback workload, the elements are indexed based on executor name
        self._pre_source_stamps_ns = {}
        self._pre_dest_stamps_ns = {}

        ##### used for increasing frequency indicator
        self._current_sub_period_ms = []
        self._current_exe_pw_workload_ms = []

        self.index_mapping = []
        print("Arch file name: ", self._arch_file)
        self._topic_names = self.get_topic_names()
        print("Topic names: ", self._topic_names)
    
    def add_new_trace_data(self, trace_data: list[dict]):
        self._lttng.add_trace_data(trace_data)
        self._app = Application(self._arch, self._lttng)

    def analyze_indicators(self):
        self._num_of_sampling += 1
        self.analyze_message_indicators()
        self.analyze_callback_indicators()

    ##############################################################
    ######## analyze message pub-sub related indicators ##########
    ##############################################################
    def analyze_message_indicators(self):
        # app = Application(self._arch, self._lttng)

        # TODO: if there are multiple subscribers or publishers, we should only focus on the last publisher or the first subscriber.
        #       this definitely should be considered in parse_msgs() function, 
        #       where it can not handle this case now
        sum_of_pub_list, _, _ = self.parse_msgs(*self._app.publishers, role_type=RoleType.PUB)
        sum_of_sub_list, sub_min_time_list, sub_max_time_list = self.parse_msgs(*self._app.subscriptions, role_type=RoleType.SUB)

        # self._pub_durations = [a - b for a, b in zip(pub_max_time_list, pub_min_time_list)]
        self._sub_durations = [a - b for a, b in zip(sub_max_time_list, sub_min_time_list)] #ns
        # print("Time duration of each sampling time: ", self._sub_durations)

        if len(self._pre_sum_of_pubs) == 0 or len(self._pre_sum_of_subs) == 0:
            self._num_of_pubs = sum_of_pub_list[:]
            self._num_of_subs = sum_of_sub_list[:]
            self.parse_index_mapping(*self._app.subscriptions) # the node/sub/pub/cbs order is not same as in task chain, by default, it is ordered based on name
            self._sum_of_drops = [0] * len(self._buffer_sizes)
            
            ##### self._sub_durations / (self._num_of_subs * 1000000) #period=ms
            ##### self._sub_durations in ns
            self._min_sub_period = [a / (b * 1000000) for a, b in zip(self._sub_durations, self._num_of_subs)]
            self._avg_sub_period = [a / (b * 1000000) for a, b in zip(self._sub_durations, self._num_of_subs)]
            self._max_sub_period = [a / (b * 1000000) for a, b in zip(self._sub_durations, self._num_of_subs)]
        else:
            self._num_of_pubs = [a - b for a, b in zip(sum_of_pub_list, self._pre_sum_of_pubs)]
            self._num_of_subs = [a - b for a, b in zip(sum_of_sub_list, self._pre_sum_of_subs)]

            for i in range(len(self._max_sub_period)):
                self._min_sub_period[i] = min(self._min_sub_period[i], self._sub_durations[i] / (self._num_of_subs[i] * 1000000))
                
                self._avg_sub_period[i] = (self._avg_sub_period[i] * (self._num_of_sampling - 1) + 
                                    self._sub_durations[i] / (self._num_of_subs[i] * 1000000)) / self._num_of_sampling
                
                self._max_sub_period[i] = max(self._max_sub_period[i], self._sub_durations[i] / (self._num_of_subs[i] * 1000000))

        ##### calculate the number of message in buffer currently
        ##### 1. number of messages dropped currently
        ##### 2. update sum of dropped messages
        ##### 3. calculate 
        # num_of_backlogs = [a - b for a, b in zip(sum_of_pub_list, sum_of_sub_list)]
        # current_sum_of_drops = [a - b if a - b > 0 else 0 for a, b in zip(num_of_backlogs, self._buffer_sizes)]
        # self.num_of_drops = [a - b if a - b > 0 else 0 for a, b in zip(current_sum_of_drops, self._sum_of_drops)]
        # self._sum_of_drops = [a + b for a, b in zip(self.num_of_drops, self._sum_of_drops)]
        # self._num_of_buffer_msgs = [a - b for a, b in zip(num_of_backlogs, self._sum_of_drops)]

        num_of_backlogs = [a - b for a, b in zip(sum_of_pub_list, sum_of_sub_list)]
        new_num_of_backlogs = [a - b if a - b > 0 else 0 for a, b in zip(num_of_backlogs, self._sum_of_drops)]
        self.num_of_drops = [a - b if a - b > 0 else 0 for a, b in zip(new_num_of_backlogs, self._buffer_sizes)]
        # print("Number of dropped messages: ", self.num_of_drops)
        self._sum_of_drops = [a + b for a, b in zip(self.num_of_drops, self._sum_of_drops)]
        self._num_of_buffer_msgs = [b if a > b else a for a, b in zip(new_num_of_backlogs, self._buffer_sizes)]

        self._pre_sum_of_pubs = sum_of_pub_list[:]
        self._pre_sum_of_subs = sum_of_sub_list[:]

    def get_message_indicators(self)->tuple[list[int], list[int], list[int], list[float], list[float], list[int], int]:
        return self.update_item_order_helper(self.num_of_drops), self.update_item_order_helper(self._num_of_pubs), self.update_item_order_helper(self._num_of_subs), self.update_item_order_helper(self._min_sub_period), self.update_item_order_helper(self._avg_sub_period), self.update_item_order_helper(self._num_of_buffer_msgs), self._num_of_commands

    ##############################################################
    ######## analyze callback workloads related indicators #######
    ##############################################################
    def analyze_callback_indicators(self):
        exe_names = self._app.executor_names # the first executor is not used
        self._current_exe_pw_workload_ms.clear()
        for exe_name in exe_names[1:]: # here we skip the first executor in task chain

            # num_of_callbacks = len(self._app.get_executor(exe_name).callbacks)

            # for idx in range(num_of_callbacks):
                # TODO: reorder the callbacks from each executor to be a chain
                #       exe_pw_workload = source_cb_start_time - sink_cb_end_time
                #       now, we assume only one callback in each executor

                # all_records = []
                # for idx in range(num_of_callbacks):
                #     records = self._app.get_executor(exe_names[0]).callbacks[idx]._to_records_core()
                #     records_data_frame = records.to_dataframe()
                #     all_records.append(records_data_frame)

                # all_data = pd.concat([all_records[0], all_records[1]], axis=1)
                # for idx in range(len(all_records)-2):
                #     all_data = pd.concat([all_data, all_records[idx+2]], axis=1)
                # print(all_data)

            records = self._app.get_executor(exe_name).callbacks[0]._to_records_core()
            target_record = records.to_dataframe()

            # Step 1. reorder columns in target_record
            # Step 2. exe_pw_workload = target_record.columns[-1] - target_record.columns[0]
            columns = records.to_dataframe().columns
            source_stamps_ns = target_record[columns[0]][:]
            dest_stamps_ns = target_record[columns[1]][:]

            exe_pw_wokload = []
            if exe_name not in self._pre_source_stamps_ns:
                exe_pw_wokload = [a - b for a, b in zip(dest_stamps_ns, source_stamps_ns)]
            else:
                new_source_stamps_ns = []
                new_dest_stamps_ns = []
                for item in source_stamps_ns[len(self._pre_source_stamps_ns[exe_name]):]:
                    new_source_stamps_ns.append(item)

                for item in dest_stamps_ns[len(self._pre_dest_stamps_ns[exe_name]):]:
                    new_dest_stamps_ns.append(item)

                exe_pw_wokload = [a - b for a, b in zip(new_dest_stamps_ns, new_source_stamps_ns)]

            self._pre_source_stamps_ns[exe_name] = source_stamps_ns
            self._pre_dest_stamps_ns[exe_name] = dest_stamps_ns
            self._current_exe_pw_workload_ms.append((sum(exe_pw_wokload)/ len(exe_pw_wokload)) / 1000000)

        self._current_sub_period_ms = [a / (b * 1000000) for a, b in zip(self._sub_durations, self._num_of_subs)]

    def get_callback_indicators(self)->tuple[list[float], list[float], list[float]]:
        return self.update_item_order_helper(self._current_sub_period_ms), self._current_exe_pw_workload_ms, self.update_item_order_helper(self._max_sub_period)

    def update_item_order_helper(self, list_to_order: list)->list:
        backup = list_to_order.copy()
        ordered_list = list_to_order.copy()
        for idx, item in enumerate(self.index_mapping):
            ordered_list[item] = backup[idx]
        return ordered_list

    def get_topic_names(self)->list[str]:
        with open(self._arch_file, 'r') as file:
            data = yaml.safe_load(file)

        return [
            node['publish_topic_name']
            for node in data['named_paths'][0]['node_chain']
            if node['publish_topic_name'] != 'UNDEFINED'
        ]

    def parse_index_mapping(
        self,
        *target_objects: TargetObjectTypes
    ):
        for sub in target_objects:
            try:
                index = self._topic_names.index(sub.topic_name)
                self.index_mapping.append(index)
            except ValueError:
                pass
        print("Index mapping: ", self.index_mapping)

    def parse_qos(
        self,
        *target_objects: TargetObjectTypes
    ) -> list[int]:
        buffer_sizes = [1] * len(self._topic_names)

        for sub in target_objects:
            try:
                index = self._topic_names.index(sub.topic_name)
                self.index_mapping.append(index)
                qos = sub.qos
                if qos is not None:
                    buffer_sizes[index] = max(buffer_sizes[index], qos.depth)
                    print("Got qos.depth information...")
            except ValueError:
                pass
        print("Index mapping: ", self.index_mapping)
        return buffer_sizes

    def get_timestamp_range(
        self,
        timeseries_records_list: list[RecordsInterface],
        item_counts: list[int],
        role_type: RoleType
    ) -> tuple[list[int], list[int]]:
        first_timestamps = []
        last_timestamps = []
        record_index = 0

        for records in timeseries_records_list:
            
            if len(records) == 0:
                record_index += 1
                continue

            item_count = item_counts[record_index]

            if len(self._pre_sum_of_pubs) == 0 or len(self._pre_sum_of_subs) == 0:
                first_timestamp = records.get_column_series(records.columns[0])[0]
                if isinstance(first_timestamp, int):
                    first_timestamps.append(first_timestamp)
            else:
                if role_type == RoleType.PUB:
                    item_index = item_count - self._pre_sum_of_pubs[record_index]
                    first_timestamp = records.get_column_series(records.columns[0])[-item_index]
                    if isinstance(first_timestamp, int):
                        first_timestamps.append(first_timestamp)
                elif role_type == RoleType.SUB:
                    item_index = item_count - self._pre_sum_of_subs[record_index]
                    first_timestamp = records.get_column_series(records.columns[0])[-item_index]
                    if isinstance(first_timestamp, int):
                        first_timestamps.append(first_timestamp)

            last_timestamp = records.get_column_series(records.columns[0])[-1]
            if isinstance(last_timestamp, int):
                last_timestamps.append(last_timestamp)
            
            record_index += 1


        if len(first_timestamps) == 0 or len(last_timestamps) == 0:
            return 0, 1  # Intended to show an empty figure.
        else:
            return first_timestamps, last_timestamps


    # TODO: if there are multiple subscribers or publishers, we should only focus on the last publisher or the first subscriber.
    #       this definitely should be considered in parse_msgs() function, 
    #       where it can not handle this case now
    def parse_msgs(
        self,
        *target_objects: TargetObjectTypes,
        role_type: RoleType
    ) -> tuple[list[int], list[int], list[int]]:
        timeseries_records_list: list[RecordsInterface] = [
            _.to_records() for _ in target_objects
        ]

        ##### count the sum of the items (i.e., the number of timestamp or messages for one pub or one sub)
        item_counts = []
        for records in timeseries_records_list:
            target_column = records.columns[0]
            count = 0
            for record in records:
                if target_column in record.columns:
                    count+=1
            item_counts.append(count)

        # TODO: the last pub in task chain is not considered, which is in location 2 of the lists 
        # the items are ordered based on topci_name
        # try to fix this for general cases
        self._num_of_commands = item_counts[2] - self._pre_sum_of_commands
        self._pre_sum_of_commands = item_counts[2]
        if role_type == RoleType.PUB:
            del item_counts[2]

        min_time_list = []
        max_time_list = []
        if role_type == RoleType.SUB:
            min_time_list, max_time_list = self.get_timestamp_range(timeseries_records_list, item_counts, role_type)
        
        return item_counts, min_time_list, max_time_list