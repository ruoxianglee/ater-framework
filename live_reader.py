import bt2
import time
import threading
from pprint import pprint
# Live Trace Reader
# Reader States: start --> read --> consume --> no events --> stop
# 
# CTF streams over LTTng live (TCP)
#              |
#              |   +--------------------+
#              |   | src.ctf.lttng-live |
#              '-->|                    |
#                  |                out @--> Sorted messages of one
#                  +--------------------+    or more streams
# inputs=URL [array of one string]
#    Use URL to connect to the LTTng relay daemon.
#    URL is an array of exactly one string of which the format is:
#        net[4]://RDHOST[:RDPORT]/host/TGTHOST/SESSION
#    RDHOST
#        LTTng relay daemon’s host name or IP address.
#    RDPORT
#        LTTng relay daemon’s listening port. If not specified, 
#        the component uses the default port (5344).
#    TGTHOST
#        Target’s host name or IP address.
#    SESSION
#        Name of the LTTng tracing session from which to receive data.

reset=1

class LiveReader:
    def __init__(self, url):
            self.url = url # 'net://localhost/host/cpsiot/e2e_sample'
            self.msg_iter = bt2.TraceCollectionMessageIterator(url)
            self.trace_data = []

            self.consume_lock = threading.Lock()
            self.trace_data_lock = threading.Lock()
            self.consuming = True
            self.count = 1

            print('Live trace reader is ready to capture events from ' + self.url)
    
    def consume(self):
        while True:
            enabled = False
            with self.consume_lock:
                enabled = self.consuming
            
            if enabled:
                try:
                    # Keep trace data
                    for msg in self.msg_iter:
                        # `bt2._EventMessageConst` is the Python type of an event message.
                        if type(msg) is bt2._EventMessageConst:
                            # An event message holds a trace event.
                            with self.trace_data_lock:
                                self.trace_data.append(msg)
                            self.count = reset
                            # print('Events consumed.')
                # Each iteration of the loop, or, more precisely, the
                # :meth:`bt2.TraceCollectionMessageIterator.__next__` method, raises
                # an exception if there's any error during the iteration process.
                except bt2.TryAgain:
                    if self.count > 20:
                        with self.consume_lock:
                            self.consuming = False
                            print('No events to provide after reading for a while.')
                            break
                    else:
                        # print('Try again. No events to provide right now.')
                        None
                    self.count+=1
                
                time.sleep(0.5)
            else:
                print('Consuming is going to be stopped.')
                break
        
        print('Consuming finished.')

    def start_consume(self):
        print('Reading live trace data...')
        consumer = threading.Thread(target=self.consume, args=(), daemon=True)
        consumer.start()

    def stop_consume(self):
        print('Stop reading live trace data...')
        with self.consume_lock:
            self.consuming = False
    
    def consume_finished(self):
        with self.consume_lock:
            if not self.consuming:
                print('Check result: finished.')
                return True
            else:
                print('Check result: reading.')
                return False
    
    def get_trace_data(self) -> list[dict]:
        with self.trace_data_lock:
            trace_data = self.trace_data.copy()
            self.trace_data.clear()
            
        return list[dict](trace_data)