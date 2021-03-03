import subprocess
import time
import os
import errno
import json
from multiprocessing import Process, Value

exec_path = "./build/src/core/simulation"
max_process_pool = 10
pool_wait_time = 30
# wait between starts, for different random seeds
wait_interval = 1

def import_from(module, name):
    module = __import__(module, fromlist=[name])
    return getattr(module, name)

class Job(Process):
    def __init__(self, counter, job_name, config_file, output_file, Checker, check_interval_seconds, batch_config):
        self.counter = counter
        self.job_name = job_name
        self.config_file = config_file
        # create file if not exist
        if not os.path.exists(os.path.dirname(output_file)):
            try:
                os.makedirs(os.path.dirname(output_file))
            except OSError as exc: # Guard against race condition
                if exc.errno != errno.EEXIST:
                    raise
        self.output_file = open(output_file, "w+")
        self.checker = Checker(output_file, batch_config)
        self.check_interval_seconds = check_interval_seconds
        super(Job, self).__init__()
    
    def run(self):
        global max_process_pool
        self.counter.value += 1
        # print(os.getcwd(), exec_path, self.config_file)
        print(self.job_name, "started ...", self.counter.value, "/", max_process_pool)
        self.proc = subprocess.Popen([exec_path, self.config_file], stdout=self.output_file)

        # start checking every check_interval_seconds
        while True:
            time.sleep(self.check_interval_seconds)
            if self.proc.poll() is not None:
                # already finished
                print(self.job_name, "run finished")
                break

            if self.checker.check():
                # experiment finished, kill program
                print(self.job_name, "needs to stop")
                # self.proc.kill()
                self.proc.send_signal(2)
                self.proc.wait()
                print(self.job_name, "process killed")
                break
        
        self.output_file.close()
        self.counter.value -= 1

class Scheduler:
    def __init__(self, config_file):
        # load config
        f = open(config_file)
        self.config = json.load(f)
        f.close()
        
        # set worker pool counter
        self.counter = Value('i', 0)
    
    def start(self):
        global max_process_pool
        global pool_wait_time
        global wait_interval
        jobs = []
        print("Processing input config ...")
        for batch in self.config:
            print("Experiment", batch["experiment_config"], "for", batch["repetitions"],"times")
        print("Config parsing finished ...")
        
        for batch in self.config:
            print("Run experiment", batch["experiment_config"], "for", batch["repetitions"],"times")
            Checker = import_from(batch["end_checking_program"], "Checker")
            start_index = 0
            if "start_index" in batch:
                start_index = batch["start_index"]
            for counter in range(start_index, start_index + batch["repetitions"]):
                new_job = Job(self.counter, batch["experiment_config"] + " run " + str(counter), batch["experiment_config"],
                              batch["output_file"] + str(counter), Checker, int(batch["check_interval_seconds"]), batch)
                new_job.start()
                jobs += [new_job]
                time.sleep(wait_interval)
                while self.counter.value >= max_process_pool:
                    time.sleep(pool_wait_time)
        
        for job in jobs:
            job.join()
        
        return 0
