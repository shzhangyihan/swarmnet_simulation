import subprocess
import time
import os
import json

class Job:
    def __init__(self, config_file, output_file):
        self.config_file = config_file
        self.output_file = open(output_file, "w")
    
    def run(self):
        # print("run")
        self.proc = subprocess.Popen(["./build/src/core/simulation", self.config_file], stdout=self.output_file)

    def end(self):
        return_code = self.proc.poll()
        if not return_code:
            self.proc.kill()
            return_code = self.proc.wait()
        print(return_code)
        self.output_file.close()

os.chdir("../")
job = Job("./config/experiment/short_test/config_49_improved_analytical.json", "dumps/dump.txt")

job.run()
time.sleep(5)
job.end()

