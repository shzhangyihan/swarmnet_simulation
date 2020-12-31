import parse

log_format = "{}|{}: {} - {} - {}"

class ProcessedData:
    id_assigned_count = 0
    id_collision_count = 0

    def __init__(self, num_robots):
        self.num_robots = num_robots
        self.robot_dict = {}

    def add_id_change(self, time, robot_id, gen_id, gen_size, checker = 0):
        self.robot_dict[robot_id] = (gen_id, gen_size, checker)
        self.data_process(time)

    def data_process(self, time):
        self.id_assigned_count = 0
        id_collision_dict = {}
        self.id_collision_count = 0

        # loop through all the robots
        for robot_id in self.robot_dict:
            gen_id_pair = self.robot_dict[robot_id]
            gen_id = gen_id_pair[0]
            gen_id_size = gen_id_pair[1]
            # gen_id_pair = (gen_id, gen_id_size)
            gen_id_pair = (gen_id)
            if gen_id_size != 32:
                # not tmp id
                self.id_assigned_count += 1
                # add to id dict for future collision count
                if gen_id_pair in id_collision_dict:
                    id_collision_dict[gen_id_pair] += 1
                else:
                    id_collision_dict[gen_id_pair] = 0

        self.id_collision_count = sum(id_collision_dict.values())

    def if_end(self):
        # print("check", self.id_assigned_count, self.id_collision_count)
        if (self.id_assigned_count == self.num_robots and 
            self.id_collision_count == 0):
            return True
        else:
            return False


class Checker:
    def __init__(self, log_file, batch_config):
        self.log_file = log_file
        self.num_robots = int(batch_config["num_robots"])
        self.processed_data = ProcessedData(self.num_robots)
    
    def check(self):
        with open(self.log_file) as f:
            for line in f.readlines():
                parsed = parse.parse(log_format, line)
                if parsed == None:
                    continue
                time = float(parsed[0])
                robot_id = int(parsed[1])
                gen_id = int(parsed[2])
                gen_id_len = int(parsed[3])
                self.processed_data.add_id_change(time, robot_id, gen_id, gen_id_len)
        
        return self.processed_data.if_end()