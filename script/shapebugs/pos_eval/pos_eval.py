import parse
import pandas as pd
import seaborn as sns
import matplotlib.pyplot as plt

def process_log():
    log_format = "{} {} {} true {}"
    log_file = "log"
    data_list = []
    dist_dict = {}
    avg_data_list = []
    with open(log_file, 'r') as f:
        for line in f.readlines():
            parsed = parse.parse(log_format, line)
            if parsed == None:
                continue
            node_id = int(parsed[0])
            time = float(parsed[1])
            dist = float(parsed[2])
            data_list += [[node_id, time, dist]]
            dist_dict[node_id] = dist
            avg_data_list += [[time, sum(dist_dict.values()) / len(dist_dict.values())]]

    df = pd.DataFrame(data_list, columns=['node_id', 'time', 'dist'])
    df.to_csv("processed.csv", sep='\t')
    avg_df = pd.DataFrame(avg_data_list, columns=['time', 'dist'])
    avg_df.to_csv("avg.csv", sep='\t')

def read_data(f):
    df = pd.read_csv(f, sep='\t', index_col=0)
    # df = df[0:100]
    print(df)
    return df

def plot_data(df):
    sns.lineplot(data=df, x="time", y="dist", ci=None)
    plt.show()

process_log()
# f = "processed.csv"
f = "avg.csv"
df = read_data(f)
plot_data(df)
