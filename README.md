# Swarm Networking Simulation

---

## To install dependencies

`sudo apt update && sudo apt install make cmake build-essential`
`pip3 install -r requirements.txt`

---

## To run the simulation

| Command | Description |
| ----------- | ----------- |
| `./start_sim -c` |  Cleanup current build |
| `./start_sim -b` | Build |
|`./start_sim -r config/experiment/comm_test/config_100.json`  | Run experiment with config file |
|`./start_sim -v 50 ./motion_log/default_log.txt output.mp4` | Render `default_log.txt` into video with 50x speed
