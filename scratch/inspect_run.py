import pandas as pd

df = pd.read_csv("mpc_v2_fail_run.csv")
print("Velocity profile over time (every 100 ms):")
pd.set_option('display.max_columns', None)
pd.set_option('display.width', 1000)
print(df[::100][['time', 'x', 'y', 'vx', 'vy', 'r', 'delta_sw', 'lateral_error']])
