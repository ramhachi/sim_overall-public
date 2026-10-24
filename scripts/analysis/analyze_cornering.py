import pandas as pd
import numpy as np

# Load cornering data
df = pd.read_csv('data/tiredata/Cornering-Hoosier18.0×6.0-10-R20.csv', skiprows=[1])

# Convert FZ to positive
df['FZ'] = df['FZ'].abs()

print('Cornering Data Analysis:')
print(f'Total rows: {len(df)}')
print(f'FZ range: [{df["FZ"].min():.2f}, {df["FZ"].max():.2f}] N')
print(f'SR range: [{df["SR"].min():.4f}, {df["SR"].max():.4f}]')
print(f'V range: [{df["V"].min():.2f}, {df["V"].max():.2f}] kph')
print(f'TSTC range: [{df["TSTC"].min():.2f}, {df["TSTC"].max():.2f}] °C')

# Apply filters one by one
print('\nApplying filters:')
df_v = df[df['V'] >= 10.0]
print(f'After V >= 10: {len(df_v)} rows')

df_fz = df_v[df_v['FZ'] >= 500.0]
print(f'After FZ >= 500: {len(df_fz)} rows')

df_sr = df_fz[df_fz['SR'].abs() <= 0.01]
print(f'After |SR| <= 0.01: {len(df_sr)} rows')

df_temp = df_sr[(df_sr['TSTC'] >= 35.0) & (df_sr['TSTC'] <= 85.0)]
print(f'After temp 35-85: {len(df_temp)} rows')

# Check SR distribution
print(f'\nSR statistics after FZ filter:')
print(f'Mean: {df_fz["SR"].mean():.4f}')
print(f'Std: {df_fz["SR"].std():.4f}')
print(f'Rows with |SR| <= 0.01: {(df_fz["SR"].abs() <= 0.01).sum()}')
print(f'Rows with |SR| <= 0.02: {(df_fz["SR"].abs() <= 0.02).sum()}')
print(f'Rows with |SR| <= 0.05: {(df_fz["SR"].abs() <= 0.05).sum()}')
