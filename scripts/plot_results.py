import numpy as np
import pandas as pd
import matplotlib.pyplot as plt

# Load the data
try:
    df = pd.read_csv('data.csv')
except FileNotFoundError:
    print("Error: Run the C++ simulation first to generate data.csv")
    exit()

df['ttc'] = df['ttc'].replace(-1.0, np.nan)
fig, (ax1, ax2, ax3, ax4) = plt.subplots(4, 1, figsize=(10, 12), sharex=True)

for ax in (ax1, ax2, ax3, ax4):
    ax.tick_params(axis='y', labelsize=9)

# 1. Distance tracking
ax1.plot(df['time'], df['distance'], label='Actual Distance', color='blue', linewidth=2)
ax1.axhline(y=40, color='gray', linestyle=':', label='Initial Target (40m)')
ax1.plot(df['time'], df['setpoint'], label='Target Distance', color='red', linestyle='--')
ax1.set_ylabel('Distance (m)')
ax1.set_title('ACC Simulation with Dynamic Time-to-Collision (TTC)')
ax1.legend()
ax1.grid(True)

# 2. Vehicle Velocity
ax2.plot(df['time'], df['velocity'], label='Ego Velocity (m/s)', color='green')
ax2.plot(df['time'], df['lead_velocity'], label='Lead Velocity (m/s)', color='purple', linestyle=':')
ax2.set_ylabel('Velocity (m/s)')
ax2.legend()
ax2.grid(True)

# 3. Acceleration (The actuator output)
ax3.step(df['time'], df['acceleration'], label='Acceleration (m/s^2)', color='orange', drawstyle='steps-post')
ax3.axhline(y=3.0, color='black', linestyle=':', alpha=0.5)  # Max Accel
ax3.axhline(y=-8.0, color='black', linestyle=':', alpha=0.5) # Max Brake
ax3.set_ylim(-9, 4)
ax3.set_ylabel('Acceleration (m/s^2)')
ax3.legend()
ax3.grid(True, which='both')

# 4. Time-to-Collision (TTC)
ax4.plot(df['time'], df['ttc'], label='TTC (s)', color='red', linewidth=2)
ax4.axhline(y=2.0, color='darkred', linestyle='--', label='TTC Critical Threshold (2.0s)')
ax4.set_ylim(0, 20) 
ax4.set_xlabel('Time (s)')
ax4.set_ylabel('TTC (seconds)')
ax4.legend()
ax4.grid(True)

plt.tight_layout()
plt.show()
