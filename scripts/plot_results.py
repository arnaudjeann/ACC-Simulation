import pandas as pd
import matplotlib.pyplot as plt

# Load the data
try:
    df = pd.read_csv('data.csv')
except FileNotFoundError:
    print("Error: Run the C++ simulation first to generate data.csv")
    exit()

# Create a figure with 3 subplots to visualize the PID performance
fig, (ax1, ax2, ax3) = plt.subplots(3, 1, figsize=(10, 10), sharex=True)

# 1. Distance tracking
ax1.plot(df['time'], df['distance'], label='Actual Distance', color='blue', linewidth=2)
ax1.axhline(y=40, color='gray', linestyle=':', label='Initial Target (40m)')
ax1.axhline(y=25, color='red', linestyle='--', label='New Target (25m)')
ax1.set_ylabel('Distance (m)')
ax1.set_title('ACC Simulation: PID Controller Performance')
ax1.legend()
ax1.grid(True)

# 2. Vehicle Velocity
ax2.plot(df['time'], df['velocity'], label='Velocity (m/s)', color='green')
ax2.set_ylabel('Velocity (m/s)')
ax2.legend()
ax2.grid(True)

# 3. Acceleration (The actuator output)
ax3.step(df['time'], df['acceleration'], label='Acceleration (m/s^2)', color='orange', drawstyle='steps-post')
ax3.axhline(y=3.0, color='black', linestyle=':', alpha=0.5)  # Max Accel
ax3.axhline(y=-8.0, color='black', linestyle=':', alpha=0.5) # Max Brake
ax3.set_ylim(-9, 4)
ax3.set_xlabel('Time (s)')
ax3.set_ylabel('Acceleration (m/s^2)')
ax3.legend()
ax3.grid(True, which='both')

plt.tight_layout()
plt.show()