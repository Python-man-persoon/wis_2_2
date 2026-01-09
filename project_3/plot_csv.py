import pandas as pd
import matplotlib.pyplot as plt

def csv():
    # Read CSV without headers
    df = pd.read_csv('stacked_inverted_pendulum.csv', header=None)
    
    print(f"CSV has {df.shape[1]} columns")
    print("First few rows:")
    print(df.head())
    
    plt.figure(figsize=(12, 8))
    
    
    plt.subplot(2, 2, 1)
    plt.plot(df[0], df[3])  # time vs pendulum_angle1
    plt.xlabel('Time (s)')
    plt.ylabel('Pendulum Angle (rad)')
    ax = plt.gca()
    #ax.set_ylim([-1,1])
    plt.title('angle of the first pendulum')
    plt.grid(True)
    
    # Plot pendulum angle (column 5)
    plt.subplot(2, 2, 2)
    plt.plot(df[0], df[4])  # time vs pendulum_angle_velocity1
    plt.xlabel('Time (s)')
    plt.ylabel('angular velocity')
    ax = plt.gca()
    #ax.set_ylim([-1,1])
    plt.title('angular velocity of the first pendulum')
    plt.grid(True)
    
    # Plot control input (column 7)
    plt.subplot(2, 2, 3)
    plt.plot(df[0], df[5])  # time vs pendulum_angle2
    ax = plt.gca()
    ax.set_ylim([-100, 100])  # Adjust y-axis limits as needed
    plt.xlabel('Time (s)')
    plt.ylabel('Pendulum Angle 2 (rad)')
    plt.title('Pendulum Angle 2 vs Time')
    plt.grid(True)
    
    # Plot input cost
    plt.subplot(2, 2, 4)
    plt.plot(df[0], df[6])  # time vs pendulum_angle_velocity2
    plt.xlabel('Time (s)')
    plt.ylabel('Pendulum Angle Velocity 2 (rad/s)')
    plt.title('Pendulum Angle Velocity 2 vs Time')
    plt.grid(True)

    
    plt.tight_layout()
    plt.show()









if __name__ == "__main__":
    csv()