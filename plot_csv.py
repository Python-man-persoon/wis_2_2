import pandas as pd
import matplotlib.pyplot as plt

def csv():
    # Read CSV without headers
    df = pd.read_csv('flywheel_inverted_pendulum.csv', header=None)
    
    print(f"CSV has {df.shape[1]} columns")
    print("First few rows:")
    print(df.head())
    
    plt.figure(figsize=(12, 8))
    
    # Plot cart position (column 3)
    plt.subplot(2, 2, 1)
    plt.plot(df[0], df[5])  # time vs cart_position
    plt.xlabel('Time (s)')
    plt.ylabel('Pendulum Angle (rad)')
    ax = plt.gca()
    ax.set_ylim([-1,1])
    plt.title('Pendulum Angle vs Time')
    plt.grid(True)
    
    # Plot pendulum angle (column 5)
    plt.subplot(2, 2, 2)
    plt.plot(df[0], df[3])  # time vs pendulum_angle
    plt.xlabel('Time (s)')
    plt.ylabel('Flywheel Angle (rad)')
    ax = plt.gca()
    ax.set_ylim([-1,1])
    plt.title('Flywheel Angle vs Time')
    plt.grid(True)
    
    # Plot control input (column 7)
    plt.subplot(2, 2, 3)
    plt.plot(df[0], df[7])  # time vs control_input
    ax = plt.gca()
    ax.set_ylim([-100, 100])  # Adjust y-axis limits as needed
    plt.xlabel('Time (s)')
    plt.ylabel('Control Force (N)')
    plt.title('Control Input vs Time')
    plt.grid(True)
    
    # Plot input cost
    plt.subplot(2, 2, 4)
    plt.plot(df[0], df[2])  # time vs cost_input
    plt.xlabel('Time (s)')
    plt.ylabel('Input Cost')
    plt.title('Input Cost vs Time')
    plt.grid(True)

    
    plt.tight_layout()
    plt.show()









if __name__ == "__main__":
    csv()