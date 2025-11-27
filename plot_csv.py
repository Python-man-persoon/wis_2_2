import pandas as pd
import matplotlib.pyplot as plt

def csv():
    # Read CSV without headers
    df = pd.read_csv('cart_inverted_pendulum.csv', header=None)
    
    print(f"CSV has {df.shape[1]} columns")
    print("First few rows:")
    print(df.head())
    
    # Based on the simulation code structure, columns are typically:
    # Column 0: time
    # Column 1: cost_state  
    # Column 2: cost_input
    # Column 3: cart_position
    # Column 4: cart_velocity
    # Column 5: pendulum_angle
    # Column 6: pendulum_angular_velocity
    # Column 7: control_input
    
    plt.figure(figsize=(12, 8))
    
    # Plot cart position (column 3)
    plt.subplot(2, 2, 1)
    plt.plot(df[0], df[3])  # time vs cart_position
    plt.xlabel('Time (s)')
    plt.ylabel('Cart Position (m)')
    plt.title('Cart Position vs Time')
    plt.grid(True)
    
    # Plot pendulum angle (column 5)
    plt.subplot(2, 2, 2)
    plt.plot(df[0], df[5])  # time vs pendulum_angle
    plt.xlabel('Time (s)')
    plt.ylabel('Pendulum Angle (rad)')
    plt.title('Pendulum Angle vs Time')
    plt.grid(True)
    
    # Plot control input (column 7)
    plt.subplot(2, 2, 3)
    plt.plot(df[0], df[7])  # time vs control_input
    plt.xlabel('Time (s)')
    plt.ylabel('Control Force (N)')
    plt.title('Control Input vs Time')
    plt.grid(True)
    
    # Plot phase portrait
    plt.subplot(2, 2, 4)
    plt.plot(df[3], df[4])  # cart_position vs cart_velocity
    plt.xlabel('Cart Position (m)')
    plt.ylabel('Cart Velocity (m/s)')
    plt.title('Cart Phase Portrait')
    plt.grid(True)

    
    plt.tight_layout()
    plt.show()

if __name__ == "__main__":
    csv()