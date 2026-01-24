import pandas as pd
import matplotlib.pyplot as plt

def csv():
    # Read CSV without headers
    try:
        df = pd.read_csv('stacked_inverted_pendulum.csv', header=None)
    except FileNotFoundError:
        print("Error: 'flywheel_inverted_pendulum.csv' not found.")
        return
    
    print(f"CSV has {df.shape[1]} columns")
    print("First few rows:")
    print(df.head())
    
    # Setting up a 4-row, 2-column grid
    plt.figure(figsize=(14, 16))
    time = df[0]

    # --- Pendulum 1 ---
    plt.subplot(4, 2, 1)
    plt.plot(time, df[3], color='b')
    plt.ylabel('Angle (rad)')
    plt.title('Pendulum 1: Angle')
    plt.grid(True)

    plt.subplot(4, 2, 2)
    plt.plot(time, df[4], color='r')
    plt.ylabel('Speed (rad/s)')
    plt.title('Pendulum 1: Angular Speed')
    plt.grid(True)

    # --- Pendulum 2 ---
    plt.subplot(4, 2, 3)
    plt.plot(time, df[5], color='b')
    plt.ylabel('Angle (rad)')
    plt.title('Pendulum 2: Angle')
    plt.grid(True)

    plt.subplot(4, 2, 4)
    plt.plot(time, df[6], color='r')
    plt.ylabel('Speed (rad/s)')
    plt.title('Pendulum 2: Angular Speed')
    plt.grid(True)

    # --- Pendulum 3 ---
    plt.subplot(4, 2, 5)
    plt.plot(time, df[7], color='b')
    plt.ylabel('Angle (rad)')
    plt.title('Pendulum 3: Angle')
    plt.grid(True)

    plt.subplot(4, 2, 6)
    plt.plot(time, df[8], color='r')
    plt.ylabel('Speed (rad/s)')
    plt.title('Pendulum 3: Angular Speed')
    plt.grid(True)

    # --- Control Input ---
    # Placing it in the 7th slot (bottom left)
    plt.subplot(4, 2, 7)
    plt.plot(time, df[2], color='g')
    plt.xlabel('Time (s)')
    plt.ylabel('Input')
    plt.title('Quadratic State Cost vs Time')
    plt.grid(True)

    plt.tight_layout()
    plt.show()

if __name__ == "__main__":
    csv()