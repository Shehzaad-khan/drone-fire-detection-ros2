import time
import csv
from datetime import datetime
import matplotlib.pyplot as plt


def fire_alert_system():
    print("🚀 Fire Alert System Started...\n")

    detections = [
        {"x": 5, "y": 3, "confidence": 0.92},
        {"x": -4, "y": 6, "confidence": 0.88}
    ]

    with open("fire_log.csv", mode="w", newline="") as file:
        writer = csv.writer(file)
        writer.writerow(["Time", "X", "Y", "Confidence"])

        for i, fire in enumerate(detections):
            timestamp = datetime.now().strftime("%H:%M:%S")

            print(f"[{timestamp}] 🔥 FIRE DETECTED #{i+1}")
            print(f"Location → X: {fire['x']} | Y: {fire['y']}")
            print(f"Confidence → {fire['confidence']}")
            print("-" * 40)

            # Save to CSV
            writer.writerow([timestamp, fire["x"], fire["y"], fire["confidence"]])

            time.sleep(2)

    print("\n✅ Monitoring Complete")
    print("📁 Log saved as fire_log.csv")


def visualize_fire():
    x_coords = []
    y_coords = []

    with open("fire_log.csv", "r") as file:
        next(file)  # skip header
        for line in file:
            _, x, y, _ = line.strip().split(",")
            x_coords.append(float(x))
            y_coords.append(float(y))

    plt.figure()
    plt.scatter(x_coords, y_coords)

    # Add labels for each fire
    for i in range(len(x_coords)):
        plt.text(x_coords[i], y_coords[i], f"Fire {i+1}")

    plt.title("Fire Detection Map")
    plt.xlabel("X Coordinate")
    plt.ylabel("Y Coordinate")
    plt.grid()

    plt.show()


if __name__ == "__main__":
    fire_alert_system()
    visualize_fire()