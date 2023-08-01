import time

class Cooler:
    def __init__(self, desired_temp, cooling_rate):
        self.desired_temp = desired_temp
        self.cooling_rate = cooling_rate
        self.current_temp = self.desired_temp  # Initialize the current temperature as the desired temperature

    def update_temp(self, current_temp, time_interval):
        rate_of_change = (current_temp - self.current_temp) / time_interval
        self.current_temp = current_temp - self.cooling_rate * rate_of_change

    def run_cooler(self):
        print(f"Cooler started. Desired temperature: {self.desired_temp}°C")

        while True:
            current_temp = float(input("Enter current temperature (in °C): "))

            if current_temp == self.desired_temp:
                print("Desired temperature achieved.")
                break

            time_interval = float(input("Enter time interval (in seconds): "))
            self.update_temp(current_temp, time_interval)

            print(f"Current temperature: {self.current_temp:.2f}°C")

            if self.current_temp < self.desired_temp:
                print("Cooler is working to cool down the house.")
            else:
                print("Cooler is off. Temperature is higher than the desired temperature.")

if __name__ == "__main__":
    desired_temperature = float(input("Enter desired temperature (in °C): "))
    cooling_rate = float(input("Enter cooling rate (recommended range: 0.1 - 0.5): "))

    cooler = Cooler(desired_temperature, cooling_rate)
    cooler.run_cooler()
