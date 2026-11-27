import math
import random
import numpy as np

def data_gen(num_customers: int,
            num_stations: int,
            depot_location: tuple = (0, 0), 
            wind: bool =False,
            Temp: float = 25.0,
            Humidity: float = 60.0,
            Traffic: float = 0.0,
            area_size: tuple = (100, 100),
            output_file: str = "data/output/generated_data.txt") -> str:
    """
    Generates a data file with depot, customers, and charging stations.

    Parameters:
    - num_customers: Number of customer nodes to generate.
    - num_stations: Number of charging station nodes to generate.
    - depot_location: Tuple representing the (x, y) coordinates of the depot.
    - area_size: Tuple representing the (width, height) of the area in which to generate nodes.

    Returns:
    - The name of the generated data file.
    """


    # Generate depot
    depot_id = "D1"
    depot_x, depot_y = depot_location
    depot_demand = 0  # Depot has no demand

    # Generate customers
    customers = []
    for i in range(1, num_customers + 1):
        customer_id = f"C{i}"
        customer_x = np.random.randint(-area_size[0]*10/2, area_size[0]*10/2)/10.0
        customer_y = np.random.randint(-area_size[1]*10/2, area_size[1]*10/2)/10.0
        customer_demand = np.random.randint(1, 151)  # Random demand between 1 and 150
        customers.append((customer_id, customer_x, customer_y, customer_demand))

    # Generate charging stations
    stations = []
    for i in range(1, num_stations + 1):
        station_id = f"S{i}"
        station_x = np.random.randint(-area_size[0]*10/2, area_size[0]*10/2)/10.0
        station_y = np.random.randint(-area_size[1]*10/2, area_size[1]*10/2)/10.0
        station_demand = 0  # Charging stations have no demand
        stations.append((station_id, station_x, station_y, station_demand))

    if wind:
        # Generate wind data
        wind_speed = np.random.randint(0, 11)  # Random wind speed between 0 and 10 m/s
        wind_direction = np.random.randint(0, 361)  # Random wind direction in degrees

    else:
        wind_speed = 0
        wind_direction = 0

    # Write to file
    filename = output_file
    with open(filename, "w") as f:
        f.write("id Type x Y demand Temp Humidity Wind  Traffic WindDir \n")  # Header
        f.write(f"{depot_id} d {depot_x} {depot_y} {depot_demand} {Temp} {Humidity} {wind_speed} {Traffic} {wind_direction}\n")
        for station in stations:
                    f.write(f"{station[0]} s {station[1]} {station[2]} {station[3]} {Temp} {Humidity} {wind_speed} {Traffic} {wind_direction}\n")
        for customer in customers:
            f.write(f"{customer[0]} c {customer[1]} {customer[2]} {customer[3]} {Temp} {Humidity} {wind_speed} {Traffic} {wind_direction}\n")
        

    return filename