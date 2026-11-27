from fileinput import filename
import math
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from data.EV_data import Truckdata, DroneData, EnvParams

## This file creates a simple mapping function designed to display the depot,customers and charging stations in the requested data
## The first sunction simply displays the bare map
## The second function displays the map with the routes of the electric vehicles and the customers served by drones
def mapper(data:str,
                save_fig: bool = False,
                output_file: str = "data/output/images/bare_map.png"
                )-> None:
    
        # Initialize data containers
    depot = []
    stations = []
    customers = []
    data_name = data.replace(".txt", "")
    data_name = data_name.split("/")[-1]  # Extract the file name without the path
    # Read and parse text file
    with open(data, "r") as file:
        lines = file.readlines()

    # Skip header
    for line in lines[1:]:

        values = line.strip().split()

        if len(values) < 5:
            continue

        node_id = values[0]
        node_type = values[1]
        x = float(values[2])
        y = float(values[3])
        demand = float(values[4])
        wind_direction= float(values[9]) 
        wind = float(values[7]) 
        temp = float(values[5])
        humidity = float(values[6])

        node = (node_id, x, y, demand)

        if node_type == "d":
            depot.append(node)

        elif node_type == "s":
            stations.append(node)

        elif node_type == "c":
            customers.append(node)

    # Initialize figure
    fig, ax = plt.subplots(figsize=(10, 8))

    # Plot depot
    if depot:
        ax.scatter(
            [n[1] for n in depot],
            [n[2] for n in depot],
            marker="*",
            s=250,
            color="red",
            edgecolors="black",
            label="Depot",
            zorder=3
        )

    # Plot charging stations
    if stations:
        ax.scatter(
            [n[1] for n in stations],
            [n[2] for n in stations],
            marker="s",
            s=100,
            color="green",
            edgecolors="black",
            label="Charging Stations",
            zorder=2
        )

    # Plot customers
    if customers:
        ax.scatter(
            [n[1] for n in customers],
            [n[2] for n in customers],
            marker="o",
            s=60,
            color="royalblue",
            edgecolors="black",
            label="Customers",
            zorder=2
        )

    # Add node labels
    for node in depot + stations + customers:
        node_id, x, y, demand = node

        ax.annotate(
            node_id,
            (x, y),
            xytext=(5, 5),
            textcoords="offset points",
            fontsize=8
        )
    dx_m_angle = [];
    dy_m_angle = [];
    distances = []
    rho_h = EnvParams().rho0 * (1 -EnvParams().humidity_coef * (humidity - EnvParams().humidity_ref))
    temp_diff = 0 #Set to zero because fuck all of these coefficients
    for angle in range(0, 361): # For 360 degrees, calculate the maximum distance a drone can travel based on wind speed and direction
        s_d = np.sqrt((DroneData().average_drone_speed/3.6)**2 + wind**2 - 2*(DroneData().average_drone_speed/3.6)*wind*math.cos(math.radians(angle-wind_direction)))
        c1 = ((rho_h*DroneData().eta*DroneData().drone_area*(s_d**2))/
              ((2*DroneData().efficency*np.exp(-EnvParams().humidity_coef*(temp_diff))**2)))
        + DroneData().additional_energy_coefficient*(DroneData().drone_curb_mass)

        c2 = ((rho_h*DroneData().eta*DroneData().drone_area*(s_d**2))/
        ((2*DroneData().efficency*np.exp(-EnvParams().humidity_coef*(temp_diff))**2)))
        + DroneData().additional_energy_coefficient*(DroneData().drone_curb_mass+DroneData().drone_load_capacity)
        delta_h = 0 #no altitude change considered in this model
        c3 =  DroneData().additional_energy_coefficient*(delta_h*(DroneData().drone_curb_mass +DroneData().drone_load_capacity) +(DroneData().drone_curb_mass)) #no altitude change considered in this model
        d_m = DroneData().drone_battery_capacity / (c1 + c2) *10**3
        distances.append(d_m)
        dx_m_angle.append(d_m*math.cos(math.radians(angle)) + depot[0][1]) #Add depot x coordinate to the distance to get the x coordinate of the maximum distance a drone can travel based on wind speed and direction
        dy_m_angle.append(d_m*math.sin(math.radians(angle)) + depot[0][2]) #Add depot y coordinate to the distance to get the y coordinate of the maximum distance a drone can travel based on wind speed and direction

    ax.fill(dx_m_angle, dy_m_angle, color="dodgerblue", alpha=0.3)
    ax.plot(dx_m_angle, dy_m_angle, color="blue")
    #print(dx_m_angle,dy_m_angle, "Max distance a drone can travel based on wind speed and direction")
    # Formatting

    ax.set_xlabel("X Coordinate")
    ax.set_ylabel("Y Coordinate")
    ax.set_title("EV-Drone Routing Problem -" + data_name)
    ax.text(0.02, 0.95, f"wind speed: {wind} m/s, wind direction: {wind_direction} degrees", transform=ax.transAxes, fontsize=10, verticalalignment='top')
    ax.legend()
    ax.grid(True, linestyle="--", alpha=0.5)
    ax.set_aspect("equal", adjustable="box")

    plt.tight_layout()

    # Save figure
    if save_fig:
        plt.savefig(output_file, dpi=300, bbox_inches="tight")

    plt.show()

## Todo write the next function

