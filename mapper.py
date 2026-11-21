from fileinput import filename
import math
import pandas as pd
import matplotlib.pyplot as plt

## This file creates a simple mapping function designed to display the depot,customers and charging stations in the requested data
## The first sunction simply displays the bare map
## The second function displays the map with the routes of the electric vehicles and the customers served by drones
def mapper_inni(data:str,
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

    # Formatting
    ax.set_xlabel("X Coordinate")
    ax.set_ylabel("Y Coordinate")
    ax.set_title("EV-Drone Routing Problem -" + data_name)

    ax.legend()
    ax.grid(True, linestyle="--", alpha=0.5)
    ax.set_aspect("equal", adjustable="box")

    plt.tight_layout()

    # Save figure
    if save_fig:
        plt.savefig(output_file, dpi=300, bbox_inches="tight")

    plt.show()

## Todo write the next function

