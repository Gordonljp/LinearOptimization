from fileinput import filename
import math
import pandas as pd
import matplotlib.pyplot as plt

## This file creates a simple mapping function designed to display the depot,customers and charging stations in the requested data
## The first sunction simply displays the bare map
## The second function displays the map with the routes of the electric vehicles and the customers served by drones
def mapper_inni(data:string,
                save_fig: bool = False,
                output_file: str = "instance_map.png"
                )-> None:
    df = pd.read_csv(filename, sep=r"\s+")

    # Identify different node types
    depot = df[df["Type"] == "d"]
    stations = df[df["Type"] == "s"]
    customers = df[df["Type"] == "c"]

    # Create figure
    fig, ax = plt.subplots(figsize=(10, 8))

    # Plot depot
    ax.scatter(
        depot["X"], depot["Y"],
        marker="*",
        s=250,
        color="red",
        edgecolors="black",
        label="Depot",
        zorder=3
    )

    # Plot charging stations
    ax.scatter(
        stations["X"], stations["Y"],
        marker="s",
        s=100,
        color="green",
        edgecolors="black",
        label="Charging stations",
        zorder=2
    )

    # Plot customers
    ax.scatter(
        customers["X"], customers["Y"],
        marker="o",
        s=60,
        color="royalblue",
        edgecolors="black",
        label="Customers",
        zorder=2
    )

    # Add node labels
    for _, row in df.iterrows():
        ax.annotate(
            row["StringID"],
            (row["X"], row["Y"]),
            xytext=(5, 5),
            textcoords="offset points",
            fontsize=8
        )

    # Formatting
    ax.set_xlabel("X Coordinate", fontsize=12)
    ax.set_ylabel("Y Coordinate", fontsize=12)
    ax.set_title("Electric Vehicle-Drone Routing Problem", fontsize=14)

    ax.legend(fontsize=10)
    ax.grid(True, linestyle="--", alpha=0.5)
    ax.set_aspect("equal", adjustable="box")

    plt.tight_layout()

    # Save figure if requested
    if save_fig:
        plt.savefig(output_file, dpi=300, bbox_inches="tight")

    plt.show()

