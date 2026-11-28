from mapper import mapper
from data.data_gen import data_gen
from model import model
from parameters import objective, instance, env

def main():
  input = data_gen(20,5, depot_location=(0, 0), wind=True, Temp=25.0, Humidity=0.5, Traffic=0.0, area_size=(50, 50))
  model = model(input, objective, instance, env)
  model.solve(time_limit=60)  # Set a time limit of 60 seconds for the solver
  return

def data_visualization(input_file: str):
  mapper(input_file, save_fig=True)
  return

if __name__ == '__main__':
  
  synth_data = data_gen(20,5, depot_location=(0, 0), wind=True, Temp=25.0, Humidity=0.5, Traffic=0.0, area_size=(50, 50))
  data_visualization(synth_data)
  
  main()

  #print("Current directory:", os.getcwd())
  #print("File exists:", os.path.exists("data.txt"))
