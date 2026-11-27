from mapper import mapper_inni
from data.data_gen import data_gen


def main():
  print('hello world')
  return

if __name__ == '__main__':

  main()
  synth_data = data_gen(10,5, depot_location=(0, 0), wind=True, Temp=25.0, Humidity=60.0, Traffic=0.0, area_size=(100, 100))
  mapper_inni(synth_data, save_fig=True)

  #print("Current directory:", os.getcwd())
  #print("File exists:", os.path.exists("data.txt"))
