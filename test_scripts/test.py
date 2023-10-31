from datetime import datetime
import csv

date_start = datetime.now().strftime("%Y-%m-%dT%H%M%S")
csv_filename = f"C:\\dev\\SpotDockerImage\\data\deictic_movements\\deictic_data_{date_start}.csv"
with open(csv_filename , 'w', newline='') as file:
    writer = csv.writer(file)
    field = ["x_person", "y_person", "x_object", "y_object", "x_robot", "y_robot", "x_robot_new", "y_robot_new"]
    writer.writerow(field)
    file.close()


data_to_save = [1, 2, 3, 4, 5, 6, 7, 8]
with open(csv_filename , 'a', newline='') as file:
    writer = csv.writer(file)
    writer.writerow(data_to_save)
    file.close()
        