import pandas as pd
import matplotlib as mpl
import matplotlib.pyplot as plt
import seaborn as sns

# Adjusting the data for custom start and finish times
data_speech = {
    "Task": ["Task 1", "Task 2", "Task 3"],
    "Start": [0, 300, 700],
    "Duration": [300, 400, 800]
}

data_gestures = [
    {
        "Task": "Say command",
        "Start": 0,
        "Duration": 200,
    },
    {
        "Task": "Recognized command",
        "Start": 0,
        "Duration": 300,
    },
    {
        "Task": "Send ROS message",
        "Start": 300,
        "Duration": 200,
    },
    {
        "Task": "Execute movement",
        "Start": 500,
        "Duration": 3000,
    },
    {
        "Task": "Start timer for beep",
        "Start": 500,
        "Duration": 4000,
    },

]

# df = pd.DataFrame(data_speech)
df = pd.DataFrame(data_gestures)
print(df)
df = df.iloc[::-1]
print(df)

# Creating a figure
plt.figure(figsize=(10, 3))

cmap =  mpl.cm.get_cmap("tab20c")

# Plotting each task
for i, row in df.iterrows():
    plt.barh(row["Task"], row["Duration"], left=row["Start"], color=cmap(i/20))

    # Adding task label
    # plt.text(row["Start"] + row["Duration"] / 2, i, f'{row["Task"]}', color='black', ha="center", va="center")

# Setting labels and title
plt.xlabel("Time [ms]")
plt.ylabel("Tasks")
plt.title("Gantt Chart with Custom Start and Finish Times")

# Adjusting the x-axis limits
# plt.xlim(0, (df["Duration"].values[0] + df["Start"].values[0])+200)

# Formatting x-axis to show milliseconds
plt.gca().xaxis.set_major_formatter(plt.FuncFormatter(lambda x, _: f'{int(x)} ms'))

plt.tight_layout()
plt.show()