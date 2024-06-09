import pandas as pd
import matplotlib.pyplot as plt
from plot_scripts import *


# Step 1: Read File
location = '/media/alejo/Elements/Alejo - Apple Pick Data/Real Apple Picks/05 - 2023 fall (Prosser-WA)/Probe/'
file = '20231101_apples_coords.csv'
apples = pd.read_csv(location + file, index_col=0)
apples['Weight [g]'].fillna(apples['Weight [g]'].mean(), inplace=True)      # Fill Nan with mean values

plt.rcParams["font.family"] = "sans-serif"
plt.rcParams["font.serif"] = ["Times New Roman"]
plt.rcParams["font.size"] = 14

# --- ARRAY OF SUBPLOTS ---
fig, ax = plt.subplots(1, 3)
apples.boxplot('Diameter [mm]', ax=ax[0])
apples.boxplot('Height [mm]', ax=ax[1])
apples.boxplot('Weight [g]', ax=ax[2])

plt.subplots_adjust(wspace=0.5)
plt.show()


