import os
import pandas as pd
import matplotlib.pyplot as plt
from plot_scripts import *

# Plot font
plt.rcParams["font.family"] = "serif"
plt.rcParams["font.serif"] = ["Times New Roman"]
plt.rcParams["font.size"] = 16
plt.rc('legend', fontsize=14)  # using a size in points


# --- Step 1: Read File ----
if os.name == 'nt':     # Windows OS
    storage = 'D:/'
else:                   # Ubuntu OS
    storage = '/media/alejo/Elements/'

folder = 'Alejo - Apple Pick Data/Real Apple Picks/05 - 2023 fall (Prosser-WA)/Probe/'
file = '20231101_apples_coords.csv'

location = storage + folder + file

apples = pd.read_csv(location, index_col=0)
apples['Weight [g]'].fillna(apples['Weight [g]'].mean(), inplace=True)      # Fill Nan with mean values


# --- Step 2: ARRAY OF SUBPLOTS ---
fig, ax = plt.subplots(1, 3)
apples.boxplot('Diameter [mm]', ax=ax[0])
apples.boxplot('Height [mm]', ax=ax[1])
apples.boxplot('Weight [g]', ax=ax[2])

plt.subplots_adjust(wspace=0.75)
plt.show()
