import pandas as pd
import matplotlib.pyplot as plt


# todos
# Hardware: * Make it wireless
#           * Place battery
# Software: * Run it with RVIZ

# open file
df = pd.read_csv('/home/alejo/cutecom.log', sep='\t')
df = df[:-1]    #last row has some missing readings, so get rid of it

print(df.iloc[:, 0])

gyr_x = df.iloc[:, 1].astype(int)
gyr_y = df.iloc[:, 2].astype(int)
gyr_z = df.iloc[:, 3].astype(int)

lin_x = df.iloc[:, 5].astype(int)
lin_y = df.iloc[:, 6].astype(int)
lin_z = df.iloc[:, 7].astype(int)

mag_x = df.iloc[:, 9].astype(int)


acc_x = df.iloc[:, 13].astype(int)
acc_y = df.iloc[:, 14].astype(int)
acc_z = df.iloc[:, 15].astype(int)


gra_x = df.iloc[:, 17].astype(int)

plt.plot(gyr_x, label='gyro_X')
plt.plot(gyr_y, label='gyro_Y')
plt.plot(gyr_z, label='gyro_Z')
plt.legend()

fig = plt.figure()
plt.plot(lin_x, label='lin_X')
plt.plot(lin_y, label='lin_Y')
plt.plot(lin_z, label='lin_Z')
plt.legend()

fig = plt.figure()
plt.plot(acc_x, label='acc_X')
plt.plot(acc_y, label='acc_Y')
plt.plot(acc_z, label='acc_Z')
plt.legend()


plt.show()
