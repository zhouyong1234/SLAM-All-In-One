import numpy as np
imu_path = '../datasets/V2_01_easy/mav0/cam1/data.csv'
imu_data = np.array(np.loadtxt(imu_path, dtype=np.str, comments='#', delimiter=','), dtype=str)
print imu_data
np.savetxt('data.txt', imu_data, fmt='%s')
