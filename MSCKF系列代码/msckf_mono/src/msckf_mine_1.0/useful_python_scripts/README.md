### some useful python scripts
---
	parase_csv2txt.py --> cahnge the .csv to .txt


```python
import numpy as np
imu_path = '../datasets/MH_01_easy/mav0/imu0/data.csv'
imu_data = np.array(np.loadtxt(imu_path, dtype=np.str, comments='#', delimiter=','), dtype=str)
print imu_data
np.savetxt('data.txt', imu_data, fmt='%s')
```

---