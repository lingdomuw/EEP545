import numpy as np
from matplotlib import pyplot as plt

data=np.loadtxt('data.txt',delimiter=',')
print(data.shape)
x=data[:,0]
y=data[:,1]
plt.figure()
plt.plot(x,y)
plt.xlabel('Iteration Index')
plt.ylabel('Error')
plt.show()