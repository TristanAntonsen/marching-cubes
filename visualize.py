import matplotlib.pyplot as plt
from stl import mesh
import pandas as pd
import sys

path = sys.argv[1]

thresh = 0

voxels = pd.read_csv(path,names=['x','y','z','value'])

def grayscale(value, threshold):
    if value <= threshold:
        return 0
    else:
        return 1

voxels['c'] = voxels.apply(lambda row: grayscale(row['value'],thresh), axis=1)
voxels = voxels[voxels['value'] <= 0]

fig = plt.figure(figsize=(6, 6))
ax = fig.add_subplot(projection='3d')

ax.scatter(voxels.x, voxels.y, voxels.z,c=voxels.value,cmap='viridis',s=2)
span = 25

ax.set_box_aspect((1,1,1))  # aspect ratio is 1:1:1
ax.set_xlim([-span, span])
ax.set_ylim([-span, span])
ax.set_zlim([-span, span])
plt.xlabel("X")
plt.ylabel("Y")
plt.show()