import numpy as np
import bezier
import pdb
import matplotlib.pyplot as plt
import seaborn
import cv2


img = cv2.imread("/home/david/Downloads/path.png")
row,col,depth = img.shape
green = img == np.asarray([[[0, 0, 255]]])
green = np.all(green, axis=2)
print(np.sum(green))
indices = np.where(green)

#nodes1 = np.asfortranarray([
#indices[0][35:], indices[1][35:]
#], dtype=np.double)
#print(indices)
nodes1 = np.asfortranarray([
    [0.0, 0.5, 1.0, 1.5, 2.0],
    [0.0, 1.0, 0.0, 1.0, 0.0],
])
curve1 = bezier.Curve(nodes1, degree=100)

#plt.plot(nodes1[1,:], nodes1[0,:])
#for i in np.arange(0, 1, 0.05):
#    print(i)
#    point = curve1.evaluate(i)
#    plt.scatter(point[0], point[1])
#plt.show()

nodes2 = np.asfortranarray([
    [0.0, 0.25,  0.5, 0.75, 1.0],
    [0.0, 2.0 , -2.0, 2.0 , 0.0],
])

pdb.set_trace()
curve2 = bezier.Curve.from_nodes(nodes2)
intersections = curve1.intersect(curve2)
s_vals = np.asfortranarray(intersections[0, :])
points = curve1.evaluate_multi(s_vals)

seaborn.set()

ax = curve1.plot(num_pts=256)
curve2.plot(num_pts=256, ax=ax)
lines = ax.plot(
    points[0, :], points[1, :],
    marker="o", linestyle="None", color="black")

ax.axis("scaled")
ax.set_xlim(-0.125, 1.125)
ax.set_ylim(-0.0625, 0.625)
plt.show()
