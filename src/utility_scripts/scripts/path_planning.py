import cv2
import numpy as np
import pdb
from dijkstar import Graph, find_path
import sklearn
import yaml

def imageToGraph(img, true_val = 255):
    """
    convert a binary representation to graphical representation
    """
    graph = Graph()
    num_rows = img.shape[0]
    num_cols = img.shape[1]
    img = np.pad(img, ((0, 1),))
    for i in range(num_rows):
        for j in range(num_cols):
            ind_current = i * num_cols + j
            ind_down    = (i + 1) * num_cols + j
            ind_right   = i * num_rows + j + 1
            
            current = img[i, j]
            down    = img[i + 1, j]
            right   = img[i, j + 1]

            if current == true_val and down == true_val:
                print("down: {} to {}".format(ind_current, ind_down))
                graph.add_edge(ind_current, ind_down, {'cost' : 1})

            if current == true_val and right == true_val:
                print("right: {} to {}".format(ind_current, ind_right))
                graph.add_edge(ind_current, ind_right, {'cost' : 1})

            #print((current, down, right))
    print(img.shape)
    print(graph)
    return graph


img = cv2.imread("../../../data/path_prob.png")
gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
binarized = (gray > 100).astype(np.uint8) * 255 
kernel = np.ones((5,5),np.uint8)
erosion = cv2.erode(binarized,kernel,iterations = 1)
dilation = cv2.dilate(erosion, kernel, iterations = 1)          
seedPoint = (img.shape[0], int(img.shape[1] / 2))
seedPoint = (500, 550)
mask = np.zeros_like(dilation)             
mask = np.pad(mask, (1,1))
flooded = cv2.floodFill(dilation, mask, seedPoint, 125)
flooded = (flooded[1]  == 125).astype(np.uint8) * 255            


with open("homography.yaml", 'r') as f:
    homography = np.asarray(yaml.load(f))

#flooded = cv2.warpPerspective(
#    flooded,
#    homography,
#    flooded.shape[:2])
#img = np.asarray([[1, 1, 1], [1, 1, 1], [1, 1, 1]])
graph = imageToGraph(flooded, 255)
pdb.set_trace()
print(graph)
#
cost_func = lambda u, v, e, prev_e: e['cost']
path = find_path(graph, img.shape[1] * 600 + 500, img.shape[1] * 160 + 999, cost_func=cost_func)
#path = find_path(graph, img.shape[1] * 0 + 0, img.shape[1] * 2 + 2, cost_func=cost_func)
print(path)

contours = cv2.findContours(flooded,cv2.RETR_TREE,cv2.CHAIN_APPROX_NONE)[-2]

output = np.zeros((flooded.shape[0], flooded.shape[1], 3), dtype=np.uint8)

cv2.imshow("", flooded)
#cv2.waitKey(20)

#sklearn.feature_extraction.image.img_to_graph(output)
for contour in contours:
   cv2.drawContours(output, contour, -1, (0, 255, 0), 3)

#for i in range(output.shape[0]):
#    print(i)
#    which_pixels = np.argwhere(output[i,:])
#    print(which_pixels.shape)
#    if which_pixels.shape[0] > 0:
#        max_pixel_location = np.max(which_pixels)
#        for j in range(max_pixel_location, 0, -1):
#            if np.isin(max_pixel_location, which_pixels):
#                output[i,j,:] = (0, 255, 0)
#            else:
#                break
#    else:
#        print(0)

#for contour in contours:
#    for i in range(output.shape[0]):
#        print(i)
#        print([x for x in contour if (x[0][1] == i)])


#cv2.imshow("", output)
#cv2.imwrite("outline.png", output)


cv2.waitKey(200000)
