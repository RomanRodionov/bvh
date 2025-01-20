import numpy as np
import matplotlib.pyplot as plt
from scipy.signal import convolve2d

from bvh import BVH


def cut_edges(img):    
    edge = np.array([[0, 1, 0], [1, -4, 1], [0, 1, 0]])
    edge_img = np.maximum(convolve2d(img, edge, mode='same'), 0) > 0

    mixed = (1 - edge_img) * img
    return mixed


loader = BVH()
loader.load_scene("suzanne2.fbx")
loader.build_bvh(15)

resolution = 1000

origin = np.array([-1, -5, 0])
pixels = np.meshgrid(np.linspace(-1, 1, resolution), np.linspace(1, -1, resolution))
pixels = np.array(pixels).reshape(2, -1).T * 1.5
pixels = np.hstack((
    pixels[:, 0:1],
    np.zeros((pixels.shape[0], 1)),
    pixels[:, 1:2],
))

origins = np.tile(origin, (pixels.shape[0], 1))
directions = pixels - origins

mask, leaf_indices, t1, t2 = loader.intersect_leaves(origins, directions)

img = t1.reshape(resolution, resolution)
mask = mask.reshape(resolution, resolution)
# img[mask] = 0
img = (img - np.min(img)) / (np.max(img) - np.min(img))
img = img * 0.8
img = cut_edges(img)

# img[mask] = 1
img[~mask] = 1

# img = 1 - img

plt.axis('off')

plt.imshow(img, cmap='gray')
plt.tight_layout()
plt.savefig('suzanne2.png')
plt.show()
