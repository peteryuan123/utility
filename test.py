import numpy as np


t0 = np.array([0, 0, 0]).reshape(3, 1)
R0 = np.eye(3)

t1 = np.array([0, 2, 0]).reshape(3, 1)
R1 = np.array([[np.cos(np.pi / 12), np.sin(np.pi / 12), 0], \
              [-np.sin(np.pi / 12), np.cos(np.pi / 12), 0], \
              [0, 0, 1]])

p0 = np.array([[1, 0, 0],
              [1, 1, 0],
              [0, 2, 1]]).T
p1 = R1.T @ (p0 - t1)

print(p0)
print(p1)
p0 = p0.T
p1 = p1.T

print(p0)
print(p1)
W = np.zeros((3, 3))
for i in range(3):
    print(p0[i])
    print(p1[i])
    W += p0[i] @ p1[i].T

U,sigma,VT = np.linalg.svd(W)
R = U @ VT
print(R)

