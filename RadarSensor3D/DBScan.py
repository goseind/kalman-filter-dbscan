from sklearn.datasets import make_moons
from matplotlib import pyplot as plt

X, y = make_moons(n_samples=500, noise=0.1, random_state=10)

plt.plot(X[:, 0][y==1], X[:, 1][y==1], "bs")
plt.plot(X[:, 0][y==0], X[:, 1][y==0], "bs")

plt.xlabel(r"$x_1$", fontsize=20)
plt.ylabel(r"$x_2$", fontsize=20)

plt.show()