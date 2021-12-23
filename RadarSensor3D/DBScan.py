'''
How does DBSCAN work: https://en.wikipedia.org/wiki/DBSCAN
'''
# deque provides an O(1) time complexity for append and pop operations instead of O(n) for lists.
from collections import deque
# dataset to toy around with.
from sklearn.datasets import make_moons

def pairwise_sq_distance(X1, X2):
    # Calculate the pairwise distance between all pairs of points from X1 and X2.
    return np.sum(X1**2, axis=1, keepdims=True) - 2*np.matmul(X1, X2.T) + np.sum(X2**2, axis=1, keepdims=True).T

class DBSCAN():
    
    def __init__(self, eps=0.5, minpts=5):
        self.eps = eps
        self.minpts = minpts
        
    def fit(self, X):
        dist = pairwise_sq_distance(X, X)
        neighbours = list(map(lambda d: np.arange(d.shape[0])[d < self.eps**2], dist))
        
        # Label all points as outliers initially.
        self.assignment = np.full((X.shape[0],), -1, dtype=np.int)
        # Find core points.
        ## Determine the number of neighbors of each point.
        N_neighbors = np.sum(dist < self.eps**2, axis=1)
        self.assignment[N_neighbors >= self.minpts] = -2
        
        # Create clusters.
        cluster = 0
        stack = deque()
        for p in range(X.shape[0]):
            if self.assignment[p] != -2:
                continue
                
            self.assignment[p] = cluster
            
            stack.extend(neighbours[p])
            # Expand cluster outwards. 
            while len(stack) > 0:
                n = stack.pop()
                label = self.assignment[n]
                # If core point include all points in ε-neighborhood.
                if label == -2:
                    stack.extend(neighbours[n])
                # If not core point (edge of cluster).
                if label < 0:
                    self.assignment[n] = cluster
            
            cluster += 1
        
    def predict(self, X):
        self.fit(X)
        return self.assignment
    
    def predict(self,X):
        return self.assignment
    
if __name__ == '__main__':
    X,y = make_moons(100)
    model = DBSCAN()
    preds = model.fit_predict(X)
    # Either low or high values are good since DBSCAN might switch class labels.
    print(f"Accuracy: {round((sum(preds == y)/len(preds))*100,2)}%")
