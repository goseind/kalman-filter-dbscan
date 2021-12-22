from sklearn.cluster import DBSCAN

class custom_dbscan:
    def scan(input):
        model = DBSCAN()
        model.fit_predict(input)
        pred = model.fit_predict(input)
        return pred

