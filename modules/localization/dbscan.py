import os
import sys
import numpy as np

from sklearn.cluster import DBSCAN
from sklearn.preprocessing import StandardScaler

_root_path = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.append(_root_path)

from modules.localization.localization import Localization


class dbscan(Localization):
    def __init__(self, eps, min_samples):
        self.operator = DBSCAN(eps=eps, min_samples=min_samples)

    def display(self, point_cloud, **kwargs):
        standardized_pcl = StandardScaler().fit_transform(point_cloud)
        db = self.operator.fit(standardized_pcl)
        core_samples_mask = np.zeros_like(db.labels_, dtype=bool)
        core_samples_mask[db.core_sample_indices_] = True
        labels = db.labels_

        # find objects center
        cluster_centers = []

        for cluster in set(db.labels_):
            if cluster != -1:
                running_sum = np.array([0.0, 0.0, 0.0])
                counter = 0

                for i in range(standardized_pcl.shape[0]):
                    if labels[i] == cluster:
                        running_sum += standardized_pcl[i]
                        counter += 1

                center = running_sum / counter
                cluster_centers.append(center)

        return None, labels, cluster_centers
