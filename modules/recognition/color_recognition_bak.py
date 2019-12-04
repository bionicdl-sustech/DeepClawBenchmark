

class color_recognition(Recognition):
    def __init__(self):
        self.size = 10

    def display(self, centers, color_image, **kwargs):
        pieces_image = []
        if len(centers)!=0:
            for center in centers:
                pieces_image.append(color_image[center[1]-self.size:center[1]+self.size, 
                                    center[0]-self.size:center[0]+self.size, :])

    def label_predice(self, piece):
        mr, mb, mg = np.mean(piece[:, :, 0]), np.mean(piece[:, :, 1]), np.mean(piece[:, :, 2])
