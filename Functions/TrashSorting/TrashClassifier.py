import numpy as np
import cv2, imutils
from Driver.Camera.RealsenseController import RealsenseController
from keras.models import load_model
from tensorflow.keras.preprocessing.image import ImageDataGenerator

# Load trained model
# model = load_model('./Functions/TrashSorting/BionicTrashnet-Aug-weights.20-0.96-DenseNet169.hdf5')
model = load_model('./Functions/TrashSorting/BionicTrashnet-Aug11-weights.20-0.957-DenseNet169.hdf5')
# model = load_model('./Functions/TrashSorting/BionicTrashnet-Aug2-weights.19-0.96-DenseNet169.hdf5')
# model = load_model('./Functions/TrashSorting/BionicTrashnet-Aug3-weights.28-0.97-DenseNet169.hdf5')
# model_33 = load_model('./Functions/TrashSorting/BionicTrashnet-Aug33-weights.20-0.954-DenseNet169.hdf5')

class FixedImageDataGenerator(ImageDataGenerator):
    def standardize(self, x):
        if self.samplewise_center:
            if len(x)==4:
                x -= np.mean(x, axis=(0,1,2), keepdims=True)
            if len(x)==3:
                x -= np.mean(x, axis=(0,1), keepdims=True)
        return x

test = ImageDataGenerator(samplewise_center=True)
test_f = FixedImageDataGenerator(samplewise_center=True)

# Function to locate trash and return cropped image of 300x300
def locate_trash(image_color):
    crop_box = [150,850,750,1300] # UR10e, crop the trash bin work space
    image_ws = image_color[crop_box[0]:crop_box[1],crop_box[2]:crop_box[3]]
    gray = cv2.cvtColor(image_ws, cv2.COLOR_BGR2GRAY)
    blurred = cv2.GaussianBlur(gray, (3, 3), 0)
    edges = cv2.Canny(blurred, 30, 200)
    cnts = cv2.findContours(edges.copy(), cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    cnts = imutils.grab_contours(cnts)
    cnts_sorted = sorted(cnts, key = cv2.contourArea, reverse = True)
    if len(cnts_sorted) ==0:
        return
    cnts_array = cnts_sorted[0]
    for cnt in cnts_sorted[1:]: #[n,1,2] (u,v)
        if cnt.shape[0]<30:
            continue
        cnts_array = np.concatenate([cnts_array,cnt])
    u_min = np.min(cnts_array[:,0,0])
    u_max = np.max(cnts_array[:,0,0])
    v_min = np.min(cnts_array[:,0,1])
    v_max = np.max(cnts_array[:,0,1])
    u_middle_ori = (u_min + u_max)/2 + crop_box[2]
    v_middle_ori = (v_min + v_max)/2 + crop_box[0]
    image = image_color[v_middle_ori-150:v_middle_ori+150, u_middle_ori-150:u_middle_ori+150].copy()
    return image, [u_middle_ori, v_middle_ori]

# Function to choose image and predict its class
def predict(image):
    image_rgb = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
    image_rgb = cv2.resize(image_rgb,(224, 224)).reshape((1, 224, 224, 3)).astype("float64")
    image_rgb_s = test.standardize(image_rgb.copy())
    predictions = model.predict(image_rgb_s)
    # image_rgb_s = test_f.standardize(image_rgb.copy())
    # predictions = model_33.predict(image_rgb_s)
    predictions = np.around(predictions,decimals = 2)
    return np.argmax(predictions)

# Function for plotting result of classification
def video_demo_localize():
    while True:
        image_color, depth_images = camera.getImage()
        # crop_box = [200,850,500,1050] # Franka
        crop_box = [150,850,750,1300] # UR10e
        image_ws = image_color[crop_box[0]:crop_box[1],crop_box[2]:crop_box[3]]
        gray = cv2.cvtColor(image_ws, cv2.COLOR_BGR2GRAY)
        blurred = cv2.GaussianBlur(gray, (3, 3), 0)
        edges = cv2.Canny(blurred, 30, 200)
        cnts = cv2.findContours(edges.copy(), cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        cnts = imutils.grab_contours(cnts)
        cnts_sorted = sorted(cnts, key = cv2.contourArea, reverse = True)
        if len(cnts_sorted) ==0:
            continue
        cnts_array = cnts_sorted[0]
        for cnt in cnts_sorted[1:]: #[n,1,2] (u,v)
            if cnt.shape[0]<30:
                continue
            cnts_array = np.concatenate([cnts_array,cnt])
        u_min = np.min(cnts_array[:,0,0])
        u_max = np.max(cnts_array[:,0,0])
        v_min = np.min(cnts_array[:,0,1])
        v_max = np.max(cnts_array[:,0,1])
        u_middle_ori = (u_min + u_max)/2 + crop_box[2]
        v_middle_ori = (v_min + v_max)/2 + crop_box[0]
        box_ori = [[u_middle_ori-150,v_middle_ori+150],[u_middle_ori-150,v_middle_ori-150],[u_middle_ori+150,v_middle_ori-150],[u_middle_ori+150,v_middle_ori+150]]
        gamebox = np.int0(box_ori)
        image = image_color[v_middle_ori-150:v_middle_ori+150, u_middle_ori-150:u_middle_ori+150].copy()
        ret = cv2.drawContours(image_color, [gamebox], 0, (0, 0, 255), 2)
        image_rgb = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
        image_rgb = cv2.resize(image_rgb,(224, 224)).reshape((1, 224, 224, 3)).astype("float64")
        image_rgb_s = test.standardize(image_rgb.copy())
        predictions = model.predict(image_rgb_s)
        # image_rgb_s = test_f.standardize(image_rgb.copy())
        # predictions = model_33.predict(image_rgb_s)
        predictions = np.around(predictions[0],decimals = 2)
        idx = np.argmax(predictions)
        labels = ['glass', 'metal', 'paper', 'plastic']
        info = "%s: %.2f,%.2f,%.2f,%.2f" %(labels[idx],predictions[0],predictions[1],predictions[2],predictions[3])
        ret = cv2.putText(image_color, text=info, org=(50, 70), fontFace=cv2.FONT_HERSHEY_SIMPLEX,
                    fontScale=3, color=(0, 0, 255), thickness=3)
        cv2.namedWindow("result", cv2.WINDOW_AUTOSIZE)
        cv2.imshow("result", image_color)
        if cv2.waitKey(1) & 0xFF == ord('q'): break
    cv2.destroyAllWindows()

if __name__ == "__main__":
    camera = RealsenseController(1920,1080)

    # Real-time trash sorting demo
    video_demo_localize()

    # Example usage
    # image_color, depth_images = camera.getImage()
    # image = locate_trash(image_color)
    # label = predict(image)
