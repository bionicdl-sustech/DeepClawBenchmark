import keras
import numpy as np
import matplotlib.pyplot as plt

from keras.preprocessing.image import img_to_array, load_img
from keras.models import load_model
from keras import backend as K
from keras.models import Model
from keras.preprocessing.image import ImageDataGenerator

from tkinter import *
from tkinter import filedialog
from tkinter import font

from PIL import Image

# Load model
model = load_model('GUI_model_DenseNet169.h5')
opt_adam = keras.optimizers.Adam(lr=0.0001, amsgrad=True)
model.compile(loss='categorical_crossentropy',
              optimizer = opt_adam,
              metrics=['accuracy'])

# Function to choose image and predict its class
def predict():
    root.filename = filedialog.askopenfilename(initialdir = "", title = "Select trash:")
    if root.filename :
        img = Image.open(root.filename)
        img = img.resize((224, 224))
        arr = np.array((img_to_array(img)),)
        arr = arr.reshape((1, 224, 224, 3))
        predictions = model.predict(arr)
        predictions = np.around(predictions,decimals = 2)
        plot(predictions)
        print(predictions)
    return

# Function for plotting result of classification
def plot(predictions):
    fig, ax = plt.subplots()
    ind = np.arange(1, 18)
    prediction = predictions.ravel()
    ca = plt.bar(1, prediction[0])
    gl = plt.bar(2, prediction[1])
    me = plt.bar(3, prediction[2])
    pa = plt.bar(4, prediction[3])
    pl = plt.bar(5, prediction[4])
    ot = plt.bar(6, prediction[5])
    ax.set_xticks(ind)
    ax.set_xticklabels(['cardboard', 'glass', 'metal', 'paper', 'plastic', 'other'])
    ax.set_ylim([0, 1])
    
    plt.tight_layout()
    plt.show(block = False)

# GUI
root = Tk()
root.geometry('500x200')
root.title("Trash Material Classifier")
font = font.Font(family='tahoma', size='15')
var = StringVar()
label = Label( root, textvariable=var, relief=RAISED, bd='0' , height=5, font=font)
var.set("Choose a file to make a prediction.")
label.pack()
button = Button(root, text=u"Choose File", command=predict, width = 15, height = 5)
button.pack(side = "top", expand = True)
root.mainloop()