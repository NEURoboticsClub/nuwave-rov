import os
import numpy as np
import cv2
import matplotlib.pyplot as plt
import pandas as pd
from sklearn import preprocessing, neighbors
import sklearn.model_selection
import tensorflow as tf

categories=["anushka sharma","katrina kaif"]
data=[]#all images as numpy arrays
lables=[]#lables of data in order
data_dir="/Users/anka/Desktop/AI"
for category in categories: 
    path = os.path.join(data_dir, category)   
    for img in os.listdir(path):
        try:
            img_array = cv2.imread(os.path.join(path, img), cv2.IMREAD_GRAYSCALE)
            image_size = 80
            new_array = cv2.resize(img_array, (image_size, image_size))
            data.append(new_array)
            lables.append(category) 
            #plt.imshow(new_array, cmap = "gray")
            #plt.show()
        except:
            print("oops",img)
            pass
data=np.array(data)
data = data.reshape(len(data), -1)
      
Xtrain, Xtest, Ytrain, Ytest = sklearn.model_selection.train_test_split(data, lables, test_size = 0.3)
a=tf.placeholder(tf.float32,[None,6400])
b=tf.placeholder(tf.float32,[6400])
accuracy=0
distance = tf.reduce_sum(tf.abs(tf.add(a, tf.negative(b))), reduction_indices=1)
pred = tf.arg_min(distance, 0) #smallest distance, 0 means row
init = tf.initialize_all_variables()
with tf.Session() as sess:
    sess.run(init)
    for i in range(len(Xtest)):
        nn_index = sess.run(pred, feed_dict={a: Xtrain, b: Xtest[i]}) #index of nearest niegbor 
        #print("Sample", i, " - Prediction:", Ytrain[nn_index], " / True Class:", Ytest[i])
        if Ytrain[nn_index]!=Ytest[i]:
            print("wrong")
        if Ytrain[nn_index] == Ytest[i]:
            accuracy += 1. / len(Xtest)
    print("Accuracy:", accuracy)
    test_array=cv2.imread("test_img.jpg", cv2.IMREAD_GRAYSCALE)
    test_array = cv2.resize(test_array, (80, 80))
    test_array = test_array.reshape(6400)
    nn_index_test = sess.run(pred, feed_dict={a: Xtrain, b: test_array})
    print (Ytrain[nn_index_test])
