import cv2
import numpy as np

#!/usr/bin/env python
"""Segmentation skeleton code for Lab 6
Course: EECS C106A, Fall 2019
Author: Grant Wang

This Python file is the skeleton code for Lab 3. You are expected to fill in
the body of the incomplete functions below to complete the lab. The 'test_..'
functions are already defined for you for allowing you to check your 
implementations.

When you believe you have completed implementations of all the incompeleted
functions, you can test your code by running python segmentation.py at the
command line and step through test images
"""

import os
import numpy as np
import numpy.ma as ma
import cv2
import matplotlib.pyplot as plt

from scipy import ndimage
from scipy.misc import imresize
from skimage import filters
from sklearn.cluster import KMeans
from sklearn.decomposition import PCA

from skimage.measure import block_reduce
import time
import pdb

this_file = os.path.dirname(os.path.abspath(__file__))
IMG_DIR = '/'.join(this_file.split('/')[:-2]) + '/img'

def read_image(img_name, grayscale=False):
    """ reads an image

    Parameters
    ----------
    img_name : str
        name of image
    grayscale : boolean
        true if image is in grayscale, false o/w
    
    Returns
    -------
    ndarray
        an array representing the image read (w/ extension)
    """

    if not grayscale:
        img = cv2.imread(img_name)
    else:
        img = cv2.imread(img_name, 0)

    # print(img.shape)
    return img

def write_image(img, img_name):
    """writes the image as a file
    
    Parameters
    ----------
    img : ndarray
        an array representing an image
    img_name : str
        name of file to write as (make sure to put extension)
    """

    cv2.imwrite(img_name, img)

def show_image(img_name, title='Fig', grayscale=False):
    """show the  as a matplotlib figure
    
    Parameters
    ----------
    img_name : str
        name of image
    tile : str
        title to give the figure shown
    grayscale : boolean
        true if image is in grayscale, false o/w
    """

    if not grayscale:
        plt.imshow(img_name)
        plt.title(title)
        plt.show()
    else:
        plt.imshow(img_name, cmap='gray')
        plt.title(title)
        plt.show()


def cluster_segment(img, n_clusters, random_state=0):
    """segment image using k_means clustering

    Parameter
    ---------
    img : ndarray
        rgb image array
    n_clusters : int
        the number of clusters to form as well as the number of centroids to generate
    random_state : int
        determines random number generation for centroid initialization

    Returns
    -------
    ndarray
        clusters of gray_img represented with similar pixel values
    """

    # Downsample img first using the mean to speed up K-means
    print(img.shape)
    img_d = block_reduce(img, block_size=(2, 2, 1), func=np.mean)

    # TODO: Generate a clustered image using K-means

    # first convert our 3-dimensional img_d array to a 2-dimensional array
    # whose shape will be (length * width, number of channels) hint: use img_d.shape
    shape = img_d.shape
    print(shape)
    img_r = np.reshape(img_d, (shape[0] * shape[1], shape[2]))
    print(img_r.shape)
    # fit the k-means algorithm on this reshaped array img_r using the
    # the scikit-learn k-means class and fit function
    # see https://scikit-learn.org/stable/modules/generated/sklearn.cluster.KMeans.html
    # the only parameters you have to worry about are n_clusters and random_state
    kmeans = KMeans(n_clusters=n_clusters, random_state=random_state).fit(img_r)

    # get the labeled cluster image using kmeans.labels_
    clusters = kmeans.labels_

    # reshape this clustered image to the original downsampled image (img_d) shape
    cluster_img = np.reshape(clusters, (shape[0], shape[1]))
    print(cluster_img)
    print(cluster_img.shape)
    # Upsample the image back to the original image (img) using nearest interpolation
    img_u = imresize(cluster_img, (img.shape[0], img.shape[1], img.shape[2]), interp='nearest')
    print(img_u.shape)

    return img_u.astype(np.uint8)


def test_cluster(img, n_clusters):
    clusters = cluster_segment(img, n_clusters).astype(np.uint8)
    # print(clusters) 
    cv2.imwrite(IMG_DIR + "/cluster.jpg", clusters)
    clusters = cv2.imread(IMG_DIR + '/cluster.jpg')
    show_image(clusters, title='cluster')

def segment():
    test_img_color = read_image(IMG_DIR + '/cv_image.jpg')     #Just doin some clustering
    clusters = cluster_segment(test_img_color, 2).astype(np.uint8)   #Clusters should be matrix of size image with 0's and 1's (we want cluster 1)
    return clusters, test_img_color
