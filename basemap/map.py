from mpl_toolkits.basemap import Basemap
import matplotlib.pyplot as plt
import matplotlib.pylab as pylab
import numpy as np
import string
import matplotlib.cm as cm
import time
from drawnow import *
from pylab import *
import random
#NB
#ORIGINAL GRAPHING CODE AND CONCEPT (of using scatter graph) FROM https://stevendkay.wordpress.com/2010/02/24/plotting-points-on-an-openstreetmap-export/
xarray = []
yarray = []
x=-9.379667 #longitudes
y=39.120951 #latitudes
plt.ion()
def makeFigs():
    m = Basemap(llcrnrlon=-9.394,llcrnrlat=39.101,urcrnrlon=-9.368,urcrnrlat=39.127,
     resolution='l',projection='merc')
    x1,y1=m(x,y)
    xarray.append(x1)
    yarray.append(y1)
    m.drawcoastlines()
    im = plt.imread("map.png")
    m.imshow(im, origin='upper')
    m.scatter(xarray,yarray,c='b',marker=".",alpha=1.0)
while(1):
    #X AND Y ARE THE RECEIVED DATA FROM THE GPS SENSOR
    #for now they have just been set to random numbers between the limits
    x = random.uniform(-9.394,-9.368)
    y = random.uniform(39.101, 39.127)
    drawnow(makeFigs)
    time.sleep(0.1)
    #SYSTEM TO START DUMPING OLD DATA - might be completely useless?
    threshold = 30 #maximum pieces of data
    if len(xarray) > threshold:
        xarray.pop(0)
        yarray.pop(0)
