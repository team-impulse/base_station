from mpl_toolkits.basemap import Basemap
import matplotlib.pyplot as plt
import matplotlib.pylab as pylab
import numpy as np
import string
import matplotlib.cm as cm
from drawnow import *
#NB
#ORIGINAL GRAPHING CODE AND CONCEPT (of using scatter graph) FROM https://stevendkay.wordpress.com/2010/02/24/plotting-points-on-an-openstreetmap-export/

x=[-9.379665] #longitudes
y=[39.120953] #latitudes

def fig():
    plt.subplot(1,1,1)
    m = Basemap(llcrnrlon=-9.394,llcrnrlat=39.101,urcrnrlon=-9.368,urcrnrlat=39.127,
                resolution='h',projection='merc')
    x1,y1=m(x,y)
    m.drawcoastlines()
    im = plt.imread("map.png")

    m.scatter(x1,y1,s=10,c='r',marker="x",cmap=cm.jet,alpha=1.0)
    plt.subplots_adjust(left=0.0, right=1.0, bottom=0.0, top=1.0)
    m.imshow(im, origin='upper')

    #plt.figure(num=1, figsize=(4.71210, 6), dpi=80, facecolor='w', edgecolor='k',Forw)

#http://stackoverflow.com/questions/875046/receiving-16-bit-integers-in-python

figure()
while(1):
    x.append(x[-1]+0.2)
    y.append(y[-1]+0.2)
    print x
    print y
    raw_input()
    drawnow(fig)
