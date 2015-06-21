from mpl_toolkits.basemap import Basemap
import matplotlib.pyplot as plt
import matplotlib.pylab as pylab
import numpy as np
import string
import matplotlib.cm as cm
#NB
#ORIGINAL GRAPHING CODE AND CONCEPT (of using scatter graph) FROM https://stevendkay.wordpress.com/2010/02/24/plotting-points-on-an-openstreetmap-export/

x=[-9.379665] #longitudes
y=[39.120953] #latitudes


m = Basemap(llcrnrlon=-9.394,llcrnrlat=39.101,urcrnrlon=-9.368,urcrnrlat=39.127,
            resolution='h',projection='merc')
x1,y1=m(x,y)
m.drawcoastlines()
im = plt.imread("map.png")
m.imshow(im, origin='upper')
m.scatter(x1,y1,s=10,c='r',marker="o",cmap=cm.jet,alpha=1.0)
plt.subplots_adjust(left=0.0, right=1.0, bottom=0.0, top=1.0)
plt.show()
#plt.figure(num=1, figsize=(4.71210, 6), dpi=80, facecolor='w', edgecolor='k',Forw)
