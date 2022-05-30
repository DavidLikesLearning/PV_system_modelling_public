import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from mpl_toolkits.mplot3d import Axes3D
from mpl_toolkits.mplot3d.art3d import Poly3DCollection 

class KmlTri:
    def __init__(self, file):
        
        ''' produces a (Number of triangles,3,3) array where each value is a coordinate 
         (lat, lon or alt). Each array is a coordinate triple. Each
         inner list is a triangle on the panels in question.'''
        
        with open(file, 'rt', encoding="utf-8") as myfile:
            doc=myfile.read()
        i = 0
        coords_start = []
        negatives = []
        coords_end = []
        while doc.find('<coord',i)>0:
            indx = doc.find('<coord',i)
            neg = doc.find('-',indx)
            indx_out = doc.find('</coord',i)
            coords_start.append(indx)
            coords_end.append(indx_out)
            negatives.append(neg)
            i=indx_out+1
        out = []
        for j in range(len(coords_start)):
            neg = negatives[j]
            end = coords_end[j]
            triangles = doc[neg:end].split('\n')[0].strip().split(' ')
            triangle = []
            for i in triangles[:-1]:
                triangle.append(np.fromstring(i,dtype=np.float64,sep=','))
            out.append(triangle)
        #storing the three triangles
        self.tris = np.asarray(out)
    
        avgcos= np.cos(np.deg2rad(np.mean(self.tris[:,:,1])))
        cartmult = np.array([111195*avgcos,111195,1])
        meters = np.multiply(self.tris,cartmult)
        #storing the three triangles in meters
        self.meters = meters
    
        vs = self.meters[:,0,:] - self.meters[:,1,:]
        us = self.meters[:,0,:] - self.meters[:,2,:]
        normsm = np.cross(vs,us,axisa=1,axisb=1)
        neg = (normsm[:,2]>0).astype(int)-1
        negM = np.tile(neg,(3,1)).T
        fix = negM + (negM+1)
        norms = normsm*fix
        #storing the normal vectors
        self.norms = norms
        
        ''' taking the l2 norm of the x and y to be r and the tilt is arctan(r/z)
        since the tilt of the normal vector is arctan(z/r)'''
        tilts = np.rad2deg(np.arctan(np.linalg.norm(norms[:,:2],axis=1)/norms[:,2]))
        #storing the tilts
        self.tilts = tilts
        
        ''' taking the arctan2 np function of the normal vectors and adjusting
        for azimuth angles being measured between 0 and 359 clockwise from vector
        [0,1], whereas arctan2 is measured bewteen -180 and 180 counterclockwise 
        from vector [1,0] '''
        raw_angles = np.rad2deg(np.arctan2(self.norms[:,1],self.norms[:,0]))
        quad2 = (raw_angles>90).astype(int)
        #to shift to azimuth from traditional angle notation, (90 - angle) works 
        #for quadrants 1,3 and 4. For quadrant 2, (450 - angle) works.
        azimuths = quad2*(450-raw_angles) + (-1*quad2+1)*(90-raw_angles)
        self.azimuths = azimuths
        
    
    def triplot(self,plot=None):
        M = self.meters - np.min(self.meters,axis=1)[0]
        if plot==None:
            plot = np.ones(M.shape[0])
        fig = plt.figure(figsize=(5,5))

        ax = fig.add_subplot(111, projection='3d')

        colors = ['r', 'y', 'g', 'c', 'b', 'k']
        polys = {}
        for k in range(M.shape[0]):
            if plot[k]:
                polys[k]= Poly3DCollection(M[k], alpha=0.8, color = colors[k])    
                ax.add_collection3d(polys[k])



        ax.set_xlim(np.min(M[:,:,0])-1,np.max(M[:,:,0])+1)

        ax.set_ylim(np.min(M[:,:,1])-1,np.max(M[:,:,1])+1)

        ax.set_zlim(np.min(M[:,:,2])-1,np.max(M[:,:,2])+1)  

        ax.set_xlabel('x')
        ax.set_ylabel('y')
        ax.set_zlabel('z')
        