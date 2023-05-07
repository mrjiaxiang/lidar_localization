## 针孔相机的成像过程  
1、世界坐标系下有一个固定的点P，世界坐标为P_w  
2、由于相机在运动，它的运动由R，t或变换矩阵T（外参）。P的相机坐标为P̃c=RPw+t  
3、这时的 P̃c 仍有X,Y,Z三个量，把它们投影到归一化平面Z=1上，得到P的归
一化相机坐标：Pc = [X/Z, Y/Z, 1]T。  
4、有畸变时，根据畸变参数计算Pc发生畸变后的坐标  


5、P的归一化坐标经过内参后，对应到它的像素坐标：Puv=KPc 。

### 2D->3D的变换

投影到归一化坐标系，直接通过内参进行计算  
x = u/fx - cx/fx;
y = v/fy - cy/fy;

r^2 = x^2 + y^2

**径向畸变**  
xdistorted = x(1+k1 * r^2+k2 * r^4 + k3 * r^6);
ydistorted = y(1+k1 * r^2+k2 * r^4 + k3 * r^6);

**切向畸变**  
xdistorted = x+2p1xy +p2(r^2 + 2x^2)
xdistorted = y+p1(r^2 + 2y^2) +2p2xy

xcorrected = x(1 + k1 r^2 + k2 r^4 + k3 r^6 ) + 2p1 xy + p2 (r^2 + 2x^2 )
ycorrected = y(1 + k1 r^2 + k2 r^4 + k3 r^6 ) + p1(r^2 + 2y^2 ) + 2p2xy


## 鱼眼相机  
