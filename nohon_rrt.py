import numpy as np
import matplotlib.pyplot as plt
from math import *
import random
N=10000
step_size=10
distance_threshold=10
orientation_threshold=20*pi/180
dt=0.1
L=7.5
max_steering_angle=0.7
vel=40
x_init=20
y_init=20
theta_init=0
x_goal=180
y_goal=180
theta_goal=pi/4
x_min=0
y_min=0
x_max=200
y_max=200
# plt.figure()
# plt.plot(x_init,y_init,'ko')
# plt.grid("on")
# plt.hold("on")
# plt.plot(x_goal,y_goal,'go')
# plt.grid("on")
# plt.hold("on")
initpoint=[x_init,y_init,theta_init,x_init,y_init,theta_init,0,0,0,-1]
#[x,y,theta,x_parent,y_parent,theta_parent,parent_steering_angle,distance,index,parent_index]
tree=[]
tree.append(initpoint)
x_new=0
y_new=0
theta_new=0
iter=1
while iter<=N:
    # x_rand=(x_max-x_min)*random.random()
    # y_rand=(y_max-y_min)*random.random()
    p=random.random()
    if(p<0.5):
        x_rand=(x_max-x_min)*random.random()
        y_rand=(x_max-x_min)*random.random()
        theta_rand=(2*pi-0.0001)*random.random()
    else:
        x_rand = x_goal
        y_rand = y_goal
        theta_rand = (2 * pi - 0.0001) * random.random()


    testmaxvalue=9999
    parent_index=0

    for i in range(len(tree)):
        distance_value=sqrt((x_rand-tree[i][0])**2+(y_rand-tree[i][1])**2)
        if (distance_value<testmaxvalue):
            testmaxvalue=distance_value
            parent_index=i

    x_near=tree[parent_index][0]
    y_near=tree[parent_index][1]
    theta_near=tree[parent_index][2]

    steering_angle=-max_steering_angle
    new_node_array=[]


    while steering_angle<=max_steering_angle:
        path = []
        path0 = [x_near, y_near, theta_near]
        path.append(path0)
        for i in range(1,step_size):
            tempx=path[i-1][0]+vel*cos(path[i-1][2])*dt
            tempy=path[i-1][1]+vel*sin(path[i-1][2])*dt
            temptheta=path[i-1][2]+(vel/L)*tan(steering_angle)*dt
            temp=[tempx,tempy,temptheta]
            path.append(temp)



        distance_new_node=sqrt((x_rand-path[-1][0])**2+(y_rand-path[-1][1])**2)

        temp_new_node_array=[distance_new_node,steering_angle,path[-1][0],path[-1][1],path[-1][2]]
        new_node_array.append(temp_new_node_array)
        steering_angle=steering_angle+0.05

    Index=0
    distance_new_node_value=99999

    for i in range(len(new_node_array)):
        if (new_node_array[i][0]<distance_new_node_value):
            distance_new_node_value=new_node_array[i][0]
            Index=i


    x_new=new_node_array[Index][2]
    y_new=new_node_array[Index][3]

    theta_new=new_node_array[Index][4]
    if theta_new<0:
        theta_new=2*pi-abs(theta_new)
    if theta_new>2*pi:
        while theta_new>2*pi:
            theta_new=theta_new-2*pi

    parent_steering_angle_new=new_node_array[Index][1]
    newpoint=[x_new,y_new,theta_new,x_near,y_near,theta_near,
              parent_steering_angle_new,distance_new_node_value,iter,parent_index]
    tree.append(newpoint)

    distance_to_goal=sqrt((x_new-x_goal)**2+(y_new-y_goal)**2)

    if ((distance_to_goal<=distance_threshold) and
        ((abs(theta_new-theta_goal)<=orientation_threshold) or
         (abs(theta_new-theta_goal-pi)<=orientation_threshold))):
        #print("success")
        plt.plot([x_goal,x_new],[y_goal,y_new],'-r')
        break


    iter=iter+1



totalindex=len(tree)-1
tree=np.asarray(tree)
totalpath=[]
#totalpath.append(tree[totalindex])
totalparent_index=int(totalindex)

# for i in range(len(tree)):
#     print(tree[i][6])


while (totalparent_index!=0):
    temptotalpath0=[tree[int(totalparent_index)][3],tree[int(totalparent_index)][4],
                    tree[int(totalparent_index)][5]]
    # print(temptotalpath0)
    temptotalpath=[]
    totalpath.append(tree[int(totalparent_index)])
    temptotalpath.append(temptotalpath0)
    temp_parent_steering_angle_new=tree[int(totalparent_index)][6]
    for i in range(1,step_size):
        totaltempx=temptotalpath[i-1][0]+vel*cos(temptotalpath[i-1][2])*dt
        totaltempy=temptotalpath[i-1][1]+vel*sin(temptotalpath[i-1][2])*dt
        totaltemptheta=temptotalpath[i-1][2]+(vel/L)*tan(temp_parent_steering_angle_new)*dt
        totaltemp=[totaltempx,totaltempy,totaltemptheta]
        temptotalpath.append(totaltemp)
    #print(temptotalpath)
    for j in range(1,len(temptotalpath)):

        plt.plot([temptotalpath[j][0],temptotalpath[j-1][0]],[temptotalpath[j][1],temptotalpath[j-1][1]],'-r')
        plt.grid("on")
        plt.hold("on")


    findex=int(totalparent_index)
    totalparent_index=tree[findex][9]


plt.show()
#print(np.asarray(totalpath))





