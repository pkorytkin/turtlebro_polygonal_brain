#!/usr/bin/env python
# -*- coding: utf-8 -*-
import math
import rospy
import geometry_msgs.msg
import math
import nav_msgs.msg
from typing import List
class Vector3(geometry_msgs.msg.Vector3):
    def __init__(self,x=0,y=0,z=0):
        self.x=x
        self.y=y
        self.z=z
    def __sub__(self,other):
        if(type(other)==Vector3):
            return Vector3(self.x-other.x,self.y-other.y,self.z-other.z)
    def __add__(self,other):
        if(type(other)==Vector3):
            return Vector3(self.x+other.x,self.y+other.y,self.z+other.z)
    def __mul__(self,other):
        if(type(other)==Vector3):
            return self.x*other.x+self.y*other.y+self.z*other.z
        elif(type(other)==float):
            return Vector3(self.x*other,self.y*other,self.z*other)
    def __str__(self):
        return "x:"+str(self.x)+" y:"+str(self.y)+" z:"+str(self.z)
    def Module(self):
        return float(math.sqrt((self.x)**2+(self.y)**2+(self.z)**2))
    
    def normalize(self):
        dist=self.Module()
        return Vector3(self.x/dist,self.y/dist,self.z/dist)

StartPoseSaved=False
#Глобальная переменная для нынешней позиции
CurrentPose=nav_msgs.msg.Odometry()
#Сохранённая стартовая позиция
StartPose=nav_msgs.msg.Odometry()
#Сообщение поворота отправляемое в Topic
twist=geometry_msgs.msg.Twist()
#Целевая точка маршрута
CurrentPointID=0
#Сохранённые координаты точек маршрута
PointList=[]
#PointList=[]
#Rate
r=any
#Число координат выпуклого многоугольника
PointsCount=4

sub=any
pub=any
def DistanceVector3(fromPosition:Vector3,toPosition:Vector3=Vector3()):
    return (fromPosition-toPosition).Module()
def DotProduct(fromVector:Vector3,toVector:Vector3):
    return fromVector*toVector
def CrossProduct(fromVector:Vector3,toVector:Vector3):
    return fromVector.x*toVector.y-fromVector.y*toVector.x
def Angle3D(fromVector:Vector3,toVector:Vector3):
    #https://www.wikihow.com/Find-the-Angle-Between-Two-Vectors
    
    fromVector=fromVector.normalize()
    toVector=toVector.normalize()
    fromVectorAngle=math.atan2(fromVector.y,fromVector.x)
    toVectorAngle=math.atan2(toVector.y,toVector.x)
    
    print("from="+str(fromVectorAngle)+" to="+str(toVectorAngle))
    angle= abs(toVectorAngle-fromVectorAngle)
    if(toVectorAngle<fromVectorAngle):
        return -angle
    return angle
    
    #angle=math.atan2(CrossProduct(fromVector,toVector),DotProduct(fromVector,toVector))
    #angle=math.acos(DotProduct(fromVector,toVector))
    #cross=CrossProduct(fromVector,toVector)
    #sign=1
    #if(DotProduct())
    #if (fromVector.x*toVector.y-fromVector.y*toVector.x<0):
    #    sign=-1
    #angle=sign*math.acos(fromVector*toVector/(DistanceVector3(fromVector)*DistanceVector3(toVector))).real
    """while angle>math.pi*2:
        angle-=math.pi*2
    while angle<-math.pi*2:
        angle+=math.pi*2
    
    if(angle>math.pi):
        angle-=math.pi*2
    if(angle<-math.pi):
        angle+=math.pi*2
    return angle"""
    
    
def euler_from_quaternion(x, y, z, w):
    """
    Convert a quaternion into euler angles (roll, pitch, yaw)
    roll is rotation around x in radians (counterclockwise)
    pitch is rotation around y in radians (counterclockwise)
    yaw is rotation around z in radians (counterclockwise)
    """
    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + y * y)
    roll_x = math.atan2(t0, t1)
    
    t2 = +2.0 * (w * y - z * x)
    t2 = +1.0 if t2 > +1.0 else t2
    t2 = -1.0 if t2 < -1.0 else t2
    pitch_y = math.asin(t2)
    
    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    yaw_z = math.atan2(t3, t4)
    
    return Vector3(roll_x,pitch_y,yaw_z)#roll_x, pitch_y, yaw_z # in radians
#Преобразование локальной позиции черепахи в мировую
def LocalPositionToWorld(Vector:Vector3):
    return CurrentPosition()+Vector
#Мировая позиция в локальную
def WorldToLocalPosition(Vector:Vector3):
    return Vector-CurrentPosition()
#Нынешняя позция
def CurrentPosition():
    #z=CurrentPose.z
    position=Vector3(x=CurrentPose.pose.pose.position.x,y=CurrentPose.pose.pose.position.y)
    return position
#Нынешняя ротация в углах эйлера
def CurrentRotation():
    #rotation=Vector3(CurrentPose.twist.twist.linear.x,CurrentPose.twist.twist.linear.y,CurrentPose.twist.twist.linear.z)
    #return rotation
    return euler_from_quaternion(CurrentPose.pose.pose.orientation.x,CurrentPose.pose.pose.orientation.y,CurrentPose.pose.pose.orientation.z,CurrentPose.pose.pose.orientation.w)
#Нынешней глобальный вектор вперёд
def CurrentGlobalForward():
    z=CurrentRotation().z
    return Vector3(x=math.cos(z).real,y=math.sin(z).real,z=0)
#Локальный вектор вперёд x=1
def CurrentLocalForward():
    return Vector3(1,0,0)



def subscriber_pose(pose:nav_msgs.msg.Odometry):
    global CurrentPose
    #rospy.loginfo(pose)
    #Сохранение стартовой позиции
    global StartPoseSaved
    global StartPose
    if(not StartPoseSaved):
        StartPose=pose
        StartPoseSaved=True
    #Сохранение нынешней позиции
    CurrentPose=pose
    #print("Saved CurrentPose="+str(pose))
    pass
def PrepareGlobals():
    global StartPoseSaved
    global CurrentPose
    global StartPose
    global r
    StartPose=nav_msgs.msg.Odometry()
    #Нужно сохранить стартовую позицию
    StartPoseSaved=False
    
    CurrentPose=nav_msgs.msg.Odometry()
    #print("Reseted CurrentPose="+str(CurrentPose))
    
    
    rospy.init_node("brain")
    r=rospy.Rate(60)#60hz
    
def PrepareWorkers():
    print("Angle="+str(math.degrees(Angle3D(Vector3(1,0,0),Vector3(-1,0,0)))))
    PrepareGlobals()
    global sub
    global pub
    sub=rospy.Subscriber("odom/",nav_msgs.msg.Odometry,tcp_nodelay=True,queue_size=1,callback=subscriber_pose)

    pub=rospy.Publisher("cmd_vel/",geometry_msgs.msg.Twist,tcp_nodelay=True,queue_size=1)

    r.sleep()

def Worker():
    global CurrentPointID
    global PointsCount
    while not(rospy.is_shutdown()):
        CurrentTargetPoint=PointList[CurrentPointID]
        print("Angle="+str(CurrentPose.twist.twist.angular))
        print("CurrentPointID="+str(CurrentPointID))
        print("CurrentTargetPoint="+str(CurrentTargetPoint))
        print("CurrentPosition="+str(CurrentPosition()))
        print("CurrentRotation="+str(CurrentRotation()))
        VectorToPoint=CurrentTargetPoint-CurrentPosition()
        print("VectorToPoint="+str(VectorToPoint))
        currentGlobalForward=CurrentGlobalForward()
        print("CurrentGlobalForward="+str(currentGlobalForward))
        AngleFromForwardToPoint=Angle3D(currentGlobalForward,VectorToPoint)
        print("AngleFromForwardToPoint="+str(AngleFromForwardToPoint))
        DistanceToPoint=DistanceVector3(CurrentPosition(),CurrentTargetPoint)
        twist.angular=Vector3(0,0,0)
        twist.linear=Vector3()
        print("DistanceToPoint="+str(DistanceToPoint))
        if(DistanceToPoint<-0.1):
            twist.linear=-1*CurrentLocalForward()
        elif(abs(AngleFromForwardToPoint)>0.03):
            twist.angular.z=AngleFromForwardToPoint
        else:
            if(DistanceToPoint>0.05):
                twist.linear=CurrentLocalForward()#(CurrentLocalForward()*max(DistanceToPoint,0.01)).convertToMSG()
            elif(DistanceToPoint<-0.05):
                twist.linear=-1*CurrentLocalForward()#(CurrentLocalForward()*(-max(DistanceToPoint,0.01))).convertToMSG()
            else:
                CurrentPointID+=1
                if(CurrentPointID==PointsCount):
                    CurrentPointID=0
        pub.publish(twist)
        print(twist)
        r.sleep()



if __name__ == '__main__':
    try:
        
        PrepareWorkers()
        #Заготавливаем точки маршрута
        currentPosition=CurrentPosition()
        i=0
        tempVector=Vector3
        while i<PointsCount:
            tempVector=Vector3(x=math.cos((2*math.pi/PointsCount)*i).real,y=math.sin((2*math.pi/PointsCount)*i).real)*0.3+currentPosition
            PointList.append(tempVector)
            i+=1
            #print(tempVector)
            continue
        i=0
        while i<PointsCount:
            
            print(PointList[i])
            i+=1
            continue
        print()
        Worker()
    except rospy.ROSInterruptException:
        pass