#!/usr/bin/env python


#El nodo tiene que suscribirse a odometria y publicar en cmd_vel

import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion, quaternion_from_euler
import math

#Inicializamos las variables que contendran la posición del robot en x e y

Posicion_x=0
Posicion_y=0
yaw=0

Posicion_x_1=0
Posicion_x_2=0
distancia_recorrida_x=0


Posicion_y_1=0
Posicion_y_2=0
distancia_recorrida_y=0




def get_position(msg: Odometry):
    global roll, pitch, yaw
    orientation_q = msg.pose.pose.orientation
    orientation_list = [orientation_q.x,orientation_q.y, orientation_q.z, orientation_q.w ]
    (roll, pitch, yaw) = euler_from_quaternion(orientation_list)
    #rospy.loginfo('Angle: ' + str(yaw))


    global Posicion_x,Posicion_y #Definimos las variables de la posición como variables globales para poder tener acceso a estas fuera de la función

    Posicion=msg.pose.pose.position
    Posicion_x=Posicion.x
    #print("Posición en x: " +str(Posicion_x))
    Posicion_y=Posicion.y
    #print("Posición en y: " +str(Posicion_y))
    





def move_robot():
    rospy.init_node('move_robot_node', anonymous=True)
    rate = rospy.Rate(2)  # Frecuencia de publicación de comandos de velocidad
    
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
    sub = rospy.Subscriber('odom', Odometry,get_position)
    twist = Twist()

    Contador=0
    
    # Velocidad lineal máxima (en m/s)
    max_linear_speed = 0.1
    # Distancia a avanzar (en metros)
    distance_to_move = 1.0
    distance_to_move_x=0
    distance_to_move_y=0
    
    # Iniciar movimiento hacia adelante
    twist.linear.x = max_linear_speed
    #twist.linear.y = max_linear_speed
    
    # Tiempo necesario para avanzar la distancia deseada
    ##time_to_move = distance_to_move/max_linear_speed
    #rospy.loginfo('tiempo a mover: ' + str(time_to_move))
    
    # Publicar comando de velocidad hasta alcanzar la distancia deseada
    #start_time = rospy.get_time()
    #rospy.loginfo('tiempo a mover: ' + str(start_time))
    while not rospy.is_shutdown():

        #El algoritmo da problemas porque no se actualiza la posición inicial hasta cierto tiempo. Hasta que esto no ocurra,
        #No vamos a tener en cuenta la distancia desplazada.
        #print(Posicion_x)
        #print(Posicion_y)
        #print(yaw)
        if Posicion_x!=0 and Posicion_y!=0 and yaw!=0:   #Posicion_x se inicializa en cero. Este valor debe cambiar cuando el robot empieza a moverse

            #La primera muestra se convierte en nuestra posición inicial   

            if Contador==0:
                
                x_inicial=Posicion_x
                print("Posición inicial en x: " +str(x_inicial))
                y_inicial=Posicion_y
                print("Posición inicial en y: " +str(y_inicial))
                distance_to_move_x=distance_to_move*math.cos(yaw)
                distance_to_move_y=distance_to_move*math.sin(yaw)
                Contador+=1
                #print(Contador)
                Posicion_x_1=x_inicial
                Posicion_y_1=y_inicial
                distancia_recorrida=0

            else:
                #print(Contador)
                Posicion_x_2=Posicion_x
                Posicion_y_2=Posicion_y
                distancia_recorrida_x=abs(abs(Posicion_x_2)-abs(Posicion_x_1))
                distancia_recorrida_y=abs(abs(Posicion_y_2)-abs(Posicion_y_1))
                distancia_recorrida=distancia_recorrida+math.sqrt(math.pow(distancia_recorrida_x,2)+math.pow(distancia_recorrida_y,2))
                #print("Distancia recorrida: "+str(distancia_recorrida))
                Posicion_x_1=Posicion_x_2
                Posicion_y_1=Posicion_y_2
                print("Posición en x: " +str(Posicion_x))
                print("Posición en y: " +str(Posicion_y))
                print("Distancia recorrida: "+str(distancia_recorrida))

            #Distancia_a_recorrer=distance_to_move+
    

                #if (Posicion_x>x_inicial+distance_to_move_x or Posicion_x<x_inicial-distance_to_move_x) and (Posicion_y>y_inicial+distance_to_move_y or Posicion_y<y_inicial-distance_to_move_y):
                
                if distancia_recorrida>distance_to_move:
                    # Detener el movimiento
                    twist.linear.x = 0
                    #twist.linear.y = 0
                    pub.publish(twist)
                    rospy.loginfo("Robot ha avanzado " + str(distance_to_move)+ " metro.")
                    break

                #current_time = rospy.get_time()
                #rospy.loginfo('tiempo actual: ' + str(current_time))
                #elapsed_time = current_time - start_time
                
                #if elapsed_time >= time_to_move:
                    # Detener el movimiento
                    #twist.linear.x = 0
                    #pub.publish(twist)
                    #rospy.loginfo("Robot ha avanzado 1 metro.")
                    #break
                
        pub.publish(twist)
        #rospy.loginfo('avance: ' + str(twist.linear.x))
        rate.sleep()

if __name__ == '__main__':
    try:
        move_robot()
    except rospy.ROSInterruptException:
        pass
