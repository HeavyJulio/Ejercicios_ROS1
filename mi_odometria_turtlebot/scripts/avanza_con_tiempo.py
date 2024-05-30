#!/usr/bin/env python



################################ ADVERTENCIA #######################################

#Es necesario reiniciar el simulador y el tiempo de la simulación cada vez que se vaya a ejecutar el programa!!!!!!!!!!!!!!!!!!!

###################################################################################

import rospy
from geometry_msgs.msg import Twist

def move_robot():
    rospy.init_node('move_robot_node', anonymous=True)
    rate = rospy.Rate(10)  # Frecuencia de publicación de comandos de velocidad
    
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
    twist = Twist()
    
    # Velocidad lineal máxima (en m/s)
    max_linear_speed = 0.05
    # Distancia a avanzar (en metros)
    distance_to_move = 1.0
    
    # Iniciar movimiento hacia adelante
    twist.linear.x = max_linear_speed
    
    # Tiempo necesario para avanzar la distancia deseada
    time_to_move = distance_to_move/max_linear_speed
    rospy.loginfo('tiempo a mover: ' + str(time_to_move))
    Contador=0
    start_time=0
    current_time=0
    elapsed_time=0
    
    # Publicar comando de velocidad hasta alcanzar la distancia deseada
    start_time = rospy.get_time()#Este comando lo he modificado
    #rospy.loginfo('tiempo inicial: ' + str(start_time))
    while not rospy.is_shutdown():
        rospy.loginfo(Contador)
        if(Contador==0):
            start_time = rospy.get_time()
            rospy.loginfo('tiempo inicial: ' + str(start_time))
            Contador +=1

        #current_time = rospy.get_time()
        current_time = rospy.get_time() #Compensamos el desfase de la simulación
        rospy.loginfo('tiempo actual: ' + str(current_time))
        #elapsed_time = current_time - start_time
        elapsed_time=current_time-start_time
        rospy.loginfo('tiempo transcurrido: ' + str(elapsed_time))
        
        if elapsed_time >= time_to_move:
            # Detener el movimiento
            twist.linear.x = 0
            pub.publish(twist)
            rospy.loginfo("Robot ha avanzado 1 metro.")
            break
        
        pub.publish(twist)
        rospy.loginfo('velocidad avance: ' + str(twist.linear.x)+' m/s')
        rate.sleep()

if __name__ == '__main__':
    try:
        move_robot()
    except rospy.ROSInterruptException:
        pass
