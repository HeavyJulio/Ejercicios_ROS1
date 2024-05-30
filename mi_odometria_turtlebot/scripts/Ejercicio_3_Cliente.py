#!/usr/bin/env python
import sys
import rospy
from mi_odometria_turtlebot.srv import Ejercicio_3


def move_robot_client(Distancia: float):

    #Esperamos al servidor 'move_robot' 

    rospy.wait_for_service('move_robot')

    try:

        #Hacemos una llamada al servidor 'move_robot' con la distancia especificada por el usuario:
        move_robot = rospy.ServiceProxy('move_robot', Ejercicio_3)
        
        Respuesta=move_robot(Distancia)
        
    except rospy.ServiceException as e:
        #Mostramos un mensaje de error si el servicio no está activo:
        print('Service call failed: %s' % e)


    
if __name__ == '__main__':

    #Pedimos al usuario la distancia que recorrerá el robot:

    Distancia=float(input("Introduce la distacia que recorrerá el robot: "))

    print('Moviendo %f m' % (Distancia))

    #Ejecutamos la función 'move_robot_client()'

    move_robot_client(Distancia)
   