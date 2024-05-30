#!/usr/bin/env python
import sys
import rospy
from mi_odometria_turtlebot.srv import Ejercicio_4,Ejercicio_4Response


def move_robot_client(Movimiento: int,Distancia: int,Giro: int):
    
    #Esperamos al servidor 'move_robot' 

    rospy.wait_for_service('move_robot')
    try:
        #Hacemos una llamada al servidor 'move_robot' con los parámetros de movimiento especificados por el usuario:
        
        move_robot = rospy.ServiceProxy('move_robot', Ejercicio_4)
        
        resp: Ejercicio_4Response = move_robot(Movimiento,Distancia,Giro)
        return resp.Resultado
    except rospy.ServiceException as e:
        #Mostramos un mensaje de error si el servicio no está activo:
        print('Service call failed: %s' % e)

 
if __name__ == '__main__':
    
    #Pedimos al usuario el tipo de movimiento que llevará a cabo el robot

    Movimiento=int(input("Introduce el tipo de movimiento para el robot (1 para lineal, 2 para rotación, 3 para salir): "))

    

    if Movimiento==1: #Movimiento lineal

            
            #Pedimos al usuario la distancia que recorrerá el robot:

            Distancia=float(input("Indica la distancia a recorrer en metros: "))

            #Ponemos a 0 el valor del giro del robot

            Giro=0

    elif Movimiento==2: #Rotación

            #Pedimos al usuario una orientación para el robot en torno al eje z (en grados)
            Giro=float(input("Introduce la orientación del robot en torno a z (en grados): "))

            #Ponemos a 0 el valor de la distancia del robot

            Distancia=0

            

    elif Movimiento==3: #El usuario ha salido sin realizar ningún movimiento
            
            #Ponemos a 0 el valor de la distancia del robot
            Distancia=0
            #Ponemos a 0 el valor del giro del robot
            Giro=0
            rospy.loginfo("Fin de la ejecución.")
            rospy.loginfo("¡Hasta la vista!")
            
        
    else: #Movimiento no válido

        #Ponemos a 0 el valor de la distancia del robot

        Distancia=0

        #Ponemos a 0 el valor del giro del robot

        Giro=0
                
        rospy.loginfo("El tipo de movimiento introducido no es válido.")
        rospy.loginfo("Fin de la ejecución.")
            


                

    
    print('Movimiento solicitado')

    #print(Movimiento)
    #print(Distancia)
    #print(Giro)

    #Ejecutamos la función 'move_robot_client()'

    move_robot_client(Movimiento,Distancia,Giro)
    