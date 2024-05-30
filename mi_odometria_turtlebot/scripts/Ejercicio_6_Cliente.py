#!/usr/bin/env python
import rospy
import actionlib

#import primer_ejemplo.msg

from mi_odometria_turtlebot.msg import *


"""""
def fibonacci_client(name):
    client = actionlib.SimpleActionClient(name, primer_ejemplo.msg.FibonacciAction)
    client.wait_for_server()

    goal = primer_ejemplo.msg.FibonacciGoal()
    goal.order = 20

    client.send_goal(goal)

    client.wait_for_result()

    return client.get_result()

"""
#Inicializamos la variable que contiene la distancia del lado del triángulo a realizar
lado=0

def move_robot_client(name):

    #Creamos una instancia del cliente
    client = actionlib.SimpleActionClient(name, mi_odometria_turtlebot.msg.FibonacciAction)
    
    #Esperamos al servidor 'move_robot'
    client.wait_for_server()

    #Definimos el objetivo de la acción

    goal = mi_odometria_turtlebot.msg.FibonacciGoal()

    #Pedimos al usuario que introduzca el tamaño del lado del triángulo a realizar


    goal.Distancia=float(input("Introduce la longitud del lado del triángulo a realizar (metros): "))


    #El primer movimiento a realizar se corresponde con un desplazamiento lineal. Vamos a definirlo:
    lado=goal.Distancia

    goal.Movimiento=1

    goal.Giro=0

    """""

    if goal.Movimiento==1:

            
            #rospy.loginfo("Introduce la distancia a recorrer (en metros): ")

            goal.Distancia=float(input("Indica la distancia a recorrer en metros: "))

            #M.Distancia=Distancia_a_recorrer

            goal.Giro=0

    elif goal.Movimiento==2:

            #rospy.loginfo("Introduce la distancia a girar (en grados): ")

            goal.Giro=float(input("Introduce la distancia a girar (en grados): "))

            goal.Distancia=0

            #M.Giro=Giro

    elif goal.Movimiento==3:
            
            goal.Distancia=0
            goal.Giro=0
            rospy.loginfo("Fin de la ejecución.")
            rospy.loginfo("¡Hasta la vista!")
            #break
        
    else:

        goal.Distancia=0
        goal.Giro=0
                
        rospy.loginfo("El tipo de movimiento introducido no es válido.")
        rospy.loginfo("Fin de la ejecución.")
            #break

    """

    #Para hacer un triángulo, el algoritmo deberá ejecutarse 3 veces:

    for i in range(3):
         
        print('Movimiento solicitado: Lineal')

        #print(goal.Movimiento)
        #print(goal.Distancia)
        #print(goal.Giro)
                
        #Mandamos el objetivo al servidor de acción
        client.send_goal(goal,feedback_cb=feedback_cb)

        #Esperamos la respuesta del servidor

        client.wait_for_result()

        #Una vez ejecutado el desplazamiento, debemos realizar un giro de 120º en el
        #caso del triángulo
    
        print('Movimiento solicitado: Giro')

        #Definimos los parámetros de giro

        goal.Movimiento=2

        goal.Distancia=0

        goal.Giro=120

        #print(goal.Movimiento)
        #print(goal.Distancia)
        #print(goal.Giro)

        #Mandamos el objetivo al servidor de acción
        client.send_goal(goal,feedback_cb=feedback_cb)

        #Esperamos la respuesta del servidor
        client.wait_for_result()

        #Preparamos el movimiento lineal para la siguiente iteración del bucle

        goal.Distancia=lado

        goal.Movimiento=1

        goal.Giro=0




    


    
    

    #print(goal.Movimiento)
    #print(goal.Distancia)
    #print(goal.Giro)

    
    



    


def feedback_cb(feedback):
      #Esta función muestra por pantalla el feedback recibido por parte del servidor de acción
      rospy.loginfo('Feedback recibido: {}'.format(feedback.Feedback))

if __name__ == '__main__':
    #Creamos un nodo para el cliente de la acción
    rospy.init_node('move_robot_client')

    #Ejecutamos la función move_robot_client para iniciar la entrada de datos por parte del usuario
    result = move_robot_client('move_robot')

    #Mostramos por pantalla el resultado devuelto por el servidor de acción.
    result_str = '[Posición_x,Posición_y,yaw]= '
    result_str += str(result)
    rospy.loginfo(result_str)