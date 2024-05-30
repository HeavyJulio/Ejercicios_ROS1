#!/usr/bin/env python
import rospy
import actionlib

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

def move_robot_client(name):

    #Creamos una instancia del cliente
    client = actionlib.SimpleActionClient(name, mi_odometria_turtlebot.msg.FibonacciAction)

    #Esperamos al servidor 'move_robot'
    client.wait_for_server()

    #Definimos el objetivo de la acción

    goal = mi_odometria_turtlebot.msg.FibonacciGoal()

    #Pedimos al usuario que introduzca el tipo de movimiento a realizar

    goal.Movimiento=int(input("Introduce el tipo de movimiento para el robot (1 para lineal, 2 para rotación, 3 para salir): "))

    

    if goal.Movimiento==1: #Movimiento lineal

            
            #Pedimos al usuario la distancia que recorrerá el robot:

            goal.Distancia=float(input("Indica la distancia a recorrer en metros: "))

            #Ponemos a 0 el valor del giro del robot

            goal.Giro=0

    elif goal.Movimiento==2: #Rotación

            #Pedimos al usuario una orientación para el robot en torno al eje z (en grados):

            goal.Giro=float(input("Introduce la distancia a girar (en grados): "))

            #Ponemos a 0 el valor de la distancia del robot

            goal.Distancia=0

            

    elif goal.Movimiento==3: #El usuario ha salido sin realizar ningún movimiento
            
            #Ponemos a 0 el valor de la distancia del robot
            goal.Distancia=0
            #Ponemos a 0 el valor del giro del robot
            goal.Giro=0
            rospy.loginfo("Fin de la ejecución.")
            rospy.loginfo("¡Hasta la vista!")
            
        
    else: #Movimiento no válido

        #Ponemos a 0 el valor de la distancia del robot

        goal.Distancia=0

        #Ponemos a 0 el valor del giro del robot

        goal.Giro=0
                
        rospy.loginfo("El tipo de movimiento introducido no es válido.")
        rospy.loginfo("Fin de la ejecución.")
            


  
    print('Movimiento solicitado')

    #print(goal.Movimiento)
    #print(goal.Distancia)
    #print(goal.Giro)

    #Mandamos el "goal" al servidor de acción. Cuando este nos devuelva feedback
    #acerca de la ejecución del proceso, ejecutamos la función "feedback_cb"

    client.send_goal(goal,feedback_cb=feedback_cb)

    #Esperamos los resultados por parte del servidor

    client.wait_for_result()

    return client.get_result()
    



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