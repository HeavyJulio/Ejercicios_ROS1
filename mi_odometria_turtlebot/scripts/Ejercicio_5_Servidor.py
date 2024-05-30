#!/usr/bin/env python
from unicodedata import name

from matplotlib.pyplot import autoscale
import rospy
import actionlib
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import math
import mi_odometria_turtlebot.msg
from mi_odometria_turtlebot.msg import FibonacciAction
from tf.transformations import euler_from_quaternion, quaternion_from_euler

#Inicializamos algunas de las variables del programa:

Posicion_x=0
Posicion_y=0
yaw=0

Posicion_x_1=0
Posicion_x_2=0
distancia_recorrida_x=0


Posicion_y_1=0
Posicion_y_2=0
distancia_recorrida_y=0

kp = 0.05


class Move_Robot_Action:

    #Creamos las variables que contienen el feedback y el resultado de la acción

    _feedback = mi_odometria_turtlebot.msg.FibonacciFeedback()
    _result = mi_odometria_turtlebot.msg.FibonacciResult()

    #Definimos el construcctor de la clase Move_Robot_Action

    def __init__(self, name):
        self._action_name = name
        self._as = actionlib.SimpleActionServer(
                            self._action_name, 
                            mi_odometria_turtlebot.msg.FibonacciAction, 
                            execute_cb=self.move_robot,      #Esto hay que cambiarlo por la función "move_robot"
                            auto_start=False
                            )
        self._as.start()

    """  

    def execute_cb(self, goal : mi_odometria_turtlebot.msg.FibonacciGoal):
        r = rospy.Rate(1)
        success = True

        self._feedback.sequence = []
        self._feedback.sequence.append(0)
        self._feedback.sequence.append(1)

        rospy.loginfo(f'{self._action_name}: Executing, creating fibonacci sequence of order {goal.order} with seeds 0, 1')

        for i in range(1, goal.order):
            if self._as.is_preempt_requested():
                rospy.loginfo(f'{self._action_name} Preempted')
                self._as.set_preempted()
                success = False
                break
            last0 = self._feedback.sequence[i]
            last1 = self._feedback.sequence[i-1]
            self._feedback.sequence.append(last0 + last1)

            self._as.publish_feedback(self._feedback)

            r.sleep()

        if success:
            self._result.sequence = self._feedback.sequence
            rospy.loginfo(f'{self._action_name}: Succeeded')
            self._as.set_succeeded(self._result)

    """   

    def get_position(self,msg: Odometry):

        #Esta función nos permite obtener los parametros de posición y orientación del robot a
        #partir de los mensajes del nodo de odometría '/odom'


        global roll, pitch, yaw #Definimos las variables de la posición angular como variables 
        #globales para poder tener acceso a estas fuera de la función



        #Extraemos los valores de orientación del mensaje en formato de cuaternio:

        orientation_q = msg.pose.pose.orientation
        orientation_list = [orientation_q.x,orientation_q.y, orientation_q.z, orientation_q.w ]


        #Tomamos el cuaternio de orientación del robot y lo pasamos a ángulos de euler.Aunque estamos
        #calculando los tres, a nosotros solo nos interesa yaw (orientación en el eje z)

        (roll, pitch, yaw) = euler_from_quaternion(orientation_list)
        
        global Posicion_x,Posicion_y #Definimos las variables de la posición como variables 
        #globales para poder tener acceso a estas fuera de la función

        #Extraemos los valores de posición en x e y del mensaje:


        Posicion=msg.pose.pose.position
        Posicion_x=Posicion.x
        
        Posicion_y=Posicion.y
        
        
    

    def move_robot(self, goal : mi_odometria_turtlebot.msg.FibonacciGoal):

        #Este método ejecuta un tipo de movimiento u otro del robot en función de los
        #parámetros especificados por el cliente:

        if goal.Movimiento==1: #Movimiento lineal

        
            rate = rospy.Rate(2)  # Frecuencia de publicación de comandos de velocidad
            
            #La velocidad del robot se controla publicando en el tópico '/cmd_vel' mensajes de tipo Twist.

            pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)

            #La posición del robot se obtiene del tópico '/odom', mediante mensajes de tipo 'Odometry'. Cada vez que el subscriptor 
            #detecte un cambio en el tópico, se ejecutara la función 'get_position()'
    
            sub = rospy.Subscriber('odom', Odometry,self.get_position)

            #Creamos un mensaje de tipo Twist

            twist = Twist()

            #Creamos un contador que nos permitirá obtener la posición inicial del robot

            Contador=0
            
            # Velocidad lineal máxima (en m/s)
            max_linear_speed = 0.1

            # Distancia a avanzar (en metros). Esta se obtiene a partir de la solicitud que hace el cliente (goal)
    
            distance_to_move = goal.Distancia
           
            # Iniciar movimiento hacia adelante
            twist.linear.x = max_linear_speed
            
            
            
            while not rospy.is_shutdown():

                #El algoritmo da problemas porque no se actualiza la posición inicial hasta cierto tiempo. Hasta que esto no ocurra,
                #No vamos a tener en cuenta la distancia desplazada.
                
                #Estas variables están inicializadas en cero, y no cambiarán hasta que el robot no empieze a mandarnos mensajes de odometría.
                #Una vez que tengamos los datos de odometría del robot, podemos iniciar el algoritmo:

                if Posicion_x!=0 and Posicion_y!=0 and yaw!=0:   #Posicion_x se inicializa en cero. Este valor debe cambiar cuando el robot empieza a moverse

                    #La primera muestra se convierte en nuestra posición inicial. La variable 'Contador' nos permite determinar la primera muestra de posición
                    #del robot:

                    if Contador==0:

                        #Inicializamos la lista con los valores del feedback

                        self._feedback.Feedback=[0,0,0]
                        
                        x_inicial=Posicion_x
                        print("Posición inicial en x: " +str(x_inicial))
                        y_inicial=Posicion_y
                        print("Posición inicial en y: " +str(y_inicial))
                        
                        #Actualizamos el contador para saber que la primera muestra ya ha sido tomada.


                        Contador+=1
                        
                        Posicion_x_1=x_inicial
                        Posicion_y_1=y_inicial
                        distancia_recorrida=0

                        


                    else:
                        
                        #Para saber cual ha sido la distancia desplazada por el robot en cada uno de los ejes, calculamos la diferencia entre
                        #la posición en cada eje entre un mensaje de odometría y el anterior.
                

                        Posicion_x_2=Posicion_x
                        Posicion_y_2=Posicion_y
                        distancia_recorrida_x=abs(abs(Posicion_x_2)-abs(Posicion_x_1))
                        distancia_recorrida_y=abs(abs(Posicion_y_2)-abs(Posicion_y_1))

                        #La distancia total recorrida en cada iteración se obtiene mediante geometría a partir de la distancia recorrida en x e y.
                        #Esta distancia recorrida se va sumando a la distancia total recorrida en las iteraciones anteriores.

                        distancia_recorrida=distancia_recorrida+math.sqrt(math.pow(distancia_recorrida_x,2)+math.pow(distancia_recorrida_y,2))
                        
                        #Se actualiza el valor del primer punto para la siguiente iteración.

                        Posicion_x_1=Posicion_x_2
                        Posicion_y_1=Posicion_y_2

                        #Mostramos por pantalla información relativa al movimiento del robot.

                        print("Posición en x: " +str(Posicion_x))
                        print("Posición en y: " +str(Posicion_y))
                        print("Distancia recorrida: "+str(distancia_recorrida))

                        #Si la distancia recorrida por el robot supera a la distancia especificada por el usuario, detenemos el robot.
                        
                        if distancia_recorrida>distance_to_move:
                            # Detener el movimiento
                            twist.linear.x = 0
                            #Publicamos el mensaje de velocidad en el tópico 'cmd_vel'
                            pub.publish(twist)
                            rospy.loginfo("Robot ha avanzado " + str(distance_to_move)+ " metro.")
                            
                            #Actualizamos los valores de feedback

                            self._feedback.Feedback[0]=Posicion_x
                            self._feedback.Feedback[1]=Posicion_y
                            self._feedback.Feedback[2]=yaw

                            #Asignamos los últimos datos de feedback al resultado y se lo devolvemos al usuario
                            self._result.Resultado=self._feedback.Feedback
                            self._as.set_succeeded(self._result)

                            break

                        
                    #Actualizamos los valores de feedback

                    self._feedback.Feedback[0]=Posicion_x
                    self._feedback.Feedback[1]=Posicion_y
                    self._feedback.Feedback[2]=yaw

                    #Devolvemos los valores de feedback al usuario

                    self._as.publish_feedback(self._feedback)

                #Publicamos el mensaje de velocidad en el tópico 'cmd_vel'        

                pub.publish(twist)
                
                rate.sleep()


        elif goal.Movimiento==2: #Movimiento de rotación

            
            rate = rospy.Rate(2)  # Frecuencia de publicación de comandos de velocidad
           

            #La velocidad del robot se controla publicando en el tópico '/cmd_vel' mensajes de tipo Twist.
        
            pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)

            #La posición del robot se obtiene del tópico '/odom', mediante mensajes de tipo 'Odometry'. Cada vez que el subscriptor 
            #detecte un cambio en el tópico, se ejecutara la función 'get_position()'
        
            sub = rospy.Subscriber('odom', Odometry,self.get_position)
            
            #Creamos un mensaje de tipo Twist


            twist = Twist()  

            #El usuario indica la posición angular en torno a z que quiere para el robot.
            #Esta viene especificada en grados en goal.Giro. Nosotros la estamos pasando a
            #radianes en target_rad, para poder compararla con la medida de yaw (en radianes).


            target_rad = goal.Giro*math.pi/180

            #Inicializamos el vector de feedback

            self._feedback.Feedback=[0,0,0]

            #El robot girará a una velocidad constante mientras que la diferencia entre el target_rad
            #y la orientación del robot sea mayor al umbral definido por kp

            angulo=yaw

            while abs(target_rad-angulo)>kp:

                angulo=yaw

                if angulo<0:
                    angulo=angulo+2*math.pi
                
                twist.angular.z = 0.5
                #Publicamos el mensaje de velocidad en el tópico 'cmd_vel'
                pub.publish(twist)
                rospy.loginfo(f'target: {target_rad}, current: {angulo}')

                #Actualizamos el feedback y se lo devolvemos al cliente
                self._feedback.Feedback[0]=Posicion_x
                self._feedback.Feedback[1]=Posicion_y
                self._feedback.Feedback[2]=yaw

                self._as.publish_feedback(self._feedback)

                #Si esta diferencia está por debajo del umbral, dejamos de girar:


                if abs(target_rad-angulo)<=kp:

                    twist.angular.z=0
                    #Publicamos el mensaje de velocidad en el tópico 'cmd_vel'
                    pub.publish(twist)
                    rospy.loginfo(f'target: {target_rad}, current: {angulo}')


                    #Asignamos el último valor del feedback al resultado y se lo devolvemos al usuario
                    self._result.Resultado=self._feedback.Feedback
                    self._as.set_succeeded(self._result)                    
                    

                
        elif goal.Movimiento==3: #El usuario ha decidido no ejecutar ningún movimiento


            print("El usuario ha cancelado la ejecución del movimiento.")
            
            #Asinamos al feedback y al resultado una lista vacia y se los pasamos al usuario
            self._feedback.Feedback=[]
            self._result.Resultado=[]

            self._as.publish_feedback(self._feedback)
            self._as.set_succeeded(self._result)   

        else: #Movimiento no válido

            print("El usuario ha introducido un movimiento no válido.")

            #Asinamos al feedback y al resultado una lista vacia y se los pasamos al usuario
            self._feedback.Feedback=[]
            self._result.Resultado=[]

            self._as.publish_feedback(self._feedback)
            self._as.set_succeeded(self._result)   









if __name__ == '__main__':
    #Iniciamos la ejecución del servidor de acción:
    rospy.init_node('move_robot_server')
    server = Move_Robot_Action('move_robot')
    rospy.loginfo("Listo para iniciar la marcha.")
    rospy.spin()