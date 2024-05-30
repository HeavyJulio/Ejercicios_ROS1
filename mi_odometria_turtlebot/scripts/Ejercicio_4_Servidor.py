#!/usr/bin/env python
import rospy
from mi_odometria_turtlebot.srv import Ejercicio_4,Ejercicio_4Response
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion, quaternion_from_euler
import math

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


def get_position(msg: Odometry):

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
    
    



def move_robot(req: Ejercicio_4):

    #Esta función ejecuta un tipo de movimiento u otro del robot en función de los
    #parámetros especificados por el cliente:

    if req.Movimiento==1: #Movimiento lineal

    
    
        rate = rospy.Rate(2)  # Frecuencia de publicación de comandos de velocidad
        
        #La velocidad del robot se controla publicando en el tópico '/cmd_vel' mensajes de tipo Twist.
    
        
        pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)


        #La posición del robot se obtiene del tópico '/odom', mediante mensajes de tipo 'Odometry'. Cada vez que el subscriptor 
        #detecte un cambio en el tópico, se ejecutara la función 'get_position()'
    
        sub = rospy.Subscriber('odom', Odometry,get_position)

        #Creamos un mensaje de tipo Twist

        twist = Twist()

        #Creamos un contador que nos permitirá obtener la posición inicial del robot

        Contador=0
        
        # Velocidad lineal máxima (en m/s)
        max_linear_speed = 0.1
        # Distancia a avanzar (en metros). Esta se obtiene a partir de la solicitud que hace el cliente (req)
    
        distance_to_move = req.Distancia
        
        
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
                        #Mostramos al usuario la distancia recorrida:
                        rospy.loginfo("Robot ha avanzado " + str(distance_to_move)+ " metro.")
                        #Devolvemos un resultado de la operación al cliente (0 si el movimiento se ha ejecutado correctamente)
                        return Ejercicio_4Response(0)
                        break

                    
            #Publicamos el mensaje de velocidad en el tópico 'cmd_vel'        
            pub.publish(twist)
            
            rate.sleep()


    elif req.Movimiento==2: #Movimiento de rotación

        
            
        rate = rospy.Rate(2)  # Frecuencia de publicación de comandos de velocidad
           

        #La velocidad del robot se controla publicando en el tópico '/cmd_vel' mensajes de tipo Twist.
       
        pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)

        #La posición del robot se obtiene del tópico '/odom', mediante mensajes de tipo 'Odometry'. Cada vez que el subscriptor 
        #detecte un cambio en el tópico, se ejecutara la función 'get_position()'
    
        sub = rospy.Subscriber('odom', Odometry,get_position)
        
        #Creamos un mensaje de tipo Twist


        twist = Twist()  

        #El usuario indica la posición angular en torno a z que quiere para el robot.
        #Esta viene especificada en grados en req.Giro. Nosotros la estamos pasando a
        #radianes en target_rad, para poder compararla con la medida de yaw (en radianes).

        target_rad = req.Giro*math.pi/180

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

            #Si esta diferencia está por debajo del umbral, dejamos de girar:

            if abs(target_rad-angulo)<=kp:

                twist.angular.z=0
                #Publicamos el mensaje de velocidad en el tópico 'cmd_vel'
                pub.publish(twist)
                rospy.loginfo(f'target: {target_rad}, current: {angulo}')
                #Devolvemos un resultado de la operación al cliente (0 si el movimiento se ha llevado a cabo correctamente)
                return Ejercicio_4Response(0)
            
    elif req.Movimiento==3: #El usuario ha decidido no ejecutar ningún movimiento

        print("El usuario ha cancelado la ejecución del movimiento.")
        #Devolvemos un resultado de la operación al cliente (1 si el usuario sale del programa)
        return Ejercicio_4Response(1)

    else: #Movimiento no válido

        print("El usuario ha introducido un movimiento no válido.")
        #Devolvemos un resultado de la operación al cliente (2 para movimiento no válido)
        return Ejercicio_4Response(2)


    
def move_robot_server():
    #Esta función genera un nodo de nombre 'move_robot_server', y crea un servidor de nombre 'move_robot'
    #que trabaja con los datos de Ejercicio_4.srv . Cada vez que un cliente le haga una petición al servidor, este
    #ejecutará la función 'move_robot()'
    rospy.init_node('move_robot_server')
    s = rospy.Service('move_robot', Ejercicio_4, move_robot)
    rospy.loginfo("Listo para iniciar la marcha.")
    rospy.spin()

if __name__ == "__main__":
    #Ejecutamos la función 'move_robot_server()'
    move_robot_server()