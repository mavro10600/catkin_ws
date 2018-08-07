#!/usr/bin/env python
# -*- coding: utf8 -*-

#
#
#
#
#
#
#
##TODO: en realidad, lo que hay que hacer es conectar este nodo al nodo Base, y leerlo en el progrma de arduino (bueno, la stellaris) y dentro del loop, cambiar el valor a 0, si el tiempo de ejecucion se sobrepasa,m los motores se detienen y se resetea el micro, en caso de operacion normal ,solo basta con ajustar el loop rate de este nodo y el tiempo de espera del micro en e loop.

import rospy
from std_msgs.msg import Int16

class Alive:
    
    def __init__(self, node_name_override = 'alive'):
		
        # El nombre del node deberìa poder especificarse como override desde alguna
        # instancia superior, por si acaso, se establece un default razonable.
        # De todos modos, el nombre del nodo que se guarda "internamente" no puede
        # asumirse igual al proporcionado, por si se invocasen al mismo tiempo varios
        # nodos del mismo "tipo", pues ROS les anexa un número al final como ID
        rospy.init_node(node_name_override)
        self.nodename = rospy.get_name()
        # loginfo sirve como medio de DEBUGGING, y para guardar un registro de la
        # ejecución del programa. Por ejemplo, algón overnodo podría hacer uso del log
        # para arreglar errores
        rospy.loginfo("alive starting with name %s", self.nodename) 
        # Quizá algún parámetro superior especifique una frecuencia de salida de datos,
        # por default, si no se encuentra, se usa una de 1 Hz 
        self.rate = rospy.get_param("alive_rate", 1)

        self.alive_val = 1

        # Publicación de la batería como entero
        self.alivePub = rospy.Publisher('alive', Int16)  

    def update(self):
		
        self.alivePub.publish(self.alive_val)

    def spin(self):
		
        # rospy.Rate y sleep permiten que el nodo pare su ejecución (cuidado, los subscriptores
        # NO están restringidos por el resto del nodo más allá de la función de callback) e 
        # intente ejecutarse cada tantos ms, dados por el rate declarado en el constructor de la
        # clase. Ahorra recursos
        r = rospy.Rate(self.rate)
        while not rospy.is_shutdown():
            self.update()
            r.sleep()


# Invocación del constructor. Básicamente, si este archivo fuera a ejecutarse como "top level", el
# intérprete le asignaría el "nombre" (protegido) __main__ y ejecutaría las siguientes lineas como
# LO PRIMERO en el archivo (salvo imports y declaraciones)    
if __name__ == '__main__':
    """ main """
    alive = Alive()
    alive.spin()

