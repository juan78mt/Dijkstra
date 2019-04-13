/*PROYECTO 1 DE ANALISIS Y DISEÑO DE ALGORITMOS  Grafos aleatorios

  Instituto Politecnico Nacional
  Centro de Investigación en Computación  

  Prof. Dr. Rolando Quintero Téllez
  Alumno. Juan Carlos López Núñez


FUENTES DE INFORMACION

https://www.youtube.com/watch?v=jC2I15OU1Rk
Árboles N-arios en JAVA con Graphviz
https://www.youtube.com/watch?v=MLdFhIOIB84
gRAPHvIZ INTRO
https://www.youtube.com/watch?v=0SGfbLrJyJI

http://aprendiendo-software.blogspot.com/2012/11/como-ejecutar-graphviz-desde-java.html
https://github.com/nidi3/graphviz-java

Ejemplo de uso de hashmap
https://www.javatpoint.com/java-hashmap

http://www.rdebug.com/2010/05/usar-graphviz-desde-java.html

Implementing a random graph
https://www.youtube.com/watch?v=Vuv4v_4n7sM
Erdos Reyni
https://www.geeksforgeeks.org/erdos-renyl-model-generating-random-graphs/

 */

package proy1grafo;



import java.io.BufferedWriter;
import java.util.HashMap;
import java.util.Map;
import java.util.Iterator;
import java.util.Set;

import java.util.Random;

import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.io.Writer;
import java.util.InputMismatchException;

import java.util.Scanner;


public class Proy1Grafo{
    
    //roy1Grafo graph = new Proy1Grafo();
  
    public static void main(String[] args) {
        
        Scanner leer = new Scanner(System.in);
        HashMap<Integer, Integer> grafo = new HashMap<>();
        Random rand = new Random();
        //generacion de numeros aleatorios
        //del 0 al 50 
        int rand_int = rand.nextInt(50);
        
        //Numero de nodos y numero de aristas
        int num_nodos, num_aristas;
              
        
        //AJUSTAR NUMERO DE NODOS Y NUMERO DE ARISTAS
        System.out.println("Ingrese el numero de nodos: ");
        num_nodos = leer.nextInt();
        System.out.println("Ingrese el numero de aristas");
        num_aristas = leer.nextInt();
        
        //generacion aleatoria de nodos y de aristas
        int randNodo1, randNodo2;       
        int val1, val2;            
       
        for(Map.Entry grafoGenerado: grafo.entrySet() ){
            //llave y valor
            randNodo1 = rand.nextInt(num_aristas);
            randNodo2 = rand.nextInt(num_aristas);
            
            grafo.put(randNodo1, randNodo2);            
        }
        
        
        //llamar a metodo para crear nodos
        
        int i;
        int e = num_aristas;
        int v = num_nodos;
        
        
        if(e > v*(v-1)/2){
            System.out.println("Demasiadas aristas");
        }
        if( e < 0)
            System.out.println("Demasiados pocas aristas");
        else{ 
            //generacion del grafo aleatorio
            Grafo grafo1 = new Grafo();

           // HashMap<Integer, Integer> establecer = new HashMap<>();
         
            //no dirigido y sin ciclos
            boolean dirigido=false;
            boolean ciclos=false;
            //obtener otros parametros         
        
                try {
                    System.out.println("¿Desea que el grafo sea dirigido?: ");
                    Scanner n = new Scanner(System.in);
                    int bn = n.nextInt();
                    if (bn == 1) {
                        dirigido = false;
                        System.out.println("El grafo no es dirigido");
                    } else{
                        dirigido = false;
                        System.out.println("El grafo no es dirigido");
                    }

                }catch (InputMismatchException k) {
                    System.out.println("Entrada invalida!");
                }
           
            
            
         
                try {
                    System.out.println("¿Permite ciclos en el grafo?: ");
                    Scanner n = new Scanner(System.in);
                    int bn = n.nextInt();
                    if (bn == 1) {
                        ciclos = false;
                        System.out.println("El grafo no tiene ciclos");
                    } else{
                        ciclos = false;
                        System.out.println("El grafo no tiene ciclos");
                    }

                }catch (InputMismatchException k) {
                    System.out.println("Entrada invalida!");
                }
                
                float rprob;
                System.out.println("Se debe usar una probabilidad entre 0 -1 para el método geometrico");
                Scanner lector2 = new Scanner(System.in);
                rprob= lector2.nextFloat();
                
                System.out.println("Indicar grado para el generador de grafos por el metodo de Barabasi ");
                Scanner lector3 = new Scanner(System.in);
                v= lector3.nextInt();

            //agregar verificacion de datos        
                      
            char c;
            String opcion="";
            
            
                      
            do{
                
            System.out.println("a.-Erdos");
            System.out.println("b.-Gilbert");
            System.out.println("c.-Geométrico");
            System.out.println("d.-Barabasi");   
            //caracter de salida
            System.out.println("Elige la opcion de generacion del grafo");
            c = leer.next().charAt(0);

                switch(c){
                        case 'a': 
                            //grafo1.crearGrafoErdos(num_nodos, e, dirigido, ciclos);
                            System.out.println(num_nodos+" " +" "+ num_aristas);
                            grafo1.crearGrafoErdos(num_nodos, num_aristas, dirigido, ciclos);
                            
                                    //graphescribirArchivo(grafo1.crearGrafoErdos(num_nodos, e, dirigido, ciclos));
                        break;
                        case 'b':
                            double p = 0.0;
                            System.out.println("Se debe usar una probabilidad entre 0 -1 ");
                            Scanner lector = new Scanner(System.in);
                            p = lector.nextDouble();

                            grafo1.crearGrafoGilbert(num_nodos, p, dirigido, ciclos);
                        break;

                        case 'c':                         
                           grafo1.crearGrafoGeo(num_nodos, rprob, dirigido, ciclos);
                        
                        break;
                        case 'd':                 
                          
                            grafo1.crearGrafoBarabasi(num_nodos, v, dirigido, ciclos);
                        break;

                        default: 
                            System.out.println("OPcion incorrecta, intente de nuevo");
                        break;
            }
            //leer caracter de salida
            System.out.println("Desea una nueva seleccion?('a'|cualquier otro caracter para salir )");
            c = leer.next().charAt(0);
            
            }while(c == 'a');
            
        }//fin de if else
        
        System.out.println("Fin del programa");
            
    
    }//fin del metodo main
    
    
    

}//fin de clase



