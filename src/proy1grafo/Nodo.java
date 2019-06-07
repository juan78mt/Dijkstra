/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

package proy1grafo;

/**
 *
 * @author labcic
 */

import java.io.*;
import java.util.*;

public class Nodo implements Comparator<Nodo>
{
    public int id;
    public char Nombre;
    public char datos;
    //coordenadas para metodo geometrico
    public float x;
    public float y;
    //Inicialmente ningun nodo ha sido visitado
    //public boolean visitado = false;
    
    public int node; 
    public int cost; 

    public Nodo(int id,float x, float y ) {
        this.id = id;
        this.x = x;
        this.y = y;
    }
    
    public Nodo(int node, int cost) 
    { 
        this.node = node; 
        this.cost = cost; 
    }

    public Nodo() {
       // throw new UnsupportedOperationException("Not supported yet."); //To change body of generated methods, choose Tools | Templates.
    }
    
    @Override
    public int compare(Nodo node1, Nodo node2) 
    { 
        if (node1.cost < node2.cost) 
            return -1; 
        if (node1.cost > node2.cost) 
            return 1; 
        return 0; 
    } 

    public int getId() {
        return id;
    }

    public void setId(int id) {
        this.id = id;
    }
    

    public void setX(float x) {
        this.x = x;
    }

    public void setY(float y) {
        this.y = y;
    }
   

    public float getX() {
        return x;
    }

    public float getY() {
        return y;
    }
    
 
    
    
    
}//fin de clase Nodo
