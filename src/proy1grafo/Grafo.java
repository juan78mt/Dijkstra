/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

package proy1grafo;


import java.io.*;
import java.util.*;

public class Grafo 
{
    //numero de nodos
    public int V;
    //LinkedList para guardar el grafo por medio de representacion de lista de adyacencia
    public LinkedList<Integer> adj[];
    public LinkedList<Integer>[] adj2;
    
    
    public LinkedList<Arista> [] adjacencylist;
    //dijkstra    
    public int origen;
    //
    public int dist[]; 
    public Set<Integer> settled; 
    public PriorityQueue<Nodo> pq; 
   // private int V; // Number of vertices 
    public List<List<Nodo> > adjdk; 
    
  
    //parametro para el algoritmo de Barabasi
    int g = 4;       
    //creacion de grafo nodos y aristas
    //nodoi -> nodoj
    public HashMap<Integer, Nodo> nodos = new HashMap<Integer,Nodo>();
    //llave y numero de nodo
    public HashMap<Integer, Integer> aristas = new HashMap<Integer,Integer>();
  
           
   // static class Graph {
    int vertices;
    ArrayList<Arista> allEdges = new ArrayList<>();
        
         
    public Grafo(int V){
        //grafo generado   
        
        this.V = V;
        adj = new LinkedList[V];
        for(int i = 0; i< V; ++i){
            adj[i] = new LinkedList();
        }
        
        adj2 = new LinkedList[V];
        for(int i = 0; i< adj2.length; i++){
            adj2[i] = new LinkedList<Integer>();
        }
        
        dist = new int[V]; 
        settled = new HashSet<Integer>(); 
        pq = new PriorityQueue<Nodo>(V, new Nodo()); 
        
        // Adjacency list representation of the  
        // connected edges 
        List<List<Nodo> > adjdk = new ArrayList<List<Nodo> >(); 
  
        // Initialize list for every node 
        for (int i = 0; i < V; i++) { 
            List<Nodo> item = new ArrayList<Nodo>(); 
            adjdk.add(item); 
        }
        
        //prim
           this.vertices = vertices;
            adjacencylist = new LinkedList[vertices];
            //initialize adjacency lists for all the vertices
            for (int i = 0; i <vertices ; i++) {
                adjacencylist[i] = new LinkedList<>();
            }
        
    
        
    }
    
  

        public void addEdge(int source, int destination, int weight) {
            Arista edgei = new Arista(source, destination, weight);
            allEdges.add(edgei); //add to total edges
        }
        
        public void kruskalMST(){
            PriorityQueue<Arista> pq = new PriorityQueue<>(allEdges.size(), Comparator.comparingInt(o -> o.peso));

            //add all the edges to priority queue, //sort the edges on weights
            for (int i = 0; i <allEdges.size() ; i++) {
                pq.add(allEdges.get(i));
            }

            //create a parent []
            int [] parent = new int[vertices];

            //makeset
            makeSet(parent);

            ArrayList<Arista> mst = new ArrayList<>();

            //process vertices - 1 edges
            int index = 0;
            while(index<vertices-1){
                Arista edge = pq.remove();
                //check if adding this edge creates a cycle
                int x_set = find(parent, edge.nodoOrigen);
                int y_set = find(parent, edge.nodoDestino);

                if(x_set==y_set){
                    //ignore, will create cycle
                }else {
                    //add it to our final result
                    mst.add(edge);
                    index++;
                    union(parent,x_set,y_set);
                }
            }
            //print MST
            System.out.println("Minimum Spanning Tree: ");
            printGraph(mst);
        }

        public void makeSet(int [] parent){
            //Make set- creating a new element with a parent pointer to itself.
            for (int i = 0; i <vertices ; i++) {
                parent[i] = i;
            }
        }

        public int find(int [] parent, int vertex){
            //chain of parent pointers from x upwards through the tree
            // until an element is reached whose parent is itself
            if(parent[vertex]!=vertex)
                return find(parent, parent[vertex]);;
            return vertex;
        }

        public void union(int [] parent, int x, int y){
            int x_set_parent = find(parent, x);
            int y_set_parent = find(parent, y);
            //make x as parent of y
            parent[y_set_parent] = x_set_parent;
        }

        
        public void printGraph(ArrayList<Arista> edgeList){
            for (int i = 0; i <edgeList.size() ; i++) {
                Arista edge = edgeList.get(i);
                System.out.println("Edge-" + i + " source: " + edge.nodoOrigen +
                        " destination: " + edge.nodoDestino +
                        " weight: " + edge.peso);
            }
        }
   // }//fin de la clase estatica grafo
    
       
     //------FIN DE LOS METODOS PARA GRAFO DE KRUSKAL-----------------------  
    
        
     //int vertices;
     //LinkedList<Arista>[] adjacencylist;
        
     static class HeapNode{
        int vertex;
        int key;
    }

    static class ResultSet {
        int parent;
        int weight;
    }



        public void addEgde(int source, int destination, int weight) {
            Arista edge = new Arista(source, destination, weight);
            adjacencylist[source].addFirst(edge);

            edge = new Arista(destination, source, weight);
            adjacencylist[destination].addFirst(edge); //for undirected graph
        }

        public void primMST(){

            boolean[] inPriorityQueue = new boolean[vertices];
            ResultSet[] resultSet = new ResultSet[vertices];
            int [] key = new int[vertices];  //keys used to store the key to know whether priority queue update is required

//          //create heapNode for all the vertices
            HeapNode [] heapNodes = new HeapNode[vertices];
            for (int i = 0; i <vertices ; i++) {
                heapNodes[i] = new HeapNode();
                heapNodes[i].vertex = i;
                heapNodes[i].key = Integer.MAX_VALUE;
                resultSet[i] = new ResultSet();
                resultSet[i].parent = -1;
                inPriorityQueue[i] = true;
                key[i] = Integer.MAX_VALUE;
            }

            //decrease the key for the first index
            heapNodes[0].key = 0;

            //add all the vertices to the priority queue
            PriorityQueue<HeapNode> pq = new PriorityQueue<>(vertices,
                    new Comparator<HeapNode>() {
                        @Override
                        public int compare(HeapNode o1, HeapNode o2) {
                            return o1.key -o2.key;
                        }
                    });
            //add all the vertices to priority queue
            for (int i = 0; i <vertices ; i++) {
                pq.offer(heapNodes[i]);
            }

            //while priority queue is not empty
            while(!pq.isEmpty()){
                //extract the min
                HeapNode extractedNode = pq.poll();

                //extracted vertex
                int extractedVertex = extractedNode.vertex;
                inPriorityQueue[extractedVertex] = false;

                //iterate through all the adjacent vertices
                LinkedList<Arista> list = adjacencylist[extractedVertex];
                for (int i = 0; i <list.size() ; i++) {
                    Arista edge = list.get(i);
                    //only if edge destination is present in heap
                    if(inPriorityQueue[edge.nodoDestino]) {
                        int destination = edge.nodoDestino;
                        int newKey = edge.peso;
                        //check if updated key < existing key, if yes, update if
                        if(key[destination]>newKey) {
                            decreaseKey(pq, newKey, destination);
                            //update the parent node for destination
                            resultSet[destination].parent = extractedVertex;
                            resultSet[destination].weight = newKey;
                            key[destination] = newKey;
                        }
                    }
                }
            }
            //print mst
            printMST(resultSet);
        }

        public void decreaseKey(PriorityQueue<HeapNode> pq, int newKey, int vertex){

            //iterate through nodes in priority queue and update the key for the vertex
            Iterator it = pq.iterator();

            while (it.hasNext()) {
                HeapNode heapNode = (HeapNode) it.next();
                if(heapNode.vertex==vertex) {
                    pq.remove(heapNode);
                    heapNode.key = newKey;
                    pq.offer(heapNode);
                    break;
                }
            }
        }

        public void printMST(ResultSet[] resultSet){
            int total_min_weight = 0;
            System.out.println("Minimum Spanning Tree: ");
            for (int i = 1; i <vertices ; i++) {
                System.out.println("Edge: " + i + " - " + resultSet[i].parent +
                        " key: " + resultSet[i].weight);
                total_min_weight += resultSet[i].weight;
            }
            System.out.println("Total minimum key: " + total_min_weight);
        }
        
        
        //-------------FIN DE LOS METODOS PARA EL ALGORITMO DE PRIM---------------------------------------------
 
    //generar numero de nodos
    public void generarNodos(int numNodos)
    {
        int rand_intVal;       
         //crear nodos llave valor de forma aleatorio
         Random rand = new Random();
         
        for(int i = 1; i < numNodos; i++){
              float coorx = rand.nextFloat();        
              float coory = rand.nextFloat();
              Nodo nodoi = new Nodo(i, coorx, coory); 
              
               nodoi.x = coorx;
               nodoi.y = coory;
               nodoi.id = i;
               nodos.put(i,nodoi);  
               
        }     
    }
    
    public void obtenerNodoxy(int n)
    {
        if(nodos.containsKey(n)){
            System.out.print("El codigo " + n + " corresponde a ");
            System.out.println(nodos.get(n));
            
      
        }else{
            System.out.println("El codigo introducido no existe");
        }
    }
    //asocia nodos
    public void crearArista(int nodoi, int nodoj)
    {
        Random rand = new Random();
        int costo = rand.nextInt(10);
        
         adj[nodoi].add(nodoj);    
         adj2[nodoi].add(nodoj);    
         aristas.put(nodoi, nodoj);
         //adjdk.get(origen).add(new Nodo(nodoi, costo));

         //valido para grafos dirigidos 
         //las aristas se seleccionan de forma aleatoria
      //  Arista arista = new Arista(nodoi, nodoj, costo);
     //   adjacencylist[origen].addFirst(arista);
    }
    
    public void crearAristaPeso(Grafo grafo1, int nodoi,int nodoj, int peso)
    {

        grafo1.addEdge(nodoi, nodoj, peso);

    }
    
    //creación del grafo
    //Erdos
    //Gilbert
    //geo
    
    /*
    n-nodos
    m- aristas o vertices
    d-dirigido o no dirigido
    a-permitir ciclos o no ciclos    
    */
    public void crearGrafoErdos(int n, int m, boolean d, boolean a)
    {
        double p = 0.0;
        Random rand = new Random();
        int prob = rand.nextInt();
       // Grafo grafo1 = new Grafo(n);
        float probNormal;
        
        //creacion de grafo para Kruskal
        Grafo grafoPeso = new Grafo(m);
        
        
       //inicializacion para dijkstra 
       List<List<Nodo> > adjdk = new ArrayList<List<Nodo> >(); 
            
        // Initialize list for every node 
        for (int ig = 0; ig < V; ig++) { 
            List<Nodo> item = new ArrayList<Nodo>(); 
            adjdk.add(item); 
        } 
    
       
        
       // grafo1.generarNodos(n);
        Random rande = new Random();
        int val1, val2, val3;
        int i = 1;
        int aristasGeneradas = 0;
        for(int n_i = 1; n_i < n && aristasGeneradas < m ; n_i++){
            for(int n_j = 1; n_j < m; n_j++){
                probNormal = rand.nextFloat();
                System.out.println(probNormal);
                if(probNormal <0.5){
                    
                    //nodos
                    val1 = rande.nextInt(n);
                    val2 = rande.nextInt(n);
                    //peso de la arista
                    val3 = rande.nextInt(n);
                    
                    crearArista(val1, val2);
                    crearAristaPeso( grafoPeso ,val1, val2, val3);
                    //adjdk.get(origen).add(new Nodo(val1, val2));
              
                    
                    aristasGeneradas++;
                    System.out.println(i+ "arista creada");
                    i++;
                    if(aristasGeneradas == m) break;
                }
            }
        }
        
        ejecutarDijkstra(origen);
        
        escribirArchivo();
        imprimirGrafo(aristas);
       
        
        
    }//fin de ERdos
    
     /*
    n-nodos
    p- probabilidad de crear un par de nodos de generar una arista
    d-dirigido o no dirigido
    a-permitir ciclos o no ciclos    
    */    
    public void crearGrafoGilbert(int n, double p, boolean d, boolean a)
    {
        
        Random rand = new Random();
        int prob = rand.nextInt();
        //Grafo grafo1 = new Grafo(n);
        //grafo1.generarNodos(n);
        int val1, val2;
       // int aristasGeneradas = 0;
        for(int n_i = 1; n_i < n; n_i++){
            for(int n_j = 1; n_j < n; n_j++){
                
                   val1 = rand.nextInt(n);
                   val2 = rand.nextInt(n);
              
                   crearArista(val1, val2);
            }
        }
        
        escribirArchivo();
        imprimirGrafo(aristas);
    }
    
     /*Geografico
    n-nodos
    r-aristas
    d-dirigido o no dirigido
    a-permitir ciclos o no ciclos    
    */
    public Grafo crearGrafoGeo(int n, float r, boolean d, boolean a)
    {        
        Random rand = new Random();
        int prob = rand.nextInt();
        Grafo grafo1 = new Grafo(n);
        grafo1.generarNodos(n);
        
        int probNormal;
    
        int id;
        
        double dist;
        Nodo r1, r2, r3,r4;
        
        float ri, rj;
        
        Random randt = new Random();
        int val1, val2;
      
        //crear aristas 
         for(int n_i = 1; n_i < n; n_i++){
                for(int n_j = 1; n_j < n; n_j++){  

                
                r1 = grafo1.nodos.get(n_i);               
                r2 = grafo1.nodos.get(n_i);
                if(n_i > 1){                    
                    r3 = grafo1.nodos.get(n_i-1);
                    r4 = grafo1.nodos.get(n_i-1);
                    
                    ri = r1.x - r3.x;
                    rj = r2.y - r4.y;
                    
                    dist = Math.sqrt(Math.pow(ri, 2) + Math.pow(rj, 2)  );
                    
               // probNormal = rand.nextInt();
                    if(dist < r){
                        val1 = randt.nextInt(n);
                        val2 = randt.nextInt(n);
                        grafo1.crearArista(val1, val2);
                    }
                }
                
                //System.out.println(r1);
             
                    
            }       
        }
        
    grafo1.escribirArchivoGeo(grafo1);
    grafo1.imprimirGrafoGeo(grafo1.aristas, grafo1);
    
    return grafo1;
           
    }
    
     /*
    n-nodos
    g = 4  grado
    d-dirigido o no dirigido
    a-permitir ciclos o no ciclos    
    */
    public void crearGrafoBarabasi(int n, int g, boolean d, boolean a)
    {
        Random rand = new Random();
        int prob = rand.nextInt();
        
      //  Grafo grafo1 = new Grafo(n);
      //  grafo1.generarNodos(n);
        //calculo de probabilidad        
        float volado;
        float p;
        int grado = 0;
        
        Random randg = new Random();
        int val1, val2;
      
        for(int n_i = 1; n_i < n; n_i++){
            for(int n_j = 1; n_j < n; n_j++){
               
                volado = rand.nextFloat();  //calcular el volado
                p = 1 - grado / g;          //obtener probabilidad
                if(volado < p){
                    val1 = randg.nextInt(n);
                    val2 = randg.nextInt(n);                    
                    crearArista(val1, val2);
                }
                
            }
            grado++;   //suma del grado
        }
        
    escribirArchivo();
    imprimirGrafo(aristas);
              
    }
    
   //GRAFO EXPLORADO

    //nodo origen s
    void BFS(int s) 
    {   
        boolean visited[] = new boolean[V]; 
        Grafo grafo2 = new Grafo(V);
       // System.out.println("s"+ s + "V" + V);

        LinkedList<Integer> queue = new LinkedList<Integer>(); 
      
        visited[s]=true; 
        queue.add(s); 
  
        while (!queue.isEmpty()){
               
            s = queue.poll(); 
            System.out.print(s+" "); 

            Iterator<Integer> i =adj[s].listIterator();
            //int i2 = 0;         
            while (i.hasNext()){
              //  System.out.println("i.hasNext  "+ s+" "); 
                int n = i.next(); 
                if (!visited[n]){
                //    System.out.println("visited  "+ s+" "); 
                    visited[n] = true; 
                    queue.add(n);
                    
                    
                    grafo2.crearArista(s, n);
                   // System.out.println(s+ "--"+ n);
                } 
            } 
        }
       // System.out.println("fin de while");
        
        escribirArchivo();
        imprimirGrafo(aristas);   
    } //fin BFS 
    

    
    public void DFSiterativo(int s)
    {        
        Vector<Boolean> visited = new Vector<Boolean>(V); 
        for (int i = 0; i < V; i++) 
            visited.add(false); 
        Grafo grafo2 = new Grafo(V);

        Stack<Integer> stack = new Stack<>(); 
        stack.push(s); 

        while(stack.empty() == false)
        { 
            s = stack.peek(); 
            stack.pop(); 
         
            if(visited.get(s) == false) 
            {
                System.out.println(s + " ");
                visited.set(s,true);
           
            } 

            Iterator<Integer> itr= adj2[s].iterator(); 

            while(itr.hasNext()){
                
                int vn = itr.next(); 
                if(!visited.get(vn)){ 
                    stack.push(vn);
                    grafo2.crearArista(s, vn); 
                }
            } 

        }//fin while
        
    escribirArchivo();
    imprimirGrafo(aristas);      
     
    }//fin DFS iterativo 
    

    
    void recursionDFS(int v,boolean visited[]) 
    {       
        visited[v] = true; 
        System.out.print(v+" "); 
        
        Grafo grafo2 = new Grafo(V);
       
        Iterator<Integer> i = adj[v].listIterator(); 
        while (i.hasNext()){            
            int n = i.next();            
            if (!visited[n]){                              
                recursionDFS( n, visited);
                crearArista(n, v); 
            } 
        }
        
      escribirArchivo();
      imprimirGrafo(aristas);
        
    }//fin de DFS recursivo 
  
    void DFSrecursivo(int v) 
    {      
        boolean visited[] = new boolean[V]; 
 
        recursionDFS(v, visited); 
    } 
    
    //--------------------ALGORITMOS BFS DFSi  y DFSr usados para el grafo geometrico-----------------------------------------
    
     //nodo origen s
    void BFSGEO(int s,Grafo grafoa) 
    {   
        boolean visited[] = new boolean[V]; 
        Grafo grafo2 = new Grafo(V);
       // System.out.println("s"+ s + "V" + V);

        LinkedList<Integer> queue = new LinkedList<Integer>(); 
      
        visited[s]=true; 
        queue.add(s); 
  
        while (!queue.isEmpty()){
               
            s = queue.poll(); 
            System.out.print(s+" "); 

            Iterator<Integer> i = grafoa.adj[s].listIterator();
            //int i2 = 0;         
            while (i.hasNext()){
              //  System.out.println("i.hasNext  "+ s+" "); 
                int n = i.next(); 
                if (!visited[n]){
                //    System.out.println("visited  "+ s+" "); 
                    visited[n] = true; 
                    queue.add(n);
                    grafo2.crearArista(s, n);
                   // System.out.println(s+ "--"+ n);
                } 
            } 
        }
       // System.out.println("fin de while");
        
        grafo2.escribirArchivoGeo(grafo2);
        grafo2.imprimirGrafoGeo(grafo2.aristas, grafo2); 
    } //fin BFS 
    

    
    public void DFSiterativoGEO(int s,Grafo grafoa)
    {        
        Vector<Boolean> visited = new Vector<Boolean>(V); 
        for (int i = 0; i < V; i++) 
            visited.add(false); 
        Grafo grafo2 = new Grafo(V);

        Stack<Integer> stack = new Stack<>(); 
        stack.push(s); 

        while(stack.empty() == false)
        { 
            s = stack.peek(); 
            stack.pop(); 
         
            if(visited.get(s) == false) 
            {
                System.out.println(s + " ");
                visited.set(s,true);
           
            } 

            Iterator<Integer> itr= grafoa.adj2[s].iterator(); 

            while(itr.hasNext()){
                
                int vn = itr.next(); 
                if(!visited.get(vn)){ 
                    stack.push(vn);
                    grafo2.crearArista(s, vn); 
                }
            } 

        }//fin while
        
    grafo2.escribirArchivoGeo(grafo2);
    grafo2.imprimirGrafoGeo(grafo2.aristas, grafo2);      
     
    }//fin DFS iterativo 
    
    void kruskal(Grafo grafoFig, int n, int m, boolean d, boolean a)
    {
        grafoFig.crearGrafoErdos( n,  m,  d,  a);
        grafoFig.kruskalMST();
        
    }
    

    
    void recursionDFSGEO(int v,boolean visited[],Grafo grafoa, Grafo grafo2) 
    {       
        visited[v] = true; 
        System.out.println(v+" "); 
        
        
       
        Iterator<Integer> i = grafoa.adj[v].listIterator(); 
        while (i.hasNext()){            
            int n = i.next();            
            if (!visited[n]){                              
                recursionDFSGEO( n, visited, grafoa, grafo2);
                grafo2.crearArista(n, v); 
            } 
        }     
        
    }//fin de DFS recursivo 
    
    void imprimirDFSGEO(Grafo grafo2)
    {
        grafo2.escribirArchivoGeo(grafo2);
        grafo2.imprimirGrafoGeo(grafo2.aristas, grafo2); 
    }
  
    void DFSrecursivoGEO(int v, Grafo grafoa) 
    {      
        boolean visited[] = new boolean[V]; 
        Grafo grafo2 = new Grafo(V);
 
        recursionDFSGEO(v, visited, grafoa, grafo2); 
        imprimirDFSGEO(grafo2);
    }
    
    
    //-------------------------------------------------------------------------------
    
    void dijkstra2(int s,Grafo grafoa) 
    {   
        boolean visited[] = new boolean[V]; 
        Grafo grafo2 = new Grafo(V);
       // System.out.println("s"+ s + "V" + V);

        LinkedList<Integer> priorityqueue = new LinkedList<Integer>(); 
      
        visited[s]=true; 
        priorityqueue.add(s); 
        
        Random aleatorio = new Random();
       
  
        while (!priorityqueue.isEmpty()){
               
            s = priorityqueue.poll(); 
            System.out.print(s+" "); 

            Iterator<Integer> i = grafoa.adj[s].listIterator();
            //int i2 = 0;         
            
            
            while (i.hasNext()){
              //  System.out.println("i.hasNext  "+ s+" "); 
                int n = i.next(); 
                if (!visited[n]){
                //    System.out.println("visited  "+ s+" "); 
                    visited[n] = true; 
                    priorityqueue.add(n);
                    grafo2.crearArista(s, n);
                   // System.out.println(s+ "--"+ n);
                    int costo = 1 + aleatorio.nextInt(10);
                    
                    //calcular distancias
                    //si la nueva distancia es menor que la distancia actua
                    //entonces esta se actualiza como la nueva distancia
                  
                     
                    
                    
                   // grafo2.adjdk.get(origen).add(new Nodo(s, costo));
                } 
            } 
        }
       // System.out.println("fin de while");
       // grafo2.ejecutarDijkstra(s);
        
        grafo2.escribirArchivoGeo(grafo2);
        grafo2.imprimirGrafoGeo(grafo2.aristas, grafo2); 
    }    
    
    
  
    //--------------------------------------------------------------------------------
    public void ejecutarDijkstra(int origen)
    {
        Grafo dpq = new Grafo(V); 
        boolean [] visited = new boolean[V];
        visited[0] = true; 
        
        this.origen = origen;
        
        Random rand = new Random();
        int costo = rand.nextInt(10);
        dijkstra2(origen,dpq); 

       // dpq.dijkstra(adjdk, origen, dpq); 
        
    }
    
    
    public void ejecutarPrim()
    {
           Grafo dpq = new Grafo(V); 
        boolean [] visited = new boolean[V];
        visited[0] = true; 
        
        this.origen = origen;
        
        Random rand = new Random();
        int costo = rand.nextInt(10);
        //dijkstra2(origen,dpq); 
        primMST();

        
    }
    
    public void ejecutarKruskal()
    {
           Grafo dpq = new Grafo(V); 
        boolean [] visited = new boolean[V];
        visited[0] = true; 
        
        this.origen = origen;
        
        Random rand = new Random();
        int costo = rand.nextInt(10);
        //dijkstra2(origen,dpq); 
        kruskalMST();

        
    }
    
    
    
    public void dijkstra(List<List<Nodo> > adjd, int origen, Grafo dpq) 
    {
       // Grafo grafod = new Grafo(V);
        
        this.adjdk = adjd; 
  
        for(int i = 0; i < V; i++) 
            dist[i] = Integer.MAX_VALUE; 
  
        // Add source node to the priority queue 
        pq.add(new Nodo(origen, 0)); 
          
        // Distance to the source is 0 
        dist[origen] = 0; 
        while(settled.size() != V){ 
  
            // remove the minimum distance node  
            // from the priority queue  
            int u = pq.remove().node; 
  
            // adding the node whose distance is 
            // finalized 
            settled.add(u); 
            
  
            e_Neighbours(u, origen);
             
        }
        
            //escribirArchivo();
            //imprimirGrafo(aristas);
            
            
              //imprime todos los nodos a partir del nodo origen   
        System.out.println("Camino mas corto del nodo origen :"); 
        for (int i = 0; i < dpq.dist.length; i++) 
            System.out.println(origen + " to " + i + " is " + dpq.dist[i]); 
   
        
        imprimirDijkstra(dpq);
        
    }
    
     // Calculate the single source shortest path 
        void imprimirDijkstra(Grafo grafo2)
        {
            grafo2.escribirArchivo();
            grafo2.imprimirGrafoDijkstra(grafo2.aristas, grafo2); 
        }
     
     // Function to process all the neighbours  
    // of the passed node 
    private void e_Neighbours(int u, int origen) 
    { 
        int edgeDistance = -1; 
        int newDistance = -1; 
  
        // All the neighbors of v 
        for (int i = 0; i < adjdk.get(u).size(); i++) 
        { 
            Nodo v = adjdk.get(u).get(i); 
  
            // If current node hasn't already been processed 
            if (!settled.contains(v.node)) { 
                edgeDistance = v.cost; 
                newDistance = dist[u] + edgeDistance; 
  
                // If new distance is cheaper in cost 
                if (newDistance < dist[v.node]) 
                    dist[v.node] = newDistance; 
                
                Random rand = new Random();
                int costo = rand.nextInt();
  
                // Add the current node to the queue 
                pq.add(new Nodo(v.node, dist[v.node])); 
                
       
            } 
        } 
    } 
    
    
    
    //-------------------------------------------------------------------------------------
    
    

    
    
    
    
    //------------------------------------------------------------------------------------
    

    
    //fomrato de archivo de grafo
    //concatenacion de los resultados del hashMap 
    //para el string  pasar el hashMap como 
    //nodoi -> nodoj
    @Override
    public String toString()
    {
        int a = 0;
        //cargar llave y valor como cadena
        String nodoCadena="";
        int val1, val2;
    
        String titulo = "digraph grafo1{"+"\n"; 
        for(Map.Entry grafo: aristas.entrySet()){
            //llave y valor
            val1 = (int) grafo.getKey();
            val2 = (int) grafo.getValue();        
 
            nodoCadena += Integer.toString(val1)+ "->"+Integer.toString(val2)+";\n";  
            
        }
        String cabecera = "}";
        
        //cadena concatenada
        String archivogv = titulo+nodoCadena+cabecera;
       // System.out.println("\n\n GRAFO IMPRESO." + adjList + "\n\n"); 
        
        return archivogv;
                
        
    }//fin de metodo ToString
    
    public void escribirArchivo()
    {
        String path="C:\\Users\\JUAN\\Desktop\\grafo1.gv";
           
        try { 
            File archivo = new File("C:\\Users\\JUAN\\Desktop\\grafo1.gv"); 
           // if (archivo.createNewFile())
            //{
                FileWriter fw = new FileWriter(archivo);             
                //BufferedWriter bw = new BufferedWriter(fw);
                //obtener la cadena
                String cadena = toString();

                // Escribir al archivo
                fw.write(cadena);

                // Cerrar conexion
                //bw.close();
                fw.close();

                System.out.println("ARCHIVO ESCRITO EXITOSAMENTE." + archivo.getName()); 
           // } else { 
            //  System.out.println("El archivo ya existe."); 
                        

            
          //  System.out.println("ARCHIVO ESCRITO EXITOSAMENTE.");
        } catch (IOException e) {
            System.out.println("UN ERROR OCURRIO.");
            //reporte de errores
            e.printStackTrace();
       } 
    }
    
    
    public void escribirArchivoGeo(Grafo grafo)
    {
        String path="C:\\Users\\JUAN\\Desktop\\grafo1.gv";
           
        try { 
            File archivo = new File("C:\\Users\\JUAN\\Desktop\\grafo1.gv"); 
           // if (archivo.createNewFile())
            //{
                FileWriter fw = new FileWriter(archivo);             
                //BufferedWriter bw = new BufferedWriter(fw);
                //obtener la cadena
                String cadena = grafo.toString();

                // Escribir al archivo
                fw.write(cadena);

                // Cerrar conexion
                //bw.close();
                fw.close();

                System.out.println("ARCHIVO ESCRITO EXITOSAMENTE." + archivo.getName()); 
           // } else { 
            //  System.out.println("El archivo ya existe."); 
                        

            
          //  System.out.println("ARCHIVO ESCRITO EXITOSAMENTE.");
        } catch (IOException e) {
            System.out.println("UN ERROR OCURRIO.");
            //reporte de errores
            e.printStackTrace();
       } 
    }
   
    public static void imprimirArbol(LinkedList list) 
    {       
        System.out.println("Arbol generado" + list);          
    } 
          
    public void imprimirGrafo(Map<Integer, Integer> Mapgrafo)  
    {
        //public void imprimirGrafo(Map<Integer, Integer> grafo)  
        if (Mapgrafo.isEmpty())  
        { 
            System.out.println("el grafo esta vacio"); 
        }           
        else
        {
            System.out.println("Valores de la adjunta");
            for(int i = 0; i <adj.length; i++)
                System.out.println(adj[i]); 
            System.out.println();
            
            int i = 1;//contador de numero de aristas
            System.out.println("grafo obtenido");
            for(Map.Entry impGrafo:  aristas.entrySet()){
                System.out.println(i +".-"+ impGrafo);
                i++;
            }
            
        } 
    }
    
    public void imprimirGrafoGeo(Map<Integer, Integer> Mapgrafo, Grafo objGrafo)  
    {
        //public void imprimirGrafo(Map<Integer, Integer> grafo)  
        if (Mapgrafo.isEmpty())  
        { 
            System.out.println("el grafo esta vacio"); 
        }           
        else
        {
            System.out.println("Valores de la adjunta");
            for(int i = 0; i < objGrafo.adj.length; i++)
                System.out.println(adj[i]); 
            System.out.println();
            
            int i = 1;//contador de numero de aristas
            System.out.println("grafo obtenido");
            for(Map.Entry impGrafo:  aristas.entrySet()){
                System.out.println(i +".-"+ impGrafo);
                i++;
            }
            
        } 
    }
    
    
    public void imprimirGrafoDijkstra(Map<Integer, Integer> Mapgrafo, Grafo objGrafo)  
    {
        //public void imprimirGrafo(Map<Integer, Integer> grafo)  
        if (Mapgrafo.isEmpty())  
        { 
            System.out.println("el grafo esta vacio"); 
        }           
        else
        {
            System.out.println("Valores de la adjunta");
            for(int i = 0; i < objGrafo.adj.length; i++)
                System.out.println(adj[i]); 
            System.out.println();
            
            int i = 1;//contador de numero de aristas
            System.out.println("grafo obtenido");
            for(Map.Entry impGrafo:  aristas.entrySet()){
                System.out.println(i +".-"+ impGrafo);
                i++;
            }
            
        } 
    } 





 
    
}//fin de la clase Grafo
