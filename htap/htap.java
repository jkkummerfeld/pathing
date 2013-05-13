import java.util.PriorityQueue;
import java.util.ArrayList;
import java.util.Collection;
import java.util.Collections;
import java.util.Hashtable;
import java.awt.Point;
import java.util.LinkedList;
import java.util.Arrays;
import java.util.Random;
import java.util.Stack;

class HTAP
{
    public static Hashtable<Point,Bloc> bloc(Vertex[] vertices, int precision)
    //Assigns each node a bloc and inserts the node into that bloc
    {
        //System.out.println("\n" + "Assigning blocs...");
        Hashtable<Point,Bloc> blocs = new Hashtable<Point,Bloc>();
        for (Vertex v : vertices) {
            int x = v.loc.x;
            int y = v.loc.y;
            int blocX = x/precision;
            int blocY = y/precision;
            Point key = new Point(blocX, blocY);
            v.bloc = key;
            Bloc currentBloc;
            if (blocs.get(key) == null) {
                currentBloc = new Bloc(blocX,blocY);
            } else {
                currentBloc = blocs.get(key);
            }
            currentBloc.add(v);
            blocs.put(key, currentBloc);
            //System.out.println("node " + v + " assigned to bloc " + currentBloc);
        }
        //System.out.println("Number of blocs formed: " + blocs.size());
        return blocs;
    }
    
    public static Vertex[] decimate(Hashtable<Point,Bloc> blocs)
    //Decimates a bloc by selecting a sole survivor node
    {
        Collection<Bloc> blocsAsCollection = blocs.values();
        Vertex[] survivors = new Vertex[blocsAsCollection.size()];
        int index = 0;
        //System.out.println("\n" + "Decimating...");
        for (Bloc b : blocsAsCollection) {
            double minRes = Double.POSITIVE_INFINITY;
            for (Vertex v : b.vertices) {
                double res = 0;
                for (Edge e : v.neighbors) {
                    res += 1/e.weight;
                }
                res = 1/res;
                if (res < minRes) {
                    minRes = res;
                    b.surv = v;
                }
            }
            survivors[index] = b.surv;
            index++;
            //System.out.println("node " + b.surv + " survived bloc " + b);
        }
        return survivors;
    }
    
    public static void voronoi(Vertex[] survivors)
    //Determines which Voronoi region each node belongs to
    //given a set of points which are at the center of the regions
    {
        //System.out.println("\n" + "Determining Voronoi Regions...");
        LinkedList<Vertex> queue = new LinkedList<Vertex>();
        for (Vertex v : survivors) {
            v.vRegion = v;
            queue.add(v);
        }
        while(!queue.isEmpty()) {
            Vertex v = queue.poll();
            LinkedList<Vertex> children = new LinkedList<Vertex>();
            for (Edge e : v.neighbors) {
                Vertex c = e.target;
                if (c.vRegion==null) {
                    children.add(c);
                }
            }
            while(!children.isEmpty()) {
                Vertex child = children.poll();
                v.verticesInVR.add(child);
                child.vRegion = v.vRegion;
                //System.out.println("node " + child + " assigned to VR " + v.vRegion);
                queue.add(child);
            }
        }
    }
    
    public static void link(Vertex[] survivors)
    //Places edges between survivor nodes whose voronoi regions are connected
    {
        //System.out.println("\n" + "Placing edges between survivor nodes...");
        for (int i = 0; i < survivors.length ; i++) {
            for (int j = i+1; j < survivors.length; j++) {
                Vertex u = survivors[i];
                Vertex v = survivors[j];
                if (u.vConnected(v) || v.vConnected(u)) {
                    Edge e1 = new Edge(u, v, 0);
                    Edge e2 = new Edge(v, u, 0);
                    if (!u.hasNeighbor(e1)) { u.newNeighbors.add(e1); }
                    if (!v.hasNeighbor(e2)) { v.newNeighbors.add(e2); }
                    //System.out.println("Added an edge between " + u + " and " + v);
                }
            }
        }
    }
    
    public static void setWeights(Vertex[] survivors)
    //Sets the weights of the newly placed edges between survivor nodes
    {
        //System.out.println("\n" + "Setting edge weights...");
        for (int i = 0; i < survivors.length ; i++) {
            for (int j = i+1; j < survivors.length ; j++) {
                Vertex u = survivors[i];
                Vertex v = survivors[j];
                if (u.vConnected(v) || v.vConnected(u)) {
                    ArrayList<Vertex> graph = new ArrayList<Vertex>();
                    for (Vertex child : u.verticesInVR) { graph.add(child); }
                    for (Vertex child : v.verticesInVR) { graph.add(child); }
                    graph.add(u);
                    graph.add(v);
                    Vertex[] graphAsArray = new Vertex[graph.size()];
                    graphAsArray = graph.toArray(graphAsArray);
                    double distUToV = Dijkstra.minDist(graphAsArray, u, v);
                    u.setNeighborWeight(v, distUToV);
                    v.setNeighborWeight(u, distUToV);
                    //System.out.println("Set weight between " + u + " and " + v + ": "
                        //+ String.format("%.3f", distUToV));
                }
            }
        }

        for (Vertex v : survivors) { v.neighbors = v.newNeighbors; }
    }
    
    public static void calibrateLocations(Vertex[] survivors) {
        for (Vertex v : survivors) { v.loc = v.bloc; }
    }
    
    public static void makePyramid(Vertex[] graph, int blocSize, int goal) {
        int size = graph.length*graph.length;
        while(size > goal) {
            Hashtable<Point,Bloc> blocs = bloc(graph,blocSize);
            size = blocs.size();
            Vertex[] survivors = decimate(blocs);
            voronoi(survivors);
            link(survivors);
            setWeights(survivors);
            calibrateLocations(survivors);
            graph = survivors;
        }
    }
    
    public static void main(String[] args)
    {
        int size = Integer.parseInt(args[0]);
        Vertex[] graph = new Vertex[size*size];
        for (int i = 0; i < size; i++) {
            for (int j = 0; j < size; j++) {
                String name = "("+Integer.toString(i)+","+Integer.toString(j)+")";
                graph[i*size+j] = new Vertex(name, i, j);
            }
        }

        int[] cardinalDirections = { 1, -1, size, -size };
        for (int index = 0; index < size*size; index++) {
            Vertex v = graph[index];
            Random rng = new Random();
            for (int offset : cardinalDirections) {
                int nIndex = index+offset;
                if ((rng.nextBoolean() || rng.nextBoolean())
                    && nIndex >= 0 && nIndex < size*size
                    && (nIndex/size == index/size || nIndex%size == index%size)) {
                    Vertex u = graph[nIndex];
                    double weight = rng.nextDouble();
                    if (!v.connected(u)) {
                        Edge e1 = new Edge(v, u, weight);
                        v.neighbors.add(e1);
                    }
                    if (!u.connected(v)) {
                        Edge e2 = new Edge(u, v, weight);
                        u.neighbors.add(e2);
                    }
                }
            }
        }
        Vertex source = graph[0];
        Vertex target = graph[size*size-1];
        
        //Dijkstra
        long startTimeD = System.nanoTime();
        double pathLength = Dijkstra.minDist(graph,source,target);
        ArrayList<Vertex> path = Dijkstra.getShortestPathTo(target);
        long endTimeD = System.nanoTime();
        long durationD = endTimeD - startTimeD;
        
        System.out.println("Dijkstra took " + durationD/1000000 + " ms");
        System.out.println("Path length: " + pathLength);
        System.out.println("Path: " + path);
        
        //HTAP
        long startTimeH = System.nanoTime();
        makePyramid(graph, 2, 4);
        Stack<Vertex> tempTargets = new Stack<Vertex>();
        Stack<Vertex> tempSources = new Stack<Vertex>();
        while (!Arrays.asList(graph).contains(target)
                || !Arrays.asList(graph).contains(source)) {
            tempTargets.push(target);
            tempSources.push(source);
            target = target.vRegion;
            source = source.vRegion;
        }
        double pathLengthH = 0.0;
        ArrayList<Vertex> pathAtThisLayer = new ArrayList<Vertex>();
        while (!tempTargets.empty()) {
            pathLengthH = Dijkstra.minDist(graph,source,target);
            pathAtThisLayer = Dijkstra.getShortestPathTo(target);
            
            ArrayList<Vertex> newGraph = new ArrayList<Vertex>();
            for (Vertex v : pathAtThisLayer) {
                for (Vertex u : v.verticesInVR) {
                    newGraph.add(u);
                }
            }
            graph = newGraph.toArray(graph);
            source = tempSources.pop();
            target = tempTargets.pop();
            long checkTime = System.nanoTime();
            if (checkTime-startTimeH > 50000000) { break; }
        }
        pathLengthH = Dijkstra.minDist(graph,source,target);
        pathAtThisLayer = Dijkstra.getShortestPathTo(target);
            
        long endTimeH = System.nanoTime();
        long durationH = endTimeH - startTimeH;
        
        System.out.println("HTAP took " + durationH/1000000 + " ms");
        System.out.println("Path length: " + pathLengthH);
        System.out.println("Path: " + pathAtThisLayer);
    }
    
    public static void printGraph(Vertex[] graph) {
        System.out.println("\n Displaying graph...");
        System.out.println("Vertices: " + Arrays.asList(graph));
        System.out.println("Voronoi Regions: ");
        for (Vertex v : graph) {
            System.out.print(v + " : ");
            for (Vertex u : v.verticesInVR) { System.out.print(u + ","); }
            System.out.println("");
        }
        System.out.println("Edges: ");
        for (Vertex v : graph) {
            for (Edge e : v.neighbors) { System.out.print(e + ","); }
            System.out.println("");
        }
    }        
}

class Vertex implements Comparable<Vertex>
{
    public final String name;
    public ArrayList<Edge> neighbors;
    public ArrayList<Edge> newNeighbors;
    public double minDistance = Double.POSITIVE_INFINITY;
    public Vertex prev;
    public Point loc;
    public Point bloc;
    public Vertex vRegion;
    public ArrayList<Vertex> verticesInVR;
    
    public Vertex(String argName, int argX, int argY) {
        name = argName;
        loc = new Point(argX, argY);
        neighbors = new ArrayList<Edge>();
        newNeighbors = new ArrayList<Edge>();
        verticesInVR = new ArrayList<Vertex>();
    }
    
    public String toString() { return name; }
    public int compareTo(Vertex v) { return Double.compare(minDistance, v.minDistance); }
    
    public boolean connected(Vertex v) {
        for (Edge e : neighbors) {
            if (e.target.name == v.name) { return true; } }
        return false;
    }
    
    public boolean vConnected(Vertex v) {
        for (Vertex u : verticesInVR) {
            for (Vertex w : v.verticesInVR) {
                if (u.connected(w)) { return true; } } }
        return false;
    }
        
    public void setNeighborWeight(Vertex target, double weight) {
        for (Edge e : newNeighbors) {
            if (e.target.name == target.name) { e.weight = weight; }
        }
    }
    
    public boolean hasNeighbor(Edge e) {
        for (Edge n : newNeighbors) {
            if (e.equals(n)) { return true; }
        }
        return false;
    }
}

class Edge
{
    public final Vertex source;
    public final Vertex target;
    public double weight;
    
    public Edge(Vertex argSource, Vertex argTarget, double argWeight) {
        source = argSource;
        target = argTarget;
        weight = argWeight;
    }
    
    public boolean equals(Edge e) {
        if (source == e.source &&
            target == e.target) {
            return true;
        } else {
            return false;
        }
    }
    
    public String toString() {
        return source.toString() + "-[" + String.format("%.3f", weight) + "]->" + target.toString();
    }
}

class Bloc
{
    public ArrayList<Vertex> vertices = new ArrayList<Vertex>();
    public Point loc;
    public Vertex surv;
    
    public Bloc(int argX, int argY) {
        loc = new Point(argX, argY);
    }
    
    public void add(Vertex v) {
        vertices.add(v);
    }
        
    public String toString() {
        return loc.x + ", " + loc.y;
    }
}

class Dijkstra
{
    public static double minDist(Vertex[] graph, Vertex source, Vertex target)
    {
        for (Vertex v : graph) { v.minDistance = Double.POSITIVE_INFINITY; }
        source.minDistance = 0.0;
        PriorityQueue<Vertex> queue = new PriorityQueue<Vertex>();
      	queue.add(source);
        double minDist = Double.POSITIVE_INFINITY;

        while (!queue.isEmpty()) {
            Vertex u = queue.poll();
            for (Edge e : u.neighbors)
            {
                Vertex v = e.target;
                if (Arrays.asList(graph).contains(v)) {
                    double weight = e.weight;
                    double distanceThroughU = u.minDistance + weight;
                    if (distanceThroughU < v.minDistance) {
                        queue.remove(v);
                        v.minDistance = distanceThroughU;
                        if (v.name == target.name) { minDist = v.minDistance; }
                        v.prev = u;
                        queue.add(v);
                    }
                }
            }
        }
        return minDist;
    }

    public static ArrayList<Vertex> getShortestPathTo(Vertex target)
    {
        ArrayList<Vertex> path = new ArrayList<Vertex>();
        for (Vertex v = target; v != null; v = v.prev)
            path.add(v);
        Collections.reverse(path);
        return path;
    }
}
        