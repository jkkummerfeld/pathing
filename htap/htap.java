import java.util.PriorityQueue;
import java.util.ArrayList;
import java.util.Collection;
import java.util.Collections;
import java.awt.Point;
import java.util.LinkedList;
import java.util.Arrays;
import java.util.Random;
import java.util.Stack;

class HTAP
{
    public static Vertex[] original;
    static int layer = 0;

    public static int squareRoot(int number) {
        return ((Double)Math.sqrt(number)).intValue();
    }
    
    public static void printGraph(Vertex[] graph) {
        int size = squareRoot(graph.length);
        System.out.println("\n Displaying graph...");
        for (int r = 0; r < size; r++) {
            for (int c = 0; c < size; c++) {
                Vertex v = graph[r*size+c];
                System.out.print(v);
            }
            System.out.print("\n");
        }
    }
    
    public static void printGraph(Vertex[] graph, ArrayList<Vertex> path) {
        int size = squareRoot(graph.length);
        System.out.println("\n Displaying graph...");
        for (int r = 0; r < size; r++) {
            for (int c = 0; c < size; c++) {
                Vertex v = graph[r*size+c];
                if (path.contains(v)) {
                    System.out.print(v.inPath());
                } else {
                    System.out.print(" ");
                }
            }
            System.out.print("\n");
        }
    }
        
    public static void main(String[] args) {
        if (args.length != 3) {
            System.out.println("usage:");
            System.out.println("HTAP <size> <connectivity> <iterations>");
            System.out.println("<size> is the length of 1 side of the square graph");
            System.out.println("<connectivity> must be 4 or 8");
            System.out.println("<iterations> is the number of searches performed");
            return;
        }
        int size = Integer.parseInt(args[0]);
        int connectivity = Integer.parseInt(args[1]);
        int iterations = Integer.parseInt(args[2]);
        original = makeGraph(size, connectivity);
        Random rng = new Random();
        
        Vertex[] sources = new Vertex[iterations];
        Vertex[] targets = new Vertex[iterations];
        for (int i = 0; i < iterations; i++) {
            sources[i] = original[rng.nextInt(size)];
            targets[i] = original[size*size-rng.nextInt(size)-1];
        }
        
        //Dijkstra
        long timerDijkstra = 0;
        ArrayList<Vertex> path = new ArrayList<Vertex>();
        for (int i = 0; i < iterations; i++) {
            long startDijkstra = System.nanoTime();
            Vertex source = sources[i];
            Vertex target = targets[i];
            Dijkstra.getPathsFrom(original, source);
            path = Dijkstra.getShortestPathTo(target);
            long endDijkstra = System.nanoTime();
            timerDijkstra += endDijkstra-startDijkstra;
            //printGraph(original, path);
        }
        System.out.println("Dijkstra took: " + timerDijkstra/1000000);
        
        //HTAP
        long timerHTAP = 0;
        long startPyramid = System.nanoTime();
        Vertex[] dummy = original;
        ArrayList<Vertex[]> pyramid = new ArrayList<Vertex[]>();
        pyramid.add(original);
        for (int i = 0; i < 2; i++) {
            Vertex[] newLayer = makeNewLayer(dummy);
            pyramid.add(newLayer);
            dummy = newLayer;
        }
        long endPyramid = System.nanoTime();
        long timerPyramid = endPyramid-startPyramid;
        
        for (int i = 0; i < iterations; i++) {
            long startHTAP = System.nanoTime();
            Vertex source = sources[i];
            Vertex target = targets[i];
            Vertex[] corridor = pyramid.get(layer);
            while (layer > 0) {
                layer -= 1;
                Dijkstra.getPathsFrom(corridor, source.allReps.get(layer));
                path = Dijkstra.getShortestPathTo(target.allReps.get(layer));
                ArrayList<Vertex> subgraph = new ArrayList<Vertex>();
                for (Vertex v : path) {
                    for (Vertex c : v.voronoiRegion) {
                        subgraph.add(c);
                    }
                }
                corridor = new Vertex[subgraph.size()];
                corridor = subgraph.toArray(corridor);
            }
            Dijkstra.getPathsFrom(corridor, source);
            path = Dijkstra.getShortestPathTo(target);
            long endHTAP = System.nanoTime();
            timerHTAP += endHTAP-startHTAP;
            //printGraph(original, path);
        }
        System.out.println("HTAP took: " + timerHTAP/1000000);
        System.out.println("Pyramid construction took: " + timerPyramid/1000000);
    }
    
    public static Vertex[] makeNewLayer(Vertex[] graph) {
        int size = squareRoot(graph.length);
        int newSize = ((Double)Math.ceil(size/3.0)).intValue();
        
        //Decimate
        Vertex[] newGraph = new Vertex[newSize*newSize];
        Vertex[] survivors = new Vertex[newSize*newSize];
        for (int r = 0; r < size; r++) {
            for (int c = 0; c < size; c++) {
                int index = r*size+c;
                Vertex v = graph[index];
                int bR = r/3;
                int bC = c/3;
                int blocIndex = bR*newSize + bC;
                if (newGraph[blocIndex] == null) {
                    newGraph[blocIndex] = new Vertex(v);
                    survivors[blocIndex] = v;
                } else if (v.getResistance() < newGraph[blocIndex].getResistance()) {
                    newGraph[blocIndex] = new Vertex(v);
                    survivors[blocIndex] = v;
                }
            }
        }
        
        //Voronoi
        LinkedList<Vertex> queue = new LinkedList<Vertex>();
        for (int index = 0; index < survivors.length; index++) {
            survivors[index].immRep = newGraph[index];
            newGraph[index].voronoiRegion = new ArrayList<Vertex>();
            newGraph[index].voronoiRegion.add(survivors[index]);
            queue.add(survivors[index]);
        }        
        while(!queue.isEmpty()) {
            Vertex v = queue.poll();
            LinkedList<Vertex> children = new LinkedList<Vertex>();
            for (Edge e : v.neighbors) {
                if (e.target.immRep==null) {
                    children.add(e.target);
                }
            }
            while(!children.isEmpty()) {
                Vertex child = children.poll();
                child.immRep = v.immRep;
                v.immRep.voronoiRegion.add(child);
                queue.add(child);
            }
        }
        for (Vertex v : original) {
            if (v.allReps.isEmpty()) {
                v.allReps.add(v.immRep);
            } else {
                Vertex newRep = v.allReps.get(layer-1).immRep;
                v.allReps.add(newRep);
            }
        }
        layer++;
        
        //Link
        for (int index1 = 0; index1 < survivors.length; index1++) {
            for (int index2 = 0; index2 < survivors.length; index2++) {
                Vertex u = newGraph[index1];
                Vertex v = newGraph[index2];
                if (u.loc == v.loc) { continue; }
                ArrayList<Vertex> subGraph = new ArrayList<Vertex>();
                for (Vertex child : u.voronoiRegion) {
                    subGraph.add(child);
                }
                for (Vertex child : v.voronoiRegion) {
                    subGraph.add(child);
                }
                Vertex[] graphAsArray = new Vertex[subGraph.size()];
                graphAsArray = subGraph.toArray(graphAsArray);
                Dijkstra.getPathsFrom(graphAsArray, survivors[index1]);
                if (survivors[index2].minDistance != Double.POSITIVE_INFINITY) {
                    Edge e = new Edge(u, v, survivors[index2].minDistance);
                    u.neighbors.add(e);
                }
            }
        }
        
        return newGraph;
    }
    
    public static Vertex[] makeGraph(int size, int connectivity) {
        if (!(connectivity == 4 || connectivity == 8)) {
            System.out.println("connectivity must be 4 or 8");
            return null;
        } else {
            Vertex[] vertices = new Vertex[size*size];
            Random rng = new Random();
            for (int i = 0; i < size; i++) {
                for (int j = 0; j < size; j++) {
                    vertices[i*size+j] = new Vertex(i, j, rng.nextDouble());
                }
            }
            Point[] cardinalDirections = {  new Point(1,0), new Point(-1,0),
                                            new Point(0,1), new Point(0, -1),
                                            new Point(1,1), new Point(-1,1),
                                            new Point(1,-1), new Point(-1,-1) };
            for (int r = 0; r < size; r++) {
                for (int c = 0; c < size; c++) {
                    Vertex v = vertices[r*size + c];
                    for (int i = 0; i < connectivity; i++) {
                        Point offset = cardinalDirections[i];
                        int nR = r+offset.x; 
                        int cR = c+offset.y;
                        if (nR >= 0 && nR < size && cR >= 0 && cR < size) {
                            int nIndex = nR*size+cR;
                            Vertex u = vertices[nIndex];
                            Edge e = new Edge(v, u, v.getCost(u));
                            v.neighbors.add(e);
                        }
                    }
                }
            }
            return vertices;
        }
    }
}

class Vertex implements Comparable<Vertex> {
    public Point loc;
    public double weight;
    public ArrayList<Edge> neighbors = new ArrayList<Edge>();
    
    public Vertex immRep = null;
    public ArrayList<Vertex> allReps = new ArrayList<Vertex>();
    public ArrayList<Vertex> voronoiRegion;
    double resistance = 0.0;
    
    public Vertex(int argX, int argY, double argWeight) {
        loc = new Point(argX, argY);
        weight = argWeight;
    }
    
    public Vertex(Vertex original) {
        loc = new Point(original.loc.x, original.loc.y);
        weight = original.weight;
    }
    
    public double getResistance() {
        if (resistance == 0.0) {
            double denom = 0;
            for (Edge e : neighbors) {
                denom += 1.0/e.weight;
            }
            resistance = 1.0/denom;
        }
        return resistance;
    }
    
    public double getCost(Vertex neighbor) {
        return (weight+neighbor.weight)/2;
    }
    
    public String getLoc() {
        return "("+loc.x+","+loc.y+")";
    }
    
    public String toString() {
        if (weight <= 0.333) { return "-"; }
        else if (weight > 0.333 && weight <= 0.666) { return "="; }
        else { return "#"; }
    } 
    
    public String inPath() {
        if (weight <= 0.333) { return "."; }
        else if (weight > 0.333 && weight <= 0.666) { return "o"; }
        else { return "O"; }
    }
    
    public double minDistance = Double.POSITIVE_INFINITY;
    public Vertex prev;
    
    public int compareTo(Vertex v) {
        return Double.compare(minDistance, v.minDistance);
    }
}

class Edge {
    Vertex source;
    Vertex target;
    final double weight;
    
    public Edge(Vertex argSource, Vertex argTarget, double argWeight) {
        source = argSource;
        target = argTarget;
        weight = argWeight;
    }
}

class Dijkstra {    
    public static void getPathsFrom(Vertex[] graph, Vertex source) {
        for (Vertex v : graph) {
            v.minDistance = Double.POSITIVE_INFINITY;
            v.prev = null;
        }
        source.minDistance = 0.0;
        PriorityQueue<Vertex> queue = new PriorityQueue<Vertex>();
        queue.add(source);

        while (!queue.isEmpty()) {
            Vertex u = queue.poll();
            for (Edge e : u.neighbors) {
                Vertex v = e.target;
                if (!Arrays.asList(graph).contains(v)) { continue; }
                double distanceThroughU = u.minDistance + e.weight;
                if (distanceThroughU < v.minDistance) {
                    queue.remove(v);
                    v.minDistance = distanceThroughU;
                    v.prev = u;
                    queue.add(v);
                }
            }
        }
    }

    public static ArrayList<Vertex> getShortestPathTo(Vertex target)
    {
        ArrayList<Vertex> path = new ArrayList<Vertex>();
        Vertex trace = target;
        while (trace != null) {
            path.add(trace);
            trace = trace.prev;
        }
        //Collections.reverse(path);
        return path;
    }
}