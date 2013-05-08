import java.util.PriorityQueue;
import java.util.ArrayList;
import java.util.Collection;
import java.util.Collections;
import java.util.Hashtable;
import java.awt.Point;
import java.util.LinkedList;
import java.util.Arrays;

class HTAP
{
    public static Hashtable<Point,Bloc> blocs = new Hashtable<Point,Bloc>();

    public static void bloc(Vertex[] vertices, int precision)
    //Assigns each node a bloc and inserts the node into that bloc
    {
        System.out.println("\n" + "Assigning blocs...");
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
            System.out.println("node " + v + " assigned to bloc " + currentBloc);
        }
    }
    
    public static Vertex[] decimate(Hashtable<Point,Bloc> blocs)
    //Decimates a bloc by selecting a sole survivor node
    {
        Collection<Bloc> blocsAsCollection = blocs.values();
        Vertex[] survivors = new Vertex[blocsAsCollection.size()];
        int index = 0;
        System.out.println("\n" + "Decimating...");
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
            System.out.println("node " + b.surv + " survived bloc " + b);
        }
        return survivors;
    }
    
    public static void voronoi(Vertex[] survivors)
    //Determines which Voronoi region each node belongs to
    //given a set of points which are at the center of the regions
    {
        System.out.println("\n" + "Determining Voronoi Regions...");
        LinkedList<Vertex> queue = new LinkedList<Vertex>();
        for (Vertex v : survivors) {
            queue.add(v);
            v.verticesInVR = new ArrayList<Vertex>();
        }
        while(!queue.isEmpty()) {
            Vertex v = queue.poll();
            LinkedList<Vertex> children = new LinkedList<Vertex>();
            for (Edge e : v.neighbors) {
                Vertex c = e.target;
                if (c.vRegion==null) { children.add(c); }
            }
            while(!children.isEmpty()) {
                Vertex child = children.poll();
                v.verticesInVR.add(child);
                child.vRegion = v;
                System.out.println("node " + child + " assigned to voronoiregion " + v);
                queue.add(child);
            }
        }
    }
    
    public static void link(Vertex[] survivors)
    //Places edges between survivor nodes whose voronoi regions are connected
    {
        System.out.println("\n" + "Placing edges between survivor nodes...");
        int index = 0;
        while (index < survivors.length) {
            Vertex s = survivors[index];
            s.oldNeighbors = s.neighbors;
            s.neighbors = new ArrayList<Edge>();
            for (Vertex v : s.verticesInVR) {
                for (Edge e : v.neighbors) {
                    Vertex u = e.target;
                    Vertex t = u.vRegion;
                    if (s.loc != t.loc) {
                        Edge newE = new Edge(s,t,0);
                        boolean notContained = true;
                        for (Edge n : s.neighbors) {
                            if (n.equals(newE)) { notContained = false; }
                        }
                        if (notContained) {
                            s.neighbors.add(newE);
                            System.out.println("Added an edge between " + s + " and " + t);
                        }
                    }
                }
            }
            index++;
        }
    }
    
    public static void setWeights(Vertex[] survivors)
    //Sets the weights of the newly placed edges between survivor nodes
    {
        System.out.println("\n" + "Setting edge weights...");
        for (int i = 0; i < survivors.length ; i++) {
            for (int j = i+1; j < survivors.length ; j++) {
                Vertex u = survivors[i];
                Vertex v = survivors[j];
                if (u.connected(v)) {
                    ArrayList<Vertex> graph = new ArrayList<Vertex>();
                    ArrayList<Edge> temp1 = u.neighbors;
                    ArrayList<Edge> temp2 = v.neighbors;
                    u.neighbors = u.oldNeighbors;
                    v.neighbors = v.oldNeighbors;
                    for (Vertex child : u.verticesInVR) { graph.add(child); }
                    for (Vertex child : v.verticesInVR) { graph.add(child); }
                    graph.add(u);
                    graph.add(v);
                    Vertex[] graphAsArray = new Vertex[graph.size()];
                    graphAsArray = graph.toArray(graphAsArray);
                    double distUToV = Dijkstra.minDist(graphAsArray, u, v);
                    u.neighbors = temp1;
                    v.neighbors = temp2;
                    u.setNeighborWeight(v, distUToV);
                    v.setNeighborWeight(u, distUToV);
                    System.out.println("Set weight between " + u + " and " + v +
                        " to " + distUToV);
                }
            }
        }       
    }
    
    public static void main(String[] args)
    {
        Vertex a = new Vertex("Aurora", 2, 3);
        Vertex d = new Vertex("Denver", 2, 2);
        Vertex g = new Vertex("Golden", 2, 1);
        Vertex b = new Vertex("Boulder", 1, 1);
        Vertex f = new Vertex("Fort Collins", 0, 2);
        Vertex s = new Vertex("Colorado Springs", 3, 1);

        a.neighbors.add(new Edge(a, d, 30));
        a.neighbors.add(new Edge(a, g, 45));
        
        d.neighbors.add(new Edge(d, a, 30));
        d.neighbors.add(new Edge(d, g, 20));
        d.neighbors.add(new Edge(d, b, 40));
        d.neighbors.add(new Edge(d, f, 60));
        d.neighbors.add(new Edge(d, s, 60));
        
        g.neighbors.add(new Edge(g, d, 20));
        g.neighbors.add(new Edge(g, a, 45));
        g.neighbors.add(new Edge(g, b, 30));
        
        b.neighbors.add(new Edge(b, g, 30));
        b.neighbors.add(new Edge(b, d, 40));
        
        f.neighbors.add(new Edge(f, d, 60));
        
        s.neighbors.add(new Edge(s, d, 60));
        
        Vertex[] vertices = { a, d, g, b, f, s };
        bloc(vertices,2);
        Vertex[] survivors = decimate(blocs);
        voronoi(survivors);
        link(survivors);
        setWeights(survivors);
    }
}

class Vertex implements Comparable<Vertex>
{
    public final String name;
    public ArrayList<Edge> neighbors;
    public ArrayList<Edge> oldNeighbors;
    public double minDistance = Double.POSITIVE_INFINITY;
    public Vertex prev;
    public final Point loc;
    public Point bloc;
    public Vertex vRegion;
    public ArrayList<Vertex> verticesInVR;
    
    public Vertex(String argName, int argX, int argY) {
        name = argName;
        loc = new Point(argX, argY);
        neighbors = new ArrayList<Edge>();
    }
    
    public String toString() {
        return name;
    }
    public int compareTo(Vertex v) {
        return Double.compare(minDistance, v.minDistance);
    }
    
    public boolean connected(Vertex v) {
        for (Edge e : neighbors) {
            if (e.target.loc == v.loc) { return true; }
        }
        for (Edge e : v.neighbors) {
            if (e.target.loc == loc) { return true; }
        }
        return false;
    }
    
    public void setNeighborWeight(Vertex target, double weight) {
        for (Edge e : neighbors) {
            if (e.target.loc == target.loc) { e.weight = weight; }
        }
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
                        if (v.loc == target.loc) { minDist = v.minDistance; }
                        v.prev = u;
                        queue.add(v);
                    }
                }
            }
        }
        return minDist;
    }
}
        