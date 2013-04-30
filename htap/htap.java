import java.util.PriorityQueue;
import java.util.ArrayList;
import java.util.Collection;
import java.util.Collections;
import java.util.Hashtable;
import java.awt.Point;

class HTAP
{
    public static Hashtable<Point,Bloc> blocs = new Hashtable<Point,Bloc>();

    public static void bloc(Vertex[] vertices, int precision)
    //Assigns each node a bloc and inserts the node into that bloc
    {
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
    
    public static void decimate(Bloc bloc)
    //Decimates a bloc by selecting a sole survivor node
    {
        double minRes = Double.POSITIVE_INFINITY;
        for (Vertex v : bloc.vertices) {
            double res = 0;
            for (Edge e : v.neighbors) {
                res += 1/e.weight;
            }
            res = 1/res;
            if (res < minRes) {
                minRes = res;
                bloc.surv = v;
            }
        }
        System.out.println("node " + bloc.surv + " survived bloc " + bloc);
    }
    
    public static void main(String[] args)
    {
        Vertex a = new Vertex("Aurora", 2, 3);
        Vertex d = new Vertex("Denver", 2, 2);
        Vertex g = new Vertex("Golden", 2, 1);
        Vertex b = new Vertex("Boulder", 1, 1);
        Vertex f = new Vertex("Fort Collins", 0, 2);
        Vertex s = new Vertex("Colorado Springs", 3, 1);

        a.neighbors = new Edge[]{   new Edge(a, d, 30),
                                    new Edge(a, g, 45) };
        d.neighbors = new Edge[]{   new Edge(d, a, 30),
                                    new Edge(d, g, 20),
                                    new Edge(d, b, 40),
                                    new Edge(d, f, 60),
                                    new Edge(d, s, 60) };
        g.neighbors = new Edge[]{   new Edge(g, d, 20),
                                    new Edge(g, a, 45),
                                    new Edge(g, b, 30) };
        b.neighbors = new Edge[]{   new Edge(b, g, 30),
                                    new Edge(b, d, 40) };
        f.neighbors = new Edge[]{   new Edge(f, d, 60) };
        s.neighbors = new Edge[]{   new Edge(s, d, 60) };
        
        Vertex[] test = { a, d, g, b, f, s };
        bloc(test,2);
        Collection<Bloc> blocsAsList = blocs.values();
        for (Bloc bloc : blocsAsList) {
            decimate(bloc);
        }
    }
}

class Vertex //implements Comparable<Vertex>
{
    public final String name;
    public Edge[] neighbors;
    /* For Dijkstra only
    public double minDistance = Double.POSITIVE_INFINITY;
    public Vertex prev; */
    public final Point loc;
    public Point bloc;
    
    public Vertex(String argName, int argX, int argY) {
        name = argName;
        loc = new Point(argX, argY);
    }
    
    public String toString() {
        return name;
    }
    
    /* Dijkstra
    public int compareTo(Vertex v) {
        return Double.compare(minDistance, v.minDistance);
    }*/
}

class Edge
{
    public final Vertex source;
    public final Vertex target;
    public final double weight;
    
    public Edge(Vertex argSource, Vertex argTarget, double argWeight) {
        source = argSource;
        target = argTarget;
        weight = argWeight;
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
        