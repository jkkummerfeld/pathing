import java.util.PriorityQueue;
import java.util.ArrayList;
import java.util.Collections;

class Vertex implements Comparable<Vertex>
{
    public final String name;
    public Edge[] neighbors;
    public double minDistance = Double.POSITIVE_INFINITY;
    public Vertex prev;
    
    public Vertex(String argName) {
        name = argName;
    }
    
    public String toString() {
        return name;
    }
    
    public int compareTo(Vertex v) {
        return Double.compare(minDistance, v.minDistance);
    }
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

class Dijkstra
{
    public static void getPathsFrom(Vertex source)
    {
        source.minDistance = 0.0;
        PriorityQueue<Vertex> queue = new PriorityQueue<Vertex>();
      	queue.add(source);

	while (!queue.isEmpty()) {
	    Vertex u = queue.poll();
            for (Edge e : u.neighbors)
            {
                Vertex v = e.target;
                double weight = e.weight;
                double distanceThroughU = u.minDistance + weight;
                if (distanceThroughU < v.minDistance) {
                    queue.remove(v);
                    v.minDistance = distanceThroughU ;
                    v.prev = u;
                    queue.add(v);
                }
            }
        }
    }

    public static ArrayList<Vertex> getShortestPathTo(Vertex target)
    {
        ArrayList<Vertex> path = new ArrayList<Vertex>();
        for (Vertex v = target; v != null; v = v.prev)
            path.add(v);
        Collections.reverse(path);
        return path;
    }

    public static void main(String[] args)
    {
        Vertex a = new Vertex("Aurora");
        Vertex d = new Vertex("Denver");
        Vertex g = new Vertex("Golden");
        Vertex b = new Vertex("Boulder");
        Vertex f = new Vertex("Fort Collins");
        Vertex s = new Vertex("Colorado Springs");

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
	Vertex[] vertices = { a, d, g, b, f, s };
        Vertex start = b;
        getPathsFrom(start);
        System.out.println("Starting from " + start);
        for (Vertex v : vertices)
        {
            if (v != start) {
                System.out.println("Distance to " + v + ": " + v.minDistance);
                ArrayList<Vertex> path = getShortestPathTo(v);
                System.out.println("Path: " + path);
            }
        }
    }
}