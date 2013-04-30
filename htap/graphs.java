import java.util.PriorityQueue;
import java.util.ArrayList;
import java.util.Collections;

public class Vertex implements Comparable<Vertex>
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

public class Edge
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