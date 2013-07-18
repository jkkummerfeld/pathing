import java.util.PriorityQueue;
import java.util.ArrayList;
import java.util.Collection;
import java.util.Collections;
import java.awt.Point;
import java.util.Random;

class Coarse {
    public static double[] squares;
    public static Vertex[] graph;
    public static Vertex[] draftCoarseGraph;
    public static Vertex[] coarseGraph;
    public static double[] gScores;
    public static int blockWidth = 5;
    public static int blockHeight = 4;

    public static void main(String[] args) {
        if (args.length != 4) {
            System.out.print("Usage: java Coarse <map size> <number of searches> ");
            System.out.println("<block width> <block height>");
            return;
        }
        int size = Integer.parseInt(args[0]);
        int graphSize = size+1;
        int connectivity = 8;
        int iterations = Integer.parseInt(args[1]);
        blockWidth = Integer.parseInt(args[2]);
        blockHeight = Integer.parseInt(args[3]);
        gScores = new double[graphSize*graphSize];
        squares = makeSquares(size);
        graph = squaresToVertices(squares, size);
        draftCoarseGraph = Corners.makeCoarseLayer(graph, squares, blockWidth, blockHeight, size);
        coarseGraph = new Vertex[graphSize*graphSize];
        
        Random rng = new Random();
        Vertex[] starts = new Vertex[iterations];
        Vertex[] goals = new Vertex[iterations];
        for (int i = 0; i < iterations; i++) {
            starts[i] = graph[rng.nextInt(graphSize*graphSize/2)];
            goals[i] = graph[graphSize*graphSize-1-rng.nextInt(graphSize*graphSize/2)];
        }
        
        int count = iterations;
        for (int i = 0; i < iterations; i++) {
            for (Vertex v : graph) {
                v.gScore = Double.POSITIVE_INFINITY;
                v.fScore = Double.POSITIVE_INFINITY;
                v.prev = null;
            }
            Vertex start = starts[i];
            Vertex goal = goals[i];
            //System.out.println("start: " + start.getLoc());
            //System.out.println("goal:  " + goal.getLoc());
            /*if (goal.loc.x%blockHeight == 0 && goal.loc.y%blockWidth == 0) {
                System.out.println("goal is a CORNER vertex");
            } else if (goal.loc.x%blockHeight != 0 && goal.loc.y%blockWidth != 0) {
                System.out.println("goal is a INSIDE vertex");
            } else {
                System.out.println("goal is a BORDER vertex");
            }*/
            for (int j = 0; i < graphSize*graphSize; i++) {
                coarseGraph[i] = draftCoarseGraph[i];
            }
            Corners.addEdges(   coarseGraph, coarseGraph[goal.loc.x*graphSize+goal.loc.y],
                                blockWidth, blockHeight, size);
            
            /*Vertex goalInCoarse = coarseGraph[goal.loc.x*graphSize+goal.loc.y];
            aStar(goalInCoarse);
            for (int row = 0; row < graphSize; row++) {
                for (int col = 0; col < graphSize; col++) {
                    int index = row*graphSize+col;
                    gScores[index] = coarseGraph[index].gScore;
                }
            }*/
            
            aStar(start, goal, 2);
            double dijkstraCost = goal.gScore;
            ArrayList<Vertex> dijkstraPath = getPath(goal);
            
            for (Vertex v : graph) {
                v.gScore = Double.POSITIVE_INFINITY;
                v.fScore = Double.POSITIVE_INFINITY;
                v.prev = null;
            }
            aStar(start, goal, 0);
            double euclidCost = goal.gScore;
            ArrayList<Vertex> euclidPath = getPath(goal);
            
            for (Vertex v : graph) {
                v.gScore = Double.POSITIVE_INFINITY;
                v.fScore = Double.POSITIVE_INFINITY;
                v.prev = null;
            }
            aStar(start, goal, 1);
            double cornerCost = goal.gScore;
            ArrayList<Vertex> cornerPath = getPath(goal);
            
            if(Math.abs(cornerCost - dijkstraCost) > 0.000001) {
                if (size < 21) {
                    count = count-1;
                    for (Vertex v : coarseGraph) {
                        System.out.print(v.getLoc()+" ");
                        for (Edge e : v.neighbors) {
                            System.out.print(e+" ");
                        }
                        System.out.println(" ");
                        System.out.println(" ");
                    }
                    printGraph(squares);
                    printGraph(Corners.newSquares);
                    
                    printGraph(graph, cornerPath);
                    for (Vertex v : cornerPath) {
                        System.out.println(v.getLoc()+": (gScore "+v.gScore+") (hScore "+v.hScore+")");
                    }
                    System.out.println("Cost (Corners)  : " + cornerCost);
                    printGraph(graph, euclidPath);
                    for (Vertex v : euclidPath) {
                        System.out.print(v.getLoc()+" ");
                    }
                    System.out.println("Cost (Euclidian): " + euclidCost);
                    printGraph(graph, dijkstraPath);
                    for (Vertex v : dijkstraPath) {
                        System.out.print(v.getLoc()+" ");
                    }
                    System.out.println("Cost (Dijkstra) : " + dijkstraCost+"\n");
                } else {
                    count = count-1;
                }
            }
        }
        System.out.println("Performance: " + count + "/" + iterations);
    }

    public static double heuristic(Vertex start, Vertex goal) {
        double xDiff = Math.abs(start.loc.x - goal.loc.x);
        double yDiff = Math.abs(start.loc.y - goal.loc.y);
        double lesser = Math.min(xDiff, yDiff);
        double greater = Math.max(xDiff, yDiff);
        return lesser*Math.sqrt(2.0) + (greater-lesser);
        //return Math.sqrt((xDiff*xDiff)+(yDiff*yDiff));
    }

    public static double coarseHeuristic(Vertex start, Vertex goal) {
        int size = squareRoot(graph.length);
        Vertex startInCoarse = coarseGraph[start.loc.x*size+start.loc.y];
        Vertex goalInCoarse = coarseGraph[goal.loc.x*size+goal.loc.y];
        aStar(startInCoarse, goalInCoarse, 0);
        return goalInCoarse.gScore;
        /*int index = start.loc.x*size+start.loc.y;
        return gScores[index];*/
    }

    public static void aStar(Vertex start, Vertex goal, int h) {
        for (Vertex v : coarseGraph) {
            v.gScore = Double.POSITIVE_INFINITY;
            v.fScore = Double.POSITIVE_INFINITY;
            v.hScore = Double.POSITIVE_INFINITY;
            v.prev = null;
        }
        ArrayList<Vertex> closedSet = new ArrayList<Vertex>();
        PriorityQueue<Vertex> openSet = new PriorityQueue<Vertex>();
        
        start.gScore = 0.0;
        start.hScore = 0.0;
        if (h == 0) {
            start.hScore = heuristic(start,goal);
        } else if (h == 1) {
            start.hScore = coarseHeuristic(start,goal);
        } else {
            start.hScore = 0.0;
        }
        start.fScore = start.gScore + start.hScore;
        openSet.add(start);
        
        while (openSet.size() > 0) {
            Vertex v = openSet.poll();
            if (v.loc == goal.loc) {
                return;
            }
            
            closedSet.add(v);
            for (Edge e : v.neighbors) {
                Vertex neighbor = e.target;
                double tempGScore = v.gScore + e.weight;
                if (closedSet.contains(neighbor) && tempGScore >= neighbor.gScore) {
                    continue;
                }
                if (!openSet.contains(neighbor) || tempGScore < neighbor.gScore) {
                    neighbor.prev = v;
                    neighbor.gScore = tempGScore;
                    neighbor.hScore = 0.0;
                    if (h == 0) {
                        neighbor.hScore = heuristic(neighbor,goal);
                    } else if (h == 1) {
                        neighbor.hScore = coarseHeuristic(neighbor,goal);
                    } else {
                        neighbor.hScore = 0.0;
                    }
                    neighbor.fScore = neighbor.gScore + neighbor.hScore;
                    if (!openSet.contains(neighbor)) {
                        openSet.add(neighbor);
                    }
                }
            }
        }
    }
    
    public static ArrayList<Vertex> getPath(Vertex goal) {
        ArrayList<Vertex> path = new ArrayList<Vertex>();
        Vertex trace = goal;
        while (trace != null) {
            path.add(trace);
            trace = trace.prev;
        }
        return path;
    }
    
    public static void printGraph(Vertex[] graph, ArrayList<Vertex> path) {
        int size = squareRoot(graph.length);
        System.out.println("\n Displaying path...");
        for (int r = 0; r < size; r++) {
            for (int c = 0; c < size; c++) {
                Vertex v = graph[r*size+c];
                if (path.contains(v)) {
                    System.out.print(v.inPath());
                } else {
                    System.out.print(v);
                }
            }
            System.out.print("\n");
        }
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
    
    public static void printGraph(double[] graph) {
        int size = squareRoot(graph.length);
        System.out.println("\n Displaying graph...");
        for (int r = 0; r < size; r++) {
            for (int c = 0; c < size; c++) {
                double weight = graph[r*size+c];
                System.out.print(weight+" ");
            }
            System.out.print("\n");
        }
    }

    public static int squareRoot(int number) {
        return ((Double)Math.sqrt(number)).intValue();
    }
    
    public static double[] makeSquares(int size) {
        double[] squares = new double[size*size];
        Random rng = new Random();
        for (int row = 0; row < size; row++) {
            for (int col = 0; col < size; col++) {
                int index = row*size+col;
                squares[index] = (double)(1 + rng.nextInt(3));
            }
        }
        return squares;
    }
    
    public static Vertex[] squaresToVertices(double[] squares, int originalSize) {
        int size = originalSize + 1;
        Vertex[] graph = new Vertex[size*size];
        for (int row = 0; row < size; row++) {
            for (int col = 0; col < size; col++) {
                int index = row*size+col;
                Vertex v = new Vertex(row, col, 0.0);
                graph[index] = v;
            }
        }
    
        Point[] cardinalOffsets = { new Point(-1,0), new Point(1,0),
                                    new Point(0,-1), new Point(0, 1) };
        Point[] cardinalSquares = { new Point(-1,-1), new Point(-1,0),
                                    new Point(0,-1), new Point(0,0),
                                    new Point(-1,-1), new Point(0,-1),
                                    new Point(-1,0), new Point(0,0) };
        Point[] diagonalOffsets = { new Point(-1,-1), new Point(-1,1),
                                    new Point(1,-1), new Point(1,1) };
        Point[] diagonalSquares = { new Point(-1,-1), new Point(-1,0),
                                    new Point(0,-1), new Point(0,0) };  
        
        for (int row = 0; row <size; row++) {
            for (int col = 0; col < size; col++) {
                int index = row*size+col;
                Vertex v = graph[index];
                for (int i = 0; i < 4; i++) {
                    Point offset = cardinalOffsets[i];
                    int newRow = row + offset.x;
                    int newCol = col + offset.y;
                    if (newRow >= 0 && newRow < size && newCol >= 0 && newCol < size) {
                        int newIndex = newRow*size+newCol;
                        Vertex u = graph[newIndex];
                        
                        double square1 = Double.POSITIVE_INFINITY;
                        double square2 = Double.POSITIVE_INFINITY;
                        Point offset1 = cardinalSquares[2*i];
                        int squareRow1 = row + offset1.x;
                        int squareCol1 = col + offset1.y;
                        if (squareRow1 >= 0 && squareRow1 < originalSize &&
                            squareCol1 >= 0 && squareCol1 < originalSize) {
                            square1 = squares[squareRow1*originalSize+squareCol1];
                        }
                        Point offset2 = cardinalSquares[(2*i)+1];
                        int squareRow2 = row + offset2.x;
                        int squareCol2 = col + offset2.y;
                        if (squareRow2 >= 0 && squareRow2 < originalSize &&
                            squareCol2 >= 0 && squareCol2 < originalSize) {
                            square2 = squares[squareRow2*originalSize+squareCol2];
                        }
                        Edge e = new Edge(v, u, Math.min(square1, square2));
                        v.neighbors.add(e);
                    }
                }
                for (int i = 0; i < 4; i++) {
                    Point offset = diagonalOffsets[i];
                    int newRow = row + offset.x;
                    int newCol = col + offset.y;
                    if (newRow >= 0 && newRow < size && newCol >= 0 && newCol < size) {
                        int newIndex = newRow*size+newCol;
                        Vertex u = graph[newIndex];
                        Point squareOffset = diagonalSquares[i];
                        int squareRow = row + squareOffset.x;
                        int squareCol = col + squareOffset.y;
                        double square = squares[squareRow*originalSize+squareCol];
                        Edge e = new Edge(v, u, square*Math.sqrt(2.0));
                        v.neighbors.add(e);
                    }
                }
            }
        }
        
        return graph;
    }
}    
    
class Vertex implements Comparable<Vertex> {
    public Point loc;
    public double weight;
    public ArrayList<Edge> neighbors = new ArrayList<Edge>();
    public double fScore = Double.POSITIVE_INFINITY;
    public double gScore = Double.POSITIVE_INFINITY;
    public double hScore = Double.POSITIVE_INFINITY;
    public Vertex prev = null;
    
    public Vertex(int argX, int argY, double argWeight) {
        loc = new Point(argX, argY);
        weight = argWeight;
    }
    
    public Vertex(Vertex original) {
        loc = new Point(original.loc.x, original.loc.y);
        weight = original.weight;
    }
    
    public double getCost(Vertex neighbor) {
        //return (weight+neighbor.weight)/2;
        return Math.max(weight, neighbor.weight);
    }
    
    public String getLoc() {
        return "("+loc.x+","+loc.y+")";
    }
    
    public String toString() {
        if (loc.x%Coarse.blockHeight == 0 && loc.y%Coarse.blockWidth == 0) { return "x"; }
        else if (loc.x%Coarse.blockHeight != 0 && loc.y%Coarse.blockWidth != 0) { return " "; }
        else if (loc.x%Coarse.blockHeight == 0) { return "-"; }
        else { return "|"; }
    }
    
    public String inPath() {
        if (weight <= 1) { return "."; }
        else if (weight > 1 && weight <= 2) { return "o"; }
        else { return "O"; }
    }
    
    public int compareTo(Vertex v) {
        return Double.compare(fScore, v.fScore);
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
    
    public String toString() {
        return "("+source.getLoc()+" to "+target.getLoc()+": " +weight+")";
    }
}

class Corners {
    public static double[] weightGraph;
    public static double[] newSquares;
    
    public static Vertex[] makeCoarseLayer(Vertex[] argGraph, double[] argSquares,
                                            int blockW, int blockH, int argSize) {
        int squaresSize = argSize;
        int graphSize = argSize + 1;
        double[] squares = argSquares;
        Vertex[] graph = argGraph;
    
        //Assign blocks and find which sqaure has the minimum weight in each block
        weightGraph = new double[(squaresSize)/blockW*(squaresSize)/blockH];
        for (int bR = 0; bR < (squaresSize)/blockH; bR++) {
            for (int bC = 0; bC < (squaresSize)/blockW; bC++) {
                int bIndex = bR*(squaresSize)/blockW+bC;
                double minWeight = Double.POSITIVE_INFINITY;
                for (int r = bR*blockH; r < (bR+1)*blockH; r++) {
                    for (int c = bC*blockW; c < (bC+1)*blockW; c++) {
                        int index = r*(squaresSize)+c;
                        minWeight = Math.min(minWeight, squares[index]);
                    }
                }
                weightGraph[bIndex] = minWeight;
            }
        }
        
        //Make a coarse copy of the original graph
        newSquares = new double[squaresSize*squaresSize];
        for (int r = 0; r < squaresSize; r++) {
            for(int c = 0; c < squaresSize; c++) {
                int index = r*squaresSize+c;
                int blockIndex = (r/blockH)*(squaresSize/blockW)+(c/blockW);
                double weight = weightGraph[blockIndex];
                newSquares[index] = weight;
            }
        }
        Vertex[] newGraph = new Vertex[graphSize*graphSize];
        for (int r = 0; r < graphSize; r++) {
            for (int c = 0; c < graphSize; c++) {
                int index = r*graphSize+c;
                Vertex v = graph[index];
                newGraph[index] = new Vertex(v);
            }
        }
        
        //Form edges within blocks - C to B, C to C, and C to I
        for (int bR = 0; bR < (squaresSize)/blockH; bR++) {
            for (int bC = 0; bC < (squaresSize)/blockW; bC++) {
                int bIndex = bR*(squaresSize)/blockW+bC;
                ArrayList<Vertex> corner = new ArrayList<Vertex>();
                ArrayList<Vertex> inside = new ArrayList<Vertex>();
                ArrayList<Vertex> border = new ArrayList<Vertex>();
                for (int r = bR*blockH; r <= (bR+1)*blockH; r++) {
                    for (int c = bC*blockW; c <= (bC+1)*blockW; c++) {
                        Vertex v = newGraph[r*graphSize+c];
                        if (r%blockH == 0 && c%blockW == 0) {
                            corner.add(v);
                        } else if (r%blockH != 0 && c%blockW != 0) {
                            inside.add(v);
                        } else {
                            border.add(v);
                        }
                    }
                }
                double blocWeight = weightGraph[bIndex];
                //C to B
                for (Vertex c : corner) {
                    for (Vertex b : border) {
                        if (c.loc.x==b.loc.x) {
                            double weight = Math.abs(c.loc.y-b.loc.y)*blocWeight;
                            Edge cToB = new Edge(c, b, weight);
                            Edge bToC = new Edge(b, c, weight);
                            c.neighbors.add(cToB);
                            b.neighbors.add(bToC);
                        } else if (c.loc.y==b.loc.y) {
                            double weight = Math.abs(c.loc.x-b.loc.x)*blocWeight;
                            Edge cToB = new Edge(c, b, weight);
                            Edge bToC = new Edge(b, c, weight);
                            c.neighbors.add(cToB);
                            b.neighbors.add(bToC);
                        }
                    }
                }
                //C to C
                for (Vertex c1 : corner) {
                    for (Vertex c2 : corner) {
                        if (c1.loc == c2.loc) { continue; }
                        int xDiff = Math.abs(c1.loc.x-c2.loc.x);
                        int yDiff = Math.abs(c1.loc.y-c2.loc.y);
                        if (c1.loc.x==c2.loc.x) {
                            double weight = yDiff*blocWeight;
                            Edge e = new Edge(c1, c2, weight);
                            c1.neighbors.add(e);
                        } else if (c1.loc.y==c2.loc.y) {
                            double weight = xDiff*blocWeight;
                            Edge e = new Edge(c1, c2, weight);
                            c1.neighbors.add(e);
                        } else {
                            double weight = Math.max(xDiff,yDiff)*blocWeight;
                            //double weight = Math.sqrt(xDiff*xDiff + yDiff*yDiff)*blocWeight;
                            Edge e = new Edge(c1, c2, weight);
                            c1.neighbors.add(e);
                        }
                    }
                }
                //C to I
                for (Vertex c : corner) {
                    for (Vertex i : inside) {
                        int xDiff = Math.abs(c.loc.x-i.loc.x);
                        int yDiff = Math.abs(c.loc.y-i.loc.y);
                        double weight = Math.max(xDiff,yDiff)*blocWeight;
                        //double weight = Math.sqrt(xDiff*xDiff + yDiff*yDiff)*blocWeight;
                        Edge cToI = new Edge(c, i, weight);
                        Edge iToC = new Edge(i, c, weight);
                        c.neighbors.add(cToI);
                        i.neighbors.add(iToC);
                    }
                }
            }
        }
        
        return newGraph;
    }
    
    public static void addEdges(Vertex[] graph, Vertex source,
                                int blockW, int blockH, int size) {
        int sourceR = source.loc.x;
        int sourceC = source.loc.y;
        for (int blockR = 0; blockR < size/blockH; blockR++) {
            for (int blockC = 0; blockC < size/blockW; blockC++) {
                if (blockR*blockH <= sourceR && sourceR <= (blockR+1)*blockH &&
                    blockC*blockW <= sourceC && sourceC <= (blockC+1)*blockW) {
                    //System.out.println("exploding block (" + blockR + "," + blockC +")");
                    explodeBlock(graph, source, blockR, blockC, blockW, blockH, size);
                }
            }
        }
    }
    
    public static void explodeBlock(Vertex[] graph, Vertex source, int blockR, int blockC,
                                    int blockW, int blockH, int size) {
        for (int r = blockR*blockH; r <= (blockR+1)*blockH; r++) {
            for (int c = blockC*blockW; c <= (blockC+1)*blockW; c++) {
                Vertex v = graph[r*(size+1)+c];
                if (v.loc == source.loc) {
                    continue;
                }
                int rowDiff = Math.abs(v.loc.x-source.loc.x);
                int colDiff = Math.abs(v.loc.y-source.loc.y);
                double weight = weightGraph[blockR*(size/blockW)+blockC];
                if (rowDiff == 0) {
                    weight = colDiff*weight;
                } else if (colDiff == 0) {
                    weight = rowDiff*weight;
                } else {
                    weight = Math.max(rowDiff,colDiff)*weight*;
                    //weight = Math.sqrt(rowDiff*rowDiff + colDiff*colDiff)*weight;
                }
                Edge e = new Edge(v, source, weight);
                v.neighbors.add(e);
            }
        }
    }
                
}

