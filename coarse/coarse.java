import java.util.PriorityQueue;
import java.util.ArrayList;
import java.util.Collection;
import java.util.Collections;
import java.awt.Point;
import java.util.Random;
import java.io.PrintWriter;
import java.io.FileNotFoundException;
import java.io.*;


class Coarse {
    public static double[][] squares;
    public static Vertex[][] graph;
    public static Vertex[][] coarseGraph;
    public static int blockWidth;
    public static int blockHeight;
    public static int[] nodesPopped = new int[4]; // index 0 = dijkstra, 1 = euclid, 2 = corners
    public static long[] times = new long[3];
    public static double[][] coarseScores;
    public static PriorityQueue<Vertex> savedFringe = new PriorityQueue<Vertex>();

    public static void main(String[] args) {
        if (args.length < 4) {
            System.out.print("Usage: java Coarse <map size> <number of searches> ");
            System.out.println("<block width> <block height> [map file] [start x] [start y]");
            System.out.println("[end x] [end y]");
            return;
        }
        int size = Integer.parseInt(args[0]);
        int graphSize = size+1;
        int connectivity = 8;
        int iterations = Integer.parseInt(args[1]);
        blockWidth = Integer.parseInt(args[2]);
        blockHeight = Integer.parseInt(args[3]);
        squares = makeSquares(size);
        if (args.length >= 5) {
            try {
                FileReader input = new FileReader(args[4]);
                BufferedReader bufRead = new BufferedReader(input);
                String row = null;
                try {
                    for (int r = 0; r < size; r++) {
                        row = bufRead.readLine();
                        String[] weights = row.split(" ");
                        if (weights.length != size) {
                            System.out.println("user input size does not match actual size of graph");
                            return;
                        }
                        int c = 0;
                        for (String s : weights) {
                            double weight = Double.parseDouble(s);
                            squares[r][c] = weight;
                            c++;
                        }
                    }
                } catch(IOException ioe) {
                    System.out.println(ioe);
                }
            } catch(FileNotFoundException fnfe) {
                System.out.println(fnfe);
            }
        }
        graph = squaresToVertices(squares, size);
        coarseGraph = Corners.makeCoarseLayer(graph, squares, blockWidth, blockHeight, size);
        coarseScores = new double[graphSize][graphSize];
        
        Random rng = new Random();
        Vertex[] starts = new Vertex[iterations];
        Vertex[] goals = new Vertex[iterations];
        if (args.length == 9) {
            if (iterations != 1) {
                System.out.println("number of searches must = 1 if start and goal specified");
                return;
            }
            int startX = Integer.parseInt(args[5]);
            int startY = Integer.parseInt(args[6]);
            int endX = Integer.parseInt(args[7]);
            int endY = Integer.parseInt(args[8]);
            starts[0] = graph[startX][startY];
            goals[0] = graph[endX][endY];
        } else {
            for (int i = 0; i < iterations; i++) {
                starts[i] = graph[rng.nextInt(graphSize-1)][rng.nextInt(graphSize-1)];
                goals[i] = graph[rng.nextInt(graphSize-1)][rng.nextInt(graphSize-1)];
            }
        }
        
        int count = iterations;
        for (int i = 0; i < iterations; i++) {
            for (Vertex[] row : graph) {
                for (Vertex v : row) {
                    v.hScore = 0.0;
                    v.gScore = Double.POSITIVE_INFINITY;
                    v.fScore = Double.POSITIVE_INFINITY;
                    v.prev = null;
                }
            }
            Vertex start = starts[i];
            Vertex goal = goals[i];
            Corners.addEdges(   coarseGraph, coarseGraph[goal.loc.x][goal.loc.y],
                                blockWidth, blockHeight, size);
            coarseScores = new double[graphSize][graphSize];
            dijkstra(coarseGraph[goal.loc.x][goal.loc.y]);
            
            long startTimer = System.nanoTime();
            aStar(start, goal, 0);
            double dijkstraCost = goal.gScore;
            ArrayList<Vertex> dijkstraPath = getPath(goal);
            long endTimer = System.nanoTime();
            times[0] += (endTimer-startTimer)/1000;
            
            for (Vertex[] row : graph) {
                for (Vertex v : row) {
                    v.hScore = 0.0;
                    v.gScore = Double.POSITIVE_INFINITY;
                    v.fScore = Double.POSITIVE_INFINITY;
                    v.prev = null;
                }
            }
            startTimer = System.nanoTime();
            aStar(start, goal, 1);
            double euclidCost = goal.gScore;
            ArrayList<Vertex> euclidPath = getPath(goal);
            endTimer = System.nanoTime();
            times[1] += (endTimer-startTimer)/1000;
            
            for (Vertex[] row : graph) {
                for (Vertex v : row) {
                    v.hScore = 0.0;
                    v.gScore = Double.POSITIVE_INFINITY;
                    v.fScore = Double.POSITIVE_INFINITY;
                    v.prev = null;
                }
            }
            startTimer = System.nanoTime();
            aStar(start, goal, 2);
            double cornerCost = goal.gScore;
            ArrayList<Vertex> cornerPath = getPath(goal);
            endTimer = System.nanoTime();
            times[2] += (endTimer-startTimer)/1000;
            
            if(Math.abs(cornerCost - dijkstraCost) > 0.000001 || iterations == 1) {
                if (Math.abs(cornerCost - dijkstraCost) > 0.000001) {
                    count = count-1;
                }
                /*for (Vertex v : coarseGraph) {
                    System.out.print(v.getLoc()+" ");
                    for (Edge e : v.neighbors) {
                        System.out.print(e+" ");
                    }
                    System.out.println(" ");
                    System.out.println(" ");
                }*/
                try {
                    PrintWriter mapWriter = new PrintWriter("map.txt");
                    mapWriter.println(printGraph(squares));
                    mapWriter.close();
                    
                    PrintWriter writer = new PrintWriter("error.txt");
                    writer.println(printGraph(squares));
                    writer.println(printGraph(Corners.newSquares));
                    for (int r = 0; r < graphSize; r++) {
                        for (int c = 0; c < graphSize; c++) {
                            writer.print(coarseScores[r][c]+" ");
                        }
                        writer.println("");
                    }
                    
                    writer.println(printGraph(graph, cornerPath));
                    for (Vertex v : cornerPath) {
                        writer.println(v.getLoc()+": (gScore "+v.gScore+") (hScore "+v.hScore+")");
                    }
                    writer.println("Cost (Corners)  : " + cornerCost);
                    writer.println(printGraph(graph, euclidPath));
                    for (Vertex v : euclidPath) {
                        writer.println(v.getLoc()+" ");
                    }
                    writer.println("Cost (Euclidian): " + euclidCost);
                    writer.println(printGraph(graph, dijkstraPath));
                    for (Vertex v : dijkstraPath) {
                        writer.println(v.getLoc()+" ");
                    }
                    writer.println("Cost (Dijkstra) : " + dijkstraCost+"\n");
                    writer.close();
                } catch(FileNotFoundException err) {
                    System.out.println(err.getMessage());
                }
            }
            Corners.removeEdges(coarseGraph);
        }
        System.out.println("Performance: " + count + "/" + iterations);
        System.out.println("Nodes Popped\n    Dijkstra: "+nodesPopped[0]);
        System.out.println("    Euclid: "+nodesPopped[1]+"\n    Corners: "+nodesPopped[2]);
        System.out.println("    Coarse: "+nodesPopped[3]);
        System.out.println("\nTimes\n    Dijkstra: "+times[0]+"\n    Euclid: "+times[1]);
        System.out.println("    Corners: "+times[2]);
    }

    public static double heuristic(Vertex start, Vertex goal, int type) {
        if (type == 0) {
            return 0;
        }
        if (type == 1) {
            double xDiff = Math.abs(start.loc.x - goal.loc.x);
            double yDiff = Math.abs(start.loc.y - goal.loc.y);
            double lesser = Math.min(xDiff, yDiff);
            double greater = Math.max(xDiff, yDiff);
            //return lesser*Math.sqrt(2.0) + (greater-lesser);
            //return Math.sqrt((xDiff*xDiff)+(yDiff*yDiff));
            return greater;
        }
        if (type == 2) {
            int size = squareRoot(graph.length);
            /**if (coarseScores[start.loc.x][start.loc.y] == 0) {
                Vertex startInCoarse = coarseGraph[start.loc.x*size+start.loc.y];
                Vertex goalInCoarse = coarseGraph[goal.loc.x*size+goal.loc.y];
                dijkstra(goalInCoarse);
            }
            return goalInCoarse.gScore;*/
            return coarseScores[start.loc.x][start.loc.y];
        }
        return 0;
    }

    public static void aStar(Vertex start, Vertex goal, int type) {
        for (Vertex[] row : coarseGraph) { // initialization of all node attributes
            for (Vertex v : row) {
                v.gScore = Double.POSITIVE_INFINITY;
                v.fScore = Double.POSITIVE_INFINITY;
                v.hScore = 0.0;
                v.prev = null;
            }
        }
        PriorityQueue<Vertex> fringe = new PriorityQueue<Vertex>();
        
        start.gScore = 0.0;
        start.hScore = heuristic(start, goal, type);
        start.fScore = start.gScore + start.hScore;
        Vertex v = start;
        
        while (v.fScore < goal.gScore) { // stopping condition for fringe popping
            nodesPopped[type] += 1;
            for (Edge e : v.neighbors) {
                Vertex neighbor = e.target;
                double tempGScore = v.gScore + e.weight;
                if (tempGScore > neighbor.gScore) { // ignores update stage if cost is greater than current cost
                    continue;
                }
                if (tempGScore <= neighbor.gScore) { // otherwise updates its neighbors and adds to fringe
                    neighbor.prev = v;
                    neighbor.gScore = tempGScore;
                    neighbor.hScore = heuristic(neighbor, goal, type);
                    neighbor.fScore = neighbor.gScore + neighbor.hScore;
                    fringe.add(neighbor);
                }
            }
            v = fringe.poll();
        }
    }
    
    public static void dijkstra(Vertex source) {
        for (Vertex[] row : coarseGraph) { // initialization of all node attributes
            for (Vertex v : row) {
                v.gScore = Double.POSITIVE_INFINITY;
                v.fScore = Double.POSITIVE_INFINITY;
                v.hScore = 0.0;
                v.prev = null;
            }
        }
            
        source.gScore = 0.0;
        PriorityQueue<Vertex> queue = new PriorityQueue<Vertex>();
      	queue.add(source);

        while (!queue.isEmpty()) {
            nodesPopped[3] += 1;
            Vertex u = queue.poll();
            for (Edge e : u.neighbors) {
                Vertex v = e.target;
                double tempGScore = u.gScore + e.weight;
                if (tempGScore < v.gScore) {
                    queue.remove(v);
                    v.gScore = tempGScore ;
                    coarseScores[v.loc.x][v.loc.y] = v.gScore;
                    v.prev = u;
                    queue.add(v);
                }
            }
        }
    }
    
    //track nodes popped from fringe, pushed to fringe
    //record performance time
    //add debug capability (save error maps/rerun code on error maps)
    
    public static ArrayList<Vertex> getPath(Vertex goal) {
        ArrayList<Vertex> path = new ArrayList<Vertex>();
        Vertex trace = goal;
        while (trace != null) {
            path.add(trace);
            trace = trace.prev;
        }
        return path;
    }
    
    public static String printGraph(Vertex[][] graph, ArrayList<Vertex> path) {
        int size = graph[0].length;
        //System.out.println("\n Displaying path...");
        String output = "";
        for (int r = 0; r < size; r++) {
            for (int c = 0; c < size; c++) {
                Vertex v = graph[r][c];
                if (path.contains(v)) {
                    output += v.inPath();
                } else {
                    output += v;
                }
            }
            output += "\n";
        }
        return output;
    }
    
    public static String printGraph(Vertex[][] graph) {
        int size = graph[0].length;
        //System.out.println("\n Displaying graph...");
        String output = "";
        for (int r = 0; r < size; r++) {
            for (int c = 0; c < size; c++) {
                Vertex v = graph[r][c];
                output += v;
            }
            output += "\n";
        }
        return output;
    }
    
    public static String printGraph(double[][] graph) {
        int size = graph[0].length;
        //System.out.println("\n Displaying graph...");
        String output = "";
        for (int r = 0; r < size; r++) {
            for (int c = 0; c < size; c++) {
                double weight = graph[r][c];
                output += weight+" ";
            }
            output += "\n";
        }
        return output;
    }

    public static int squareRoot(int number) {
        return ((Double)Math.sqrt(number)).intValue();
    }
    
    public static double[][] makeSquares(int size) {
        double[][] squares = new double[size][size];
        Random rng = new Random();
        for (int row = 0; row < size; row++) {
            for (int col = 0; col < size; col++) {
                squares[row][col] = (double)(1 + rng.nextInt(3));
            }
        }
        return squares;
    }
    
    public static Vertex[][] squaresToVertices(double[][] squares, int originalSize) {
        int size = originalSize + 1;
        Vertex[][] graph = new Vertex[size][size];
        for (int row = 0; row < size; row++) {
            for (int col = 0; col < size; col++) {
                Vertex v = new Vertex(row, col, 0.0);
                graph[row][col] = v;
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
                Vertex v = graph[row][col];
                for (int i = 0; i < 4; i++) {
                    Point offset = cardinalOffsets[i];
                    int newRow = row + offset.x;
                    int newCol = col + offset.y;
                    if (newRow >= 0 && newRow < size && newCol >= 0 && newCol < size) {
                        Vertex u = graph[newRow][newCol];
                        
                        double square1 = Double.POSITIVE_INFINITY;
                        double square2 = Double.POSITIVE_INFINITY;
                        Point offset1 = cardinalSquares[2*i];
                        int squareRow1 = row + offset1.x;
                        int squareCol1 = col + offset1.y;
                        if (squareRow1 >= 0 && squareRow1 < originalSize &&
                            squareCol1 >= 0 && squareCol1 < originalSize) {
                            square1 = squares[squareRow1][squareCol1];
                        }
                        Point offset2 = cardinalSquares[(2*i)+1];
                        int squareRow2 = row + offset2.x;
                        int squareCol2 = col + offset2.y;
                        if (squareRow2 >= 0 && squareRow2 < originalSize &&
                            squareCol2 >= 0 && squareCol2 < originalSize) {
                            square2 = squares[squareRow2][squareCol2];
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
                        Vertex u = graph[newRow][newCol];
                        Point squareOffset = diagonalSquares[i];
                        int squareRow = row + squareOffset.x;
                        int squareCol = col + squareOffset.y;
                        double square = squares[squareRow][squareCol];
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
    public double hScore = 0.0;
    public Vertex prev = null;
    public boolean marked = false;
    
    public Vertex(int argX, int argY, double argWeight) {
        loc = new Point(argX, argY);
        weight = argWeight;
    }
    
    public Vertex(Vertex original) {
        loc = new Point(original.loc.x, original.loc.y);
        weight = original.weight;
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
        if (Double.compare(fScore, v.fScore) == 0) {
            return Double.compare(gScore, v.gScore);
        } else {
            return Double.compare(fScore, v.fScore);
        }
    }
}

class Edge {
    Vertex source;
    Vertex target;
    final double weight;
    public boolean marked = false;
    
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
    public static double[][] weightGraph;
    public static double[][] newSquares;
    
    public static Vertex[][] makeCoarseLayer(Vertex[][] argGraph, double[][] argSquares,
                                            int blockW, int blockH, int argSize) {
        int squaresSize = argSize;
        int graphSize = argSize + 1;
        double[][] squares = argSquares;
        Vertex[][] graph = argGraph;
    
        //Assign blocks and find which square has the minimum weight in each block
        weightGraph = new double[squaresSize/blockH][squaresSize/blockW];
        for (int bR = 0; bR < squaresSize/blockH; bR++) {
            for (int bC = 0; bC < squaresSize/blockW; bC++) {
                double minWeight = Double.POSITIVE_INFINITY;
                for (int r = bR*blockH; r < (bR+1)*blockH; r++) {
                    for (int c = bC*blockW; c < (bC+1)*blockW; c++) {
                        minWeight = Math.min(minWeight, squares[r][c]);
                    }
                }
                weightGraph[bR][bC] = minWeight;
            }
        }
        
        //Make a coarse copy of the original graph
        newSquares = new double[squaresSize][squaresSize];
        for (int r = 0; r < squaresSize; r++) {
            for(int c = 0; c < squaresSize; c++) {
                double weight = weightGraph[r/blockH][c/blockW];
                newSquares[r][c] = weight;
            }
        }
        Vertex[][] newGraph = new Vertex[graphSize][graphSize];
        for (int r = 0; r < graphSize; r++) {
            for (int c = 0; c < graphSize; c++) {
                Vertex v = graph[r][c];
                newGraph[r][c] = new Vertex(v);
            }
        }
        
        //Form edges within blocks - C to B, C to C, and C to I
        for (int bR = 0; bR < squaresSize/blockH; bR++) {
            for (int bC = 0; bC < squaresSize/blockW; bC++) {
                ArrayList<Vertex> corner = new ArrayList<Vertex>();
                ArrayList<Vertex> inside = new ArrayList<Vertex>();
                ArrayList<Vertex> border = new ArrayList<Vertex>();
                for (int r = bR*blockH; r <= (bR+1)*blockH; r++) {
                    for (int c = bC*blockW; c <= (bC+1)*blockW; c++) {
                        Vertex v = newGraph[r][c];
                        if (r%blockH == 0 && c%blockW == 0) {
                            corner.add(v);
                        } else if (r%blockH != 0 && c%blockW != 0) {
                            inside.add(v);
                        } else {
                            border.add(v);
                        }
                    }
                }
                double blocWeight = weightGraph[bR][bC];
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
                            double weight = diag(xDiff,yDiff)*blocWeight*delta;
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
                        double weight = diag(xDiff,yDiff)*blocWeight*delta;
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
    public static double delta = 1.0;
    
    public static void addEdges(Vertex[][] graph, Vertex source,
                                int blockW, int blockH, int size) {
        int sourceR = source.loc.x;
        int sourceC = source.loc.y;
        for (int blockR = 0; blockR < size/blockH; blockR++) {
            for (int blockC = 0; blockC < size/blockW; blockC++) {
                if (blockR*blockH <= sourceR && sourceR <= (blockR+1)*blockH &&
                    blockC*blockW <= sourceC && sourceC <= (blockC+1)*blockW) {
                    explodeBlock(graph, source, blockR, blockC, blockW, blockH, size);
                }
            }
        }
    }
    
    public static void explodeBlock(Vertex[][] graph, Vertex source, int blockR, int blockC,
                                    int blockW, int blockH, int size) {
        source.marked = true;
        for (int r = blockR*blockH; r <= (blockR+1)*blockH; r++) {
            for (int c = blockC*blockW; c <= (blockC+1)*blockW; c++) {
                Vertex v = graph[r][c];
                if (v.loc == source.loc) {
                    continue;
                }
                int rowDiff = Math.abs(v.loc.x-source.loc.x);
                int colDiff = Math.abs(v.loc.y-source.loc.y);
                double weight = weightGraph[blockR][blockC];
                if (rowDiff == 0) {
                    weight = colDiff*weight;
                } else if (colDiff == 0) {
                    weight = rowDiff*weight;
                } else {
                    weight = diag(rowDiff,colDiff)*weight*delta;
                }
                Edge e = new Edge(v, source, weight);
                Edge f = new Edge(source, v, weight);
                e.marked = true;
                f.marked = true;
                v.neighbors.add(e);
                source.neighbors.add(f);
                v.marked = true;
            }
        }
    }
    
    public static void removeEdges(Vertex[][] graph) {
        for (Vertex[] row : graph) {
            for (Vertex v : row) {
                if (v.marked) {
                    ArrayList<Edge> newNeighbors = new ArrayList<Edge>();
                    for (Edge e : v.neighbors) {
                        if (e.marked) {
                            continue;
                        } else {
                            newNeighbors.add(e);
                        }
                    }
                    v.neighbors = newNeighbors;
                    v.marked = false;
                }
            }
        }
    }
    
    public static double diag(double leg1, double leg2) {
        double squareSum = leg1*leg1+leg2*leg2;
        //return Math.sqrt(squareSum);
        //return Math.max(leg1,leg2);
        return Math.min(leg1,leg2);
    }
}
