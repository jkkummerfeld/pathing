import java.util.*;
import java.io.*;
import java.awt.Point;

class Coarse {
    public static int counter1 = 0;
    public static int counter2 = 0;
    public static double[][] squares;
    public static Vertex[][] graph;
    public static Vertex[][] coarseGraph;
    public static int mapHeight;
    public static int mapWidth;
    public static int graphWidth;
    public static int graphHeight;
    public static int blockWidth;
    public static int blockHeight;
    public static int[] nodesPopped = new int[4]; // index 0 = dijkstra, 1 = euclid, 2 = corners
    public static long[] times = new long[3];
    public static double[][] coarseScores;
    public static PriorityQueue<Vertex> savedFringe = null;
    public static Vertex globalStart;
    public static Vertex globalGoal;

    public static void main(String[] args) {
        if (args.length < 5) {
            System.out.print("Usage: java Coarse <map height> <map width> <number of searches> ");
            System.out.println("<block height> <block width> [map file] [start x] [start y]");
            System.out.println("[end x] [end y]");
            return;
        }
        mapHeight = Integer.parseInt(args[0]);
        mapWidth = Integer.parseInt(args[1]);
        graphWidth = mapWidth+1;
        graphHeight = mapHeight+1;
        int connectivity = 8;
        int iterations = Integer.parseInt(args[2]);
        blockHeight = Integer.parseInt(args[3]);
        blockWidth = Integer.parseInt(args[4]);
        squares = makeSquares(mapHeight, mapWidth);
        if (args.length >= 6) {
            try {
                FileReader input = new FileReader(args[5]);
                BufferedReader bufRead = new BufferedReader(input);
                String row = null;
                try {
                    for (int r = 0; r < mapHeight; r++) {
                        row = bufRead.readLine();
                        if (row.startsWith("Map size:")) {
                            String[] line1 = row.split(" ");
                            System.out.println(line1);
                        } else {
                            // parse regular map
                            String[] weights = row.split(" ");
                            if (weights.length != mapWidth) {
                                System.out.println("user input size does not match actual size of graph");
                                return;
                            }
                        11!    int c = 0;
                            for (String s : weights) {
                                double weight = Double.parseDouble(s);
                                squares[r][c] = weight;
                                c++;
                            }
                        }
                    }
                } catch(IOException ioe) {
                    System.out.println(ioe);
                }
            } catch(FileNotFoundException fnfe) {
                System.out.println(fnfe);
            }
        }
        graph = squaresToVertices(squares, mapHeight, mapWidth);
        long startCoarseTimer = System.nanoTime();
        coarseGraph = Corners.makeCoarseLayer(graph, squares, blockWidth, blockHeight, mapHeight, mapWidth);
        long endCoarseTimer = System.nanoTime();
        
        Random rng = new Random();
        Vertex[] starts = new Vertex[iterations];
        Vertex[] goals = new Vertex[iterations];
        if (args.length == 10) {
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
                starts[i] = graph[rng.nextInt(graphHeight-1)][rng.nextInt(graphWidth-1)];
                goals[i] = graph[rng.nextInt(graphHeight-1)][rng.nextInt(graphWidth-1)];
            }
        }
        
        long timeAddingEdges = 0;
        long timeReinitialize = 0;
        
        int count = iterations;
        for (int i = 0; i < iterations; i++) {
            savedFringe = null;
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
            globalStart = start;
            globalGoal = goal;
            long startAddingEdges = System.nanoTime();
            if (goal.loc.x%blockHeight!=0 || goal.loc.y%blockWidth!=0) {
                Corners.addEdges(   coarseGraph, coarseGraph[goal.loc.x][goal.loc.y],
                                    blockHeight, blockWidth, mapHeight, mapWidth);
            }
            long endAddingEdges = System.nanoTime();
            timeAddingEdges += (endAddingEdges - startAddingEdges)/1000;
            coarseScores = new double[graphHeight][graphWidth];
            for (double[] row : coarseScores) {
                for (int index = 0; index < row.length; index++) {
                    row[index] = -1.0;
                }
            }
            //coarseScores = new double[graphSize][graphSize];
            //dijkstra(coarseGraph[goal.loc.x][goal.loc.y]);
            
            long startTimer = System.nanoTime();
            aStar(start, goal, 0);
            double dijkstraCost = goal.gScore;
            ArrayList<Vertex> dijkstraPath = getPath(goal);
            long endTimer = System.nanoTime();
            times[0] += (endTimer-startTimer)/1000;
            
            long startReinitialize = System.nanoTime();
            for (Vertex[] row : graph) {
                for (Vertex v : row) {
                    v.hScore = 0.0;
                    v.gScore = Double.POSITIVE_INFINITY;
                    v.fScore = Double.POSITIVE_INFINITY;
                    v.prev = null;
                }
            }
            long endReinitialize = System.nanoTime();
            timeReinitialize += (endReinitialize-startReinitialize)/1000;
            
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
                    for (int r = 0; r < graphHeight; r++) {
                        for (int c = 0; c < graphWidth; c++) {
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
        System.out.println("Nodes Popped\n    Dijkst: "+nodesPopped[0]);
        System.out.println("    Euclid: "+nodesPopped[1]+"\n    Corner: "+nodesPopped[2]);
        System.out.println("    Coarse: "+nodesPopped[3]);
        System.out.println("\nTimes\n    Dijkst: "+times[0]+"\n    Euclid: "+times[1]);
        System.out.println("    Corner: "+times[2]);
        System.out.println("\nTime to generate coarse graph: "+(endCoarseTimer-startCoarseTimer)/1000);
        System.out.println("\nCalls to Coarse Search: "+counter1);
        System.out.println("In-table heuristic reads: "+counter2);
        System.out.println("# times coarse search starts over: "+countNewCoarseSearch);
        System.out.println("# times coarse search resumes: "+countResumeCoarseSearch+"\n");
        System.out.println("Time to add edges in goal block: "+timeAddingEdges);
        System.out.println("Time to reinitialize graphs: "+timeReinitialize);
        System.out.println("Time searching at coarse level: "+timeDijkstra);
    }
    static long timeDijkstra = 0;
    static int countNewCoarseSearch = 0;
    static int countResumeCoarseSearch = 0;

    public static double heuristic(Vertex start, Vertex goal, int type) {
        if (type == 0) { //Dijkst
            return 0;
        }
        if (type == 1) { //Euclid
            double xDiff = Math.abs(start.loc.x - goal.loc.x);
            double yDiff = Math.abs(start.loc.y - goal.loc.y);
            double lesser = Math.min(xDiff, yDiff);
            double greater = Math.max(xDiff, yDiff);
            return lesser*Math.sqrt(2.0) + (greater-lesser);
            //return Math.sqrt((xDiff*xDiff)+(yDiff*yDiff));
            //return greater;
        }
        if (type == 2) { //Corner
            int r = start.loc.x;
            int c = start.loc.y;
            if (coarseScores[start.loc.x][start.loc.y] == -1.0) {
                counter1++;
                Vertex goalInCoarse = coarseGraph[goal.loc.x][goal.loc.y];
                double min = Double.POSITIVE_INFINITY;
                if (r%blockHeight==0 && c%blockWidth==0) {
                    Vertex startInCoarse = coarseGraph[r][c];
                    dijkstra(goalInCoarse, startInCoarse);
                    return coarseScores[r][c];
                } else if (r%blockHeight!=0 && c%blockWidth!=0) {
                    if (coarseScores[r/blockHeight][c/blockWidth] == -1 ||
                        coarseScores[1+r/blockHeight][c/blockWidth] == -1 ||
                        coarseScores[r/blockHeight][1+c/blockWidth] == -1 ||
                        coarseScores[1+r/blockHeight][1+c/blockWidth] == -1) {
                        ArrayList<Vertex> stoppers = new ArrayList<Vertex>();
                        stoppers.add(coarseGraph[r/blockHeight][c/blockWidth]);
                        stoppers.add(coarseGraph[1+r/blockHeight][c/blockWidth]);
                        stoppers.add(coarseGraph[r/blockHeight][1+c/blockWidth]);
                        stoppers.add(coarseGraph[1+r/blockHeight][1+c/blockWidth]);
                        dijkstra(goalInCoarse, stoppers);
                    }
                    for (int row = r/blockHeight; row < r/blockHeight+1; row++) {
                        for (int col = c/blockWidth; col < c/blockWidth+1; col++) {
                            double dist = coarseScores[row][col];
                            double weight = Corners.weightGraph[r/blockHeight][c/blockWidth];
                            dist += Math.min(Math.abs(row-r), Math.abs(col-c))*weight;
                            min = Math.min(min, dist);
                        }
                    }
                    coarseScores[r][c] = min;
                    return min;
                } else if (r%blockHeight == 0) {
                    if (coarseScores[r/blockHeight][c/blockWidth] == -1 ||
                        coarseScores[r/blockHeight][1+c/blockWidth] == -1) {
                        ArrayList<Vertex> stoppers = new ArrayList<Vertex>();
                        stoppers.add(coarseGraph[r/blockHeight][c/blockWidth]);
                        stoppers.add(coarseGraph[r/blockHeight][1+c/blockWidth]);
                        dijkstra(goalInCoarse, stoppers);
                    }
                    for (int col = c/blockWidth; col < c/blockWidth+1; col++) {
                        int row = r/blockHeight;
                        double dist = coarseScores[row][col];
                        double weight1 = Double.POSITIVE_INFINITY;
                        double weight2 = Double.POSITIVE_INFINITY;
                        if (r/blockHeight < mapHeight/blockHeight) {
                            weight1 = Corners.weightGraph[r/blockHeight][c/blockWidth];
                        }
                        if (r/blockHeight-1 >= 0) {
                            weight2 = Corners.weightGraph[r/blockHeight-1][c/blockWidth];
                        }
                        double weight = Math.min(weight1,weight2);
                        dist += Math.abs(col-c)*weight;
                        min = Math.min(min, dist);
                    }
                    coarseScores[r][c] = min;
                    return min;
                } else {
                    if (coarseScores[r/blockHeight][c/blockWidth] == -1 ||
                        coarseScores[1+r/blockHeight][c/blockWidth] == -1) {
                        ArrayList<Vertex> stoppers = new ArrayList<Vertex>();
                        stoppers.add(coarseGraph[r/blockHeight][c/blockWidth]);
                        stoppers.add(coarseGraph[1+r/blockHeight][c/blockWidth]);
                        dijkstra(goalInCoarse, stoppers);
                    } 
                    for (int row = r/blockHeight; row < r/blockHeight+1; row++) {
                        int col = c/blockWidth;
                        double dist = coarseScores[row][col];
                        double weight1 = Double.POSITIVE_INFINITY;
                        double weight2 = Double.POSITIVE_INFINITY;
                        if (c/blockWidth < mapWidth/blockWidth) {
                            weight1 = Corners.weightGraph[r/blockHeight][c/blockWidth];
                        }
                        if (c/blockWidth-1 >= 0) {
                            weight2 = Corners.weightGraph[r/blockHeight][c/blockWidth-1];
                        }
                        double weight = Math.min(weight1,weight2);
                        dist += Math.abs(row-r)*weight;
                        min = Math.min(min, dist);
                    } 
                    coarseScores[r][c] = min;
                    return min;
                }
            } else {
                counter2++;
            }
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
        fringe.add(start);
        Vertex v = start;
        
        while (v.fScore < goal.gScore) { // stopping condition for fringe popping
            nodesPopped[type] += 1;
            v = fringe.poll();
            for (Edge e : v.neighbors) {
                Vertex neighbor = e.target;
                double tempGScore = v.gScore + e.weight;
                if (tempGScore >= neighbor.gScore) { // ignores update stage if cost is greater than current cost
                    continue;
                }
                if (tempGScore < neighbor.gScore) { // otherwise updates its neighbors and adds to fringe
                    neighbor.prev = v;
                    neighbor.gScore = tempGScore;
                    neighbor.hScore = heuristic(neighbor, goal, type);
                    neighbor.fScore = neighbor.gScore + neighbor.hScore;
                    fringe.add(neighbor);
                }
            }
        }
    }
    
    public static void dijkstra(Vertex source, Vertex stopper) {
        long startDijkstra = System.nanoTime();
        PriorityQueue<Vertex> queue = new PriorityQueue<Vertex>();
        Hashtable<Point, Vertex> closed = new Hashtable<Point, Vertex>();
        if (savedFringe == null) {
            countNewCoarseSearch++;
            for (Vertex[] row : coarseGraph) { // initialization of all node attributes
                for (Vertex v : row) {
                    v.gScore = Double.POSITIVE_INFINITY;
                    v.fScore = Double.POSITIVE_INFINITY;
                    v.hScore = 0.0;
                    v.prev = null;
                }
            }
            source.gScore = 0.0;
            source.hScore = heuristic(source, globalStart, 1);
            source.fScore = source.gScore + source.hScore;
            queue.add(source);
        } else {
            countResumeCoarseSearch++;
            queue = savedFringe;
        }
        boolean resume = true;

        while (!queue.isEmpty() && resume == true) {
            nodesPopped[3] += 1;
            Vertex current = queue.poll();
            //closed.put(current.loc, current);
            for (Edge e : current.neighbors) {
                Vertex neighbor = e.target;
                double tempGScore = current.gScore + e.weight;
                if (closed.get(neighbor.loc) != null) {
                    continue;
                }
                if (tempGScore < neighbor.gScore) {
                    queue.remove(neighbor);
                    neighbor.gScore = tempGScore ;
                    neighbor.hScore = heuristic(neighbor, globalStart, 1);
                    neighbor.fScore = neighbor.gScore + neighbor.hScore;
                    coarseScores[neighbor.loc.x][neighbor.loc.y] = neighbor.gScore;
                    neighbor.prev = current;
                    queue.add(neighbor);
                }
            }
            if (stopper.equals(current)) {
                resume = false;
                savedFringe = queue;
            }
        }
        long endDijkstra = System.nanoTime();
        timeDijkstra += (endDijkstra - startDijkstra)/1000;
    }
    
    public static void dijkstra(Vertex source, ArrayList<Vertex> stoppers) {
        long startDijkstra = System.nanoTime();
        PriorityQueue<Vertex> queue = new PriorityQueue<Vertex>();
        Hashtable<Point, Vertex> closed = new Hashtable<Point, Vertex>();
        if (savedFringe == null) {
            countNewCoarseSearch++;
            for (Vertex[] row : coarseGraph) { // initialization of all node attributes
                for (Vertex v : row) {
                    v.gScore = Double.POSITIVE_INFINITY;
                    v.fScore = Double.POSITIVE_INFINITY;
                    v.hScore = 0.0;
                    v.prev = null;
                }
            }
            source.gScore = 0.0;
            coarseScores[source.loc.x][source.loc.y] = 0.0;    
            source.hScore = heuristic(source, globalStart, 1);
            source.fScore = source.gScore + source.hScore;
            queue.add(source);
        } else {
            countResumeCoarseSearch++;
            queue = savedFringe;
        }
        boolean resume = true;

        while (!queue.isEmpty() && resume == true) {
            nodesPopped[3] += 1;
            Vertex current = queue.poll();
            closed.put(current.loc, current);
            for (Edge e : current.neighbors) {
                Vertex neighbor = e.target;
                double tempGScore = current.gScore + e.weight;
                if (closed.get(neighbor.loc) != null) {
                    continue;
                }
                if (tempGScore < neighbor.gScore) {
                    queue.remove(neighbor);
                    neighbor.gScore = tempGScore ;
                    neighbor.hScore = heuristic(neighbor, globalStart, 1);
                    neighbor.fScore = neighbor.gScore + neighbor.hScore;
                    coarseScores[neighbor.loc.x][neighbor.loc.y] = neighbor.gScore;
                    neighbor.prev = current;
                    queue.add(neighbor);
                }
            }
            if (stoppers.contains(current)) {
                stoppers.remove(current);
                if (stoppers.isEmpty()) {
                    resume = false;
                    savedFringe = queue;
                }
            }
        }
        long endDijkstra = System.nanoTime();
        timeDijkstra += (endDijkstra - startDijkstra)/1000;
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
        //System.out.println("\n Displaying path...");
        String output = "";
        for (Vertex[] row : graph) {
            for (Vertex v : row) {
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
        //System.out.println("\n Displaying graph...");
        String output = "";
        for (Vertex[] row : graph) {
            for (Vertex v : row) {
                output += v;
            }
            output += "\n";
        }
        return output;
    }
    
    public static String printGraph(double[][] graph) {
        //System.out.println("\n Displaying graph...");
        String output = "";
        for (double[] row : graph) {
            for (double weight : row) {
                output += weight+" ";
            }
            output += "\n";
        }
        return output;
    }

    public static int squareRoot(int number) {
        return ((Double)Math.sqrt(number)).intValue();
    }
    
    public static double[][] makeSquares(int height, int width) {
        double[][] squares = new double[height][width];
        Random rng = new Random();
        /*int rowIndex = 0;
        int colIndex = 0;
        int rowStop = 0;
        int colStop = 0;
        while (rowIndex < height && colIndex < width) {
            rowStop += rng.nextInt(height);
            colStop += rng.nextInt(width);
            double weight = (double)(1 + rng.nextInt(10));
            for (int r = rowIndex; r < rowStop && r < height; r++) {
                for (int c = colIndex; c < colStop && c < width; c++) {
                    squares[r][c] = weight;
                    colIndex++;
                }
                rowIndex++;
            }
        }*/
        
        double weight = (double)(1 + rng.nextInt(5));
        for (int row = 0; row < height; row++) {
            for (int col = 0; col < width; col++) {
                if (rng.nextBoolean()) {
                    weight +=1;
                    if (weight > 5) {
                        weight = 3;
                    }
                    squares[row][col] = weight;
                } else {
                    weight -=1;
                    if (weight < 1) {
                        weight = 2;
                    }
                    squares[row][col] = weight;
                }
            }
        }
        return squares;
    }
    
    public static Vertex[][] squaresToVertices(double[][] squares, int mapHeight, int mapWidth) {
        int graphHeight = mapHeight + 1;
        int graphWidth = mapWidth + 1;
        Vertex[][] graph = new Vertex[graphHeight][graphWidth];
        for (int row = 0; row < graphHeight; row++) {
            for (int col = 0; col < graphWidth; col++) {
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
        
        for (int row = 0; row < graphHeight; row++) {
            for (int col = 0; col < graphWidth; col++) {
                Vertex v = graph[row][col];
                for (int i = 0; i < 4; i++) {
                    Point offset = cardinalOffsets[i];
                    int newRow = row + offset.x;
                    int newCol = col + offset.y;
                    if (newRow >= 0 && newRow < graphHeight && newCol >= 0 && newCol < graphWidth) {
                        Vertex u = graph[newRow][newCol];
                        
                        double square1 = Double.POSITIVE_INFINITY;
                        double square2 = Double.POSITIVE_INFINITY;
                        Point offset1 = cardinalSquares[2*i];
                        int squareRow1 = row + offset1.x;
                        int squareCol1 = col + offset1.y;
                        if (squareRow1 >= 0 && squareRow1 < mapHeight &&
                            squareCol1 >= 0 && squareCol1 < mapWidth) {
                            square1 = squares[squareRow1][squareCol1];
                        }
                        Point offset2 = cardinalSquares[(2*i)+1];
                        int squareRow2 = row + offset2.x;
                        int squareCol2 = col + offset2.y;
                        if (squareRow2 >= 0 && squareRow2 < mapHeight &&
                            squareCol2 >= 0 && squareCol2 < mapWidth) {
                            try {
                                square2 = squares[squareRow2][squareCol2];
                            } catch (IndexOutOfBoundsException ioobe) {
                                System.out.println("tried to access "+squareRow2+","+squareCol2);
                                System.out.println("but map size is "+mapHeight+","+mapWidth);
                                System.exit(0);
                            }
                        }
                        Edge e = new Edge(v, u, Math.min(square1, square2));
                        v.neighbors.add(e);
                    }
                }
                for (int i = 0; i < 4; i++) {
                    Point offset = diagonalOffsets[i];
                    int newRow = row + offset.x;
                    int newCol = col + offset.y;
                    if (newRow >= 0 && newRow < graphHeight && newCol >= 0 && newCol < graphWidth) {
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
    
    public boolean equals(Vertex v) {
        return (loc.x == v.loc.x && loc.y == v.loc.y);
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
                                            int blockH, int blockW, int mapHeight, int mapWidth) {
        int graphHeight = mapHeight+1;
        int graphWidth = mapWidth+1;
        int newMapHeight = mapHeight/blockH;
        int newMapWidth = mapWidth/blockW;
        int newGraphHeight = newMapHeight+1;
        int newGraphWidth = newMapWidth+1;
        double[][] squares = argSquares;
        Vertex[][] graph = argGraph;
    
        //Assign blocks and find which square has the minimum weight in each block
        weightGraph = new double[newMapHeight][newMapWidth];
        newSquares = weightGraph;
        for (int bR = 0; bR < newMapHeight; bR++) {
            for (int bC = 0; bC < newMapWidth; bC++) {
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
        Vertex[][] newGraph = new Vertex[graphHeight][graphWidth];
        for (int r = 0; r < graphHeight; r++) {
            for (int c = 0; c < graphWidth; c++) {
                newGraph[r][c] = new Vertex(r, c, 0.0);
            }
        }
        
        //Form edges between vertices
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
        double weight;
        for (int row = 0; row < newGraphHeight; row++) {
            for (int col = 0; col < newGraphWidth; col++) {
                Vertex v = newGraph[row*blockH][col*blockW];
                for (int i = 0; i < 4; i++) {
                    Point offset = cardinalOffsets[i];
                    int newRow = row + offset.x;
                    int newCol = col + offset.y;
                    if (newRow >= 0 && newRow < newGraphHeight && newCol >= 0 && newCol < newGraphWidth) {
                        Vertex u = newGraph[newRow*blockH][newCol*blockW];
                        
                        double square1 = Double.POSITIVE_INFINITY;
                        double square2 = Double.POSITIVE_INFINITY;
                        Point offset1 = cardinalSquares[2*i];
                        int squareRow1 = row + offset1.x;
                        int squareCol1 = col + offset1.y;
                        if (squareRow1 >= 0 && squareRow1 < newMapHeight &&
                            squareCol1 >= 0 && squareCol1 < newMapWidth) {
                            square1 = newSquares[squareRow1][squareCol1];
                        }
                        Point offset2 = cardinalSquares[(2*i)+1];
                        int squareRow2 = row + offset2.x;
                        int squareCol2 = col + offset2.y;
                        if (squareRow2 >= 0 && squareRow2 < newMapHeight &&
                            squareCol2 >= 0 && squareCol2 < newMapWidth) {
                            try {
                                square2 = newSquares[squareRow2][squareCol2];
                            } catch (IndexOutOfBoundsException ioobe) {
                                System.out.println("tried to access "+squareRow2+","+squareCol2);
                                System.out.println("but map size is "+newMapHeight+","+newMapWidth);
                                System.exit(0);
                            }
                        }
                        if (i < 2) {
                            weight = blockH*Math.min(square1, square2);
                        } else {
                            weight = blockW*Math.min(square1, square2);
                        }
                        Edge e = new Edge(v, u, weight);
                        v.neighbors.add(e);
                    }
                }
                for (int i = 0; i < 4; i++) {
                    Point offset = diagonalOffsets[i];
                    int newRow = row + offset.x;
                    int newCol = col + offset.y;
                    if (newRow >= 0 && newRow < newGraphHeight && newCol >= 0 && newCol < newGraphWidth) {
                        Vertex u = newGraph[newRow*blockH][newCol*blockW];
                        Point squareOffset = diagonalSquares[i];
                        int squareRow = row + squareOffset.x;
                        int squareCol = col + squareOffset.y;
                        double square = newSquares[squareRow][squareCol];
                        Edge e = new Edge(v, u, square*Math.max(blockW,blockH));
                        v.neighbors.add(e);
                    }
                }
            }
        }
        
        return newGraph;
    }
    public static double delta = 1.0;
    
    public static void addEdges(Vertex[][] coarseGraph, Vertex source,
                                int blockH, int blockW, int mapHeight, int mapWidth) {
        int sourceR = source.loc.x;
        int sourceC = source.loc.y;
        for (int blockR = 0; blockR < mapHeight/blockH; blockR++) {
            for (int blockC = 0; blockC < mapWidth/blockW; blockC++) {
                if (blockR*blockH <= sourceR && sourceR <= (blockR+1)*blockH &&
                    blockC*blockW <= sourceC && sourceC <= (blockC+1)*blockW) {
                    explodeBlock(coarseGraph, source, blockR, blockC, blockH, blockW);
                }
            }
        }
    }
    
    public static void explodeBlock(Vertex[][] coarseGraph, Vertex source,
                                    int blockR, int blockC, int blockH, int blockW) {
        source.marked = true;
        for (int r = blockR*blockH; r <= (blockR+1)*blockH; r++) {
            for (int c = blockC*blockW; c <= (blockC+1)*blockW; c++) {
                Vertex v = coarseGraph[r][c];
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
                    weight = diag(rowDiff,colDiff)*weight;
                }
                Edge e1 = new Edge(v, source, weight);
                Edge e2 = new Edge(source, v, weight);
                e1.marked = true;
                e2.marked = true;
                v.neighbors.add(e1);
                source.neighbors.add(e2);
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
        //return Math.min(leg1,leg2);
        double lesser = Math.min(leg1, leg2);
        double greater = Math.max(leg1, leg2);
        return lesser*Math.sqrt(2.0) + (greater-lesser);
    }
}
