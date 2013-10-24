import java.util.*;
import java.io.*;
import java.text.*;
import java.awt.Point;

class Search {
    public static int mapHeight;
    public static int mapWidth;
    public static int blockH;
    public static int blockW;
    public static int iterations;
    public static Map map;
    public static Vertex start;
    public static Vertex goal;
    public static Map coarseMap;
    public static Graph graph;
    public static Graph coarseGraph;
    public static boolean reopen;

    public static void main(String[] args) {
        
        if (args.length < 4) {
            System.out.println("\nUsage: <map height> <map width> <block height> <block width>");
            System.out.println("       [number of searches] [map file]");
            System.out.println("       [start x] [start y] [goal x] [goal y]");
            System.exit(0);
        }
        
        mapHeight = Integer.parseInt(args[0]);
        mapWidth = Integer.parseInt(args[1]);
        blockH = Integer.parseInt(args[2]);
        blockW = Integer.parseInt(args[3]);
        
        if (args.length >= 5) {
            iterations = Integer.parseInt(args[4]);
        } else {
            iterations = 1;
        }
        
        if (args.length >= 6) {
            map = new Map(mapHeight, mapWidth, args[5]);
        } else {
            map = new Map(mapHeight, mapWidth);
        }
        graph = new Graph(map);
        
        Vertex[] starts = new Vertex[iterations];
        Vertex[] goals = new Vertex[iterations];
        
        if (args.length >= 10 && iterations == 1) {
            starts[0] = graph.get(Integer.parseInt(args[6]), Integer.parseInt(args[7]));
            goals[0] = graph.get(Integer.parseInt(args[8]), Integer.parseInt(args[9]));
        } else if (map.startR != -1 && map.startC != -1
            && map.goalR != -1 && map.goalC != -1 && iterations == 1) {
            starts[0] = graph.get(map.startR, map.startC);
            goals[0] = graph.get(map.goalR, map.goalC);
        } else {
            Random rng = new Random();
            if (iterations == 1) {
                starts[0] = graph.get(0, 0);
                goals[0] = graph.get(mapHeight, mapWidth);
            }
            else {
                for (int i = 0; i < iterations; i++) {
                    starts[i] = graph.get(rng.nextInt(mapHeight), rng.nextInt(mapWidth));
                    goals[i] = graph.get(rng.nextInt(mapHeight), rng.nextInt(mapWidth));
                    while (starts[i] == goals[i]) {
                        starts[i] = graph.get(rng.nextInt(mapHeight), rng.nextInt(mapWidth));
                        goals[i] = graph.get(rng.nextInt(mapHeight), rng.nextInt(mapWidth));
                    }
                }
            }
        }
        
        // make coarse layer and add goal/start nodes to it
        coarseMap = map.makeCoarse(blockH, blockW);
        coarseGraph = new Graph(coarseMap, blockH, blockW);
        
        long dijkstraTimer = 0;
        long cornersTimer = 0;
        long euclidTimer = 0;
        long coarseTimer = 0;
        int dijkstraNodes = 0;
        int cornersNodes = 0;
        int euclidNodes = 0;
        int coarseNodes = 0;
        double error = 0.0;
        double totalError = 0.0;
        double nodeDiff = 0.0;
        double timeDiff = 0.0;
        
        for (int i = 0; i < iterations; i++) {
            start = starts[i];
            goal = goals[i];
            coarseGraph.addStart(start, blockH, blockW);
            coarseGraph.addGoal(goal, blockH, blockW);
            
            // test corners
            Heuristic c = new CornersHeuristic(graph, coarseGraph, coarseMap);
            aStar(start, goal, c, graph);
            double cornersScore = goal.gScore;
            ArrayList<Vertex> cornersPath = getPath(goal);
            cornersTimer += graph.timer;
            cornersNodes += c.getNodesPopped();
            coarseTimer += ((CornersHeuristic)c).timer;
            coarseNodes += ((CornersHeuristic)c).getCoarseNodesPopped();
            if (iterations == 1)
                System.out.println("finished running CORNERS");
                        
            graph.clear();
            coarseGraph.clear();
            
            // test euclid
            Heuristic e = new EuclidHeuristic(graph);
            aStar(start, goal, e, graph);
            double euclidScore = goal.gScore;
            ArrayList<Vertex> euclidPath = getPath(goal);
            euclidTimer += graph.timer;
            euclidNodes += e.getNodesPopped();
            if (iterations == 1)
                System.out.println("finished running EUCLID");
            
            graph.clear();
            coarseGraph.clear();
            
            // test dijkstra
            Heuristic d = new DijkstraHeuristic(graph);
            aStar(start, goal, d, graph);
            double dijkstraScore = goal.gScore;
            ArrayList<Vertex> dijkstraPath = getPath(goal);
            dijkstraTimer += graph.timer;
            dijkstraNodes += d.getNodesPopped();
            if (iterations == 1)
                System.out.println("finished running DIJKSTRA");
            
            error = (Math.abs(dijkstraScore - cornersScore)/dijkstraScore)*100;
            totalError += error;
            
            graph.clear();
            coarseGraph.clear();
            
            if (error > 10 || i == iterations-1) {
                try {
                    /*PrintWriter mapWriter = new PrintWriter("map.txt");
                    mapWriter.println(map);
                    mapWriter.close();*/
                    
                    PrintWriter writer = new PrintWriter("results.txt");
                    writer.println(map);
                    writer.println(coarseMap);
                    
                    writer.println("START: "+start);
                    writer.println("GOAL: "+goal);
                    
                    writer.println("COARSE LAYER");
                    writer.println("Nodes popped: "+((CornersHeuristic)c).getCoarseNodesPopped());
                    writer.println("Time taken: "+((CornersHeuristic)c).timer+"\n");
                    
                    writer.println("CORNERS");
                    writer.println("Cost: "+cornersScore);
                    writer.println("Nodes popped: "+c.getNodesPopped());
                    writer.println("Time taken: "+cornersTimer+"\n");
                    
                    writer.println("DIJSKTRA");
                    writer.println("Cost: "+dijkstraScore);
                    writer.println("Nodes popped: "+d.getNodesPopped());
                    writer.println("Time taken: "+dijkstraTimer+"\n");
                    
                    writer.println("EUCLID");
                    writer.println("Cost: "+euclidScore);
                    writer.println("Nodes popped: "+e.getNodesPopped());
                    writer.println("Time taken: "+euclidTimer+"\n");
                    
                    if (mapHeight*mapWidth < 10000) {
                        writer.println("H-Score lookup table:");
                        writer.println(((CornersHeuristic)c).tableAsString());
                        writer.println("\nPath taken by CORNERS:");
                        writer.println(graph.toString(cornersPath, blockH, blockW));
                        writer.println("\nPath taken by DIJKSTRA:");
                        writer.println(graph.toString(dijkstraPath, blockH, blockW));
                        writer.println("\nPath taken by EUCLID:");
                        writer.println(graph.toString(euclidPath, blockH, blockW));
                    }
                    writer.close();
                } catch(FileNotFoundException err) {
                    System.out.println("File was not found in method: MAIN");
                }
                System.out.println("\n\nreturned a non-optimal path - breaking operation at iteration "+i);
                break;
            }
        }
        nodeDiff = ((double)cornersNodes/Math.min(euclidNodes,dijkstraNodes))*100.0;
        timeDiff = ((double)cornersTimer/Math.min(euclidTimer,dijkstraTimer))*100.0;
        
        System.out.println("COARSE LAYER");
        System.out.println("Nodes popped: "+coarseNodes);
        System.out.println("Time taken: "+coarseTimer+"\n");
        
        System.out.println("CORNERS");
        System.out.println("Nodes popped: "+cornersNodes);
        System.out.println("Time taken: "+cornersTimer+"\n");
        
        System.out.println("EUCLID");
        System.out.println("Nodes popped: "+euclidNodes);
        System.out.println("Time taken: "+euclidTimer+"\n");
        
        System.out.println("DIJSKTRA");
        System.out.println("Nodes popped: "+dijkstraNodes);
        System.out.println("Time taken: "+dijkstraTimer+"\n");
        
        System.out.println("CORNERS heuristic on average... ");
        System.out.printf("was %.2f %% within optimal score\n",(100-totalError/iterations));
        System.out.printf("popped %.2f %% of nodes popped by others\n",nodeDiff);
        System.out.printf("took %.2f %% of the time took by others\n",timeDiff);
    }
    
    public static class Map {
        // A map comprised of weighted unit squares
        public double[][] map;
        public int height;
        public int width;
        public int startR = -1;
        public int startC = -1;
        public int goalR = -1;
        public int goalC = -1;
        
        public Map(int argHeight, int argWidth) {
            height = argHeight;
            width = argWidth;
            map = new double[height][width];
            Random rng = new Random();
            int inc = 2+rng.nextInt((int)Math.sqrt(height));
            for (int br = 0; br < height; br+=inc) {
                for (int bc = 0; bc < width; bc+=inc) {
                    int weight = 1+rng.nextInt(5);
                    for (int r = br; r < Math.min(height, br+inc); r++) {
                        for (int c = bc; c < Math.min(width, bc+inc); c++) {
                            map[r][c] = weight;
                        }
                    }
                }
            }
        }
        
        public Map(int argHeight, int argWidth, double[][] argMap) {
            height = argHeight;
            width = argWidth;
            map = argMap;
        }
        
        public Map(int argHeight, int argWidth, String mapName) {
            height = argHeight;
            width = argWidth;
            map = new double[height][width];
            try {
                FileReader input = new FileReader(mapName);
                BufferedReader bufRead = new BufferedReader(input);
                try {
                    bufRead.mark(10);
                    String row = bufRead.readLine();
                    if (row.contains("Map size:")) {
                    // starcraft map
                        String[] readSize = row.split(" ");
                        int readHeight = Integer.parseInt(readSize[4]);
                        int readWidth = Integer.parseInt(readSize[2]);
                        if (readWidth != width) {
                            System.out.println("user input map width "+width+" does not match map's width "+readWidth);
                            System.exit(0);
                        }
                        if (readHeight != height) {
                            System.out.println("user input map height "+height+ "does not match map's height "+readHeight);
                            System.exit(0);
                        }
                        row = bufRead.readLine(); // skip line
                        row = bufRead.readLine();
                        String[] startCoord = row.split(" ");
                        startR = Integer.parseInt(startCoord[1]);
                        startC = Integer.parseInt(startCoord[0]);
                        
                        row = bufRead.readLine();
                        String[] goalCoord = row.split(" ");
                        goalR = Integer.parseInt(goalCoord[1]);
                        goalC = Integer.parseInt(goalCoord[0]);
                        
                        row = bufRead.readLine(); // skip line
                        row = bufRead.readLine(); // skip line
                        
                        // start WALKABLE
                        for (int r = 0; r < height; r++) {
                            for (int c = 0; c < width; c++) {
                                row = bufRead.readLine();
                                String[] walkable = row.split(" ");
                                int readRow = Integer.parseInt(walkable[1]);
                                int readCol = Integer.parseInt(walkable[0]);
                                if (walkable[2] == "false") {
                                    map[readRow][readCol] = Double.POSITIVE_INFINITY;
                                } else {
                                    map[readRow][readCol] = 0.0;
                                }
                            }
                        }
                        
                        row = bufRead.readLine(); // skip line
                        row = bufRead.readLine(); // skip line
                        
                        // start THREAT
                        for (int r = 0; r < height; r++) {
                            for (int c = 0; c < width; c++) {
                                row = bufRead.readLine();
                                String[] threat = row.split(" ");
                                int readRow = Integer.parseInt(threat[1]);
                                int readCol = Integer.parseInt(threat[0]);
                                double readThreat = Double.parseDouble(threat[2]);
                                map[readRow][readCol] += Double.parseDouble(threat[2]);
                            }
                        }
                        
                        System.out.println("finished parsing map");
                    } else {
                    // .txt map
                        bufRead.reset();
                        for (int r = 0; r < height; r++) {
                            row = bufRead.readLine();
                            String[] weights = row.split("\\s+");
                            if (weights.length != width) {
                                System.out.println("user input size does not match actual size of map");
                                return;
                            }
                            int c = 0;
                            for (String s : weights) {
                                double weight = Double.parseDouble(s);
                                map[r][c] = weight;
                                c++;
                            }
                        }
                    }
                } catch (IOException ioe) {
                    System.out.println("Caught an IO Exception in Map<init>");
                }
            } catch(FileNotFoundException fnfe) {
                System.out.println("Caught a FileNotFound Exception in Map<init>");
                System.exit(1);
            }
        }
        
        public Map makeCoarse(int blockH, int blockW) {
            int newH = height/blockH;
            int newW = width/blockW;
            double[][] newMap = new double[newH][newW];
            for (int newR = 0; newR < newH; newR++) {
                for (int newC = 0; newC < newW; newC++) {
                    double minWeight = Double.POSITIVE_INFINITY;
                    for (int oldR = newR*blockH; oldR < (newR+1)*blockH; oldR++) {
                        for (int oldC = newC*blockW; oldC < (newC+1)*blockW; oldC++) {
                            minWeight = Math.min(minWeight, map[oldR][oldC]);
                        }
                    }
                    newMap[newR][newC] = minWeight;
                }
            }
            System.out.println("finished creating coarse layer");
            return new Map(newH, newW, newMap);
        }
        
        public double get(int row, int col) {
            return map[row][col];
        }
        
        public void print() {
            for (int r = 0; r < height; r++) {
                for (int c = 0; c < width; c++) {
                    System.out.print(map[r][c]+" ");
                }
                System.out.println();
            }
        }
        
        public String toString() {
            String output = "";
            DecimalFormat df = new DecimalFormat("00");
            for (int r = 0; r < height; r++) {
                for (int c = 0; c < width; c++) {
                    output+=df.format(map[r][c])+" ";
                }
                output+="\n";
            }
            return output;
        }
    }
    
    public static class Graph {
        // A graph comprised of vertices whose edge weights are determined by a Map
        public Vertex[][] graph;
        public int height;
        public int width;
        public Vertex goal;
        public Vertex start;
        public int goalType; // 0 = CORNER, 1 = INSIDE, 2 = HORIZONTAL, 3 = VERTICAL
        public Map coarseMap;
        public long timer;
        
        public Graph(Map map) {
            // Construct a graph from a regular Map
            int mapH = map.height;
            int mapW = map.width;
            height = mapH+1;
            width = mapW+1;
            graph = new Vertex[height][width];
            for (int r = 0; r < height; r++) {
                for (int c = 0; c < width; c++) {
                    Vertex v = new Vertex(r, c);
                    graph[r][c] = v;
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
                                        
            for (int r = 0; r < height; r++) {
                for (int c = 0; c < width; c++) {
                    Vertex v = graph[r][c];
                    //Connect cardinal directional neighbors (NSEW)
                    for (int i = 0; i < 4; i++) {
                        Point offset = cardinalOffsets[i];
                        int newR = r+offset.x;
                        int newC = c+offset.y;
                        if (newR >= 0 && newR < height && newC >= 0 && newC < width) {
                            Vertex neighbor = graph[newR][newC];
                            double square1 = Double.POSITIVE_INFINITY;
                            double square2 = Double.POSITIVE_INFINITY;
                            Point offset1 = cardinalSquares[2*i];
                            int mapR1 = r+offset1.x;
                            int mapC1 = c+offset1.y;
                            if (mapR1 >= 0 && mapR1 < mapH && mapC1 >= 0 && mapC1 < mapW) {
                                square1 = map.get(mapR1,mapC1);
                            }
                            Point offset2 = cardinalSquares[(2*i)+1];
                            int mapR2 = r+offset2.x;
                            int mapC2 = c+offset2.y;
                            if (mapR2 >= 0 && mapR2 < mapH && mapC2 >= 0 && mapC2 < mapW) {
                                square2 = map.get(mapR2,mapC2);
                            }
                            Edge e = new Edge(v, neighbor, Math.min(square1, square2));
                            v.neighbors.add(e);
                        }
                    }
                    //Connect diagonal neighbors
                    for (int i = 0; i < 4; i++) {
                        Point offset = diagonalOffsets[i];
                        int newR = r + offset.x;
                        int newC = c + offset.y;
                        if (newR >= 0 && newR < height && newC >= 0 && newC < width) {
                            Vertex neighbor = graph[newR][newC];
                            Point squareOffset = diagonalSquares[i];
                            int mapR = r + squareOffset.x;
                            int mapC = c + squareOffset.y;
                            double square = map.get(mapR,mapC);
                            Edge e = new Edge(v, neighbor, square*Math.sqrt(2.0));
                            v.neighbors.add(e);
                        }
                    }
                }
            }
        }
        
        public Graph(Map map, int blockH, int blockW) {
            // Construct a graph from a coarse Map
            int mapH = map.height;
            int mapW = map.width;
            height = mapH+1;
            width = mapW+1;
            coarseMap = map;
            graph = new Vertex[height][width];
            for (int r = 0; r < height; r++) {
                for (int c = 0; c < width; c++) {
                    Vertex v = new Vertex(r*blockH, c*blockW);
                    graph[r][c] = v;
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
                                        
            for (int r = 0; r < height; r++) {
                for (int c = 0; c < width; c++) {
                    Vertex v = graph[r][c];
                    //Connect cardinal directional neighbors (NSEW)
                    for (int i = 0; i < 4; i++) {
                        Point offset = cardinalOffsets[i];
                        int newR = r+offset.x;
                        int newC = c+offset.y;
                        if (newR >= 0 && newR < height && newC >= 0 && newC < width) {
                            Vertex neighbor = graph[newR][newC];
                            double square1 = Double.POSITIVE_INFINITY;
                            double square2 = Double.POSITIVE_INFINITY;
                            double weight;
                            Point offset1 = cardinalSquares[2*i];
                            int mapR1 = r+offset1.x;
                            int mapC1 = c+offset1.y;
                            if (mapR1 >= 0 && mapR1 < mapH && mapC1 >= 0 && mapC1 < mapW) {
                                square1 = map.get(mapR1,mapC1);
                            }
                            Point offset2 = cardinalSquares[(2*i)+1];
                            int mapR2 = r+offset2.x;
                            int mapC2 = c+offset2.y;
                            if (mapR2 >= 0 && mapR2 < mapH && mapC2 >= 0 && mapC2 < mapW) {
                                square2 = map.get(mapR2,mapC2);
                            }
                            if (i < 2) {
                                weight = Math.min(square1, square2)*blockH;
                            } else {
                                weight = Math.min(square1, square2)*blockW;
                            }
                            Edge e = new Edge(v, neighbor, weight);
                            v.neighbors.add(e);
                        }
                    }
                    //Connect diagonal neighbors
                    for (int i = 0; i < 4; i++) {
                        Point offset = diagonalOffsets[i];
                        int newR = r + offset.x;
                        int newC = c + offset.y;
                        if (newR >= 0 && newR < height && newC >= 0 && newC < width) {
                            Vertex neighbor = graph[newR][newC];
                            Point squareOffset = diagonalSquares[i];
                            int mapR = r + squareOffset.x;
                            int mapC = c + squareOffset.y;
                            double square = map.get(mapR,mapC);
                            double weight = square*diagonal(blockH, blockW);
                            Edge e = new Edge(v, neighbor, weight);
                            v.neighbors.add(e);
                        }
                    }
                }
            }
            
            System.out.println("finished creating coarse Graph");
        }
        
        public Vertex get(int row, int col) {
            return graph[row][col];
        }
        
        public void addStart(Vertex s, int blockH, int blockW) {
            // Only applicable for coarse Graphs
            start = new Vertex(s.row, s.col);
        }
        
        public void addGoal(Vertex g, int blockH, int blockW) {
            // Only applicable for coarse Graphs
            if (g.row%blockH == 0 && g.col%blockW == 0) {
            // Goal is a CORNER
                goal = graph[g.row/blockH][g.col/blockW];
                goalType = 0;
            } else if (g.row%blockH != 0 && g.col%blockW != 0) {
            // Goal is an INSIDE
                goal = new Vertex(g.row, g.col);
                goalType = 1;
                for (int dr = 0; dr < 2; dr++) {
                // dr = change in row
                    for (int dc = 0; dc < 2; dc++) {
                    // dc = change in col
                        Vertex neighbor = graph[dr+g.row/blockH][dc+g.col/blockW];
                        int rowDiff = (int)Math.abs(neighbor.row-g.row);
                        int colDiff = (int)Math.abs(neighbor.col-g.col);
                        double square = coarseMap.get(g.row/blockH, g.col/blockW);
                        double weight = diagonal(rowDiff, colDiff)*square;
                        goal.neighbors.add(new Edge(goal, neighbor, weight));
                        neighbor.neighbors.add(new Edge(neighbor, goal, weight));
                    }
                }
            } else if (g.row%blockH == 0) {
            // Goal is a HORIZONTAL BORDER
                goal = new Vertex(g.row, g.col);
                goalType = 2;
                for (int dc = 0; dc < 2; dc++) {
                    Vertex neighbor = graph[g.row/blockH][dc+g.col/blockW];
                    int colDiff = (int)Math.abs(neighbor.col-g.col);
                    double square1 = Double.POSITIVE_INFINITY;
                    if (g.row/blockH-1 >= 0 && g.col/blockW < coarseMap.width) {
                        square1 = coarseMap.get(g.row/blockH-1, g.col/blockW);
                    }
                    double square2 = Double.POSITIVE_INFINITY;
                    if (g.row/blockH < coarseMap.height && g.col/blockW < coarseMap.width) {
                        square2 = coarseMap.get(g.row/blockH, g.col/blockW);
                    }
                    double weight = Math.min(square1, square2)*colDiff;
                    goal.neighbors.add(new Edge(goal, neighbor, weight));
                    neighbor.neighbors.add(new Edge(neighbor, goal, weight));
                }
            } else {
            // Goal is a VERTICAL BORDER
                goal = new Vertex(g.row, g.col);
                goalType = 3;
                for (int dr = 0; dr < 2; dr++) {
                    Vertex neighbor = graph[dr+g.row/blockH][g.col/blockW];
                    int rowDiff = (int)Math.abs(neighbor.row-g.row);
                    double square1 = Double.POSITIVE_INFINITY;
                    if (g.row/blockH < coarseMap.height && g.col/blockW-1 >= 0) {
                        square1 = coarseMap.get(g.row/blockH, g.col/blockW-1);
                    }
                    double square2 = Double.POSITIVE_INFINITY;
                    if (g.row/blockH < coarseMap.height && g.col/blockW < coarseMap.width) {
                        square2 = coarseMap.get(g.row/blockH, g.col/blockW);
                    }
                    double weight = Math.min(square1, square2)*rowDiff;
                    goal.neighbors.add(new Edge(goal, neighbor, weight));
                    neighbor.neighbors.add(new Edge(neighbor, goal, weight));
                }
            }
        }
        
        public double diagonal(int x, int y) {
            return Math.max(x,y);
            /*if (x > y) {
                return y*Math.sqrt(2.0)+(x-y);
            } else {
                return x*Math.sqrt(2.0)+(y-x);
            }*/
        }
        
        public void clear() {
        // resets all the Scores and prev of every Vertex in the graph
        // Use between AStar Searches
            for (int r = 0; r < height; r++) {
                for (int c = 0; c < width; c++) {
                    Vertex v = graph[r][c];
                    v.prev = null;
                    v.fScore = Double.POSITIVE_INFINITY;
                    v.gScore = Double.POSITIVE_INFINITY;
                    v.hScore = Double.POSITIVE_INFINITY;
                }
            }
            if (goal != null) {
                goal.prev = null;
                goal.gScore = Double.POSITIVE_INFINITY;
            }
            if (start != null) {
                start.prev = null;
                start.gScore = Double.POSITIVE_INFINITY;
            }
        }
        
        public void print() {
            for (int r = 0; r < height; r++) {
                for (int c = 0; c < width; c++) {
                    System.out.print(graph[r][c]+" ");
                }
                System.out.println();
            }
        }
        
        public void print(ArrayList<Vertex> path, int blockH, int blockW) {
            for (int r = 0; r < height; r++) {
                for (int c = 0; c < width; c++) {
                    String s = graph[r][c].inPath(path, blockH, blockW);
                    System.out.print(s+" ");
                }
                System.out.println();
            }
        }
        
        public String toString() {
            String output = "";
            for (int r = 0; r < height; r++) {
                for (int c = 0; c < width; c++) {
                    output += graph[r][c]+" ";
                }
            output += "\n";
            }
            return output;
        }
        
        public String toString(ArrayList<Vertex> path, int blockH, int blockW) {
            String output = "";
            for (int r = 0; r < height; r++) {
                for (int c = 0; c < width; c++) {
                    String s = graph[r][c].inPath(path, blockH, blockW);
                    output += s;
                }
                output += "\n";
            }
            return output;
        }
    }
    
    public static class Vertex implements Comparable<Vertex>{
        public int row;
        public int col;
        public ArrayList<Edge> neighbors;
        public double fScore;
        public double gScore;
        public double hScore;
        public Vertex prev;
        
        public Vertex(int argRow, int argCol) {
            row = argRow;
            col = argCol;
            neighbors = new ArrayList<Edge>();
            gScore = Double.POSITIVE_INFINITY;
        }
        
        public Vertex(Vertex v) {
            row = v.row;
            col = v.col;
            neighbors = new ArrayList<Edge>();
            for (Edge e : v.neighbors) {
                neighbors.add(e);
            }
            fScore = v.fScore;
            gScore = v.gScore;
            hScore = v.hScore;
        }
        
        public boolean equals(Vertex v) {
            return (v.row == row && v.col == col);
        }
        
        public String toString() {
            return "("+row+","+col+")";
        }
        
        public String inPath(ArrayList<Vertex> path, int blockH, int blockW) {
            if (path.contains(this)) {
                return "o";
            } else {
                if (row%blockH == 0 && col%blockW == 0) {
                    return "+";
                } else if (row%blockH != 0 && col%blockW != 0) {
                    return " ";
                } else if (row%blockH == 0) {
                    return "-";
                } else {
                    return "|";
                }
            }
        }
        
        public int compareTo(Vertex v) {
            if (Double.compare(fScore, v.fScore) == 0) {
                return Double.compare(gScore, v.gScore);
            } else {
                return Double.compare(fScore, v.fScore);
            }
        }
    }
    
    public static class Edge {
        Vertex start;
        Vertex end;
        double weight;
        
        public Edge(Vertex argStart, Vertex argEnd, double argWeight) {
            start = argStart;
            end = argEnd;
            weight = argWeight;
        }
        
        public String toString() {
            return "("+start+","+end+","+weight+")";
        }
    }

    public static interface Heuristic {    
        public double getScore(Vertex v, Vertex goal);
        
        public void inc();
        
        public int getNodesPopped();
    }
    
    public static class EuclidHeuristic implements Heuristic {
        Graph graph;
        int nodesPopped;
    
        public EuclidHeuristic(Graph argGraph) {
            graph = argGraph;
            nodesPopped = 0;
        }
    
        public double getScore(Vertex v, Vertex goal) {
            int rowDiff = (int)Math.abs(v.row-goal.row);
            int colDiff = (int)Math.abs(v.col-goal.col);
            if (rowDiff > colDiff) {
                return colDiff*Math.sqrt(2.0)+(rowDiff-colDiff);
            } else {
                return rowDiff*Math.sqrt(2.0)+(colDiff-rowDiff);
            }
        }
        
        public void inc() {
            nodesPopped++;
        }
        
        public int getNodesPopped() {
            return nodesPopped;
        }
    }
    
    public static class CornersHeuristic implements Heuristic {
        // getScore is not static; need instance of CornersHeuristic to use
        int blockH;
        int blockW;
        Graph graph;
        int graphH;
        int graphW;
        public double[][] hTable;
        Graph coarseGraph;
        Map coarseMap;
        PriorityQueue<Vertex> savedFringe;
        int nodesPopped;
        int coarseNodesPopped;
        long timer;
        
        public double diagonal(int x, int y) {
            return Math.max(x, y);
            /*if (x > y) {
                return y*Math.sqrt(2.0)+(x-y);
            } else {
                return x*Math.sqrt(2.0)+(y-x);
            }*/
        }
        
        public CornersHeuristic(Graph argGraph, Graph argCoarseGraph, Map argCoarseMap) {
            blockH = Search.blockH;
            blockW = Search.blockW;
            graph = argGraph;
            graphH = graph.height;
            graphW = graph.width;
            hTable = new double[graphH][graphW];
            coarseGraph = argCoarseGraph;
            coarseMap = argCoarseMap;
            savedFringe = null;
            nodesPopped = 0;
            coarseNodesPopped = 0;
            timer = 0;
            for (int row = 0; row < graphH; row++) {
                for (int col = 0; col < graphW; col++) {
                    hTable[row][col] = -1.0;
                }
            }
        }
        
        public double getScore(Vertex v, Vertex goal) {
            int row = v.row;
            int col = v.col;
            
            if (hTable[row][col] != -1.0) {
                return hTable[row][col];
            } else {
                int vBlockR = v.row/blockH;
                int vBlockC = v.col/blockW;
                int gBlockR = goal.row/blockH;
                int gBlockC = goal.col/blockW;
                double tempHScore = Double.POSITIVE_INFINITY;
                Vertex corner = null;
                double cornerScore = Double.POSITIVE_INFINITY;
                int rowDiff = 0;
                int colDiff = 0;
                double weight = 0;
                double square1 = Double.POSITIVE_INFINITY;
                double square2 = Double.POSITIVE_INFINITY;
                
                if (row%blockH == 0 && col%blockW == 0) {
                // v is a CORNER vertex
                    smartAStar(coarseGraph.goal, coarseGraph.start, v);
                    return hTable[row][col];
                } else if (row%blockH != 0 && col%blockW != 0) {
                // v is an INSIDE vertex
                    for (int dr = 0; dr < 2; dr++) {
                        for (int dc = 0; dc < 2; dc++) {
                            corner = coarseGraph.get(dr+vBlockR, dc+vBlockC);
                            if (hTable[(dr+vBlockR)*blockH][(dc+vBlockC)*blockW] == -1) {
                                smartAStar(coarseGraph.goal, coarseGraph.start, corner);
                            }
                            cornerScore = hTable[(dr+vBlockR)*blockH][(dc+vBlockC)*blockW];
                            rowDiff = (int)Math.abs(corner.row-row);
                            colDiff = (int)Math.abs(corner.col-col);
                            weight = coarseMap.get(vBlockR, vBlockC);
                            tempHScore = Math.min(tempHScore, cornerScore+diagonal(rowDiff,colDiff)*weight);
                        }
                    }
                    if (coarseGraph.goalType == 1 &&
                        vBlockR == gBlockR &&
                        vBlockC == gBlockC) {
                    // goal is an INSIDE vertex and v is in the same block as goal
                        rowDiff = (int)Math.abs(goal.row-row);
                        colDiff = (int)Math.abs(goal.col-col);
                        square1 = coarseMap.get(vBlockR, vBlockC);
                        hTable[row][col] = diagonal(rowDiff, colDiff)*square1;
                        return hTable[row][col];
                    } else if (coarseGraph.goalType == 2 &&
                        (vBlockR == gBlockR || vBlockR == gBlockR-1) &&
                        vBlockC == gBlockC) {
                    // goal is a HORIZONTAL vertex and v is in one of two goal blocks
                        rowDiff = (int)Math.abs(goal.row-row);
                        colDiff = (int)Math.abs(goal.col-col);
                        square1 = coarseMap.get(row/blockH, col/blockW);
                        tempHScore = Math.min(tempHScore, diagonal(rowDiff,colDiff)*square1);
                    } else if (coarseGraph.goalType == 3 &&
                        vBlockR == gBlockR &&
                        (vBlockC == gBlockC || vBlockC == gBlockC-1)) {
                    // goal is a VERTICAL vertex and v is in one of the two goal blocks
                        rowDiff = (int)Math.abs(goal.row-row);
                        colDiff = (int)Math.abs(goal.col-col);
                        square1 = coarseMap.get(vBlockR, vBlockC);
                        tempHScore = Math.min(tempHScore, diagonal(rowDiff,colDiff)*square1);
                    }
                    hTable[row][col] = tempHScore;
                    return hTable[row][col];                        
                } else if (row%blockH == 0) {
                // v is a HORIZONTAL BORDER vertex
                    if (vBlockR-1 >= 0 && vBlockC < coarseMap.width) {
                        square1 = coarseMap.get(vBlockR-1, vBlockC);
                    }
                    if (vBlockR < coarseMap.height && vBlockC < coarseMap.width) {
                        square2 = coarseMap.get(vBlockR, vBlockC);
                    }
                    for (int dr = -1; dr < 2; dr++) {
                        if (dr+vBlockR >= 0 && dr+vBlockR < coarseGraph.height) {
                            for (int dc = 0; dc < 2; dc++) {
                                corner = coarseGraph.get(dr+vBlockR, dc+vBlockC);
                                if (hTable[(dr+vBlockR)*blockH][(dc+vBlockC)*blockW] == -1) {
                                    smartAStar(coarseGraph.goal, coarseGraph.start, corner);
                                }
                                cornerScore = hTable[(dr+row/blockH)*blockH][(dc+col/blockW)*blockW];
                                rowDiff = (int)Math.abs(corner.row-row);
                                colDiff = (int)Math.abs(corner.col-col);
                                if (dr == 0) {
                                    weight = Math.min(square1, square2);
                                } else if (dr == -1) {
                                    weight = square1;
                                } else {
                                    weight = square2;
                                }
                                tempHScore = Math.min(tempHScore, cornerScore+diagonal(rowDiff,colDiff)*weight);
                            }
                        }
                    }
                    if (coarseGraph.goalType == 1 &&
                        vBlockC == gBlockC &&
                        (vBlockR == gBlockR || vBlockR-1 == gBlockR)) {
                    // goal is an INSIDE vertex and is in one of the two blocks that v touches
                        rowDiff = (int)Math.abs(goal.row-row);
                        colDiff = (int)Math.abs(goal.col-col);
                        if (vBlockR-1 == gBlockR) {
                            weight = square1;
                        } else {
                            weight = square2;
                        }
                        tempHScore = Math.min(tempHScore, diagonal(rowDiff, colDiff)*weight);
                    } else if (coarseGraph.goalType == 2 &&
                        v.row == goal.row &&
                        vBlockC == gBlockC) {
                    // goal is also on this HORIZONTAL BORDER
                        colDiff = (int)Math.abs(goal.col-col);
                        weight = Math.min(square1, square2);
                        tempHScore = Math.min(tempHScore, colDiff*weight);
                    }
                    hTable[row][col] = tempHScore;
                    return tempHScore;
                } else {
                // v is a VERTICAL BORDER vertex
                    if (vBlockR < coarseMap.height && vBlockC-1 >= 0) {
                        square1 = coarseMap.get(vBlockR, vBlockC-1);
                    }
                    if (vBlockR < coarseMap.height && vBlockC < coarseMap.width) {
                        square2 = coarseMap.get(vBlockR, vBlockC);
                    }
                    for (int dc = -1; dc < 2; dc++) {
                        if (dc+vBlockC >= 0 && dc+vBlockC < coarseGraph.width) {
                            for (int dr = 0; dr < 2; dr++) {
                                corner = coarseGraph.get(dr+vBlockR, dc+vBlockC);
                                if (hTable[(dr+vBlockR)*blockH][(dc+vBlockC)*blockW] == -1) {
                                    smartAStar(coarseGraph.goal, coarseGraph.start, corner);
                                }
                                cornerScore = hTable[(dr+vBlockR)*blockH][(dc+vBlockC)*blockW];
                                rowDiff = (int)Math.abs(corner.row-row);
                                colDiff = (int)Math.abs(corner.col-col);
                                if (dc == 0) {
                                    weight = Math.min(square1, square2);
                                } else if (dc == -1) {
                                    weight = square1;
                                } else {
                                    weight = square2;
                                }
                                tempHScore = Math.min(tempHScore, cornerScore+diagonal(rowDiff,colDiff)*weight);
                            }
                        }
                    }
                    if (coarseGraph.goalType == 1 &&
                        vBlockR == gBlockR &&
                        (vBlockC == gBlockC || vBlockC-1 == gBlockC)) {
                    // goal is an INSIDE vertex and is in one of the two blocks that v touches
                        rowDiff = (int)Math.abs(goal.row-row);
                        colDiff = (int)Math.abs(goal.col-col);
                        if (vBlockC-1 == gBlockC) {
                            weight = square1;
                        } else {
                            weight = square2;
                        }
                        tempHScore = Math.min(tempHScore, diagonal(rowDiff, colDiff)*weight);
                    } else if (coarseGraph.goalType == 3 &&
                        v.col == goal.col &&
                        vBlockR == gBlockR) {
                    // goal is also on this VERTICLE BORDER
                        rowDiff = (int)Math.abs(goal.row-row);
                        weight = Math.min(square1, square2);
                        tempHScore = Math.min(tempHScore, rowDiff*weight);
                    }
                    hTable[row][col] = tempHScore;
                    return tempHScore;
                }
            }
        }
        
        public void smartAStar(Vertex source, Vertex end, Vertex stopper) {
            long startTimer = System.nanoTime();
            PriorityQueue<Vertex> queue = new PriorityQueue<Vertex>();
            ArrayList<Vertex> closed = new ArrayList<Vertex>();
            Heuristic h = new EuclidHeuristic(coarseGraph);
            if (savedFringe == null) {
                source.gScore = 0.0;
                hTable[source.row][source.col] = 0.0;    
                source.hScore = h.getScore(source, end);
                source.fScore = source.gScore + source.hScore;
                queue.add(source);
            } else {
                queue = savedFringe;
            }
            boolean resume = true;

            while (!queue.isEmpty() && resume == true) {
                coarseNodesPopped++;
                Vertex current = queue.poll();
                closed.add(current);
                for (Edge e : current.neighbors) {
                    Vertex neighbor = e.end;
                    double tempGScore = current.gScore + e.weight;
                    if (closed.contains(neighbor)) {
                        continue;
                    } else if (tempGScore < neighbor.gScore) {
                        queue.remove(neighbor);
                        neighbor.gScore = tempGScore ;
                        neighbor.prev = current;
                        neighbor.hScore = h.getScore(neighbor, end);
                        neighbor.fScore = neighbor.gScore + neighbor.hScore;
                        hTable[neighbor.row][neighbor.col] = tempGScore;
                        queue.add(neighbor);
                    }
                }
                if (current.equals(stopper)) {
                    resume = false;
                    savedFringe = queue;
                }
            }
            long endTimer = System.nanoTime();
            timer += (endTimer-startTimer)/1000;
        }
        
        public String tableAsString() {
            String output = "";
            DecimalFormat df = new DecimalFormat("000");
            for (int r = 0; r < graphH; r++) {
                for (int c = 0; c < graphW; c++) {
                    String formatted = df.format(hTable[r][c]);
                    if (hTable[r][c] == -1) {
                        output += "--- ";
                    } else {
                        output += formatted+" ";
                    }
                }
                output += "\n";
            }
            return output;
        }
        
        public void inc() {
            nodesPopped++;
        }
        
        public int getNodesPopped() {
            return nodesPopped;
        }
        
        public int getCoarseNodesPopped() {
            return coarseNodesPopped;
        }
    }
    
    public static class DijkstraHeuristic implements Heuristic {
        Graph graph;
        int nodesPopped;
    
        public DijkstraHeuristic(Graph argGraph) {
            graph = argGraph;
            nodesPopped = 0;
        }
    
        public double getScore(Vertex v, Vertex goal) {
            return 0.0;
        }
        
        public void inc() {
            nodesPopped++;
        }
        
        public int getNodesPopped() {
            return nodesPopped;
        }
    }
    
    public static void aStar(Vertex start, Vertex goal, Heuristic h, Graph graph) {
        long startTimer = System.nanoTime();
        PriorityQueue<Vertex> fringe = new PriorityQueue<Vertex>();
        Hashtable<Point, Vertex> closed = new Hashtable<Point, Vertex>();
        
        start.gScore = 0.0;
        start.hScore = h.getScore(start, goal);
        start.fScore = start.gScore + start.hScore;
        fringe.add(start);
        Vertex v = start;
        int size = 1;
        
        while (v.fScore < goal.gScore && size != 0) { // stopping condition for fringe popping
            h.inc();
            v = fringe.poll();
            size = size-1;
            closed.put(new Point(v.row, v.col), v);
            for (Edge e : v.neighbors) {
                Vertex neighbor = e.end;
                double tempGScore = v.gScore + e.weight;
                if (tempGScore > neighbor.gScore ) { 
                // ignores update stage if cost is greater than current cost
                    continue;
                }
                //if (closed.get(new Point(neighbor.row, neighbor.col)) != null) continue;
                if (tempGScore <= neighbor.gScore) { 
                // otherwise updates its neighbors and adds to fringe
                    boolean check = fringe.remove(neighbor);
                    if (check) {
                        size = size-1;
                    }
                    neighbor.prev = v;
                    neighbor.gScore = tempGScore;
                    neighbor.hScore = h.getScore(neighbor, goal);
                    neighbor.fScore = neighbor.gScore + neighbor.hScore;
                    fringe.add(neighbor);
                    size = size+1;
                }
            }
        }
        long endTimer = System.nanoTime();
        graph.timer = (endTimer-startTimer)/1000;
    }
    
    public static void dijkstra(Vertex start, Vertex goal, Graph graph) {
        Heuristic h = new DijkstraHeuristic(graph);
        aStar(start, goal, h, graph);
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
}
