compile:    javac search.java
run:        java [-cp .] Search <MAP HEIGHT> <MAP WIDTH> <BLOCK HEIGHT> <BLOCK WIDTH>
                                [# SEARCHES] [MAP FILE]
                                [START ROW] [START COL] [GOAL ROW] [GOAL COL]
defaults:   # SEARCHES      : 1
            MAP             : randomly generated
            if # SEARCHES=1 : START   = (0, 0)
                              GOAL    = (H, W)
            else            : START and GOAL are randomly selected points on the map
            
notes:      If more than 1 search is run, the program will automatically terminate if the
            CORNERS heuristic returns a path that is outside 5% of optimal. The search
            which caused this, as well as the map and relevant search data, is written to
            a file named 'results.txt'