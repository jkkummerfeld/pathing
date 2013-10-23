compile:    javac search.java
run:        java [-cp .] Search <MAP HEIGHT> <MAP WIDTH> <BLOCK HEIGHT> <BLOCK WIDTH>
                                [# SEARCHES] [MAP FILE]
                                [START ROW] [START COL] [GOAL ROW] [GOAL COL]
defaults:   # SEARCHES      : 1
            MAP             : randomly generated
            if # SEARCHES=1 : START   = (0, 0)
                              GOAL    = (H, W)
            else            : START and GOAL are randomly selected points on the map
            
notes:      If # SEARCHES is more than 1, the program will automatically terminate if the
            CORNERS heuristic returns a path that is more than 5% away from optimal. The 
            search that caused this, as well as the map and some data about the search
            is written to 'results.txt'