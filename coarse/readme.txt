compile:    javac search.java
run:        java [-cp .] Search <MAP HEIGHT> <MAP WIDTH> <BLOCK HEIGHT> <BLOCK WIDTH>
                                [# SEARCHES] [MAP FILE]
                                [START ROW] [START COL] [GOAL ROW] [GOAL COL]
defaults:   # SEARCHES      : 1
            MAP             : randomly generated
            if # SEARCHES=1 : START   = (0, 0)
                              GOAL    = (H, W)
            else            : START and GOAL are randomly selected points on the map