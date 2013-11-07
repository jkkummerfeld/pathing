.SUFFIXES: .java .class

.java.class:
	javac $<
    
SOURCES = search.java

CLASSES = Search.class Search$*.class

all:
	mkdir -p bin
	javac -O -d bin search.java

clean:
	rm -rf bin
	rm -rf *.class
	
test:
	java -Xrunhprof:cpu=samples,depth=12 -cp ./bin Search 64 64 4 4 100
	java -cp ./bin Search 40 40 5 5 100 maps/checkers.txt
	java -cp ./bin Search 20 20 5 5 100 maps/checkers2.txt
	java -cp ./bin Search 40 40 5 5 100 maps/hhlarge.txt
	java -cp ./bin Search 40 40 5 5 1 maps/bullseye.txt 40 40 20 20
	
debug:
	java -cp ./bin Search 30 30 5 5 1 maps/debug.txt 17 29 28 2
	
destination:
	java -Xrunhprof:cpu=samples,depth=12 -cp ./bin Search 512 384 8 8 1 dest/destination011.txt
	java -cp ./bin Search 512 384 16 16 1 dest/destination011.txt
	java -cp ./bin Search 512 384 32 32 1 dest/destination011.txt