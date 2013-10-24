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
	java -Xrunhprof:cpu=samples,depth=12 -cp ./bin Search 64 64 8 8 100