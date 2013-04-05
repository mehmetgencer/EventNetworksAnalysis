LIBS=libs/json-simple-1.1.1.jar:libs/commons-cli-1.2.jar

compile: clear
	javac -Xlint -classpath $(LIBS) -d bin src/ena/*.java
clear:
	rm -fr bin/*
testrun10: compile
	java -classpath bin:$(LIBS) ena.enatest -v 2 -f 200 testdata/test10.enet
testrun10nd: compile
	java -classpath bin:$(LIBS) ena.enatest -v 1 -f 200 testdata/test10.enet
testrun: compile
	java -classpath bin:$(LIBS) ena.enatest -v 3 -f 500 testdata/scalainternals.enet
testrunnd: compile
	java -classpath bin:$(LIBS) ena.enatest -v 1 -f 500 testdata/scalainternals.enet
papertex:
	cd paper; pdflatex ena.tex;bibtex ena.aux;pdflatex ena.tex;pdflatex ena.tex
