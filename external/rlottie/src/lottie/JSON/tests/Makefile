CXXFLAGS:=-O0 -g -DJSONPartialParsing

all: JSONDump JSONPartial JSONSaxTest

JSONDump: JSONDump.o
	$(CXX) $(CXXFLAGS) -o $@ $^
JSONPartial: JSONPartialTest.o
	$(CXX) $(CXXFLAGS) -o $@ $^
JSONSaxTest: JSONSaxTest.o
	$(CXX) $(CXXFLAGS) -o $@ $^

.cpp.o:
	$(CXX) $(CXXFLAGS) -c $< -o $@

clean:
	-rm JSONDump
	-rm JSONPartial
	-rm JSONSaxTest
	-rm *.o
