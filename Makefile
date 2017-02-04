# Student name: Hichem Rehouma
# Student ID: V00811045
# ELEC 586: Project 

CFLAGS = `pkg-config --cflags opencv`
LIBS = `pkg-config --libs opencv`


all: project_soft 


project_soft : project_soft.o 
	g++ $(CFLAGS)  -o $@ $^ $(LIBS)

project_soft.o: project_soft.cpp
	g++ $(CFLAGS)  -c $< $(LIBS)


.PHONY: all clean


