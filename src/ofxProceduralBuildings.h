#pragma once

#include "ofMain.h"
#include "ofxProceduralCity.h"

struct Building {
    Building();
};

class ofxProceduralBuildings{
private:
    vector<Building> buildings;
    
public:
    void setup();
    void generate();
    void draw();
    
    ofxProceduralBuildings();
};
