#pragma once
#include "ofMain.h"

class ofxProceduralCity;
class ofxProceduralMap;

class ofxProceduralTerrain{
private:
    ofxProceduralCity *city;
    ofxProceduralMap *elevation;
    
    ofMesh mesh;
    
    void displace();
    
public:
    void reset();
    void setup(ofxProceduralMap *_elevation);
    void generate();
    
    void draw();
    void drawDebug(ofEasyCam* cam);
    
    ofxProceduralTerrain(ofxProceduralCity *_city): city(_city){};
};
