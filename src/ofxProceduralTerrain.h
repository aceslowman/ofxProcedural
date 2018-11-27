#pragma once
#include "ofMain.h"
#include "ofxGui.h"

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
    
    ofParameterGroup params;
    ofParameter<float> zscale;
    ofParameter<bool> show_elevation;
    
    ofxProceduralTerrain(ofxProceduralCity *_city): city(_city){
        params.setName("Terrain");
        params.add(zscale.set("z-scale", 100, 0, 500));
        params.add(show_elevation.set("Show Elev.", true));
    };
};
