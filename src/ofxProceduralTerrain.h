#pragma once
#include "ofMain.h"
#include "ofxGui.h"

class ofxProceduralCity;
class ofxProceduralMap;

class ofxProceduralTerrain{
private:
    ofxProceduralCity *city;
    ofxProceduralMap *elevation;
    
    ofVboMesh mesh;
    
    void displace();
    
public:
    void reset();
    void setup(ofxProceduralMap *_elevation);
    void generate();
    void generatePlane();
    
    void draw();
    void drawDebug(ofEasyCam* cam);
    
    ofNode node;
    
    ofParameterGroup params;
    ofParameter<float> zscale;
    ofParameter<bool> show_elevation;
    ofParameter<float> detail_level;
    
    ofxProceduralTerrain(ofxProceduralCity *_city): city(_city){
        params.setName("Terrain");
        params.add(zscale.set("z-scale", 100, 0, 500));
        params.add(show_elevation.set("Show Elev.", true));
        params.add(detail_level.set("Detail Level", 1.0, 0.0, 1.0));
    };
};
