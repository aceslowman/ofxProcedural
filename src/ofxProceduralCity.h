#pragma once

#include "ofMain.h"
#include "ofxGui.h"
#include "ofxProceduralMap.h"
#include "ofxProceduralRoads.h"
#include "ofxProceduralBuildings.h"
#include "ofxProceduralTerrain.h"

class ofxProceduralCity {
    
private:

public:
    ofxProceduralMap population_map;
    ofxProceduralMap elevation_map;
    
    ofxProceduralRoads roads;
    ofxProceduralBuildings buildings;
    ofxProceduralTerrain terrain;
    
    ofNode node;
    
    bool globalBoundsCheck(ofVec3f &a);
    
    void reset();
    void setup();
    void setupGui();
    void draw();
    
    ofVec2f dimensions;
    int global_walk;
    
    ofxPanel gui;
    ofParameterGroup params;
    ofParameter<bool> regen_all;
    
    void regenClicked(bool &val);
    
    ofxProceduralCity() : roads(this), buildings(this), terrain(this){}
};

/*
 https://stackoverflow.com/questions/30694183/utility-functions-in-a-namespace-or-in-a-class
 */
namespace proc_utils {
    bool getLineIntersection(ofVec2f p0, ofVec2f p1, ofVec2f p2, ofVec2f p3, ofVec2f &intersection);
    float getDistanceBetweenPointandLine( Crossing _a, Crossing _b, ofVec3f _p );
    bool sortByDistance(ofVec2f A, ofVec2f B, ofVec2f pt);
}
