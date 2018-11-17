#pragma once

#include "ofMain.h"
#include "ofxProceduralMap.h"
#include "ofxProceduralRoads.h"

class ofxProceduralCity {
    
private:

public:
    ofxProceduralMap population_map;
    ofxProceduralMap elevation_map;
    
    ofxProceduralRoads roads;
    
    bool globalBoundsCheck(ofVec3f &a);
    
    void reset();
    void setup();
    void draw();
    
    int map_size;
    int global_walk;
    
    ofxProceduralCity() : roads(this){}
};

/*
 https://stackoverflow.com/questions/30694183/utility-functions-in-a-namespace-or-in-a-class
 */
namespace proc_utils {
    bool getLineIntersection(ofVec2f p0, ofVec2f p1, ofVec2f p2, ofVec2f p3, ofVec2f &intersection);
    float getDistanceBetweenPointandLine( ofVec2f a, ofVec2f b, ofVec2f p );
    bool sortByDistance(ofVec2f A, ofVec2f B, ofVec2f pt);
}
