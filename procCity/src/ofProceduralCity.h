#pragma once

#include "ofMain.h"

class RoadSegment {
    /*
     ri - the geometrical properties of the segment
     metadata - any additional metadata associated with the segment
     */
public:
    ofVec2f start, end;
    
    float time_delay;
    float metadata;

    ofColor color;
    
    // metadata
    float population;
    
    ofPolyline debugLine;
    
    RoadSegment(float _ti, ofVec2f _start, ofVec2f _end);
};

class ofProceduralCity {

public:
    int global_walk;
    
    void setup();
    void draw();
    void printDebug();

    ofImage pop_map;
    float map_size;
    
    float road_scalar;
    
    ofMesh mesh;
    vector<ofVec3f> points;
    
private:
    void setupDebug();
    
    bool localConstraints(RoadSegment &r);
    vector<RoadSegment> globalGoals(RoadSegment &r);
    
    bool globalBoundsCheck(ofVec2f &node);
    bool getLineIntersection(ofVec2f p0, ofVec2f p1, ofVec2f p2, ofVec2f p3, ofVec2f *intersection);
    static bool sortByDelay(RoadSegment A, RoadSegment B);
    
    vector<RoadSegment> priority_list;
    vector<RoadSegment> segment_list;
    
    int global_counter;
};
