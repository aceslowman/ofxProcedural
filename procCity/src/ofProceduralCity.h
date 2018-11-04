#pragma once

#include "ofMain.h"

class RoadSegment {

public:
    ofVec2f start, end;
    
    float time_delay;
    float population;
    
    ofPolyline line;
    
    float getSlope();
    float getRotation();
    
    RoadSegment(float _ti, ofVec2f _start, ofVec2f _end);
};

class ofProceduralCity {

public:

    int global_walk;
    int global_counter;
    
    vector<RoadSegment> pending_list;
    vector<RoadSegment> placed_list;
    
    ofImage pop_map;
    float map_size;
    
    int road_limit;
    float road_scalar;
    
    ofMesh mesh;
    ofMesh intersectionMesh;
    vector<ofVec3f> points;
    
    // setup
    void setup();
    void setupDebug();
    
    // generation
    void generateRoads();
    
    // constraint
    bool localConstraints(RoadSegment &a);
    vector<RoadSegment> globalGoals(RoadSegment &a);
    bool constrainToIntersections(RoadSegment &a);
    bool constrainToCityPattern(RoadSegment &prev, RoadSegment &next);
    
    // checks
    bool globalBoundsCheck(ofVec2f &a);
    
    // utility
    int samplePopulation(ofVec2f s);
    static bool sortByDelay(RoadSegment A, RoadSegment B);
    static bool sortByDistance(ofVec2f A, ofVec2f B, ofVec2f pt);
    bool getLineIntersection(ofVec2f p0, ofVec2f p1, ofVec2f p2, ofVec2f p3, ofVec2f &intersection);
    float getRoadAngle(ofVec2f A, ofVec2f B, ofVec2f C);
    
    // city rules
    void BasicRule();
    void NewYorkRule();
    void ParisRule();
    void SanFranRule();
    
    // drawing
    void draw();
    void printDebug();
};
