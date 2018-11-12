#pragma once

#include "ofMain.h"

struct RoadSegment {
    ofVec2f start, end;
    
    float time_delay;
    
    ofPolyline line;
    
    RoadSegment(float _ti, ofVec2f _start, ofVec2f _end);
};

struct Building {};

class ofProceduralCity {
    
private:
    vector<RoadSegment> pending_list;
    vector<RoadSegment> placed_list;
    vector<ofVec2f> crossing_list;
    
    ofImage pop_map;

    int road_limit;
    float road_scalar;

    ofMesh mesh;
    ofMesh crossingMesh;

    void setupDebug();
    
    // generation
    void generateRoads();
    void generatePopulationMap();
    
    // constraint
    bool localConstraints(RoadSegment &a);
    vector<RoadSegment> globalGoals(RoadSegment &a);
    bool checkForCrossings(RoadSegment &a);
    bool checkForNearby(RoadSegment &a);
    
    // checks
    bool globalBoundsCheck(ofVec2f &a);
    
    // utility
    int samplePopulation(ofVec2f s);
    bool getLineIntersection(ofVec2f p0, ofVec2f p1, ofVec2f p2, ofVec2f p3, ofVec2f &intersection);
    float getDistanceBetweenPointandLine( ofVec2f a, ofVec2f b, ofVec2f p );
    static bool sortByDelay(RoadSegment A, RoadSegment B);
    static bool sortByDistance(ofVec2f A, ofVec2f B, ofVec2f pt);
    
    // city rules
    bool constrainToRightAngles(RoadSegment &prev, ofVec2f &end);
    bool constrainToPopulation(RoadSegment &prev, ofVec2f &end);
    
public:
    int map_size;
    int global_walk;
    
    // setup
    void reset();
    void setup();

    // drawing
    void draw(bool debug);
};
