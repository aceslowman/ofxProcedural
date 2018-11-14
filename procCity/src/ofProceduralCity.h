#pragma once

#include "ofMain.h"

struct Road {
    shared_ptr<Road> prev;
    ofVec2f node;
    
    float time_delay;

    vector<shared_ptr<Road>> siblings;
    
    Road(float _time_delay, shared_ptr<Road> _prev, ofVec2f _node);
};

struct Building {};

class ofProceduralCity {
    
private:
    vector<shared_ptr<Road>> pending_list;
    vector<shared_ptr<Road>> placed_list;
    
    vector<ofVec2f> crossing_list;
    vector<ofVec2f> building_list;
    
    ofImage pop_map;

    int road_limit;
    float road_scalar;

    ofMesh mesh, roadMesh;
    ofMesh crossingMesh;
    ofMesh buildingMesh;

    void setupDebug();
    
    // generation
    void generateRoads();
    void generatePopulationMap();
    void divideIntoLots();
    
    // constraint
    bool localConstraints(shared_ptr<Road> a);
    vector<shared_ptr<Road>> globalGoals(shared_ptr<Road> a);
    bool checkForCrossings(shared_ptr<Road> a);
    bool checkForNearby(shared_ptr<Road> a);
    
    // checks
    bool globalBoundsCheck(ofVec2f &a);
    
    // utility
    int samplePopulation(ofVec2f s);
    bool getLineIntersection(ofVec2f p0, ofVec2f p1, ofVec2f p2, ofVec2f p3, ofVec2f &intersection);
    float getDistanceBetweenPointandLine( ofVec2f a, ofVec2f b, ofVec2f p );
    static bool sortByDelay(shared_ptr<Road> A, shared_ptr<Road> B);
    static bool sortByDistance(ofVec2f A, ofVec2f B, ofVec2f pt);
    
    // city rules
    bool constrainToRightAngles(Road &prev, ofVec2f &end);
    bool constrainToPopulation(Road &prev, ofVec2f &end);
    
public:
    int map_size;
    int global_walk;
    
    // setup
    void reset();
    void setup();

    // drawing
    void draw(bool debug);
};
