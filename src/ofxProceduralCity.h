#pragma once

#include "ofMain.h"

struct Road {
    shared_ptr<Road> prev;
    ofVec3f node;
    
    float time_delay;
    
    ofPolyline line;

    vector<shared_ptr<Road>> siblings;
    
    Road(float _time_delay, shared_ptr<Road> _prev, ofVec3f _node);
};

struct Building {
    ofPath path;
};

class ofxProceduralCity {
    
private:
    vector<shared_ptr<Road>> pending_list; //use lists?
    vector<shared_ptr<Road>> placed_list; //use lists?
    
    vector<ofVec3f> crossing_list;
    vector<Building> buildings;
    
    ofImage population_map;
    ofImage elevation_map;

    int road_limit;
    float road_scalar;

    ofMesh mesh;
    ofMesh crossingMesh;
    ofMesh buildingMesh;

    void setupDebug();
    
    // generation
    void generateRoads();
    void generateMaps();
    void divideIntoLots();
    
    // constraint
    bool localConstraints(shared_ptr<Road> a);
    vector<shared_ptr<Road>> globalGoals(shared_ptr<Road> a);
    bool checkForCrossings(shared_ptr<Road> a);
    bool checkForNearby(shared_ptr<Road> a);
    
    // checks
    bool globalBoundsCheck(ofVec3f &a);
    
    // utility
    int sampleMap(ofVec2f s, ofImage &img);
    bool getLineIntersection(ofVec2f p0, ofVec2f p1, ofVec2f p2, ofVec2f p3, ofVec2f &intersection);
    float getDistanceBetweenPointandLine( ofVec2f a, ofVec2f b, ofVec2f p );
    static bool sortByDelay(shared_ptr<Road> A, shared_ptr<Road> B);
    static bool sortByDistance(ofVec2f A, ofVec2f B, ofVec2f pt);
    
    // city rules
    bool constrainToRightAngles(shared_ptr<Road> prev, ofVec3f &end);
    bool constrainToPopulation(shared_ptr<Road> prev, ofVec3f &end);
    
public:
    int map_size;
    int global_walk;
    
    // setup
    void reset();
    void setup();

    // drawing
    void draw(bool debug);
    void drawPopMap();
    void drawElevationMap();
};
