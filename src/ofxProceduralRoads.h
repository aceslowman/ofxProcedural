#pragma once
#include "ofMain.h"

struct Road {
    vector<shared_ptr<Road>> siblings;
    shared_ptr<Road> prev;
    
    ofVec3f node;
    
    float time_delay;
    
    ofPolyline line; // for debug
    
    Road(float _time_delay, shared_ptr<Road> _prev, ofVec3f _node): time_delay(_time_delay), prev(_prev), node(_node){};
};

class ofxProceduralCity; // forward declaration

class ofxProceduralRoads{
private:
    ofxProceduralCity *city;
    // local constraints
    bool localConstraints(shared_ptr<Road> a);
    bool checkForCrossings(shared_ptr<Road> a, float tolerance);
    bool checkForNearby(shared_ptr<Road> a, float tolerance);
    
    // global goals
    void globalGoals(shared_ptr<Road> a, vector<shared_ptr<Road>> &t_priority);
    bool rightAngleGoal(shared_ptr<Road> prev, ofVec3f &end, float range, float tendency);
    bool populationGoal(shared_ptr<Road> prev, ofVec3f &end, float range, int numRays, int numSample);
    
    vector<shared_ptr<Road>> pending_list; //use lists?
    vector<shared_ptr<Road>> placed_list; //use lists?
    
    vector<ofVec3f> crossing_list;
    
    ofMesh mesh;
    ofMesh crossingMesh;
    
public:
    void reset();
    void setup();
    void generate();
    void draw(bool debug);
    
    // utility
    static bool sortByDelay(shared_ptr<Road> A, shared_ptr<Road> B);
    
    int road_limit;
    float road_scalar;
    
    ofxProceduralRoads(ofxProceduralCity *_city): city(_city){};
};
