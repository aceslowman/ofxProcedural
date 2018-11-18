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
    vector<shared_ptr<Road>> globalGoals(shared_ptr<Road> a, int mode);
    vector<shared_ptr<Road>> rightAngleGoal(shared_ptr<Road> a);
    vector<shared_ptr<Road>> populationGoal(shared_ptr<Road> a);
    
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
