#pragma once
#include "ofMain.h"
#include "ofxGui.h"

struct Road {
    vector<shared_ptr<Road>> siblings;
    shared_ptr<Road> prev;
    
    ofVec3f node;
    
    float time_delay;
    
    void addSibling(shared_ptr<Road> r);
    void removeSibling(shared_ptr<Road> r);
    
    Road(float _time_delay, shared_ptr<Road> _prev, ofVec3f _node): time_delay(_time_delay), prev(_prev), node(_node){};
    
//    ~Road();
};

struct Crossing {
    shared_ptr<Road> a; //impending road
    shared_ptr<Road> b; //right road
    shared_ptr<Road> c; //left road
    
    ofVec2f location;
    
    Crossing(shared_ptr<Road> _a,shared_ptr<Road> _b, shared_ptr<Road> _c, ofVec2f _location): a(_a), b(_b), c(_c), location(_location){};
};

class ofxProceduralCity; // forward declaration

class ofxProceduralRoads{
private:
    ofxProceduralCity *city;

    bool localConstraints(shared_ptr<Road> a);
    bool checkForCrossings(shared_ptr<Road> a);
    bool checkForDuplicates(shared_ptr<Road> a);
    
    vector<shared_ptr<Road>> globalGoals(shared_ptr<Road> a, int mode);
    vector<shared_ptr<Road>> angleGoal(shared_ptr<Road> a, float range, float tendency);
    vector<shared_ptr<Road>> populationGoal(shared_ptr<Road> a, float range, int numRays);
    
    vector<shared_ptr<Road>> pending_list; //use lists?
    
    vector<ofVec3f> crossing_list, duplicate_list;
    
    ofMesh mesh;
    ofMesh crossingMesh, duplicateMesh;
    
public:
    void reset();
    void setup();
    void generate();
    void draw();
    void drawDebug(ofEasyCam* cam, ofVec3f mouse, bool numbers);
    
    static bool sortByDelay(shared_ptr<Road> A, shared_ptr<Road> B);
    
    vector<shared_ptr<Road>> placed_list; //use lists?
    
    int road_limit;
//    float road_scalar;
    
    ofNode node;
    
    ofParameterGroup params;
    ofParameter<float> road_scalar;
    ofParameter<bool> popGoal;
    ofParameter<bool> rightGoal;
    ofParameter<float> mergeRadius;
    ofParameter<bool> show_dots;
    
    ofxProceduralRoads(ofxProceduralCity *_city): city(_city){
        params.setName("Roads");
        params.add(road_scalar.set("Road Scalar", 50, 1, 100));
        params.add(popGoal.set("Population", true));
        params.add(rightGoal.set("Right Angle", false));
        params.add(mergeRadius.set("Merge Radius", 0.0, 0.0, 50));
        params.add(show_dots.set("Show Dots", true));
    };
};
