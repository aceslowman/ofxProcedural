#pragma once
#include "ofMain.h"

struct Building{
    ofPath path;
    
    Building();
};

struct Lot {
    ofPath path;
    
    Lot();
};

struct Block {
    ofMesh mesh;
    
    Block();
};

class ofxProceduralCity;
class ofxProceduralRoads;
class Road;

class ofxProceduralBuildings{
private:
    ofxProceduralCity *city;
    ofxProceduralRoads *roads;
    
    vector<Building> buildings;
    vector<Block> blocks;
    vector<Lot> lots;
    
public:
    void reset();
    void setup(ofxProceduralRoads *_roads);
    void generate();
    void draw();
    
    void generateBlocks(Block &block, shared_ptr<Road> root, shared_ptr<Road> current, int test_count);
    void generateLots();
    void generateBuildings();
    
    ofxProceduralBuildings(ofxProceduralCity *_city): city(_city){};
};
