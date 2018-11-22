#include "ofxProceduralBuildings.h"
#include "ofxProceduralRoads.h"
#include "ofxProceduralCity.h"

/*

 I think my primary issue comes down to improper sibling relationships on the roads. Prev should be a sibling,
 and I should be sure that whatever points are dropped at crossings do not duplicate and create improper
 relationships.
 
 The logic here is looking OK, I'm sure there is plenty more to fix, but I can't help but think there is a
 general incoherence of the Road system.
 
*/

//-----------------------------------------------------------------------------

Building::Building(){}
Block::Block(){}
Lot::Lot(){}

//-----------------------------------------------------------------------------

void ofxProceduralBuildings::setup(ofxProceduralRoads *_roads){
    roads = _roads;
    
    generate();
}

void ofxProceduralBuildings::generate(){
    //first, I need to take in the roads, and begin creating BLOCKS from the network
    
    for(auto r : roads->placed_list){
        Block block;
        block.mesh.setMode(OF_PRIMITIVE_POINTS);
        
        for(auto sib : r->siblings){ // generate a new block for each sibling
            block.mesh.addVertex(r->node);
            
            generateBlocks(block, r, sib, 0);
            
            blocks.push_back(block);
            
            break;
        }
        
        break;
    }
}

void ofxProceduralBuildings::generateBlocks(Block &block, shared_ptr<Road> root, shared_ptr<Road> current, int test_count){
    block.mesh.addVertex(current->node);
    
    float left = 0;
    shared_ptr<Road> next = current;
    for(auto sib : current->siblings){ // goal is only to find the next node (furthest left)
        ofVec3f a = current->prev->node;
        ofVec3f b = current->node;
        ofVec3f p = sib->node;

        float t_left = (p.x - a.x)*(b.y - a.y) - (p.y - a.y)*(b.x - a.x);

        if(t_left >= left){
            ofLog(OF_LOG_NOTICE, "Found one further left.");
            left = t_left;
            next = sib;
        }
    }
    
    test_count++;

    // now that we are done with the left check
    if(next != root && test_count < 5){
        generateBlocks(block, root, next, test_count);
    }
}



//-----------------------------------------------------------------------------

void ofxProceduralBuildings::draw(){
    
    for(auto b : blocks){
        b.mesh.draw();
    }
}
