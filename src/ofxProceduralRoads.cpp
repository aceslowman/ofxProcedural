#include "ofxProceduralRoads.h"
#include "ofxProceduralCity.h"

#include <algorithm>    // std::shuffle
#include <array>        // std::array
#include <random>       // std::default_random_engine
#include <chrono>       // std::chrono::system_clock

//-----------------------------------------------------------------------------
//  Setup
//-----------------------------------------------------------------------------

void ofxProceduralRoads::reset(){
    pending_list.clear();
    placed_list.clear();
    crossing_list.clear();
    
    mesh.clear();
    crossingMesh.clear();
    
    setup();
}

void ofxProceduralRoads::setup(){
    road_limit = 10000;
    road_scalar = city->map_size/30.0f;
    
    generate();
    
    mesh.setMode(OF_PRIMITIVE_POINTS);
    crossingMesh.setMode(OF_PRIMITIVE_POINTS);
    
    for(auto &r : placed_list){
        mesh.addVertex(r->node);
        
        if(r->prev != nullptr){
            r->line.addVertex(r->prev->node);
            r->line.addVertex(r->node);
        }
    }
    
    for(auto &i : crossing_list){
        crossingMesh.addVertex(i);
    }
}

void ofxProceduralRoads::generate(){
    ofVec3f map_center = ofVec3f(city->map_size/2,city->map_size/2,0);
    
    shared_ptr<Road> initial_road = make_shared<Road>(0, nullptr, map_center);
    
    pending_list.push_back(initial_road);
    placed_list.empty();
    
    while(pending_list.size() > 0){
        std::sort (pending_list.begin(), pending_list.end(), sortByDelay);
        shared_ptr<Road> r = pending_list.front();
        
        bool accepted = true;
        if(r->prev != nullptr){
            accepted = localConstraints(r);
        }
        
        if(accepted){
            if(r->prev != nullptr){
                r->prev->siblings.push_back(r);
            }
            
            placed_list.push_back(r);
            pending_list.erase(pending_list.begin());
            
            int mode = 0;
            
            for(auto i : globalGoals(r, mode)){
                pending_list.push_back(i);
            }
        }else{
            ofLog(OF_LOG_NOTICE, "Road failed local constraints! Removing from pending.");
            pending_list.erase(pending_list.begin());
        }
    }
}

//-----------------------------------------------------------------------------
//  Local Constraints
//-----------------------------------------------------------------------------

bool ofxProceduralRoads::localConstraints(shared_ptr<Road> a){
    bool crossings = checkForCrossings(a, 10);
    bool nearby = checkForNearby(a, 10);
    
    if(crossings && nearby){
        //calculate final elevation
        //maybe this should happen somewhere else.
        //        a->node.z = sampleMap((ofVec2f)a->node, elevation_map);
        return true;
    }else{
        return false;
    }
}

bool ofxProceduralRoads::checkForCrossings(shared_ptr<Road> a, float tolerance){
    vector<ofVec3f> crossings;
    for(auto b : placed_list){
        if (b->prev == nullptr) { continue; }
        if (a->prev->node == b->prev->node || a->node == b->node || a->prev->node == b->node || a->node == b->prev->node) { continue; } //can probably do some pointer comparison here
        
        ofVec2f crossing;
        
        bool intersect = proc_utils::getLineIntersection(a->prev->node, a->node, b->prev->node, b->node, crossing);
        
        if(intersect){
            crossings.push_back(crossing);
        }
    }
    
    bool intersects = crossings.size() > 0;
    if(intersects){
        std::sort(crossings.begin(), crossings.end(), std::bind(proc_utils::sortByDistance, std::placeholders::_1, std::placeholders::_2, a->prev->node));
        
        a->node = crossings.front();
        
        crossing_list.push_back(a->node);
    }
    
    return true;
}

bool ofxProceduralRoads::checkForNearby(shared_ptr<Road> a, float tolerance){
    /*
     This still needs a check for nearby, but uncomplete connections. not only do I have
     to do a crossing check, but I need to merge with nearby roads if within range
     */
    
    bool snapped = false;
    
    for(auto b : placed_list){
        if(b->prev == nullptr){ continue; }
        bool close_to_start = a->node.distance(b->prev->node) < tolerance;
        bool close_to_end = a->node.distance(b->node) < tolerance;
        
        if(close_to_start && close_to_end){
            ofLog(OF_LOG_NOTICE, "discarding by proximity to crossing");
            return false;
        }else if(close_to_start){
            a->node = b->prev->node;
            snapped = true;
        }else if(close_to_end){
            a->node = b->node;
            snapped = true;
        }
    }
    
    if(snapped){
        bool cross = checkForCrossings(a, tolerance);
        
        if(!cross){
            ofLog(OF_LOG_NOTICE, "checkForCrossings failed");
            return false;
        }
    }
    
    return true;
}

//-----------------------------------------------------------------------------
//  Global Goals
//-----------------------------------------------------------------------------

vector<shared_ptr<Road>> ofxProceduralRoads::globalGoals(shared_ptr<Road> a, int mode){
    /*
        There are ways that I can move the stuff from this for loop below, down into the *Goal methods.
        Essentially they would be more responsible for building *all*
     */
    
    switch(mode){
        case 0:
            return rightAngleGoal(a);
        case 1:
            return populationGoal(a);
            
            // can probably check for nullptr here, and have a default strategy for the first two nodes.
    }
}

vector<shared_ptr<Road>> ofxProceduralRoads::rightAngleGoal(shared_ptr<Road> a){
    vector<shared_ptr<Road>> t_priority;
    
    int max_goals = 2;
    float range = 0;
    float tendency = 90;
    
    (placed_list.size() < road_limit) ? max_goals = 2 : max_goals = 0;
    
    std::array<int,4> quadrants {1,2,3,4};
    
    unsigned seed = std::chrono::system_clock::now().time_since_epoch().count();
    std::shuffle(quadrants.begin(), quadrants.end(), std::default_random_engine(seed));

    for(int i = 0; i < max_goals; i++){
        ofVec3f new_direction;
        
        if(a->prev != nullptr){
            ofVec3f prev_direction = ofVec3f(a->node - a->prev->node).normalize();
            new_direction = prev_direction;
            
            float angle = ofRandom((tendency * quadrants[i]) - range,(tendency * quadrants[i]) + range);
            new_direction.rotate(angle, ofVec3f(0,0,1));
            
            ofVec3f end = a->node + (new_direction * road_scalar);
            
            shared_ptr<Road> new_road = make_shared<Road>(a->time_delay + 1, a, end);
            t_priority.push_back(new_road);
        }else{ // define default point
            new_direction = ofVec3f(ofRandom(-1,1),ofRandom(-1,1),0).normalize();
            
            ofVec3f end = a->node + (new_direction * road_scalar);
            
            shared_ptr<Road> new_road = make_shared<Road>(a->time_delay + 1, a, end);
            t_priority.push_back(new_road);
            
            break;
        }
    }
    
    return t_priority;
}

vector<shared_ptr<Road>> ofxProceduralRoads::populationGoal(shared_ptr<Road> a){
    vector<shared_ptr<Road>> t_priority;
    
    float range = 30;
    int numRays = 3;
    int numSample = 3;
    int max_goals = 2;
    
    (placed_list.size() < road_limit) ? max_goals = 2 : max_goals = 0;
    
    for(int i = 0; i < max_goals; i++){
        if(a->prev != nullptr){
            ofPolyline t_ray;
            float t_sum = 0;
            
            for(int i = 0; i < numRays; i++){
                ofPolyline ray;
                float sum = 0;
        
                ray.addVertex(a->prev->node);
        
                ofVec3f direction = ofVec3f(a->node - a->prev->node).normalize();
        
                float random_angle = ofRandom(-range,range);
                direction.rotate(random_angle,ofVec3f(0,0,1));
        
                ofVec3f t_end = a->node + (direction * road_scalar);
        
                ray.addVertex(t_end);
        
                for(int j = 0; j < numSample; j++){
                    ofVec3f p = ray.getPointAtIndexInterpolated((1.0f/3)*j);
                    if(city->globalBoundsCheck(p)){
                        sum += city->population_map.sample((ofVec2f)p);
                    }
                }
        
                if(sum > t_sum){
                    t_sum = sum;
                    t_ray = ray;
                }
            }
            
            ofVec3f end = t_ray.getPointAtIndexInterpolated(1);
            
            shared_ptr<Road> new_road = make_shared<Road>(a->time_delay + 1, a, end);
            t_priority.push_back(new_road);
        }else{
            ofVec3f new_direction = ofVec3f(ofRandom(-1,1),ofRandom(-1,1),0);
            ofVec3f end = a->node + (new_direction * road_scalar);
            
            shared_ptr<Road> new_road = make_shared<Road>(a->time_delay + 1, a, end);
            t_priority.push_back(new_road);
        }
    }
    
    return t_priority;
}

//-----------------------------------------------------------------------------
//  Drawing
//-----------------------------------------------------------------------------

void ofxProceduralRoads::draw(bool debug){
    ofSetColor(ofColor(255,255,255));
    for(auto r : placed_list){
        if(r->time_delay < city->global_walk){
            r->line.draw();
        }
        
        int selection_rng = 4;
        if((ofGetMouseX() < (r->node.x + selection_rng)) && (ofGetMouseX() > (r->node.x - selection_rng))
           && (ofGetMouseY() < (r->node.y + selection_rng)) && (ofGetMouseY() > (r->node.y - selection_rng))){
            for(auto sib : r->siblings){
                ofSetColor(ofColor(255,165,0)); // ORANGE
                ofSetLineWidth(3);
                ofDrawArrow(r->node, sib->node);
            }
            if(r->prev != nullptr){
                ofSetColor(ofColor(148,0,211)); // PURPLE
                ofDrawArrow(r->node, r->prev->node);
            }
            ofSetLineWidth(1);
            ofSetColor(ofColor(255,255,255));
        }
    }
    
    if(debug){
        glPointSize(5);
        ofSetColor(ofColor(0,255,0));
        mesh.draw();
        glPointSize(2);
        ofSetColor(ofColor(255,0,0));
        crossingMesh.draw();
        ofSetColor(ofColor(255,255,255));
        
        ofSetColor(ofColor(0));
        ofDrawRectangle(0,0,200,30);
        ofSetColor(ofColor(255));
        ofDrawBitmapString("Total Nodes: " + ofToString(placed_list.size()), 10, 10);
        ofDrawBitmapString("Global Walk: " + ofToString(city->global_walk), 10, 25);
    }
}

//-----------------------------------------------------------------------------
//  Utility
//-----------------------------------------------------------------------------

bool ofxProceduralRoads::sortByDelay(shared_ptr<Road> A, shared_ptr<Road> B){
    return (A->time_delay < B->time_delay);
}
