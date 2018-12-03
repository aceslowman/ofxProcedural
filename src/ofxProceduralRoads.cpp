#include "ofxProceduralRoads.h"
#include "ofxProceduralCity.h"

#include <algorithm>    // std::shuffle
#include <array>        // std::array
#include <random>       // std::default_random_engine
#include <chrono>       // std::chrono::system_clock


void Road::addSibling(shared_ptr<Road> r){
    this->siblings.push_back(r);
}

void Road::removeSibling(shared_ptr<Road> r){
    this->siblings.erase(std::remove(this->siblings.begin(), this->siblings.end(), r), this->siblings.end());
}

//-----------------------------------------------------------------------------
//  Setup
//-----------------------------------------------------------------------------

void ofxProceduralRoads::reset(){
    pending_list.clear();
    placed_list.clear();
    crossing_list.clear();
    duplicate_list.clear();
    
    mesh.clear();
    crossingMesh.clear();
    duplicateMesh.clear();
    
    setup();
}

void ofxProceduralRoads::setup(){
    node.setParent(city->node);
    
    city->global_walk = 50;
    road_limit = 100;
//    road_scalar = city->dimensions.x/10.0f;
    
    generate();
    
    mesh.setMode(OF_PRIMITIVE_POINTS);
    crossingMesh.setMode(OF_PRIMITIVE_POINTS);
    duplicateMesh.setMode(OF_PRIMITIVE_POINTS);
    
    for(auto &r : placed_list){
        mesh.addVertex(r->node);
    }
    
    for(auto &i : crossing_list){
        crossingMesh.addVertex(i);
    }
    
    for(auto &i : duplicate_list){
        duplicateMesh.addVertex(i);
    }
}

void ofxProceduralRoads::generate(){
    ofVec3f map_center = ofVec3f(city->dimensions.x/2,city->dimensions.x/2,0);
    
    shared_ptr<Road> initial_road = make_shared<Road>(0, nullptr, map_center);
    
    pending_list.push_back(initial_road);
    placed_list.empty();
    
    while(pending_list.size() > 0){
        std::sort (pending_list.begin(), pending_list.end(), sortByDelay);
        shared_ptr<Road> a = pending_list.front();
        
        bool accepted = localConstraints(a);
        
        if(accepted){
            if(a->prev != nullptr){
                a->prev->addSibling(a);
                a->addSibling(a->prev); // all nodes should be siblings (there can be an implied prev for preventing duplication)
            }
            
//            a->node.z = city->population_map.sample((ofVec2f)a->node);
            // need to apply elevation using city->terrain!
            
            placed_list.push_back(a);
            pending_list.erase(pending_list.begin());
            
            int mode = 1;
            
            for(auto i : globalGoals(a, mode)){
                pending_list.push_back(i);
            }
        }else{
            pending_list.erase(pending_list.begin()); //properly removed
        }
    }
}

//-----------------------------------------------------------------------------
//  Local Constraints
//-----------------------------------------------------------------------------

bool ofxProceduralRoads::localConstraints(shared_ptr<Road> a){
    if(a->prev == nullptr){ return true; }
    
    bool crossings = checkForCrossings(a);
    bool dedupe = checkForDuplicates(a);
//    bool dedupe = true;
    
    return dedupe;
}

bool ofxProceduralRoads::checkForCrossings(shared_ptr<Road> a){
    vector<Crossing> crossings;

    for(auto b : placed_list){
        if (b->prev == nullptr || a->prev->node == b->prev->node || a->prev->node == b->node) { continue; }
        
        ofVec2f crossing;
        
        shared_ptr<Road> c = b->prev;
        
        bool intersect = proc_utils::getLineIntersection(a->prev->node, a->node, b->node, c->node, crossing);
        
        if(intersect){
            Crossing match(a, b, c, (ofVec3f)crossing);
            crossings.push_back(match);
        }
    }
    
    if(crossings.size() > 0){
        std::sort(crossings.begin(), crossings.end(), [&](Crossing A, Crossing B){
            return (a->prev->node.distance(A.location) < a->prev->node.distance(B.location));
        });
        
        Crossing match = crossings.front();
        crossing_list.push_back(match.location);

        a->node = match.location;
        match.c->removeSibling(match.b);
        a->addSibling(match.c);
        a->addSibling(match.b);
        match.b->addSibling(a);
        match.c->addSibling(a);

        return true;
    }
    
    return false;
}

bool ofxProceduralRoads::checkForDuplicates(shared_ptr<Road> a){ // prob here
    bool ok = true;

    for(auto b : placed_list){
        if(a->node.distance(b->node) == 0){
            a->prev->addSibling(b);
            
            for(auto sib : a->siblings){
                b->addSibling(sib);
            }
            
            duplicate_list.push_back(b->node);
            
            ok = false;
            
            break;
        }
    }
    
    return ok;
}

//-----------------------------------------------------------------------------
//  Global Goals
//-----------------------------------------------------------------------------

vector<shared_ptr<Road>> ofxProceduralRoads::globalGoals(shared_ptr<Road> a, int mode){
    switch(mode){
        case 0:
            return angleGoal(a, 0, 90);
        default:
            return populationGoal(a, 60, 3);
    }
}

vector<shared_ptr<Road>> ofxProceduralRoads::angleGoal(shared_ptr<Road> a, float range, float tendency){
    vector<shared_ptr<Road>> t_priority;
    
    int max_goals = 2;
    
    (placed_list.size() < road_limit) ? max_goals = 2 : max_goals = 0;
    
    std::array<int,3> quadrants {1,2,3}; // i dont think it should be 4
    
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
            
            if(city->globalBoundsCheck(end)){
                shared_ptr<Road> new_road = make_shared<Road>(a->time_delay + 1, a, end);
                t_priority.push_back(new_road);
            }
            
        }else{ // define default point
            new_direction = ofVec3f(ofRandom(-1,1),ofRandom(-1,1),0).normalize();
            
            ofVec3f end = a->node + (new_direction * road_scalar);
            
            if(city->globalBoundsCheck(end)){
                shared_ptr<Road> new_road = make_shared<Road>(a->time_delay + 1, a, end);
                t_priority.push_back(new_road);
            }
            
            break;
        }
    }
    
    return t_priority;
}

vector<shared_ptr<Road>> ofxProceduralRoads::populationGoal(shared_ptr<Road> a, float range, int numRays){
    vector<shared_ptr<Road>> t_priority;
    
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
            
            if(city->globalBoundsCheck(end)){
                shared_ptr<Road> new_road = make_shared<Road>(a->time_delay + 1, a, end);
                t_priority.push_back(new_road);
            }
        }else{
            ofVec3f new_direction = ofVec3f(ofRandom(-1,1),ofRandom(-1,1),0);
            ofVec3f end = a->node + (new_direction * road_scalar);
            
            if(city->globalBoundsCheck(end)){
                shared_ptr<Road> new_road = make_shared<Road>(a->time_delay + 1, a, end);
                t_priority.push_back(new_road);
            }
        }
    }
    
    return t_priority;
}

//-----------------------------------------------------------------------------
//  Drawing
//-----------------------------------------------------------------------------

void ofxProceduralRoads::draw(){
    ofDisableDepthTest();
    glLineWidth(2);
    ofSetColor(ofColor(255,255,255));

    for(auto r : placed_list){
        if(r->time_delay < city->global_walk){
            for(auto sib : r->siblings){
                ofDrawLine(r->node, sib->node);
            }
        }
    }
    ofEnableDepthTest();
}

void ofxProceduralRoads::drawDebug(ofEasyCam* cam, ofVec3f mouse, bool numbers){
    glPointSize(5);
    cam->begin();
    
    ofSetColor(ofColor(255,255,255));
    
    int it = 0;
    for(auto r : placed_list){
        float nearestDistance = 0;
        ofVec2f nearestVertex;
        ofVec3f cur = cam->worldToScreen(r->node);
        float distance = cur.distance(mouse);
        if(r->prev != nullptr || distance < nearestDistance){
            nearestDistance = distance;
            nearestVertex = cur;
            
            int selection_rng = 4;
            if(nearestDistance < selection_rng){
                for(auto sib : r->siblings){
                    ofSetColor(ofColor(255,165,0,180)); // ORANGE
                    ofSetLineWidth(8);
                    ofDrawArrow(r->node, sib->node);
                }
                if(r->prev != nullptr){
                    ofSetColor(ofColor(148,0,211,180)); // PURPLE
                    ofSetLineWidth(3);
                    ofDrawArrow(r->node, r->prev->node);
                }
                ofSetLineWidth(2);
                ofSetColor(ofColor(255,255,255));
            }
        }
        
        if(numbers){
            ofDrawBitmapString(ofToString(it), r->node);
        }
        
        it++;
    }
    

    glPointSize(10);
    ofSetColor(ofColor(0,0,255, 100));
    duplicateMesh.draw();
//    for(auto d : duplicate_list){
//        ofDrawCircle(d, 5);
//    }
    glPointSize(6);
    ofSetColor(ofColor(0,255,0));
    mesh.draw();
    glPointSize(3);
    ofSetColor(ofColor(255,0,0));
    crossingMesh.draw();
    ofSetColor(ofColor(255,255,255));
    
    ofSetColor(ofColor(255,255,255));
    
    cam->end();
    
//    ofSetColor(ofColor(0));
//    ofDrawRectangle(0,0,200,30);
    ofSetColor(ofColor(255));
//    ofDrawBitmapString("Total Nodes: " + ofToString(placed_list.size()), 10, 10);
//    ofDrawBitmapString("Global Walk: " + ofToString(city->global_walk), 10, 25);
}

//-----------------------------------------------------------------------------
//  Utility
//-----------------------------------------------------------------------------

bool ofxProceduralRoads::sortByDelay(shared_ptr<Road> A, shared_ptr<Road> B){
    return (A->time_delay < B->time_delay);
}
