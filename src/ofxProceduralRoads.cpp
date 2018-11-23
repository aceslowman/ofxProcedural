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
    city->global_walk = 50;
    road_limit = 1000;
    road_scalar = city->map_size/15.0f;
    
    generate();
    
    mesh.setMode(OF_PRIMITIVE_POINTS);
    crossingMesh.setMode(OF_PRIMITIVE_POINTS);
    
    for(auto &r : placed_list){
        mesh.addVertex(r->node);
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
        shared_ptr<Road> a = pending_list.front();
        
        bool accepted = true;
        if(a->prev != nullptr){
            accepted = localConstraints(a);
        }
        
        if(accepted){
            if(a->prev != nullptr){
                a->prev->siblings.push_back(a);
            }
            
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
    bool crossings = checkForCrossings(a, 10);
    
    if(!crossings){
        return checkForDuplicates(a, 10);
    }
    
    return crossings;
}

bool ofxProceduralRoads::checkForCrossings(shared_ptr<Road> a, float tolerance){
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
            return (a->node.distance(A.location) > a->node.distance(B.location));
        });
        
        Crossing match = crossings.front();
        a->node = match.location;
        
        bool dupes = checkForDuplicates(a, tolerance);

        if(dupes){
            a->siblings.push_back(match.b);
            a->siblings.push_back(match.c);
            match.b->siblings.push_back(a); //!!!
            match.c->siblings.push_back(a); //!!!
            match.b->siblings.erase(std::remove(match.b->siblings.begin(), match.b->siblings.end(), match.c), match.b->siblings.end()); // b should not have the sibling of c
            match.c->siblings.erase(std::remove(match.c->siblings.begin(), match.c->siblings.end(), match.b), match.c->siblings.end()); // c should not have the sibling of b
            
            crossing_list.push_back(match.location);
            
            return true;
        }else{
            return false;
        }
    }
    
    return false;
}

bool ofxProceduralRoads::checkForDuplicates(shared_ptr<Road> a, float tolerance){
    /* Problem between this and checkCrossings. For some reason, some interesections are not properly recognized. I suspect sibling relationships...*/
    bool close_to_node = false;
    
    for(auto b : placed_list){
        if(a->node.distance(b->node) <= tolerance){
            close_to_node = true;
            
            a->node = b->node;
            
            b->siblings.push_back(a->prev);
        }
    }
    
    return !close_to_node;
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

void ofxProceduralRoads::draw(){
    glPointSize(1);
    ofSetColor(ofColor(255,255,255));

    for(auto r : placed_list){
        if(r->time_delay < city->global_walk){
            for(auto sib : r->siblings){
                ofDrawLine(r->node, sib->node);
            }
        }
    }
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
    

    glPointSize(7);
    ofSetColor(ofColor(0,255,0));
    mesh.draw();
    glPointSize(5);
    ofSetColor(ofColor(255,0,0));
    crossingMesh.draw();
    ofSetColor(ofColor(255,255,255));
    
    cam->end();
    
    ofSetColor(ofColor(0));
    ofDrawRectangle(0,0,200,30);
    ofSetColor(ofColor(255));
    ofDrawBitmapString("Total Nodes: " + ofToString(placed_list.size()), 10, 10);
    ofDrawBitmapString("Global Walk: " + ofToString(city->global_walk), 10, 25);
}

//-----------------------------------------------------------------------------
//  Utility
//-----------------------------------------------------------------------------

bool ofxProceduralRoads::sortByDelay(shared_ptr<Road> A, shared_ptr<Road> B){
    return (A->time_delay < B->time_delay);
}
