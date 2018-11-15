#include "ofxProceduralCity.h"

//-----------------------------------------------------------------------------

Road::Road(float _time_delay, shared_ptr<Road> _prev, ofVec2f _node): time_delay(_time_delay), prev(_prev), node(_node){}

//-----------------------------------------------------------------------------
// SETUP
//-----------------------------------------------------------------------------

void ofxProceduralCity::reset(){
    //clear
    pending_list.clear();
    placed_list.clear();
    crossing_list.clear();
    
    pop_map.clear();
    
    mesh.clear();
    crossingMesh.clear();

    setup();
}

void ofxProceduralCity::setup(){
    road_limit = 100;
    global_walk = 0;
    map_size = 512;
    generatePopulationMap();
    
    ofSetWindowShape(map_size, map_size);
    ofSetWindowPosition((ofGetScreenWidth() / 2.0)-(map_size/2.0), (ofGetScreenHeight() / 2.0)-(map_size/2.0));
    
    road_scalar = map_size/10.0f;
    
    generateRoads();
    divideIntoLots();
    
    setupDebug();
}

void ofxProceduralCity::setupDebug(){
    mesh.setMode(OF_PRIMITIVE_POINTS);
    crossingMesh.setMode(OF_PRIMITIVE_POINTS);
    
    for(auto &r : placed_list){
        mesh.addVertex((ofVec3f)r->node);
        
        if(r->prev != nullptr){
            r->line.addVertex((ofVec3f)r->prev->node);
            r->line.addVertex((ofVec3f)r->node);
        }
    }
    
    for(auto &i : crossing_list){
        crossingMesh.addVertex((ofVec3f)i);
    }
}

//-----------------------------------------------------------------------------
// GENERATION
//-----------------------------------------------------------------------------

void ofxProceduralCity::generateRoads(){
    ofVec2f map_center = ofVec2f(map_size/2,map_size/2);
    
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
                r->prev->siblings.push_back(r); //only add the sibling to the prev if all conditions have been met
            }
            
            placed_list.push_back(r);
            pending_list.erase(pending_list.begin());
            
            for(auto i : globalGoals(r)){
                pending_list.push_back(i);
            }
        }else{
            ofLog(OF_LOG_NOTICE, "Road failed local constraints! Removing from pending.");
            pending_list.erase(pending_list.begin());
        }
    }
}

void ofxProceduralCity::generatePopulationMap(){
    ofPixels t_pop;
    t_pop.allocate(map_size, map_size, OF_IMAGE_GRAYSCALE);
    pop_map.allocate(map_size, map_size, OF_IMAGE_GRAYSCALE);
    
    ofSeedRandom();
    
    float scale = 500;
    float offset = ofRandom(100);
    
    for(int i = 0; i < map_size; i++){
        for(int j = 0; j < map_size; j++){
            t_pop.setColor(i, j, ofColor(ofNoise(i/scale + offset, j/scale + offset)*255));
        }
    }
    
    pop_map.setFromPixels(t_pop);
}

void ofxProceduralCity::divideIntoLots(){
    //for each block
    //treat it's points as one polygon
    //in which you will subdivide it's longest edges
    //until the individual blocks
    //have an area below a certain threshold
}

//-----------------------------------------------------------------------------
// LOCAL CONSTRAINTS
//-----------------------------------------------------------------------------

bool ofxProceduralCity::localConstraints(shared_ptr<Road> a){
    bool crossings = checkForCrossings(a);
    bool nearby = checkForNearby(a);

    if(crossings && nearby){
        return true;
    }else{
        return false;
    }
}

bool ofxProceduralCity::checkForCrossings(shared_ptr<Road> a){
    float tolerance = 10.0f;

    vector<ofVec2f> crossings;
    for(auto b : placed_list){
        if (b->prev == nullptr) { continue; }
        if (a->prev->node == b->prev->node || a->node == b->node || a->prev->node == b->node || a->node == b->prev->node) { continue; } //can probably do some pointer comparison here

        ofVec2f crossing;

        bool intersect = getLineIntersection(a->prev->node, a->node, b->prev->node, b->node, crossing);

        if(intersect){
            crossings.push_back(crossing);
        }
    }

    bool intersects = crossings.size() > 0;
    if(intersects){
        std::sort(crossings.begin(), crossings.end(), std::bind(sortByDistance, std::placeholders::_1, std::placeholders::_2, a->prev->node));

        a->node = crossings.front();

        crossing_list.push_back(a->node);
    }
    
    return true;
}

bool ofxProceduralCity::checkForNearby(shared_ptr<Road> a){
    float tolerance = 10.0f; // parameterize
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

    if(snapped){ // end points have moved, check again for intersections
        bool cross = checkForCrossings(a);

        if(!cross){
            ofLog(OF_LOG_NOTICE, "checkForCrossings failed");
            return false;
        }
    }

    /*
        This still needs a check for nearby, but uncomplete connections. not only do I have
        to do a crossing check, but I need to merge with nearby roads if within range
     */

    return true;
}

//-----------------------------------------------------------------------------
// GLOBAL GOALS
//-----------------------------------------------------------------------------

vector<shared_ptr<Road>> ofxProceduralCity::globalGoals(shared_ptr<Road> a){
    vector<shared_ptr<Road>> t_priority;
    
    int max_goals = 2;
    
//    (placed_list.size() < road_limit) ? max_goals = 2 : max_goals = 0;
    
    for(int i = 0; i < max_goals; i++){
        ofVec2f direction = ofVec2f(ofRandom(-1,1),ofRandom(-1,1));
        ofVec2f next_node = a->node + (direction * road_scalar);
        
        bool accepted = true;
        if(a->prev != nullptr){
            bool pattern_check = constrainToRightAngles(a, next_node);
            bool pop_check = constrainToPopulation(a, next_node);
            bool in_bounds = globalBoundsCheck(next_node);

            accepted = pop_check && in_bounds;
        }
        
        if(accepted){
            shared_ptr<Road> new_road = make_shared<Road>(a->time_delay + 1, a, next_node);
            t_priority.push_back(new_road);
        }
    }
    
    
    return t_priority;
}

bool ofxProceduralCity::constrainToRightAngles(shared_ptr<Road> prev, ofVec2f &end){
    if(prev->prev == nullptr){ return true; }
    /*
        I am wondering if there are efficiency issues tied to the number of points we are dropping without
        checking distance from crossings, but for now that will be a local constraint task.
    */
    
    /* TODO: two roads should not be generated in the same quadrant. */

    ofVec2f prev_direction = ofVec2f(prev->node - prev->prev->node).normalize();
    ofVec2f new_direction = prev_direction;
    
    int quadrant = (int)ofRandom(4);
    float range = 5.0f;
    float tendency = 90.0f; // 45 degree angle results in spiral!
    
    float random_angle = ofRandom((tendency * quadrant) - range,(tendency * quadrant) + range);
    new_direction.rotate(random_angle);
    
    //end is only used if accepted. so maybe I should return to the old use of (prev, end)
    end = prev->node + (new_direction * road_scalar);

    return true;
}

bool ofxProceduralCity::constrainToPopulation(shared_ptr<Road> prev, ofVec2f &end){
    if(prev->prev == nullptr){ return true; }
    
    float range = 30; // paramaterize!!!
    int numRays = 3;
    int numSample = 3;

    ofPolyline t_ray;
    float t_sum = 0;

    for(int i = 0; i < numRays; i++){
        ofPolyline ray;
        float sum = 0;

        ray.addVertex((ofVec3f)prev->prev->node);

        ofVec2f direction = ofVec2f(prev->node - prev->prev->node).normalize();

        float random_angle = ofRandom(-range,range);
        direction.rotate(random_angle);

        ofVec2f t_end = prev->node + (direction * road_scalar);

        ray.addVertex((ofVec3f)t_end);

        // at three points along the ray, sample the population map and add to the sum
        for(int j = 0; j < numSample; j++){
            ofVec2f p = (ofVec2f)ray.getPointAtIndexInterpolated((1.0f/3)*j);
            sum += samplePopulation(p);
        }

        if(sum > t_sum){
            t_sum = sum;
            t_ray = ray;
        }
    }

    end = (ofVec2f)t_ray.getPointAtIndexInterpolated(1);

    return true;
}

//-----------------------------------------------------------------------------
// CHECKS
//-----------------------------------------------------------------------------

bool ofxProceduralCity::globalBoundsCheck(ofVec2f &a){
    if(a.x >= map_size || a.x <= 0 || a.y >= map_size || a.y <= 0){
        ofLog(OF_LOG_ERROR, "OUT OF BOUNDS");
        return false;
    }
    
    return true;
}

//-----------------------------------------------------------------------------
// UTILITY
//-----------------------------------------------------------------------------

int ofxProceduralCity::samplePopulation(ofVec2f s){
    ofPixels &pix = pop_map.getPixels();
    
    return pix.getColor(s.x,s.y).r;
}

bool ofxProceduralCity::getLineIntersection(ofVec2f p0, ofVec2f p1, ofVec2f p2, ofVec2f p3, ofVec2f &intersection){
    /*
        From Andre LeMothe's "Tricks of the Windows Game Programming Gurus"
        via https://stackoverflow.com/questions/563198/how-do-you-detect-where-two-line-segments-intersect
    */
    
    float s1_x, s1_y, s2_x, s2_y;
    s1_x = p1.x - p0.x;
    s1_y = p1.y - p0.y;
    s2_x = p3.x - p2.x;
    s2_y = p3.y - p2.y;
    
    float s, t;
    s = (-s1_y * (p0.x - p2.x) + s1_x * (p0.y - p2.y)) / (-s2_x * s1_y + s1_x * s2_y);
    t = ( s2_x * (p0.y - p2.y) - s2_y * (p0.x - p2.x)) / (-s2_x * s1_y + s1_x * s2_y);
    
    if (s >= 0 && s <= 1 && t >= 0 && t <= 1){
        intersection.x = p0.x + (t * s1_x);
        intersection.y = p0.y + (t * s1_y);
        return true;
    }
    
    return false;
}

float ofxProceduralCity::getDistanceBetweenPointandLine( ofVec2f a, ofVec2f b, ofVec2f p ){
    ofVec2f n = b - a;
    ofVec2f pa = a - p;
    ofVec2f c = n * (pa.dot(n) / n.dot(n));
    ofVec2f d = pa - c;
    return sqrt( d.dot(d) );
}

bool ofxProceduralCity::sortByDelay(shared_ptr<Road> A, shared_ptr<Road> B){
    return (A->time_delay < B->time_delay);
}

bool ofxProceduralCity::sortByDistance(ofVec2f A, ofVec2f B, ofVec2f pt){
    return (pt.distance(A) < pt.distance(B));
}

//-----------------------------------------------------------------------------
// DRAWING
//-----------------------------------------------------------------------------

void ofxProceduralCity::draw(bool debug){
    if(debug){
        pop_map.draw(0,0,map_size,map_size);
    }
    
    ofSetColor(ofColor(255,255,255));
    for(auto r : placed_list){
        if(r->time_delay < global_walk){
            r->line.draw();
        }
        
        int selection_rng = 4;
        if((r->prev != nullptr) && (ofGetMouseX() < (r->node.x + selection_rng)) && (ofGetMouseX() > (r->node.x - selection_rng))
           && (ofGetMouseY() < (r->node.y + selection_rng)) && (ofGetMouseY() > (r->node.y - selection_rng))){
            for(auto sib : r->siblings){
                ofSetColor(ofColor(255,165,0)); // ORANGE
                ofSetLineWidth(3);
                ofDrawArrow((ofVec3f)r->node, (ofVec3f)sib->node);
            }
            ofSetColor(ofColor(148,0,211)); // PURPLE
            ofDrawArrow((ofVec3f)r->node, (ofVec3f)r->prev->node);
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
        ofSetColor(ofColor(0,0,255));
        buildingMesh.draw();
        ofSetColor(ofColor(255,255,255));
        
        ofDrawBitmapString("Total Nodes: " + ofToString(placed_list.size()), 10, 10);
        ofDrawBitmapString("Global Walk: " + ofToString(global_walk), 10, 25);
    }
}
