#include "ofProceduralCity.h"

//-----------------------------------------------------------------------------

RoadSegment::RoadSegment(float _time_delay, ofVec2f _start, ofVec2f _end){
    time_delay = _time_delay;
    start      = _start;
    end        = _end;
}

float RoadSegment::getSlope(){
    float slope = (this->end.y - this->start.y) / (this->end.x - this->start.x);
    ofLog(OF_LOG_NOTICE, "  Slope: " + ofToString(slope));
    
    return slope;
}

float RoadSegment::getRotation(){
    return this->start.angle(end);
}

//-----------------------------------------------------------------------------
// SETUP
//-----------------------------------------------------------------------------

void ofProceduralCity::setup(){
    road_limit = 500;
    global_walk = 0;
    pop_map.load("noise512.png");
    
    // set up maps
    map_size = pop_map.getWidth();
    ofSetWindowShape(map_size, map_size);
    ofSetWindowPosition((ofGetScreenWidth() / 2.0)-(map_size/2.0), (ofGetScreenHeight() / 2.0)-(map_size/2.0));
    
    road_scalar = map_size/4.0f;
    
    generateRoads();
    
    setupDebug();
}

void ofProceduralCity::setupDebug(){
    mesh.setMode(OF_PRIMITIVE_POINTS);
    intersectionMesh.setMode(OF_PRIMITIVE_POINTS);

    
    for(auto &r : placed_list){
        mesh.addVertex((ofVec3f)r.end);
//        mesh.addColor(ofColor(r.population));
        
        r.line.addVertex((ofVec3f)r.start);
        r.line.addVertex((ofVec3f)r.end);
    }
}

//-----------------------------------------------------------------------------
// GENERATION
//-----------------------------------------------------------------------------

void ofProceduralCity::generateRoads(){
    ofVec2f i_start = ofVec2f(map_size/2,map_size/2);
    ofVec2f direction = ofVec2f(ofRandom(-1.0f, 1.0f),ofRandom(-1.0f, 1.0f));
    ofVec2f i_end = i_start + (direction * road_scalar);
    
    if(!globalBoundsCheck(i_end)){
        direction = ofVec2f(ofRandom(-1.0f, 1.0f),ofRandom(-1.0f, 1.0f));
        i_end = i_start + (direction * road_scalar);
    }
    
    RoadSegment initial_road = RoadSegment(0, i_start, i_end);

    pending_list.push_back(initial_road);
    placed_list.empty();
    
    while(pending_list.size() > 0){
        std::sort (pending_list.begin(), pending_list.end(), sortByDelay);
        RoadSegment &r = pending_list.front();
        
        bool accepted = localConstraints(r);
        
        if(accepted){
            placed_list.push_back(r);

            for(auto i : globalGoals(r)){
                pending_list.push_back(i);
            }
        }
        
        pending_list.erase(pending_list.begin());
    }
}

//-----------------------------------------------------------------------------
// CONSTRAINT
//-----------------------------------------------------------------------------

bool ofProceduralCity::localConstraints(RoadSegment &a){
    ofLog(OF_LOG_NOTICE, "checking LOCAL CONSTRAINTS at point: ("+ofToString(a.start)+")("+ofToString(a.end)+")");
    
    constrainToIntersections(a);
//    constrainCloseTo(a);
    
    return true;
}

vector<RoadSegment> ofProceduralCity::globalGoals(RoadSegment &a){
    ofLog(OF_LOG_NOTICE, "checking GLOBAL GOALS at point: ("+ofToString(a.start)+")("+ofToString(a.end)+")");
    
    vector<RoadSegment> t_priority;
    
    int max_goals;
    
    if(placed_list.size() < road_limit){
        max_goals = 2;
    }else{
        max_goals = 0;
    }
    
    for(int i = 0; i < max_goals; i++){
        ofVec2f start = a.end;
        ofVec2f direction = ofVec2f(ofRandom(-1.0f, 1.0f),ofRandom(-1.0f, 1.0f));
        ofVec2f end = start + (direction * road_scalar);

        bool in_bounds = globalBoundsCheck(end);

        if(in_bounds){
            RoadSegment new_road(a.time_delay + 1, start, end);

            bool pattern_check = constrainToCityPattern(a, new_road);

            if(pattern_check){
                new_road.population = samplePopulation(end);
                t_priority.push_back(new_road);
            }else{
                ofLog(OF_LOG_WARNING, "     FAILED PATTERN CHECK");
            }
        }else{
            ofLog(OF_LOG_WARNING, "     Attempting to sample out of bounds!");
        }
    }
    
    return t_priority;
}

bool ofProceduralCity::constrainToIntersections(RoadSegment &a){
    /*
     Checks for intersection between segments, truncating the initial segment
     at the intersection point
     */
    vector<ofVec2f> intersections;
    
    for(auto b : placed_list){ // within this loop, I can do much more than constrain to intersections right?
        if (a.start == b.start || a.end == b.end || a.start == b.end || a.end == b.start) { continue; }
        
        ofVec2f intersection = ofVec2f(0,0);

        bool intersect = getLineIntersection(a.start, a.end, b.start, b.end, intersection);
        if(intersect){
            ofLog(OF_LOG_NOTICE, "      Intersection found at: " + ofToString(intersection));
            intersections.push_back(intersection);
        }
    }
    
    if(intersections.size() > 0){
        std::sort (intersections.begin(), intersections.end(),
                   std::bind(sortByDistance, std::placeholders::_1, std::placeholders::_2, a.start));
        
        a.end = intersections.front();
        a.population = samplePopulation(a.end);
        intersectionMesh.addVertex(ofVec3f(a.end.x,a.end.y,0));
    }
    
    return true;
}

bool ofProceduralCity::constrainToCityPattern(RoadSegment &prev, RoadSegment &next){
    float random_angle = ofRandom(80, 100);

//    next.end = ofVec2f(0,0);
//    next.end.rotate(random_angle,next.start);
    
    return true;
}

//-----------------------------------------------------------------------------
// CHECKS
//-----------------------------------------------------------------------------
bool ofProceduralCity::globalBoundsCheck(ofVec2f &a){
    if(a.x >= map_size || a.x <= 0 || a.y >= map_size || a.y <= 0){
        return false;
    }
    
    return true;
}

//-----------------------------------------------------------------------------
// UTILITY
//-----------------------------------------------------------------------------

int ofProceduralCity::samplePopulation(ofVec2f s){
    ofPixels &pix = pop_map.getPixels();
    ofColor color = ofColor(0);
    
    return pix.getColor(s.x,s.y).r;
}

bool ofProceduralCity::sortByDelay(RoadSegment A, RoadSegment B){
    return (A.time_delay < B.time_delay);
}

bool ofProceduralCity::sortByDistance(ofVec2f A, ofVec2f B, ofVec2f pt){
    return (pt.distance(A) < pt.distance(B));
}

/*
    From Andre LeMothe's "Tricks of the Windows Game Programming Gurus"
    via https://stackoverflow.com/questions/563198/how-do-you-detect-where-two-line-segments-intersect
 */
bool ofProceduralCity::getLineIntersection(ofVec2f p0, ofVec2f p1, ofVec2f p2, ofVec2f p3, ofVec2f &intersection){
    
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

/* https://stackoverflow.com/questions/26829161/the-angle-between-3-points-in-c */
float ofProceduralCity::getRoadAngle(ofVec2f A, ofVec2f B, ofVec2f C){
    float atanA = atan2(A.x - B.x, A.y - B.y);
    float atanC = atan2(C.x - B.x, C.y - B.y);
    float diff = atanC - atanA;

    if (diff > PI) diff -= PI;
    else if (diff < -PI) diff += PI;

    diff *= 180 / PI;

    return diff;
}

//-----------------------------------------------------------------------------
// DRAWING
//-----------------------------------------------------------------------------

void ofProceduralCity::draw(){
    ofSetColor(ofColor(255,255,255));
    for(auto r : placed_list){
        if(r.time_delay < global_walk){
            r.line.draw();
//            ofDrawArrow((ofVec3f)r.start,(ofVec3f)r.end,5.0);
        }
    }

    glPointSize(9);
    ofSetColor(ofColor(0,255,0));
    mesh.draw();
    glPointSize(5);
    ofSetColor(ofColor(255,0,0));
    intersectionMesh.draw();
    ofSetColor(ofColor(255,255,255));
}

void ofProceduralCity::printDebug(){
    ofDrawBitmapString("Priority List: " + ofToString(pending_list.size()), 10, 15);
    ofDrawBitmapString("Segment List: " + ofToString(placed_list.size()), 10, 30);
    ofDrawBitmapString("Total Nodes: " + ofToString(mesh.getVertices().size()), 10, 45);
    ofDrawBitmapString("Global Walk: " + ofToString(global_walk), 10, 75);
}
