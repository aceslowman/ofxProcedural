#include "ofProceduralCity.h"

//-----------------------------------------------------------------------------

RoadSegment::RoadSegment(float _time_delay, ofVec2f _start, ofVec2f _end){
    time_delay = _time_delay;
    start      = _start;
    end        = _end;
    
    debugLine.addVertex(start.x, start.y);
    debugLine.addVertex(end.x, end.y);
}

//-----------------------------------------------------------------------------

void ofProceduralCity::setup(){
    global_counter = 0; //temporary
    pop_map.load("noise512.png");
    
    // set up maps
    map_size = pop_map.getWidth();
    ofSetWindowShape(map_size, map_size);
    ofSetWindowPosition((ofGetScreenWidth() / 2.0)-(map_size/2.0), (ofGetScreenHeight() / 2.0)-(map_size/2.0));
    
    road_scalar = map_size/5.0f;
    
    priority_list.push_back(RoadSegment(0, ofVec2f(map_size/2,map_size/2), ofVec2f(map_size/2,map_size/2)));
    segment_list.empty();
    
    while(priority_list.size() > 0){
        std::sort (priority_list.begin(), priority_list.end(), sortByDelay);
        RoadSegment r = priority_list.front();
        
        bool accepted = localConstraints(r);
        
        if(accepted){
            segment_list.push_back(r);
            
            for(auto _r : globalGoals(r)){
                priority_list.push_back(_r);
                global_counter++;
            }
        }
        
        priority_list.erase(priority_list.begin());
    }
    
    setupDebug();
}

void ofProceduralCity::setupDebug(){
    mesh.setMode(OF_PRIMITIVE_POINTS);
    glPointSize(5);
    
    for(auto r : segment_list){
        mesh.addVertex(ofVec3f(r.end.x,r.end.y,0));
        mesh.addColor(ofColor(r.population));
    }
}

//-----------------------------------------------------------------------------
bool ofProceduralCity::localConstraints(RoadSegment &r){
    /*
     Determine constraints from environment
     
     checks the segment for compatibility with all previously placed segments and may modify its geometry if necessary, for example to join the end of the segment to a nearby junction.
     */
    
    // TODO: DO I PERFORM INTERSECTION CHECKS HERE IN LOCAL? OR SHOULD I IN GLOBAL?
    
    return true;
}

//-----------------------------------------------------------------------------
vector<RoadSegment> ofProceduralCity::globalGoals(RoadSegment &r){
    vector<RoadSegment> t_priority;
    
    int max_goals;
    
    if(global_counter <= 1000){
        max_goals = 2;
    }else{
        max_goals = 0;
    }
    
    for(int i = 0; i < max_goals; i++){
        ofVec2f start = r.end;
        ofVec2f direction = ofVec2f(ofRandom(-1.0f, 1.0f),ofRandom(-1.0f, 1.0f));
        ofVec2f end = start + (direction * road_scalar);

        ofLog(OF_LOG_NOTICE, "-------------------------------");

        bool valid = globalBoundsCheck(end);
        
        if(valid){
            RoadSegment _r(r.time_delay + 1, start, end);
            
            ofPixels &pix = pop_map.getPixels();
            ofColor color = ofColor(0);
            
            _r.population = pix.getColor(end.x,end.y).r;
            
            ofLog(OF_LOG_NOTICE, "start: " + ofToString(start));
            ofLog(OF_LOG_NOTICE, "end: " + ofToString(end));
            ofLog(OF_LOG_NOTICE, "population: " + ofToString(_r.population));
            
            t_priority.push_back(_r);
        }else{
            ofLog(OF_LOG_NOTICE, "Attempting to sample out of bounds!");
        }
    }
    
    return t_priority;
}

bool ofProceduralCity::globalBoundsCheck(ofVec2f &node){
    if(node.x >= map_size || node.x <= 0 || node.y >= map_size || node.y <= 0){
        return false;
    }
    
    return true;
}

bool ofProceduralCity::sortByDelay(RoadSegment A, RoadSegment B){
    return (A.time_delay < B.time_delay);
}

//-----------------------------------------------------------------------------

bool onSegment(ofVec2f p, ofVec2f q, ofVec2f r)
{
    if (q.x <= max(p.x, r.x) && q.x >= min(p.x, r.x) &&
        q.y <= max(p.y, r.y) && q.y >= min(p.y, r.y))
        return true;
    
    return false;
}

int checkOrientation(ofVec2f p, ofVec2f q, ofVec2f r){
    int val = (q.y - p.y) * (r.x - q.x) -
    (q.x - p.x) * (r.y - q.y);
    
    if (val == 0) return 0;  // colinear
    
    return (val > 0)? 1: 2; // clock or counterclock wise
}

bool ofProceduralCity::checkIntersecting(RoadSegment A){
    
//    How is Orientation useful here?
//    Two segments (p1,q1) and (p2,q2) intersect if and only if one of the following two conditions is verified
//
//        1. General Case:
//        – (p1, q1, p2) and (p1, q1, q2) have different orientations and
//        – (p2, q2, p1) and (p2, q2, q1) have different orientations.
    
    for(auto B : segment_list){
        bool o1 = checkOrientation(A.start, A.end, B.start);
        bool o2 = checkOrientation(A.start, A.end, B.end);
        bool o3 = checkOrientation(B.start, B.end, A.start);
        bool o4 = checkOrientation(B.start, B.end, A.end);
        
        //let's return the point at which the intersection occurs...
        
        
        // General case
        if (o1 != o2 && o3 != o4){
            return true;
        }
        
        // Special Cases
        // p1, q1 and p2 are colinear and p2 lies on segment p1q1
        if (o1 == 0 && onSegment(A.start, B.start, A.end)) return true;
        
        // p1, q1 and q2 are colinear and q2 lies on segment p1q1
        if (o2 == 0 && onSegment(A.start, B.end, A.end)) return true;
        
        // p2, q2 and p1 are colinear and p1 lies on segment p2q2
        if (o3 == 0 && onSegment(B.start, A.start, B.end)) return true;
        
        // p2, q2 and q1 are colinear and q1 lies on segment p2q2
        if (o4 == 0 && onSegment(B.start, A.end, B.end)) return true;
        
        return false; // Doesn't fall in any of the above cases
    }
}

//-----------------------------------------------------------------------------

void ofProceduralCity::draw(){
    global_walk = (int)(ofGetElapsedTimeMillis()/500.0);
    
    for(auto r : segment_list){
        if(r.time_delay < global_walk){
            r.debugLine.draw();
        }
    }
    
    mesh.draw();
}

void ofProceduralCity::printDebug(){
    ofDrawBitmapString("Priority List: " + ofToString(priority_list.size()), 10, 15);
    ofDrawBitmapString("Segment List: " + ofToString(segment_list.size()), 10, 30);
    ofDrawBitmapString("Total Nodes: " + ofToString(mesh.getVertices().size()), 10, 45);
    ofDrawBitmapString("Global Walk: " + ofToString(global_walk), 10, 75);
    
}

//-----------------------------------------------------------------------------

//for(auto _r : priority_list){
//    std::cout << "vector (SORTED):";
//    std::cout << _r.time_delay;
//    std::cout << endl;
//    }
