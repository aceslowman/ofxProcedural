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
        
        //currently, each node START is equal to the first node's END?
        bool accepted = localConstraints(r); //TODO: pay attention to intersections, starting here
        
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
        Checks for intersection between segments, truncating the initial segment
        at the intersection point
     */
    ofVec2f intersection = ofVec2f(0,0);
    
    for(auto b : segment_list){ // this is gonna be a lot of iterations...
            bool intersect = getLineIntersection(r.start, r.end, b.start, b.end, &intersection);
            if(intersect){
                ofLog(OF_LOG_NOTICE, "Intersection found at: " + ofToString(intersection));
//                r.end = intersection;
            }
    }
    
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

/*
    From Andre LeMothe's "Tricks of the Windows Game Programming Gurus"
    via https://stackoverflow.com/questions/563198/how-do-you-detect-where-two-line-segments-intersect
 */

bool ofProceduralCity::getLineIntersection(ofVec2f p0, ofVec2f p1, ofVec2f p2, ofVec2f p3, ofVec2f *intersection){
    
    float s1_x, s1_y, s2_x, s2_y;
    s1_x = p1.x - p0.x;     s1_y = p1.y - p0.y;
    s2_x = p3.x - p2.x;     s2_y = p3.y - p2.y;
    
    float s, t;
    s = (-s1_y * (p0.x - p2.x) + s1_x * (p0.y - p2.y)) / (-s2_x * s1_y + s1_x * s2_y);
    t = ( s2_x * (p0.y - p2.y) - s2_y * (p0.x - p2.x)) / (-s2_x * s1_y + s1_x * s2_y);
    
    if (s >= 0 && s <= 1 && t >= 0 && t <= 1){
        intersection->x = p0.x + (t * s1_x);
        intersection->y = p0.y + (t * s1_y);
        return true;
    }
    
    return false;
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
