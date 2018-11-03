#include "ofProceduralCity.h"

//-----------------------------------------------------------------------------

RoadSegment::RoadSegment(float _time_delay, ofVec2f _start, ofVec2f _end){
    time_delay = _time_delay;
    start      = _start;
    end        = _end;
}

//-----------------------------------------------------------------------------

void ofProceduralCity::setup(){
    global_walk = 0;
    global_counter = 0; //temporary
    pop_map.load("noise512.png");
    
    // set up maps
    map_size = pop_map.getWidth();
    ofSetWindowShape(map_size, map_size);
    ofSetWindowPosition((ofGetScreenWidth() / 2.0)-(map_size/2.0), (ofGetScreenHeight() / 2.0)-(map_size/2.0));
    
    road_scalar = map_size/2.0f;
    
    ofVec2f i_start = ofVec2f(map_size/2,map_size/2);
    ofVec2f i_end = ofVec2f(map_size/2,map_size/2);
    
    pending_list.push_back(RoadSegment(0, i_start, i_end));
    placed_list.empty();
    
    while(pending_list.size() > 0){
        std::sort (pending_list.begin(), pending_list.end(), sortByDelay);
        RoadSegment &r = pending_list.front();

        bool accepted = localConstraints(r);
        
        if(accepted){
            placed_list.push_back(r);
            generateFromGlobalGoals(r);
        }
        
        pending_list.erase(pending_list.begin());
    }
    
    setupDebug();
}

void ofProceduralCity::setupDebug(){
    mesh.setMode(OF_PRIMITIVE_POINTS);
    intersectionMesh.setMode(OF_PRIMITIVE_POINTS);

    
    for(auto &r : placed_list){
        mesh.addVertex(ofVec3f(r.end.x,r.end.y,0));
        mesh.addColor(ofColor(r.population));
        
        r.debugLine.addVertex(ofVec3f(r.start.x,r.start.y,0));
        r.debugLine.addVertex(ofVec3f(r.end.x,r.end.y,0));
    }
}

//-----------------------------------------------------------------------------

bool ofProceduralCity::localConstraints(RoadSegment &a){
    ofLog(OF_LOG_NOTICE, "  checking local constraints  at point: ("+ofToString(a.start)+")("+ofToString(a.end)+")");
    
    constrainIntersections(a);
    
    return true;
}

void ofProceduralCity::generateFromGlobalGoals(RoadSegment a){
    ofLog(OF_LOG_NOTICE, "  generating global goals at point: ("+ofToString(a.start)+")("+ofToString(a.end)+")");
    
    vector<RoadSegment> t_priority;
    
    int max_goals;
    
    if(global_counter <= 100){
        max_goals = 2;
    }else{
        max_goals = 0;
    }
    
    for(int i = 0; i < max_goals; i++){
        ofVec2f start = a.end;
        ofVec2f direction = ofVec2f(ofRandom(-1.0f, 1.0f),ofRandom(-1.0f, 1.0f));
        ofVec2f end = start + (direction * road_scalar);

        // right here I can implement the road pattern check
        // what is the angle between the start/end pair, and it's previous (a.start/a.end)?
        
        //find the slope of each line ( start/end ) and ( a.start/a.end )
        //To find the slope, you divide the difference of the y-coordinates of 2 points on a line by the difference of the x-coordinates of those same 2 points .
        
//        float slope_1 = (a.end.y - a.end.y) / (a.start.x - a.start.y);
//        float slope_2 = (end.y - start.y) / (end.x - start.y);
//
//        float inc_1 = atan(slope_1);
//        float inc_2 = atan(slope_2);
//
//        float result = inc_2 - inc_1;
        
        bool valid = globalBoundsCheck(end);
        
        if(valid){
            RoadSegment new_road(a.time_delay + 1, start, end);
            
            new_road.population = samplePopulation(end);
         
            pending_list.push_back(new_road);
            global_counter++;
        }else{
            ofLog(OF_LOG_WARNING, "     Attempting to sample out of bounds!");
        }
    }
    
    return t_priority;
}

//-----------------------------------------------------------------------------

int ofProceduralCity::samplePopulation(ofVec2f s){
    ofPixels &pix = pop_map.getPixels();
    ofColor color = ofColor(0);
    
    return pix.getColor(s.x,s.y).r;
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

bool ofProceduralCity::sortByDistance(ofVec2f A, ofVec2f B, ofVec2f pt){
    return (pt.distance(A) > pt.distance(B));
}

//-----------------------------------------------------------------------------

bool ofProceduralCity::constrainIntersections(RoadSegment &a){
    /*
     Checks for intersection between segments, truncating the initial segment
     at the intersection point
     */
    vector<ofVec2f> intersections;
    
    for(auto b : placed_list){ // b is pulled from placed... are there elements that aren't in placed yet?
        if(&a != &b){
            ofVec2f intersection = ofVec2f(0,0);
            
            if (
                (((int)a.start.x == (int)b.start.x) && ((int)a.start.y == (int)b.start.y)) ||
                ((int)(a.end.x == b.end.x) && (int)(a.end.y == b.end.y)) ||
                ((int)(a.start.x == b.end.x) && (int)(a.start.y == b.end.y)) ||
                ((int)(a.end.x == b.start.x) && (int)(a.end.y == b.start.y))){
                break;
            }
            
            //find intersection between A and B
            bool intersect = getLineIntersection(a.start, a.end, b.start, b.end, intersection);
            
            ofLog(OF_LOG_NOTICE, "      Comparing:");
            ofLog(OF_LOG_NOTICE, "          A");
            ofLog(OF_LOG_NOTICE, "              Start: (" + ofToString(a.start) + ")");
            ofLog(OF_LOG_NOTICE, "              End: (" + ofToString(a.end) + ")");
            ofLog(OF_LOG_NOTICE, "          B");
            ofLog(OF_LOG_NOTICE, "              Start: (" + ofToString(b.start) + ")");
            ofLog(OF_LOG_NOTICE, "              End: (" + ofToString(b.end) + ")");
            
            if(intersect){
                ofLog(OF_LOG_NOTICE, "      Intersection found at: " + ofToString(intersection));
                intersections.push_back(intersection);
                break;
            }
        }
    }
    
    if(intersections.size() > 0){
        std::sort (intersections.begin(), intersections.end(),
                   std::bind(sortByDistance, std::placeholders::_1, std::placeholders::_2, a.start));
        
        a.end = intersections.front();
        a.population = samplePopulation(a.end);
        intersectionMesh.addVertex(ofVec3f(a.end.x,a.end.y,0));
        intersectionMesh.addColor(ofColor(255,0,0));
    }
    
    return true;
}

/*
    From Andre LeMothe's "Tricks of the Windows Game Programming Gurus"
    via https://stackoverflow.com/questions/563198/how-do-you-detect-where-two-line-segments-intersect
 */

bool ofProceduralCity::getLineIntersection(ofVec2f p0, ofVec2f p1, ofVec2f p2, ofVec2f p3, ofVec2f &intersection){
    
    float s1_x, s1_y, s2_x, s2_y;
    s1_x = p1.x - p0.x;     s1_y = p1.y - p0.y;
    s2_x = p3.x - p2.x;     s2_y = p3.y - p2.y;
    
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

//-----------------------------------------------------------------------------

void ofProceduralCity::draw(){
//    global_walk = (int)(ofGetElapsedTimeMillis()/500.0);
    
    for(auto r : placed_list){
        if(r.time_delay < global_walk){
            r.debugLine.draw();
        }
    }
    
    glPointSize(9);
    mesh.draw();
    glPointSize(5);
    intersectionMesh.draw();
}

void ofProceduralCity::printDebug(){
    ofDrawBitmapString("Priority List: " + ofToString(pending_list.size()), 10, 15);
    ofDrawBitmapString("Segment List: " + ofToString(placed_list.size()), 10, 30);
    ofDrawBitmapString("Total Nodes: " + ofToString(mesh.getVertices().size()), 10, 45);
    ofDrawBitmapString("Global Walk: " + ofToString(global_walk), 10, 75);
}
