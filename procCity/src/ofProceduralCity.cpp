#include "ofProceduralCity.h"

//-----------------------------------------------------------------------------

RoadSegment::RoadSegment(float _time_delay, ofVec2f _start, ofVec2f _end){
    time_delay = _time_delay;
    start      = _start;
    end        = _end;
}

//-----------------------------------------------------------------------------
// SETUP
//-----------------------------------------------------------------------------

void ofProceduralCity::setup(){
    road_limit = 1500;
    global_walk = 0;
    pop_map.load("noise512.png");
    
    // set up maps
    map_size = pop_map.getWidth();
    ofSetWindowShape(map_size, map_size);
    ofSetWindowPosition((ofGetScreenWidth() / 2.0)-(map_size/2.0), (ofGetScreenHeight() / 2.0)-(map_size/2.0));
    
    road_scalar = map_size/15.0f;
    
    generateRoads();
    
    setupDebug();
}

void ofProceduralCity::setupDebug(){
    mesh.setMode(OF_PRIMITIVE_POINTS);
    crossingMesh.setMode(OF_PRIMITIVE_POINTS);

    
    for(auto &r : placed_list){
        mesh.addVertex((ofVec3f)r.end);
//        mesh.addColor(ofColor(r.population));
        
        r.line.addVertex((ofVec3f)r.start);
        r.line.addVertex((ofVec3f)r.end);
    }
    
    for(auto &i : crossing_list){
        crossingMesh.addVertex((ofVec3f)i);
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
    bool crossings = checkForCrossings(a);
    bool nearby = checkForNearby(a);

    if(crossings && nearby){
        return true;
    }else{
        ofLog(OF_LOG_NOTICE, "Road failed local constraints! Removing from pending.");
        return false;
    }
}

vector<RoadSegment> ofProceduralCity::globalGoals(RoadSegment &a){
    vector<RoadSegment> t_priority;
    
    int max_goals = 2;
    
    /*
        I now need to work towards removing limits...
     */
    
    (placed_list.size() < road_limit) ? max_goals = 2 : max_goals = 0;
    
    for(int i = 0; i < max_goals; i++){
        ofVec2f start = a.end;
        ofVec2f direction = ofVec2f(ofRandom(-1,1),ofRandom(-1,1));
        ofVec2f end = a.end + (direction*road_scalar);
    
        bool pattern_check = constrainToRightAngles(a, end);
//        bool pattern_check = true;
        
        bool pop_check = constrainToPopulation(end);
        bool in_bounds = globalBoundsCheck(end);
    
        bool accepted = pattern_check && in_bounds;
        if(accepted){
            RoadSegment new_road(a.time_delay + 1, start, end);
            new_road.population = samplePopulation(end);
            t_priority.push_back(new_road);
        }
    }
    
    return t_priority;
}

bool ofProceduralCity::checkForCrossings(RoadSegment &a){
    float tolerance = 10.0f;
    /*
        Checks for crossings between segments, truncating the initial segment
        at the crossing point
     */
    vector<ofVec2f> crossings;
    for(auto b : placed_list){
        if (a.start == b.start || a.end == b.end || a.start == b.end || a.end == b.start) { continue; }
        
        ofVec2f crossing;

        bool intersect = getLineIntersection(a.start, a.end, b.start, b.end, crossing);
        
        if(intersect){
            crossings.push_back(crossing);
        }
    }
    
    bool intersects = crossings.size() > 0;
    if(intersects){
        std::sort(crossings.begin(), crossings.end(), std::bind(sortByDistance, std::placeholders::_1, std::placeholders::_2, a.start));
        
        a.end = crossings.front();
        a.population = samplePopulation(a.end);

        crossing_list.push_back(a.end);
    }
    
    return true;
}

bool ofProceduralCity::checkForNearby(RoadSegment &a){
    float tolerance = 10.0f;
    bool snapped = false;

    for(auto b : placed_list){
        bool close_to_start = a.end.distance(b.start) < tolerance;
        bool close_to_end = a.end.distance(b.end) < tolerance;
        
        if(close_to_start && close_to_end){
            return false;
        }else if(close_to_start){
            a.end = b.start;
            snapped = true;
        }else if(close_to_end){
            a.end = b.end;
            snapped = true;
        }
    }
    
    if(snapped){ // end points have moved, check again for intersections
        bool cross = checkForCrossings(a);
        
        if(!cross){
            return false;
        }
    }

    return true;
}

bool ofProceduralCity::constrainToRightAngles(RoadSegment &prev, ofVec2f &end){
    /*
        I am wondering if there are efficiency issues tied to the number of points we are dropping without
        checking distance from crossings, but for now that will be a local constraint task.
    */
    
    /* TODO: two roads should not be generated in the same quadrant. */
    ofVec2f prev_direction = ofVec2f(prev.end - prev.start).normalize();
    ofVec2f new_direction = prev_direction;

    int quadrant = (int)ofRandom(4);
    float range = 5.0f;
    float tendency = 90.0f; // 45 degree angle results in spiral!

    float random_angle = ofRandom((tendency * quadrant) - range,(tendency * quadrant) + range);
    new_direction.rotate(random_angle);

    end = prev.end + (new_direction * road_scalar);
    
    return true;
}

bool ofProceduralCity::constrainToPopulation(ofVec2f &end){
    // skew the end towards area of highest population
    
    // generate 3 rays within a given angle range
    // along each of the 3 rays, sample the population map at 5 points and sum
    // chose the ray with the greatest sum
    
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

bool ofProceduralCity::getLineIntersection(ofVec2f p0, ofVec2f p1, ofVec2f p2, ofVec2f p3, ofVec2f &intersection){
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

float ofProceduralCity::getDistanceBetweenPointandLine( ofVec2f a, ofVec2f b, ofVec2f p ){
    ofVec2f n = b - a;
    ofVec2f pa = a - p;
    ofVec2f c = n * (pa.dot(n) / n.dot(n));
    ofVec2f d = pa - c;
    return sqrt( d.dot(d) );
}

bool ofProceduralCity::sortByDelay(RoadSegment A, RoadSegment B){
    return (A.time_delay < B.time_delay);
}

bool ofProceduralCity::sortByDistance(ofVec2f A, ofVec2f B, ofVec2f pt){
    return (pt.distance(A) < pt.distance(B));
}

//-----------------------------------------------------------------------------
// DRAWING
//-----------------------------------------------------------------------------

void ofProceduralCity::draw(){
    ofSetColor(ofColor(255,255,255));
    for(auto r : placed_list){
        if(r.time_delay < global_walk){
            r.line.draw();
        }
    }
}

void ofProceduralCity::drawDebug(){
    glPointSize(5);
    ofSetColor(ofColor(0,255,0));
    mesh.draw();
    glPointSize(2);
    ofSetColor(ofColor(255,0,0));
    crossingMesh.draw();
    ofSetColor(ofColor(255,255,255));
    
    ofDrawBitmapString("Total Nodes: " + ofToString(mesh.getVertices().size()), 10, 10);
    ofDrawBitmapString("Global Walk: " + ofToString(global_walk), 10, 25);
}
