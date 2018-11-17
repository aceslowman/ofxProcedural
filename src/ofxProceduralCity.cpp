#include "ofxProceduralCity.h"

void ofxProceduralCity::reset(){
    roads.reset();
    population_map.reset(3, 150, 500, map_size);
    elevation_map.reset(9, 150, 500, map_size);
    elevation_map.invert();
}

void ofxProceduralCity::setup(){
    global_walk = 0;
    map_size = 512;
    
    ofSetWindowShape(map_size, map_size);
    ofSetWindowPosition((ofGetScreenWidth() / 2.0)-(map_size/2.0), (ofGetScreenHeight() / 2.0)-(map_size/2.0));
    
    population_map.setup(3, 150, 500, map_size);
    elevation_map.setup(9, 150, 500, map_size);
    elevation_map.invert();
    
    roads.setup();
}

void ofxProceduralCity::draw(){
    //
}

bool ofxProceduralCity::globalBoundsCheck(ofVec3f &a){
    if(a.x >= map_size || a.x <= 0 || a.y >= map_size || a.y <= 0){
        ofLog(OF_LOG_ERROR, "OUT OF BOUNDS");
        return false;
    }
    
    return true;
}

//----------------------------------------------------------------

namespace proc_utils {
    
    bool getLineIntersection(ofVec2f p0, ofVec2f p1, ofVec2f p2, ofVec2f p3, ofVec2f &intersection){
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
    
    float getDistanceBetweenPointandLine( ofVec2f a, ofVec2f b, ofVec2f p ){
        ofVec2f n = b - a;
        ofVec2f pa = a - p;
        ofVec2f c = n * (pa.dot(n) / n.dot(n));
        ofVec2f d = pa - c;
        return sqrt( d.dot(d) );
    }
    
    bool sortByDistance(ofVec2f A, ofVec2f B, ofVec2f pt){
        return (pt.distance(A) < pt.distance(B));
    }
}


