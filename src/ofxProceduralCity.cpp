#include "ofxProceduralCity.h"

void ofxProceduralCity::reset(){
//    roads.reset();
    gui.clear();
    setup();
}

void ofxProceduralCity::setup(){
    global_walk = 0;
    dimensions = ofVec2f(512,512);
    
//    ofSetWindowShape(dimensions.x, dimensions.y);
//    ofSetWindowPosition((ofGetScreenWidth() / 2.0)-(dimensions.x/2.0), (ofGetScreenHeight() / 2.0)-(dimensions.y/2.0));
    
    population_map.setup(3, 150, 500, dimensions);
    elevation_map.setup(9, 150, 500, dimensions);
    elevation_map.invert();
    
//    roads.setup();
    terrain.setup(&elevation_map);
//    buildings.setup(&roads);

    params.setName("Procedural");
    params.add(regen_all.set("Regenerate", false));
    regen_all.addListener(this, &ofxProceduralCity::regenClicked);
    
    gui.setup(params);
    gui.add(terrain.params);
    gui.add(elevation_map.params);
    gui.add(roads.params);
}

void ofxProceduralCity::draw(){
    gui.draw();
}

bool ofxProceduralCity::globalBoundsCheck(ofVec3f &a){
    if(a.x >= dimensions.x || a.x <= 0 || a.y >= dimensions.y || a.y <= 0){
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

void ofxProceduralCity::regenClicked(bool &val){
    if(val){
        reset();
        
        val = false;
    }
}
