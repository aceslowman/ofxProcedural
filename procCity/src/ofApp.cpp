#include "ofApp.h"

//--------------------------------------------------------------
void ofApp::setup(){
    debug = true;
    
    city.setup();
}

//--------------------------------------------------------------
void ofApp::update(){
    
}

//--------------------------------------------------------------
void ofApp::draw(){
    ofSetBackgroundColor(ofColor(0));

//    cam.begin();
    
    if(debug){
        city.pop_map.draw(0,0,city.map_size,city.map_size);
        ofDrawGrid(city.map_size/10.0f, 10, false, false, false, true);
    }
    
    city.draw();
//    cam.end();
    
    city.printDebug();
}

//--------------------------------------------------------------
void ofApp::keyPressed(int key){
    if(key == 'd'){
        debug = !debug;
    }
}

//--------------------------------------------------------------
void ofApp::keyReleased(int key){
    
}

//--------------------------------------------------------------
void ofApp::mouseMoved(int x, int y ){
    
}

//--------------------------------------------------------------
void ofApp::mouseDragged(int x, int y, int button){
    
}

//--------------------------------------------------------------
void ofApp::mousePressed(int x, int y, int button){
    
}

//--------------------------------------------------------------
void ofApp::mouseReleased(int x, int y, int button){
    
}

//--------------------------------------------------------------
void ofApp::mouseEntered(int x, int y){
    
}

//--------------------------------------------------------------
void ofApp::mouseExited(int x, int y){
    
}

//--------------------------------------------------------------
void ofApp::windowResized(int w, int h){
    
}

//--------------------------------------------------------------
void ofApp::gotMessage(ofMessage msg){
    
}

//--------------------------------------------------------------
void ofApp::dragEvent(ofDragInfo dragInfo){
    
}

