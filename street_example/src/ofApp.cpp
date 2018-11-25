#include "ofApp.h"

//--------------------------------------------------------------
void ofApp::setup(){
    debug = true;
   
    city.setup();
    
    

    
    drawElev = false;
    drawPop = true;
    
    show_numbers = false;
}

//--------------------------------------------------------------
void ofApp::update(){

}

//--------------------------------------------------------------
void ofApp::draw(){
    ofSetBackgroundColor(ofColor(0));

    cam.begin();
    
    if(debug){
        if(drawPop){
//            city.population_map.draw();
        }else if(drawElev){
//            city.elevation_map.draw();
        }
    }

    if(debug){
        ofSetColor(ofColor(0,0,255,100));
        ofDrawGrid(city.map_size/10.0f, 10, false, false, false, true);
    }
    
//    glPointSize(10);
//    city.buildings.draw();
//    glPointSize(2);
    
//    city.roads.draw();
    city.terrain.draw();
    
    cam.end();
    
    city.terrain.drawDebug(&cam);
//    city.roads.drawDebug(&cam, ofVec2f(mouseX, mouseY), show_numbers);

//    if(drawPop){
//        ofSetColor(ofColor(0));
//        ofDrawRectangle(0,ofGetHeight()-20,200,ofGetHeight());
//        ofSetColor(ofColor(255));
//        ofDrawBitmapString("(1) Population",10,ofGetHeight()-6);
//    }else if(drawElev){
//        ofSetColor(ofColor(0));
//        ofDrawRectangle(0,ofGetHeight()-20,200,ofGetHeight());
//        ofSetColor(ofColor(255));
//        ofDrawBitmapString("(2) Elevation",10,ofGetHeight()-6);
//    }
}

//--------------------------------------------------------------
void ofApp::keyPressed(int key){
    if(key == 'd'){
        debug = !debug;
    }
    if(key == 'i'){
        show_numbers = !show_numbers;
    }
    if(key == '-' && city.global_walk != 0){
        city.global_walk--;
    }
    if(key == '='){
        city.global_walk++;
    }
    if(key == 'b'){
        cam.setGlobalPosition(city.map_size/2,city.map_size/2,0);
        city.reset();
    }
    if(key == 'z'){
        drawPop = true;
        drawElev = false;
    }
    if(key == 'x'){
        drawPop = false;
        drawElev = true;
    }
    
    
    if(key == '1'){
        cam.removeAllInteractions();
        cam.addInteraction(ofEasyCam::TRANSFORM_TRANSLATE_XY,OF_MOUSE_BUTTON_LEFT);
        cam.addInteraction(ofEasyCam::TRANSFORM_TRANSLATE_Z, OF_MOUSE_BUTTON_RIGHT);
        
        cam.enableOrtho();
        cam.setNearClip(-1000000);
        cam.setFarClip(1000000);
        cam.setVFlip(true);
        cam.setAutoDistance(false);
        
        cam.setGlobalPosition(city.map_size/2,city.map_size/2,0);
    }
    
    if(key == '2'){
        
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

