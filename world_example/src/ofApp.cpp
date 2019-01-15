#include "ofApp.h"

//--------------------------------------------------------------
void ofApp::setup(){
    debug = true;
   
    city.setup();
//    city.setupGui();

    drawElev = false;
    drawPop = false;
    
    show_numbers = false;
    ofEnableDepthTest();
    
    orthoCam.removeAllInteractions();
    orthoCam.addInteraction(ofEasyCam::TRANSFORM_TRANSLATE_XY,OF_MOUSE_BUTTON_LEFT);
    orthoCam.addInteraction(ofEasyCam::TRANSFORM_TRANSLATE_Z, OF_MOUSE_BUTTON_RIGHT);
    orthoCam.enableOrtho();
    orthoCam.setNearClip(-1000000);
    orthoCam.setFarClip(1000000);
    orthoCam.setVFlip(true);
    orthoCam.setAutoDistance(false);

    cam = &orthoCam;
}

//--------------------------------------------------------------
void ofApp::update(){

}

//--------------------------------------------------------------
void ofApp::draw(){
    ofEnableDepthTest();
    ofSetBackgroundColor(ofColor(0));

//    cam->begin();
        city.roads.draw();
//    cam->end();
    
    if(debug){
        city.roads.drawDebug(cam, ofVec2f(ofGetMouseX(), ofGetMouseY()), true);
    }
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
    
    
    if(key == '1'){ //orthographic
        cam = &orthoCam;
        cam->reset();
    }
    
    if(key == '2'){ //angled
        cam = &normalCam;
        
        cam->resetTransform();

        cam->setPosition(city.dimensions.x * 1.1, city.dimensions.x * 1.1, city.dimensions.x * 1.1);
        cam->lookAt(ofVec3f(0,0,0),ofVec3f(-1,-1,0));
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

