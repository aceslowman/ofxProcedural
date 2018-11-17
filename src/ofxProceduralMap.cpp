#include "ofxProceduralMap.h"

//-----------------------------------------------------------------------------

ofxProceduralMap::ofxProceduralMap(){}

//-----------------------------------------------------------------------------

void ofxProceduralMap::reset(int _oct, float _amp, float _freq, int _size){
    img.clear();
    
    setup(_oct, _amp, _freq, _size);
}

void ofxProceduralMap::setup(int _oct, float _amp, float _freq, int _size){
    octaves = _oct;
    amplitude = _amp;
    frequency = _freq;
    size = _size;
    
    generate();
}

void ofxProceduralMap::generate(){
    ofPixels pix;
    pix.allocate(size, size, OF_IMAGE_GRAYSCALE);
    img.allocate(size, size, OF_IMAGE_GRAYSCALE);
    
    ofSeedRandom();
    
    float offset = ofRandom(100);
    
    amplitude = 150;
    frequency = 500;
    
    for(int x = 0; x < size; x++){
        for(int y = 0; y < size; y++){
            ofVec2f st = ofVec2f(x,y);
            
            float value = 0.0;
            float amp = amplitude;
            float freq = frequency;
            
            for(int i = 0; i < octaves; i++){
                value += amp * ofNoise( st / freq + offset );
                st *= 2.0;
                amp *= 0.5;
                
                pix.setColor(x,y, ofColor(255 - ofClamp(value,0,255)));
            }
        }
    }
    
    img.setFromPixels(pix);
}

void ofxProceduralMap::invert(){
    ofPixels &pix = img.getPixels();
    
    for(int i = 0; i < pix.size(); i++){
        pix[i] = 255 - pix[i];
    }
    
    img.update();
}

//-----------------------------------------------------------------------------

void ofxProceduralMap::draw(){
    img.draw(0,0);
}

int ofxProceduralMap::sample(ofVec2f st){
    ofPixels &pix = img.getPixels();
    
    return pix.getColor(st.x,st.y).r;
}
