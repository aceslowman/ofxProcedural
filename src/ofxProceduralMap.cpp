#include "ofxProceduralMap.h"

//-----------------------------------------------------------------------------

void ofxProceduralMap::reset(int _oct, float _amp, float _freq, ofVec2f _size){
    tex.clear();
    pix.clear();
    
    setup(_oct, _amp, _freq, _size);
}

void ofxProceduralMap::setup(int _oct, float _amp, float _freq, ofVec2f _size){
//    octaves = _oct;
//    amplitude = _amp;
//    frequency = _freq;
    size = _size;
    
    generate();
}

void ofxProceduralMap::generate(){
    pix.allocate(size.x, size.y,  OF_PIXELS_GRAY);
    tex.allocate(size.x, size.y, GL_R16F); //also GL_R32F for much higher precision
    
    ofSeedRandom();
    
    float offset = ofRandom(100);
    
    float amplitude = noise_range->y;
    
    for(int x = 0; x < size.x; x++){
        for(int y = 0; y < size.y; y++){
            ofVec2f st = ofVec2f(x,y);
            
            float value = 0.0;
            float amp = amplitude;
            ofVec2f freq = noise_frequency;
            
            for(int i = 0; i < noise_octaves; i++){
                value += amp * ofNoise( st / freq + offset );
                st *= 2.0;
                amp *= 0.5;
                
                ofFloatColor c = ofFloatColor(value);
                
                pix.setColor(x,y,c);
            }
        }
    }
    
    tex.loadData(pix);
}

void ofxProceduralMap::invert(){
    for(int i = 0; i < pix.size(); i++){
        pix[i] = noise_range->y - pix[i];
    }
    
    tex.loadData(pix);
}

//-----------------------------------------------------------------------------

void ofxProceduralMap::draw(){
    tex.draw(0,0);
}

float ofxProceduralMap::sample(ofVec2f st){
    return pix.getColor(floor(st.x),floor(st.y)).r;
}
