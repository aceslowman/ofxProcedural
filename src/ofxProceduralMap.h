#pragma once
#include "ofMain.h"
#include "ofxGui.h"

class ofxProceduralMap{
private:
    
public:
    void reset(int octaves, float amplitude, float frequency, ofVec2f _size);
    void setup(int octaves, float amplitude, float frequency, ofVec2f _size);
    void generate();
    void invert();
    void draw();
    
    int sample(ofVec2f st);
    
    ofTexture tex;
    ofPixels pix;
    ofVec2f size;
    
    int octaves;
    float amplitude;
    float frequency;
    
    ofParameterGroup params;
    ofParameter<ofVec2f> noise_frequency;
    ofParameter<ofVec2f> noise_range;
    
    ofxProceduralMap(){
        params.setName("Map");
        params.add(noise_range.set("Noise Range", ofVec2f(0,1), ofVec2f(-1,-1), ofVec2f(1,1)));
        params.add(noise_frequency.set("Noise Frequency", ofVec2f(500,500), ofVec2f(0,0), ofVec2f(1000,1000)));
    };
};
