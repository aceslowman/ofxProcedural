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
    
    ofImage img;
    ofVec2f size;
    
    int octaves;
    float amplitude;
    float frequency;
    
    ofParameterGroup params;
    ofParameter<bool> signed_mode, normalized_mode, bit_mode;
    ofParameter<float> noise_frequency;
    ofParameter<ofVec2f> noise_size, noise_range;
    
    ofxProceduralMap();
};
