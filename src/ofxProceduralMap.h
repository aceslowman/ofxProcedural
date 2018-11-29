#pragma once
#include "ofMain.h"
#include "ofxGui.h"

class ofxProceduralMap{
private:
    ofFloatPixels pix;
    
public:
    void reset(int octaves, float amplitude, float frequency, ofVec2f _size);
    void setup(int octaves, float amplitude, float frequency, ofVec2f _size);
    void generate();
    void invert();
    void draw();
    
    float sample(ofVec2f st);

    ofTexture tex;
    ofVec2f size;
    
//    int octaves;
//    float amplitude;
    
    ofParameterGroup params;
    ofParameter<ofVec2f> noise_frequency;
    ofParameter<ofVec2f> noise_range;
    ofParameter<int> noise_octaves;
    
    ofxProceduralMap(){
        params.setName("Map");
        params.add(noise_range.set("Range", ofVec2f(0,1), ofVec2f(-1,-1), ofVec2f(1,1)));
        params.add(noise_frequency.set("Frequency", ofVec2f(500,500), ofVec2f(0,0), ofVec2f(1000,1000)));
        params.add(noise_octaves.set("Octaves", 5, 0, 10));
    };
};
