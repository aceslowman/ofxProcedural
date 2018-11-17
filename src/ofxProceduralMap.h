#pragma once
#include "ofMain.h"

class ofxProceduralMap{
private:
    
public:
    void reset(int octaves, float amplitude, float frequency, int _size);
    void setup(int octaves, float amplitude, float frequency, int _size);
    void generate();
    void invert();
    void draw();
    
    int sample(ofVec2f st);
    
    ofImage img;
    int size;
    
    int octaves;
    float amplitude;
    float frequency; 
    
    ofxProceduralMap();
};
