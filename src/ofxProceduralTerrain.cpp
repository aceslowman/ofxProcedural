#include "ofxProceduralTerrain.h"
#include "ofxProceduralCity.h"

//-----------------------------------------------------------------------------
//  Setup
//-----------------------------------------------------------------------------

void ofxProceduralTerrain::reset(){
    mesh.clear();
    setup(elevation);
}

void ofxProceduralTerrain::setup(ofxProceduralMap *_elevation){
    elevation = _elevation;
    generate();
}

void ofxProceduralTerrain::generate(){
    float detail_level = 0.05; // parameterize
    
    mesh = mesh.plane(city->map_size, city->map_size, city->map_size * detail_level, city->map_size * detail_level);
    
    displace();
}

void ofxProceduralTerrain::displace(){
    for(int i = 0; i < mesh.getNumVertices(); i++){
        ofVec3f position = mesh.getVertex(i);
        position.z = elevation->sample((ofVec2f)position);
        mesh.setVertex(i, position);
    }
}

//-----------------------------------------------------------------------------
//  Drawing
//-----------------------------------------------------------------------------

void ofxProceduralTerrain::draw(){
    ofSetColor(ofColor(255,255,0,100));
    mesh.draw();
}

void ofxProceduralTerrain::drawDebug(ofEasyCam* cam){
    ofSetColor(ofColor(255,0,100));
    cam->begin();
    mesh.drawWireframe();
    cam->end();
}
