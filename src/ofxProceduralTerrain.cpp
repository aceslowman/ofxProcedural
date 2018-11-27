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
    
    params.setName("Terrain");
    params.add(zscale.set("z-scale", 100, 0, 500));
    params.add(show_elevation.set("Show Elev.", true));
    
    generate();
}

void ofxProceduralTerrain::generate(){
    float detail_level = 0.05; // parameterize
    
    mesh = mesh.plane(city->dimensions.x, city->dimensions.y, city->dimensions.x * detail_level, city->dimensions.y * detail_level, OF_PRIMITIVE_TRIANGLE_FAN);
    
    displace();
}

void ofxProceduralTerrain::displace(){
    for(int i = 0; i < mesh.getNumVertices(); i++){
        ofVec3f position = mesh.getVertex(i);

        position.z = elevation->sample(position + (city->dimensions/2.0)) * zscale; // bug here.
        mesh.setVertex(i, position);
        mesh.addTexCoord((ofVec2f)position);
        mesh.addColor(ofColor(position.z));
    }
}

//-----------------------------------------------------------------------------
//  Drawing
//-----------------------------------------------------------------------------

void ofxProceduralTerrain::draw(){
    if(show_elevation){
        mesh.enableColors();
    }else{
        mesh.disableColors();
    }
    
    mesh.draw();
}

void ofxProceduralTerrain::drawDebug(ofEasyCam* cam){
    cam->begin();
            ofSetColor(ofColor(255,0,100));
            ofDisableDepthTest();
                mesh.drawWireframe();
            ofEnableDepthTest();
    
            ofSetColor(ofColor(0,0,255,100));
            ofDrawGrid(city->dimensions.x/10.0f, 10, false, false, false, true);
            elevation->img.draw(-city->dimensions.x/2.0,-city->dimensions.y/2.0);
    cam->end();
}
