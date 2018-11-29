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
    node.setParent(city->node);
    elevation = _elevation;
    
    generate();
}

void ofxProceduralTerrain::generate(){
    node.setPosition(city->dimensions.x / 2.0, city->dimensions.y / 2.0, 0);

    generatePlane();
//    mesh = mesh.plane(city->dimensions.x - 1.0, city->dimensions.y - 1.0, city->dimensions.x * detail_level, city->dimensions.y * detail_level, OF_PRIMITIVE_TRIANGLES);
    //now can I apply the node transformation to the mesh somehow?
    
    
    displace();
}

void ofxProceduralTerrain::generatePlane(){
    mesh.setMode(OF_PRIMITIVE_TRIANGLES);

    float width = city->dimensions.x;
    float height = city->dimensions.y;
    
    int rows = city->dimensions.x * detail_level;
    int columns = city->dimensions.y * detail_level;

    ofVec2f texcoord;
    ofVec3f vert;
    ofVec3f normal = ofVec3f(0,1,0); // y is always up

    for(int iy = 0; iy != rows; iy++) {
        for(int ix = 0; ix != columns; ix++) {
            
            // normalized tex coords //
            texcoord.x =       ((float)ix/((float)columns-1));
            texcoord.y = 1.f - ((float)iy/((float)rows-1));
            
            vert.x = texcoord.x * width;
            vert.y = -(texcoord.y-1) * height;
            
            mesh.addVertex(vert);
            mesh.addTexCoord(texcoord);
            mesh.addNormal(normal);
        }
    }

    for(int y = 0; y < rows-1; y++) {
        for(int x = 0; x < columns-1; x++) {
            // first triangle //
            mesh.addIndex((y)*columns + x);
            mesh.addIndex((y)*columns + x+1);
            mesh.addIndex((y+1)*columns + x);
            
            // second triangle //
            mesh.addIndex((y)*columns + x+1);
            mesh.addIndex((y+1)*columns + x+1);
            mesh.addIndex((y+1)*columns + x);
        }
    }
}

void ofxProceduralTerrain::displace(){
    for(int i = 0; i < mesh.getNumVertices(); i++){
        ofVec3f position = mesh.getVertex(i);
        
//        float sample = elevation->sample(position + (city->dimensions / 2.0));
        float sample = elevation->sample(position);
        
        mesh.addColor(ofFloatColor(sample));
        
        position.z = sample;
        position.z *= zscale;
        
        mesh.setVertex(i, position);
        mesh.addTexCoord((ofVec2f)position);
    }
}

//-----------------------------------------------------------------------------
//  Drawing
//-----------------------------------------------------------------------------

void ofxProceduralTerrain::draw(){
    ofDrawCircle(-city->dimensions.x,-city->dimensions.y,0,10);
    
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
            elevation->tex.draw(-city->dimensions.x/2.0,-city->dimensions.y/2.0);
    cam->end();
}
