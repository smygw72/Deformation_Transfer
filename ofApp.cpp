#include "ofApp.h"

using namespace Eigen;

namespace {
	std::string undeformed_source = "./data/kawai_N.obj";
	std::string deformed_source = "./data/kawai_a.obj";
	std::string undeformed_target = "./data/mizo_N.obj";
	std::string deformed_target = "./data/mizo_a.obj";	// copy "./data/mizo_N.obj" 
}
EigenMatrixXs X;

//--------------------------------------------------------------
void ofApp::setup(){


	undeformed_source_mesh = new ObjMesh();
	deformed_source_mesh = new ObjMesh();
	undeformed_target_mesh = new ObjMesh();
	deformed_target_mesh = new ObjMesh();

	undeformed_source_mesh->read_from_file(undeformed_source);
	deformed_source_mesh->read_from_file(deformed_source);
	undeformed_target_mesh->read_from_file(undeformed_target);
	deformed_target_mesh->read_from_file(deformed_target);
	
	// deformation transfer
	Deformation_Transfer(undeformed_source_mesh, deformed_source_mesh, undeformed_target_mesh, X);
	deformed_target_mesh->setm_V(X);
//	deformed_target_mesh->setm_F();

	//compute Normal
	undeformed_source_mesh->Init();
	deformed_source_mesh->Init();
	undeformed_target_mesh->Init();	
	deformed_target_mesh->Init();

	// lights setting 
	lights.resize(2);
	float light_distance = 300.;
	lights[0].setPosition(2.0*light_distance, 1.0*light_distance, 0.);
	lights[1].setPosition(-2.0*light_distance, -1.0*light_distance, 1.0* light_distance);
	
}

//--------------------------------------------------------------
void ofApp::update(){

}

//--------------------------------------------------------------
void ofApp::draw(){

	ofBackgroundGradient(ofColor(100), ofColor(25));

	glEnable(GL_DEPTH_TEST);

	ofEnableLighting();
	for (int i = 0; i < lights.size(); ++i) { lights[i].enable(); }

	cam.begin(); 

	ofPushMatrix();
	ofTranslate(-200, 200);
	ofSetColor(ofColor(133, 180, 250));
	undeformed_source_mesh->renderPoly();
	ofPopMatrix();
	
	ofPushMatrix();
	ofTranslate(200, 200);
	ofSetColor(ofColor(133, 180, 250));
	deformed_source_mesh->renderPoly();
	ofPopMatrix();

	ofPushMatrix();
	ofTranslate(-200, -200);
	ofSetColor(ofColor(133, 180, 250));
	undeformed_target_mesh->renderPoly();
	ofPopMatrix();

	ofPushMatrix();
	ofTranslate(200, -200);
	ofSetColor(ofColor(133, 180, 250));
	deformed_target_mesh->renderPoly();
	ofPopMatrix();

	cam.end();

	for (int i = 0; i < lights.size(); i++) { lights[i].disable(); }
	ofDisableLighting();
	glDisable(GL_DEPTH_TEST);
}

//--------------------------------------------------------------
void ofApp::keyPressed(int key){

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
