#pragma once

#include "ofMain.h"
#include "mesh.h"
#include "deformation_transfer.h"


class ofApp : public ofBaseApp{

	public:
		void setup();
		void update();
		void draw();

		void keyPressed(int key);
		void keyReleased(int key);
		void mouseMoved(int x, int y );
		void mouseDragged(int x, int y, int button);
		void mousePressed(int x, int y, int button);
		void mouseReleased(int x, int y, int button);
		void mouseEntered(int x, int y);
		void mouseExited(int x, int y);
		void windowResized(int w, int h);
		void dragEvent(ofDragInfo dragInfo);
		void gotMessage(ofMessage msg);

public:
	ofEasyCam cam;
	std::vector<ofLight> lights;
	Mesh* undeformed_source_mesh;
	Mesh* deformed_source_mesh;
	Mesh* undeformed_target_mesh;
	Mesh* deformed_target_mesh;
};
