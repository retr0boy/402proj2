#include <cstddef>
#include <sys/types.h>
#include <sys/time.h>

#include <GL/glut.h>
#include <GL/glu.h>
#include <GL/gl.h>
#include <iostream>
#include <cmath>
#include <cstdlib>
#include <thread>

using namespace std;

// Global quaternion variables (Start with identity: w=1, x=0, y=0, z=0)
float qw = 1.0f, qx = 0.0f, qy = 0.0f, qz = 0.0f;

// Normalize the quaternion to prevent scaling/distortion
void normalizeQuat(float &w, float &x, float &y, float &z) {
    float mag = sqrt(w*w + x*x + y*y + z*z);
    if (mag > 0.0f) {
        w /= mag; x /= mag; y /= mag; z /= mag;
    }
}

// Multiply two quaternions (q1 * q2)
void multiplyQuat(float w1, float x1, float y1, float z1,
                  float w2, float x2, float y2, float z2,
                  float &wOut, float &xOut, float &yOut, float &zOut) {
    wOut = w1*w2 - x1*x2 - y1*y2 - z1*z2;
    xOut = w1*x2 + x1*w2 + y1*z2 - z1*y2;
    yOut = w1*y2 - x1*z2 + y1*w2 + z1*x2;
    zOut = w1*z2 + x1*y2 - y1*x2 + z1*w2;
}

// Convert quaternion to an OpenGL 4x4 rotation matrix (Column-major)
void quatToMatrix(float w, float x, float y, float z, float* m) {
    float xx = x * x; float yy = y * y; float zz = z * z;
    float xy = x * y; float xz = x * z; float yz = y * z;
    float wx = w * x; float wy = w * y; float wz = w * z;

    m[0]  = 1.0f - 2.0f * (yy + zz);
    m[1]  = 2.0f * (xy + wz);
    m[2]  = 2.0f * (xz - wy);
    m[3]  = 0.0f;

    m[4]  = 2.0f * (xy - wz);
    m[5]  = 1.0f - 2.0f * (xx + zz);
    m[6]  = 2.0f * (yz + wx);
    m[7]  = 0.0f;

    m[8]  = 2.0f * (xz + wy);
    m[9]  = 2.0f * (yz - wx);
    m[10] = 1.0f - 2.0f * (xx + yy);
    m[11] = 0.0f;

    m[12] = 0.0f; m[13] = 0.0f; m[14] = 0.0f; m[15] = 1.0f;
}

// Draw a colored cube so orientation is easily visible
void drawCube() {
    glBegin(GL_QUADS);
    // Front face (Red)
    glColor3f(1.0f, 0.0f, 0.0f);
    glVertex3f(-1.0f, -1.0f,  1.0f); glVertex3f( 1.0f, -1.0f,  1.0f);
    glVertex3f( 1.0f,  1.0f,  1.0f); glVertex3f(-1.0f,  1.0f,  1.0f);
    // Back face (Green)
    glColor3f(0.0f, 1.0f, 0.0f);
    glVertex3f(-1.0f, -1.0f, -1.0f); glVertex3f(-1.0f,  1.0f, -1.0f);
    glVertex3f( 1.0f,  1.0f, -1.0f); glVertex3f( 1.0f, -1.0f, -1.0f);
    // Top face (Blue)
    glColor3f(0.0f, 0.0f, 1.0f);
    glVertex3f(-1.0f,  1.0f, -1.0f); glVertex3f(-1.0f,  1.0f,  1.0f);
    glVertex3f( 1.0f,  1.0f,  1.0f); glVertex3f( 1.0f,  1.0f, -1.0f);
    // Bottom face (Yellow)
    glColor3f(1.0f, 1.0f, 0.0f);
    glVertex3f(-1.0f, -1.0f, -1.0f); glVertex3f( 1.0f, -1.0f, -1.0f);
    glVertex3f( 1.0f, -1.0f,  1.0f); glVertex3f(-1.0f, -1.0f,  1.0f);
    // Right face (Cyan)
    glColor3f(0.0f, 1.0f, 1.0f);
    glVertex3f( 1.0f, -1.0f, -1.0f); glVertex3f( 1.0f,  1.0f, -1.0f);
    glVertex3f( 1.0f,  1.0f,  1.0f); glVertex3f( 1.0f, -1.0f,  1.0f);
    // Left face (Magenta)
    glColor3f(1.0f, 0.0f, 1.0f);
    glVertex3f(-1.0f, -1.0f, -1.0f); glVertex3f(-1.0f, -1.0f,  1.0f);
    glVertex3f(-1.0f,  1.0f,  1.0f); glVertex3f(-1.0f,  1.0f, -1.0f);
    glEnd();
}

void display() {
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    glLoadIdentity();

    // Move the camera back a bit to see the cube
    glTranslatef(0.0f, 0.0f, -5.0f);

    // Apply the Quaternion Rotation
    float rotMatrix[16];
    quatToMatrix(qw, qx, qy, qz, rotMatrix);
    glMultMatrixf(rotMatrix);

    drawCube();

    glutSwapBuffers();
}

void reshape(int width, int height) {
    glViewport(0, 0, width, height);
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    gluPerspective(45.0, (double)width / (double)height, 0.1, 100.0);
    glMatrixMode(GL_MODELVIEW);
}

// Handle keyboard inputs to rotate the cube
void keyboard(unsigned char key, int x, int y) {
    // A small rotation step (approx 5 degrees)
    float angle = 5.0f * (M_PI / 180.0f);
    float s = sin(angle / 2.0f);
    float c = cos(angle / 2.0f);
    
    // Delta quaternion for the key press
    float dw = 1.0f, dx = 0.0f, dy = 0.0f, dz = 0.0f;

    switch (key) {
        case 'w': dw = c; dx = s; break;  // Rotate +X
        case 's': dw = c; dx = -s; break; // Rotate -X
        case 'a': dw = c; dy = -s; break; // Rotate -Y
        case 'd': dw = c; dy = s; break;  // Rotate +Y
        case 'q': dw = c; dz = s; break;  // Rotate +Z
        case 'e': dw = c; dz = -s; break; // Rotate -Z
        case 27: exit(0); break;          // ESC to quit
        default: return;
    }

    // Multiply current orientation by the delta
    float nw, nx, ny, nz;
    multiplyQuat(dw, dx, dy, dz, qw, qx, qy, qz, nw, nx, ny, nz);
    qw = nw; qx = nx; qy = ny; qz = nz;
    
    normalizeQuat(qw, qx, qy, qz);
    
    cout << "Current Quat (w, x, y, z): " << qw << " " << qx << " " << qy << " " << qz << endl;
    
    glutPostRedisplay(); // Redraw the screen
}

void readIMUData() {
	float w, x, y, z;
	
	while (cin >> w >> x >> y >> z){
		qw = w;
		qx = x;
		qy = y;
		qz = z;
		normalizeQuat(qw, qx, qy, qz);
		glutPostRedisplay();
	}
}

int main(int argc, char** argv) {
    // Parse command line arguments if provided (w x y z)
    if (argc == 5) {
        qw = atof(argv[1]);
        qx = atof(argv[2]);
        qy = atof(argv[3]);
        qz = atof(argv[4]);
        normalizeQuat(qw, qx, qy, qz);
        cout << "Loaded initial quaternion from arguments." << endl;
    } else {
        cout << "Using default identity quaternion." << endl;
        cout << "Usage: ./cube_quat [w] [x] [y] [z]" << endl;
    }

    cout << "\nControls:" << endl;
    cout << "  W / S : Rotate Pitch (X-axis)" << endl;
    cout << "  A / D : Rotate Yaw (Y-axis)" << endl;
    cout << "  Q / E : Rotate Roll (Z-axis)" << endl;
    cout << "  ESC   : Quit\n" << endl;

    // Initialize GLUT
    glutInit(&argc, argv);
    glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGB | GLUT_DEPTH);
    glutInitWindowSize(800, 600);
    glutCreateWindow("IMU Quaternion Visualization");

    glEnable(GL_DEPTH_TEST); // Enable 3D depth

    // Register callbacks
    glutDisplayFunc(display);
    glutReshapeFunc(reshape);
    glutKeyboardFunc(keyboard);
    
    std::thread imuThread(readIMUData);
    imuThread.detach();

    // Enter the main loop
    glutMainLoop();

    return 0;
}
        
