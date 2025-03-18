#include <vector>
#include <string>
#include <fstream>
#include <nlohmann/json.hpp>
#include <array>

const float PI = 3.14159265358979323846f;
const float G = 6.67E-11;
const int MESH_RES = 50;
const int TRAJ_RES = 1;
const int TRAJ_BUFF_SIZE = 250;

const int SCREEN_WIDTH = 1024;
const int SCREEN_HEIGTH = 1024;

class Body
{
public:    
    int id = 0;
    std::vector<double> position;
    std::vector<double> velocity;
    std::vector<double> rgb;
    double mass;
    double radius;

    std::array<std::vector<double>, TRAJ_BUFF_SIZE> trajectory;
    int ix = 0;
    int endPos = 0;

    Body(std::vector<double> initPos, std::vector<double> initVel, double m, double r, std::vector<double> color);

    void updatePosition();

    void updateVelocity(std::vector<double> deltaV);

    void drawBody();

    void drawTrajectory();
};

class GravitySimulator
{
public:
    std::vector<Body> bodies;
    static GravitySimulator* currentInstance;
    
    GravitySimulator();

    void addBody(Body& body);

    double getDistance(Body& b1, Body& b2);

    std::vector<double> getGravForce(Body& b1, Body& b2);

    void renderScene(void);

    static void renderSceneStatic()
    {
        if (currentInstance)
            currentInstance->renderScene();
    }

    void loadSceneFromFile(std::string filePath);
};

namespace GUIUtils
{
    void drawLine(float x1, float y1, float x2, float y2)
    {
        glBegin(GL_LINES);
        glVertex2f(x1, y1);
        glVertex2f(x2, y2);
        glEnd();
    }
    
    void drawArrow(Body& b1, Body& b2)
    {
        double forceAbs = 50; //(G * b1.mass * b2.mass) / pow(getDistance(b1, b2), 2);
        double angle = atan2((b2.position.at(1) - b1.position.at(1)), (b2.position.at(0) - b1.position.at(0)));
        double x2 = (forceAbs * cos(angle)) + b1.position.at(0);
        double y2 = (forceAbs * sin(angle)) + b1.position.at(1);
        
        glBegin(GL_LINES);
        glVertex2f(b1.position.at(0), b1.position.at(1));
        glVertex2f(x2, y2);

        double arrowheadX1 = x2 - 20 * cos(angle - PI / 6);
        double arrowheadY1 = y2 - 20 * sin(angle - PI / 6);
        double arrowheadX2 = x2 - 20 * cos(angle + PI / 6);
        double arrowheadY2 = y2 - 20 * sin(angle + PI / 6);

        glBegin(GL_TRIANGLES);
        glVertex2f(x2, y2);  // Tip of the arrow
        glVertex2f(arrowheadX1, arrowheadY1); // First point of the arrowhead
        glVertex2f(arrowheadX2, arrowheadY2); // Second point of the arrowhead

        glEnd();
    }
}