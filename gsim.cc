#include <GL/glut.h>
#include <cmath>
#include <iostream>
#include <vector>
#include <array>
#include <thread>
#include <chrono>
#include <stdio.h>
#include "gsim.h"

Body::Body(std::vector<double> initPos, std::vector<double> initVel, double m, double r, std::vector<double> color)
{
    this->position.push_back(initPos.at(0));
    this->position.push_back(initPos.at(1));

    this->velocity.push_back(initVel.at(0));
    this->velocity.push_back(initVel.at(1));

    this->rgb.push_back(color.at(0));
    this->rgb.push_back(color.at(1));
    this->rgb.push_back(color.at(2));

    this->mass = m;
    this->radius = r;
}

void Body::updatePosition()
{
    static int it = 0;

    if (it++ % TRAJ_RES == 0)
    {
        trajectory[this->ix] = {this->position.at(0), this->position.at(1)};
        this->ix = (this->ix + 1) % TRAJ_BUFF_SIZE;

        if (this->endPos < TRAJ_BUFF_SIZE)
            this->endPos++;
    }

    this->position.at(0) += this->velocity.at(0);
    this->position.at(1) += this->velocity.at(1);
}

void Body::updateVelocity(std::vector<double> deltaV)
{
    this->velocity.at(0) += deltaV.at(0);
    this->velocity.at(1) += deltaV.at(1);
}

void Body::drawBody()
{
    glColor3f(this->rgb.at(0), this->rgb.at(1), this->rgb.at(2));
    glBegin(GL_TRIANGLE_FAN); // Use triangle fan to draw the circle
    glVertex2f(this->position.at(0), this->position.at(1)); // Center of the circle (the first vertex is the center)

    for (int i = 0; i <= MESH_RES; ++i) {
        float angle = (i * 2.0f * PI) / MESH_RES; // Calculate angle for each vertex
        float dx = radius * cos(angle); // X component
        float dy = radius * sin(angle); // Y component
        glVertex2f(this->position.at(0) + dx, this->position.at(1) + dy); // Add each vertex to the circle
    }

    glEnd();
}

void Body::drawTrajectory()
{
    //glColor3f(this->rgb.at(0), this->rgb.at(1), this->rgb.at(2));
    glBegin(GL_POINTS);

    for (int i = 0; i < this->endPos; i++)
    {
        std::vector<double> p = trajectory.at(i);
        glVertex2f(p.at(0), p.at(1));
    }

    glEnd();
}

GravitySimulator::GravitySimulator()
{

}

void GravitySimulator::addBody(Body& body)
{
    auto nBodies = bodies.size();
    body.id = nBodies;
    bodies.push_back(body);
}

double GravitySimulator::getDistance(Body& b1, Body& b2)
{
    return sqrt(pow(b2.position.at(1) - b1.position.at(1), 2) + pow(b2.position.at(0) - b1.position.at(0), 2));
}

std::vector<double> GravitySimulator::getGravForce(Body& b1, Body& b2)
{
    double forceAbs = (G * b1.mass * b2.mass) / pow(getDistance(b1, b2), 2);
    double alpha = atan2((b2.position.at(1) - b1.position.at(1)), (b2.position.at(0) - b1.position.at(0)));

    return {forceAbs*cos(alpha), forceAbs*sin(alpha)};
}

void GravitySimulator::renderScene(void)
{
    glClear(GL_COLOR_BUFFER_BIT);

    for(Body& bi : this->bodies)
    {
        std::array<double, 2> forces {0,0};
        bi.drawBody();

        for(Body& by : this->bodies)
        {
            if (bi.id != by.id)
            {
                std::vector<double> f = getGravForce(bi, by);
                forces.at(0) += f.at(0);
                forces.at(1) += f.at(1);
            }
        }

        double ax = forces.at(0) / bi.mass;
        double ay = forces.at(1) / bi.mass;

        //printf("[%d] ax: %lf ay: %lf px: %lf py: %lf\n", bi.id, ax, ay, bi.position.at(0), bi.position.at(1));

        bi.updateVelocity({ax, ay});
        bi.updatePosition();   
        bi.drawTrajectory();         
    }

    //printf("==============================\n");

    glFlush();
    glutSwapBuffers();
}

void GravitySimulator::loadSceneFromFile(std::string filePath)
{
    using json = nlohmann::json;
    std::ifstream inFile(filePath);

    json data = json::parse(inFile);

    for (auto& body : data)
    {
        Body b ({body["initialPosition"][0], 
                 body["initialPosition"][1],},
                {body["initialVelocity"][0], 
                 body["initialVelocity"][1]},
                 body["mass"],
                 body["radius"],
                {body["colorRgb"][0],
                 body["colorRgb"][1],
                 body["colorRgb"][2]});
        this->addBody(b);
        std::cout << "Adding " << body << std::endl;
    }
}

GravitySimulator* GravitySimulator::currentInstance = nullptr;

int main(int argc, char** argv) {

    GravitySimulator gSim;

    gSim.loadSceneFromFile("config.json");

    glutInit(&argc, argv); // Initialize GLUT
    glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGB); // Set display mode for double buffering and RGB color
    glutInitWindowSize(SCREEN_WIDTH, SCREEN_HEIGTH); // Set window size
    glutCreateWindow("Gravity"); // Create the window

    // Set up orthogonal projection (to avoid perspective distortion)
    glOrtho(0, SCREEN_WIDTH, 0, SCREEN_HEIGTH, 0, 1);

    gSim.currentInstance = &gSim;
    glutDisplayFunc(GravitySimulator::renderSceneStatic);
    glutIdleFunc(GravitySimulator::renderSceneStatic);
    glutMainLoop(); // Enter the GLUT main loop

    return 0;
}