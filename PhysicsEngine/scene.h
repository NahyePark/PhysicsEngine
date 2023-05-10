////////////////////////////////////////////////////////////////////////
// The scene class contains all the parameters needed to define and
// draw a simple scene, including:
//   * Geometry
//   * Light parameters
//   * Material properties
//   * Viewport size parameters
//   * Viewing transformation values
//   * others ...
//
// Some of these parameters are set when the scene is built, and
// others are set by the framework in response to user mouse/keyboard
// interactions.  All of them can be used to draw the scene.

#include "shapes.h"
#include "object.h"
#include "texture.h"
#include "fbo.h"
#include "gbuffer.h"

enum ObjectIds {
    nullId	= 0,
    skyId	= 1,
    seaId	= 2,
    groundId	= 3,
    roomId	= 4,
    boxId	= 5,
    frameId	= 6,
    lPicId	= 7,
    rPicId	= 8,
    teapotId	= 9,
    spheresId	= 10,
    floorId     = 11,
    lionId      = 12,
    sphere1Id    = 13,
    sphere2Id    = 14
};

class Shader;

class Scene
{
public:
    GLFWwindow* window;

    // @@ Declare interactive viewing variables here. (spin, tilt, ry, front back, ...)

    // Light parameters
    float lightSpin, lightTilt, lightDist;
    glm::vec3 lightPos;
    glm::vec3 lightCol;
    glm::vec3 lightDir;

    // @@ Perhaps declare additional scene lighting values here. (lightVal, lightAmb)
    
    //buffer
    GBuffer gBuffer;
    FBO shadowBuffer;

    bool drawReflective;
    bool nav;
    int key;
    float spin, tilt, speed, ry, front, back;
    glm::vec3 eye, tr;
    float last_time;
    int smode; // Shadow on/off/debug mode
    int rmode; // Extra reflection indicator hooked up some keys and sent to shader
    int lmode; // BRDF mode
    int tmode; // Texture mode
    int imode; // Image Based Lighting mode
    bool flatshade;
    int mode; // Extra mode indicator hooked up to number keys and sent to shader
    
    // Viewport
    int width, height;

    // Transformations
    glm::mat4 WorldProj, WorldView, WorldInverse;
    glm::mat4 lightProj, lightView, shadowMatrix;

    // All objects in the scene are children of this single root object.
    Object* objectRoot;
    Object* central, *floor, *sphere1, *sphere2, *box, *box2;
    Object* quad;

    Shape* boxPolygons, * spherePolygons;

    // Shader programs
    ShaderProgram* lightingProgram;
    ShaderProgram* defferedShadingProgram;
    ShaderProgram* gBufferProgram;
    ShaderProgram* shadowProgram;
    ShaderProgram* computehorizonProgram;
    ShaderProgram* computeverticalProgram;
    // @@ Declare additional shaders if necessary

    //Physics

    int frameCount = 0;
    float timeElapesed = 0.0f;
    float fps;
    float deltaTime;
    float lastFrame;
    bool play = false;
    bool oneStep = false;
    unsigned int vaoID;
    GLuint pntBuff;
    GLuint indBuff;

    // Options menu stuff
    bool show_demo_window;
    int gBuffer_num;
    float alpha;

    void ClearAll();
    void CreateQuad();
    void DrawQuad();
    void InitializeScene();
    void BuildTransforms();
    void DrawMenu();
    void DrawScene();
    void DrawContactPoints(ShaderProgram* program);
};
