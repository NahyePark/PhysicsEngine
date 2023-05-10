////////////////////////////////////////////////////////////////////////
// The scene class contains all the parameters needed to define and
// draw a simple scene, including:
//   * Geometry
//   * Light parameters
//   * Material properties
//   * viewport size parameters
//   * Viewing transformation values
//   * others ...
//
// Some of these parameters are set when the scene is built, and
// others are set by the framework in response to user mouse/keyboard
// interactions.  All of them can be used to draw the scene.

const bool fullPolyCount = true; // Use false when emulating the graphics pipeline in software

#include "math.h"
#include <iostream>
#include <stdlib.h>

#include <glbinding/gl/gl.h>
#include <glbinding/Binding.h>
using namespace gl;

#include <glu.h>                // For gluErrorString


#define GLM_FORCE_RADIANS
#define GLM_SWIZZLE
#include <glm/glm.hpp>
#include <glm/ext.hpp>          // For printing GLM objects with to_string
//#include <glm/gtc/matrix_transform.hpp>

#include "framework.h"
#include "texture.h"
#include "Physics.h"
#include "transform.h"

#include <memory>

const float FRAME_LIMIT = 1.f / 60.f;

const float PI = 3.14159f;
const float rad = PI/180.0f;    // Convert degrees to radians

glm::mat4 Identity;

const float grndSize = 100.0;    // Island radius;  Minimum about 20;  Maximum 1000 or so
const float grndOctaves = 4.0;  // Number of levels of detail to compute
const float grndFreq = 0.03;    // Number of hills per (approx) 50m
const float grndPersistence = 0.03; // Terrain roughness: Slight:0.01  rough:0.05
const float grndLow = -3.0;         // Lowest extent below sea level
const float grndHigh = 5.0;        // Highest extent above sea level

std::unique_ptr<Physics> g_Physics;
std::unique_ptr<DebugDrawing> debugDraw;

////////////////////////////////////////////////////////////////////////
// This macro makes it easy to sprinkle checks for OpenGL errors
// throughout your code.  Most OpenGL calls can record errors, and a
// careful programmer will check the error status *often*, perhaps as
// often as after every OpenGL call.  At the very least, once per
// refresh will tell you if something is going wrong.
#define CHECKERROR {GLenum err = glGetError(); if (err != GL_NO_ERROR) { fprintf(stderr, "OpenGL error (at line scene.cpp:%d): %s\n", __LINE__, gluErrorString(err)); exit(-1);} }


// Create an RGB color from human friendly parameters: hue, saturation, value
glm::vec3 HSV2RGB(const float h, const float s, const float v)
{
    if (s == 0.0)
        return glm::vec3(v,v,v);

    int i = (int)(h*6.0) % 6;
    float f = (h*6.0f) - i;
    float p = v*(1.0f - s);
    float q = v*(1.0f - s*f);
    float t = v*(1.0f - s*(1.0f-f));
    if      (i == 0)     return glm::vec3(v,t,p);
    else if (i == 1)  return glm::vec3(q,v,p);
    else if (i == 2)  return glm::vec3(p,v,t);
    else if (i == 3)  return glm::vec3(p,q,v);
    else if (i == 4)  return glm::vec3(t,p,v);
    else   /*i == 5*/ return glm::vec3(v,p,q);
}

void Scene::ClearAll()
{
    for (size_t i = 0; i < objectRoot->instances.size(); ++i)
        delete objectRoot->instances[i].first;

    delete objectRoot;

    delete quad;
    delete boxPolygons;
    delete spherePolygons;

    delete lightingProgram;
    delete defferedShadingProgram;
    delete gBufferProgram;
    delete shadowProgram;
    delete computehorizonProgram;
    delete computeverticalProgram;
}

////////////////////////////////////////////////////////////////////////
// InitializeScene is called once during setup to create all the
// textures, shape VAOs, and shader programs as well as setting a
// number of other parameters.
void Scene::InitializeScene()
{
    debugDraw = std::make_unique<DebugDrawing>();
    glEnable(GL_DEPTH_TEST);
    CHECKERROR;

    glfwGetFramebufferSize(window, &width, &height);

    // @@ Initialize interactive viewing variables here. (spin, tilt, ry, front back, ...)
    
    // Set initial light parameters
    lightSpin = 150.0;
    lightTilt = -45.0;
    lightDist = 50.0;
    lightCol = { 3.0f, 3.0f, 3.0f };
    // @@ Perhaps initialize additional scene lighting values here. (lightVal, lightAmb)
    
    key = 0;
    nav = false;
    spin = 0.0;
    tilt = 30.0;
    eye = glm::vec3(0.0, -20.0, 0.0);
    speed = 300.0/30.0;
    last_time = glfwGetTime();
    tr = glm::vec3(0.0, 0.0, 25.0);

    ry = 0.4;
    front = 0.5;
    back = 5000.0;

    alpha = 0.0003;
    CHECKERROR;
    objectRoot = new Object("root", NULL, nullId,NULL);

    
    // Enable OpenGL depth-testing
    glEnable(GL_DEPTH_TEST);

    //FBO
    gBuffer.Init(width, height);
    shadowBuffer.CreateFBO(2048, 2048);


    // Create the lighting shader program from source code files.
    // @@ Initialize additional shaders if necessary
    lightingProgram = new ShaderProgram();
    lightingProgram->AddShader("shaders/lightingPhong.vert", GL_VERTEX_SHADER);
    lightingProgram->AddShader("shaders/lightingPhong.frag", GL_FRAGMENT_SHADER);

    glBindAttribLocation(lightingProgram->programId, 0, "vertex");
    glBindAttribLocation(lightingProgram->programId, 1, "vertexNormal");
    glBindAttribLocation(lightingProgram->programId, 2, "vertexTexture");
    glBindAttribLocation(lightingProgram->programId, 3, "vertexTangent");
    lightingProgram->LinkProgram();

    shadowProgram = new ShaderProgram();
    shadowProgram->AddShader("shaders/shadow.vert", GL_VERTEX_SHADER);
    shadowProgram->AddShader("shaders/shadow.frag", GL_FRAGMENT_SHADER);

    glBindAttribLocation(shadowProgram->programId, 0, "vertex");
    glBindAttribLocation(shadowProgram->programId, 1, "vertexNormal");
    glBindAttribLocation(shadowProgram->programId, 2, "vertexTexture");
    glBindAttribLocation(shadowProgram->programId, 3, "vertexTangent");
    shadowProgram->LinkProgram();

    gBufferProgram = new ShaderProgram();
    gBufferProgram->AddShader("shaders/gBuffer.vert", GL_VERTEX_SHADER);
    gBufferProgram->AddShader("shaders/gBuffer.frag", GL_FRAGMENT_SHADER);

    glBindAttribLocation(gBufferProgram->programId, 0, "vertex");
    glBindAttribLocation(gBufferProgram->programId, 1, "vertexNormal");
    glBindAttribLocation(gBufferProgram->programId, 2, "vertexTexture");
    glBindAttribLocation(gBufferProgram->programId, 3, "vertexTangent");
    gBufferProgram->LinkProgram();

    computehorizonProgram = new ShaderProgram();
    computehorizonProgram->AddShader("shaders/blur_horizon.comp", GL_COMPUTE_SHADER);
    computehorizonProgram->LinkProgram();

    computeverticalProgram = new ShaderProgram();
    computeverticalProgram->AddShader("shaders/blur_vertical.comp", GL_COMPUTE_SHADER);
    computeverticalProgram->LinkProgram();
    
    
    g_Physics = std::make_unique<Physics>();
    // Create all the Polygon shapes
    
    boxPolygons = new Box();
    spherePolygons = new Sphere(32);
    Shape* FloorPolygons = new Plane(10.0, 10);
    Shape* QuadPolygons = new Quad();

    // Various colors used in the subsequent models
    glm::vec3 woodColor(87.0/255.0, 51.0/255.0, 35.0/255.0);
    glm::vec3 brickColor(134.0/255.0, 60.0/255.0, 56.0/255.0);
    glm::vec3 floorColor(222.0/255.0, 184.0/255.0, 135.0/255.0);
    glm::vec3 brassColor(0.5, 0.5, 0.1);
    glm::vec3 grassColor(62.0/255.0, 102.0/255.0, 38.0/255.0);
    glm::vec3 waterColor(0.3, 0.3, 1.0);
    glm::vec3 greyColor(224.0/255.0, 224.0 / 255.0, 224.0 / 255.0 );
    glm::vec3 yellowColor(204.0 / 255.0, 204.0 / 255.0, 0);

    glm::vec3 black(0.0, 0.0, 0.0);
    glm::vec3 brightSpec(0.5, 0.5, 0.5);
    glm::vec3 polishedSpec(0.3, 0.3, 0.3);
 
    // @@ To change an object's surface parameters (Kd, Ks, or alpha),
    // modify the following lines.
    
    floor      = new Object("floor", boxPolygons, floorId, boxPolygons, floorColor, black, 1, BoundingType::CONVEX, true);
    //sphere1 = new Object(SpherePolygons, sphere1Id, SpherePolygons, greyColor, brightSpec, 120, BoundingType::SPHERE, true);
    sphere1 = new Object("sphere1", spherePolygons, sphere1Id, spherePolygons, greyColor, brightSpec, 120, BoundingType::SPHERE, true);
    sphere2     = new Object("sphere2", spherePolygons, sphere2Id, spherePolygons, yellowColor, brightSpec, 120, BoundingType::SPHERE, true);
    box = new Object("box", boxPolygons, 0, boxPolygons, greyColor, brightSpec, 120, BoundingType::CONVEX, true);
    //box2 = new Object("box", boxPolygons, 0, boxPolygons, greyColor, brightSpec, 120, BoundingType::CONVEX, true);
    quad = new Object("quad", QuadPolygons, nullId, NULL);

    


    // @@ To change the scene hierarchy, examine the hierarchy created
    // by the following object->add() calls and adjust as you wish.
    // The objects being manipulated and their polygon shapes are
    // created above here.

    //objectRoot->add(central);
#ifndef REFL
    //objectRoot->add(room,  Translate(0.0, 0.0, 0.02));
#endif
    objectRoot->add(floor, glm::vec3(0.0, 0.0, 0.0),glm::vec3(30,30,1.0));
    objectRoot->add(sphere1, glm::vec3(5.0, 0.0, 10.0));
    //objectRoot->add(box, glm::vec3(0.0, 0.0, 7.0), glm::vec3(1,1,1), glm::vec3(0,1,0), 70.f);
    objectRoot->add(box, glm::vec3(0.0, 0.0, 7.0));
    //objectRoot->add(box2, glm::vec3(0.0, 0.0, 10.0));
    objectRoot->add(sphere2, glm::vec3(-5.0, 0.0, 5.0));

    floor->rigidbody.SetDynamic(false);

    g_Physics->AddPhysicsObject(floor);
    g_Physics->AddPhysicsObject(sphere1);
    g_Physics->AddPhysicsObject(box);
    //g_Physics->AddPhysicsObject(box2);
    g_Physics->AddPhysicsObject(sphere2);


    g_Physics->Init();
    lastFrame = (float)glfwGetTime();

    CHECKERROR;

    // Options menu stuff
    show_demo_window = false;
    gBuffer_num = 0;

    glGenVertexArrays(1, &vaoID);
    glGenBuffers(1, &pntBuff);
    glGenBuffers(1, &indBuff);

}

void Scene::DrawMenu()
{
    ImGui_ImplOpenGL3_NewFrame();
    ImGui_ImplGlfw_NewFrame();
    ImGui::NewFrame();

    static bool isDockspaceOpen = true;

    static bool isPPMImagesWindowOpen = true;

    
    ImGui::Begin("Settings", &isPPMImagesWindowOpen);
    ImGui::Text("Frame Per Second : %d ms", static_cast<int>(fps));
    ImGui::Text("Number of Objects : %i", g_Physics->GetDynamicPhysicsObjects().size() + g_Physics->GetStaticPhysicsObjects().size());
    if (ImGui::Button("Play"))
        play = true;
    ImGui::SameLine();
    if (ImGui::Button("Pause"))
        play = false;
    ImGui::SameLine();
    if (ImGui::Button("Step"))
    {
        play = true;
        oneStep = true;
    }
    if (ImGui::Button("Reset"))
    {
        objectRoot->Reset();
        objectRoot->removeStressObjects();
        debugDraw->contactPoints.clear();
        debugDraw->contactIndex.clear();
        g_Physics->RemoveStressObjects();
        g_Physics->ClearCollisionQueue();
    }
    if (ImGui::Button("Remove Stress Test Objects"))
    {
        objectRoot->removeStressObjects();
        debugDraw->contactPoints.clear();
        debugDraw->contactIndex.clear();
        g_Physics->RemoveStressObjects();
        g_Physics->ClearCollisionQueue();
    }
    if (ImGui::Button("Remove All Objects"))
    {
        g_Physics->RemoveAllDynamicObjects();
        objectRoot->removeAll();
        objectRoot->removeStressObjects();
        debugDraw->contactPoints.clear();
        debugDraw->contactIndex.clear();
        g_Physics->RemoveStressObjects();
        g_Physics->ClearCollisionQueue();
    }
    

    ImGui::Checkbox("Stress Test : ", &debugDraw->stressTest);
    ImGui::SameLine();
    if (ImGui::Checkbox("spheres", &debugDraw->stressSphere))
    {
        debugDraw->stressBox = false;
    }ImGui::SameLine();
    if (ImGui::Checkbox("Boxes", &debugDraw->stressBox))
    {
        debugDraw->stressSphere = false;
    }

    //ImGui::Checkbox("Show Dockspace", &isDockspaceOpen);

    if (ImGui::CollapsingHeader("Graphics"))
    {
        if (ImGui::CollapsingHeader("Light"))
        {
            ImGui::SliderFloat("Light Spin", &lightSpin, 0.0f, 360.f);
            ImGui::SliderFloat("Light Tilt", &lightTilt, -90.0f, 90.f);
            ImGui::SliderFloat("Light Dist", &lightDist, 50.0f, 150.f);
        }
        ImGui::Separator();

        if (ImGui::CollapsingHeader("Shadow"))
            ImGui::DragFloat("alpha", &alpha, 0.000001, 0.0000001, 0.005, "%8f");
        ImGui::Separator();

    }

    if (ImGui::CollapsingHeader("Physics"))
    {
        ImGui::Checkbox("Gravity", &g_Physics->m_EnableGravity);
        ImGui::Checkbox("Draw Collider", &debugDraw->colliderDrawing);
        ImGui::Checkbox("Draw Velocity", &debugDraw->velocityDrawing);
        ImGui::Checkbox("Draw Tree", &debugDraw->treeDrawing);
        ImGui::Checkbox("Draw Contact Points", &debugDraw->contactDraw);
        ImGui::SliderFloat("Bias Factor", &debugDraw->biasFactor, 0.0f, 1.0f);
        ImGui::SliderInt("Velocity Solver Iterations", &g_Physics->m_velocitySolveIt, 1, 200);
        ImGui::Separator();

        if (ImGui::Button("Save Scene"))
        {
            for (int i = 0; i < objectRoot->instances.size(); ++i)
            {
                auto curr_obj = objectRoot->instances[i].first;
                curr_obj->initial.m_position = curr_obj->rigidbody.m_collider->m_position;
                curr_obj->initial.m_scale = curr_obj->rigidbody.m_collider->m_scale;
                curr_obj->initial.m_rotation = curr_obj->rigidbody.m_collider->m_rotation;
                curr_obj->initial.m_objTr = curr_obj->rigidbody.m_collider->m_objTr;
            }
        }
        if (ImGui::CollapsingHeader("Objects"))
        {
            std::string name = "objectNum" + std::to_string(debugDraw->objectNum);
            ImGui::InputText("Name", const_cast<char*>(name.c_str()), 20); 

            if (ImGui::Checkbox("Box", &debugDraw->isBox))
                debugDraw->isSphere = !debugDraw->isBox;
            ImGui::SameLine();
            if (ImGui::Checkbox("Sphere", &debugDraw->isSphere))
                debugDraw->isBox = !debugDraw->isSphere;

            if (ImGui::Button("Add \Object"))
            {
                Shape* newShape = boxPolygons;
                BoundingType newBounding = BoundingType::CONVEX;

                if (debugDraw->isSphere)
                {
                    newShape = spherePolygons;
                    newBounding = BoundingType::SPHERE;
                }

                debugDraw->objectNum++;
                glm::vec3 p = { 0, 0, 15.f };
                Object* newobj = new Object(name.c_str(), newShape, 0, newShape, { 0.8,0.8,0.8 }, { 0.5,0.5,0.5 }, 120, newBounding, true);
                objectRoot->add(newobj, p);
                g_Physics->AddPhysicsObject(newobj);
            }

            ImGui::Separator();
            
            for (int i = 0; i < objectRoot->instances.size(); ++i)
            {
                std::string curr_name = objectRoot->instances[i].first->name;
                if (curr_name == "stressObject")
                    continue;
                if (ImGui::CollapsingHeader(curr_name.c_str()))
                {
                    ImGui::Text("Position");
                    auto curr_obj = objectRoot->instances[i].first;
                    bool change = false;
                    if (ImGui::DragFloat("pos x", &curr_obj->rigidbody.m_collider->m_position.x, 0.1f, -50.f, 50.f, "%.1f"))
                        change = true;
                    if (ImGui::DragFloat("pos y", &curr_obj->rigidbody.m_collider->m_position.y, 0.1f, -50.f, 50.f, "%.1f"))
                        change = true;
                    if (ImGui::DragFloat("pos z", &curr_obj->rigidbody.m_collider->m_position.z, 0.1f, -50.f, 50.f, "%.1f"))
                        change = true;

                    ImGui::Text("Rotation");
                    glm::vec3 rot = glm::degrees(glm::eulerAngles(curr_obj->rigidbody.m_collider->m_rotation));
                    if (ImGui::DragFloat("rot x", &rot.x, 0.1f, -180.f, 180.f, "%.1f"))
                        change = true;
                    if (ImGui::DragFloat("rot y", &rot.y, 0.1f, -180.f, 180.f, "%.1f"))
                        change = true;
                    if (ImGui::DragFloat("rot z", &rot.z, 0.1f, -180.f, 180.f, "%.1f"))
                        change = true;

                    ImGui::Text("Scale");
                    if (ImGui::DragFloat("scale x", &curr_obj->rigidbody.m_collider->m_scale.x, 0.1f, 1.f, 30.f, "%.1f"))
                        change = true;
                    if (ImGui::DragFloat("scale y", &curr_obj->rigidbody.m_collider->m_scale.y, 0.1f, 1.f, 30.f, "%.1f"))
                        change = true;
                    if (ImGui::DragFloat("scale z", &curr_obj->rigidbody.m_collider->m_scale.z, 0.1f, 1.f, 30.f, "%.1f"))
                        change = true;

                    if (change)
                    {
                        curr_obj->rigidbody.m_collider->m_aabb.m_upper = curr_obj->rigidbody.m_collider->m_scale;
                        curr_obj->rigidbody.m_collider->m_aabb.m_lower = -curr_obj->rigidbody.m_collider->m_scale;
                        curr_obj->rigidbody.m_collider->m_rotation = glm::quat(glm::radians(rot));
                        objectRoot->instances[i].first->rigidbody.m_collider->m_objTr = Translate(curr_obj->rigidbody.m_collider->m_position.x, curr_obj->rigidbody.m_collider->m_position.y, curr_obj->rigidbody.m_collider->m_position.z)
                            * glm::toMat4(curr_obj->rigidbody.m_collider->m_rotation)
                            * Scale(curr_obj->rigidbody.m_collider->m_scale.x, curr_obj->rigidbody.m_collider->m_scale.y, curr_obj->rigidbody.m_collider->m_scale.z);
                        g_Physics->tree->Remove(g_Physics->tree->FindIndex(&curr_obj->rigidbody));
                        g_Physics->tree->Insert(&curr_obj->rigidbody);
                        
                        objectRoot->instances[i].first->rigidbody.m_collider->UpdateAABB();
                        g_Physics->tree->Update();
                    }

                    if (ImGui::Button("Remove"))
                    {
                        objectRoot->remove(curr_name);
                        g_Physics->RemovePhysicsObject(curr_obj);
                    }
                }
            }
        }

        
    }

    ImGui::End();

    ImGui::Render();

    ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());
}

void Scene::BuildTransforms()
{
    
    // Work out the eye position as the user move it with the WASD keys.
    float now = glfwGetTime();
    float dist = (now-last_time)*speed;
    last_time = now;
    if (key == GLFW_KEY_W)
        eye += dist*glm::vec3(sin(spin*rad), cos(spin*rad), 0.0);
    if (key == GLFW_KEY_S)
        eye -= dist*glm::vec3(sin(spin*rad), cos(spin*rad), 0.0);
    if (key == GLFW_KEY_D)
        eye += dist*glm::vec3(cos(spin*rad), -sin(spin*rad), 0.0);
    if (key == GLFW_KEY_A)
        eye -= dist*glm::vec3(cos(spin*rad), -sin(spin*rad), 0.0);

    //eye[2] = proceduralground->HeightAt(eye[0], eye[1]) + 2.0;

    CHECKERROR;

    if (nav)
        WorldView = Rotate(0, tilt-90)*Rotate(2, spin) *Translate(-eye[0], -eye[1], -eye[2]);
    else
        WorldView = Translate(tr[0], tr[1], -tr[2]) *Rotate(0, tilt-90)*Rotate(2, spin);
    WorldProj = Perspective((ry*width)/height, ry, front, (mode==0) ? 1000 : back);

    // @@ Print the two matrices (in column-major order) for
    // comparison with the project document.
    //std::cout << "WorldView: " << glm::to_string(WorldView) << std::endl;
    //std::cout << "WorldProj: " << glm::to_string(WorldProj) << std::endl;
}

////////////////////////////////////////////////////////////////////////
// Procedure DrawScene is called whenever the scene needs to be
// drawn. (Which is often: 30 to 60 times per second are the common
// goals.)
void Scene::DrawScene()
{
    // Set the viewport
    glfwGetFramebufferSize(window, &width, &height);
    glViewport(0, 0, width, height);

    CHECKERROR;

    //////////////////UPDATE/////////////////////////////
    float currFrame = (float)glfwGetTime();
    deltaTime = currFrame - lastFrame;
    lastFrame = currFrame;

    frameCount++;
    timeElapesed += deltaTime;
    if (timeElapesed > -1.0f)
    {
        fps = (float)frameCount / timeElapesed;
        timeElapesed = 0.0f;
        frameCount = 0.0f;
    }
    fps = 1.0f/deltaTime;
    if (deltaTime <= FRAME_LIMIT && play)
    {
        if (static_cast<int>(1.0 / deltaTime) < 30)
            debugDraw->stressTest = false;

        if (debugDraw->stressTest)
        {
            glm::vec3 p = { rand() % 60 - 30.f, rand() % 60 - 30.f, rand() % 30 + 10.f };
            Object* newobj;
            if(debugDraw->stressSphere)
                newobj = new Object("stressObject", spherePolygons, 0, spherePolygons, { 0.8,0.8,0.8 }, { 0.5,0.5,0.5 }, 120, BoundingType::SPHERE, true);
            else if(debugDraw->stressBox)
                newobj = new Object("stressObject", boxPolygons, 0, boxPolygons, { 0.8,0.8,0.8 }, { 0.5,0.5,0.5 }, 120, BoundingType::CONVEX, true);
            objectRoot->add(newobj, p);
            g_Physics->AddPhysicsObject(newobj);
        }

        debugDraw->contactPoints.clear();
        debugDraw->contactIndex.clear();
        g_Physics->Update(deltaTime);
        if (oneStep)
        {
            play = false;
            oneStep = false;
        }
    }
    //////////////////////////////////////////////////////////////


    // Calculate the light's position from lightSpin, lightTilt, lightDist
    lightPos = glm::vec3(lightDist * cos(lightSpin * rad) * sin(lightTilt * rad),
        lightDist * sin(lightSpin * rad) * sin(lightTilt * rad),
        lightDist * cos(lightTilt * rad));
    lightDir = -glm::normalize(lightPos);
    lightView = glm::lookAt(lightPos, lightDir, glm::vec3(0, 1, 0));
    lightProj = glm::perspective(glm::radians(90.f), 1.f, front, 100.f);
    shadowMatrix = Translate(0.5f, 0.5f, 0.5f) * Scale(0.5f, 0.5f, 0.5f) * lightProj * lightView;

    BuildTransforms();

    // The lighting algorithm needs the inverse of the WorldView matrix
    WorldInverse = glm::inverse(WorldView);

    ////////////////////////////////////////////////////////////////////////////////
    // Anatomy of a pass:
    //   Choose a shader  (create the shader in InitializeScene above)
    //   Choose and FBO/Render-Target (if needed; create the FBO in InitializeScene above)
    //   Set the viewport (to the pixel size of the screen or FBO)
    //   Clear the screen.
    //   Set the uniform variables required by the shader
    //   Draw the geometry
    //   Unset the FBO (if one was used)
    //   Unset the shader
    ////////////////////////////////////////////////////////////////////////////////

    CHECKERROR;
    int loc, programId;

    gBufferProgram->UseShader();
    programId = gBufferProgram->programId;

    gBuffer.BindFBO();
    GLenum DrawBuffers[] = { GL_COLOR_ATTACHMENT0, GL_COLOR_ATTACHMENT1, GL_COLOR_ATTACHMENT2, GL_COLOR_ATTACHMENT3 };
    glDrawBuffers(4, DrawBuffers);

    glViewport(0, 0, width, height);
    //glClearColor(0, 0, 0, 1.0);
    glClearColor(225.f/255.f, 246.f / 255.f, 255.f / 255.f, 1.0);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    loc = glGetUniformLocation(programId, "WorldProj");
    glUniformMatrix4fv(loc, 1, GL_FALSE, Pntr(WorldProj));
    loc = glGetUniformLocation(programId, "WorldView");
    glUniformMatrix4fv(loc, 1, GL_FALSE, Pntr(WorldView));


    CHECKERROR;

    objectRoot->Draw(gBufferProgram);

    if (debugDraw->treeDrawing)
        g_Physics->tree->Draw(gBufferProgram->programId);

    if (debugDraw->contactDraw && debugDraw->contactPoints.size())
        DrawContactPoints(gBufferProgram);

    gBuffer.UnbindFBO();
    gBufferProgram->UnuseShader();


    ////////////////////////////////////////////////////////////////////////////////
    // Lighting pass
    ////////////////////////////////////////////////////////////////////////////////
    
    // Choose the lighting shader
    lightingProgram->UseShader();
    programId = lightingProgram->programId;

    // Set the viewport, and clear the screen
    glViewport(0, 0, width, height);
    //glClearColor(0, 0, 0, 1.0);
    glClearColor(225.f / 255.f, 246.f / 255.f, 255.f / 255.f, 1.0);
    glClear(GL_COLOR_BUFFER_BIT| GL_DEPTH_BUFFER_BIT);
    
    loc = glGetUniformLocation(programId, "WorldProj");
    glUniformMatrix4fv(loc, 1, GL_FALSE, Pntr(WorldProj));
    loc = glGetUniformLocation(programId, "WorldView");
    glUniformMatrix4fv(loc, 1, GL_FALSE, Pntr(WorldView));
    loc = glGetUniformLocation(programId, "WorldInverse");
    glUniformMatrix4fv(loc, 1, GL_FALSE, Pntr(WorldInverse));
    loc = glGetUniformLocation(programId, "shadowMatrix");
    glUniformMatrix4fv(loc, 1, GL_FALSE, Pntr(shadowMatrix));
    loc = glGetUniformLocation(programId, "lightPos");
    glUniform3fv(loc, 1, &(lightPos[0]));   
    loc = glGetUniformLocation(programId, "lightCol");
    glUniform3fv(loc, 1, &(lightCol[0]));
    loc = glGetUniformLocation(programId, "mode");
    glUniform1i(loc, mode);
    loc = glGetUniformLocation(programId, "width");
    glUniform1i(loc, width);
    loc = glGetUniformLocation(programId, "height");
    glUniform1i(loc, height);
    loc = glGetUniformLocation(programId, "lightPos");
    glUniform3fv(loc, 1, &(lightPos[0]));
    loc = glGetUniformLocation(programId, "eyePos");
    glUniform3fv(loc, 1, &(eye[0]));
    loc = glGetUniformLocation(programId, "gBuffer_num");
    glUniform1i(loc, gBuffer_num);
    loc = glGetUniformLocation(programId, "alpha");
    glUniform1f(loc, alpha);
    CHECKERROR;

    gBuffer.BindTexture(0, lightingProgram->programId, "gBufferWorldPos", 0);
    gBuffer.BindTexture(1, lightingProgram->programId, "gBufferNormal", 1);
    gBuffer.BindTexture(2, lightingProgram->programId, "gBufferDiffuse", 2);
    gBuffer.BindTexture(3, lightingProgram->programId, "gBufferSpecular", 3);
    shadowBuffer.BindTexture(4, lightingProgram->programId, "shadowMap");
    CHECKERROR;

    quad->Draw(lightingProgram);
    CHECKERROR; 

    
    // Turn off the shader
    lightingProgram->UnuseShader();

    ////////////////////////////////////////////////////////////////////////////////
    // End of Lighting pass
    ////////////////////////////////////////////////////////////////////////////////

}

void Scene::DrawContactPoints(ShaderProgram* program)
{
    glm::vec3 color(1.0f, 1.0f, 0.0f);
    int loc = glGetUniformLocation(program->programId, "lineColor");
    glUniform3fv(loc, 1, &color[0]);

    loc = glGetUniformLocation(program->programId, "diffuse");
    glUniform3fv(loc, 1, &color[0]);

    loc = glGetUniformLocation(program->programId, "mode");
    glUniform1i(loc, 1);

    loc = glGetUniformLocation(program->programId, "ModelTr");
    glUniformMatrix4fv(loc, 1, GL_FALSE, Pntr(glm::mat4()));

    loc = glGetUniformLocation(program->programId, "NormalTr");
    glUniformMatrix4fv(loc, 1, GL_FALSE, Pntr(glm::mat4()));

    glBindVertexArray(vaoID);

    glBindBuffer(GL_ARRAY_BUFFER, pntBuff);
    glBufferData(GL_ARRAY_BUFFER, sizeof(float) * 4 * debugDraw->contactPoints.size(),
        &debugDraw->contactPoints[0][0], GL_STATIC_DRAW);
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(0, 4, GL_FLOAT, GL_FALSE, 0, 0);
    glBindBuffer(GL_ARRAY_BUFFER, 0);

    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, indBuff);
    glBufferData(GL_ELEMENT_ARRAY_BUFFER, sizeof(int) * debugDraw->contactIndex.size(),
        &debugDraw->contactIndex[0], GL_STATIC_DRAW);
    
    glPointSize(10.0f);
    glDrawElements(GL_POINTS, debugDraw->contactPoints.size(), GL_UNSIGNED_INT, 0);
    glBindBuffer(GL_ARRAY_BUFFER, 0);
    glBindVertexArray(0);

}

