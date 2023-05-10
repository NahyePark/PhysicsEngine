////////////////////////////////////////////////////////////////////////
// A lightweight class representing an instance of an object that can
// be drawn onscreen.  An Object consists of a shape (batch of
// triangles), and various transformation, color and texture
// parameters.  It also contains a list of child objects to be drawn
// in a hierarchical fashion under the control of parent's
// transformations.
//
// Methods consist of a constructor, and a Draw procedure, and an
// append for building hierarchies of objects.

#ifndef _OBJECT
#define _OBJECT

#include "texture.h"
#include "RigidBody.h"

#include <utility>              // for pair<Object*,glm::mat4>

class ShaderProgram;
class Object;

typedef std::pair<Object*,glm::mat4> INSTANCE;


struct DebugDrawing
{
    bool colliderDrawing = false;
    bool velocityDrawing = false;
    bool treeDrawing = false;
    bool stressTest = false;
    bool stressSphere = false;
    bool stressBox = true;
    bool contactDraw = true;
    bool isBox = true;
    bool isSphere = false;
    int objectNum = 1;
    char buff[20] = "object";
    float biasFactor = 0.1f;

    std::vector<glm::vec4> contactPoints;
    std::vector<int> contactIndex;
};

extern std::unique_ptr<DebugDrawing> debugDraw;

// Object:: A shape, and its transformations, colors, and textures and sub-objects.
class Object 
{
 public:
    Shape* shape;               // Polygons 
    glm::mat4 animTr;                // This model's animation transformation
    int objectId;               // Object id to be sent to the shader
    bool drawMe;                // Toggle specifies if this object (and children) are drawn.
    std::string name;

    glm::vec3 diffuseColor;          // Diffuse color of object
    glm::vec3 specularColor;         // Specular color of object
    float shininess;            // Surface roughness value

    std::vector<INSTANCE> instances; // Pairs of sub-objects and transformations
    Trans initial;
    
    //Physics
    RigidBody rigidbody;
    bool hasRigidBody;
    Line linearVelocity;
    Line angularVelocity;

    Object(const char* _name, Shape* _shape, const int objectId, Shape* boundingShape = NULL,
           const glm::vec3 _d=glm::vec3(), const glm::vec3 _s=glm::vec3(), const float _n=1, BoundingType bt = BoundingType::NUMSHAPES, bool rb = false);


    // If this object is to be drawn with a texture, this is a good
    // place to store the texture id (a small positive integer).  The
    // texture id should be set in Scene::InitializeScene and used in
    // Object::Draw.
    
    void Draw(ShaderProgram* program);

    void Reset()
    {
        if (hasRigidBody)
        {
            rigidbody.m_collider->ResetTrans(initial);
            rigidbody.Reset();
            if(rigidbody.m_collider->m_type == BoundingType::BOX)
                std::static_pointer_cast<OBBCollider>(rigidbody.m_collider)->OBBResetCollider(glm::toMat4(rigidbody.m_collider->m_rotation));
            else if (rigidbody.m_collider->m_type == BoundingType::CONVEX)
                std::static_pointer_cast<ConvexCollider>(rigidbody.m_collider)->ConvexResetCollider(glm::toMat4(rigidbody.m_collider->m_rotation));
            else if (rigidbody.m_collider->m_type == BoundingType::SPHERE)
                std::static_pointer_cast<SphereCollider>(rigidbody.m_collider)->SphereResetCollider(glm::toMat4(rigidbody.m_collider->m_rotation));
        }
        for (int i = 0; i < instances.size(); i++) {
            instances[i].first->Reset();
        }
    }
    //void add(Object* m, glm::mat4 tr=glm::mat4()) { instances.push_back(std::make_pair(m,tr)); }

    void add(Object* m, glm::vec3 trans = glm::vec3(0,0,0), glm::vec3 scale = glm::vec3(1,1,1), glm::vec3 rotAxis = glm::vec3(0, 0, 0), float angle = 0)
    { 
        if (m->shape != nullptr)
        {
            m->rigidbody.m_collider->m_position = trans;
            m->rigidbody.m_collider->m_rotation = glm::angleAxis(angle, rotAxis);
            m->rigidbody.m_collider->m_scale = scale;
            m->rigidbody.m_collider->m_aabb.m_lower = m->shape->minP * scale + trans;
            m->rigidbody.m_collider->m_aabb.m_upper = m->shape->maxP * scale + trans;
            glm::mat4 rot = glm::toMat4(m->rigidbody.m_collider->m_rotation);
            m->rigidbody.m_collider->m_objTr = Translate(m->rigidbody.m_collider->m_position.x, m->rigidbody.m_collider->m_position.y, m->rigidbody.m_collider->m_position.z)
                * rot
                * Scale(m->rigidbody.m_collider->m_scale.x, m->rigidbody.m_collider->m_scale.y, m->rigidbody.m_collider->m_scale.z);
            //if (rigidbody.m_collider->m_type == BoundingType::BOX)
            //{
            //    rigidbody.m_collider = std::make_shared<OBBCollider>();
            //    std::dynamic_pointer_cast<OBBCollider>(m->rigidbody.m_collider)->OBBResetCollider(glm::toMat4(m->rigidbody.m_collider->m_rotation));
            //}
            //else if (rigidbody.m_collider->m_type == BoundingType::CONVEX)
            //{
             //   std::dynamic_pointer_cast<ConvexCollider>(m->rigidbody.m_collider)->ConvexResetCollider(glm::toMat4(m->rigidbody.m_collider->m_rotation));
            //}
            //else if (rigidbody.m_collider->m_type == BoundingType::SPHERE)
            //{
            //    std::dynamic_pointer_cast<SphereCollider>(m->rigidbody.m_collider)->SphereResetCollider(glm::toMat4(m->rigidbody.m_collider->m_rotation));
            //
            //}
            m->rigidbody.Initialize();
            m->initial = Trans(trans, scale, rotAxis, angle);
            m->initial.m_objTr = m->rigidbody.m_collider->m_objTr;
        }
        instances.push_back(std::make_pair(m, m->rigidbody.m_collider->m_objTr));

    }

    void removeStressObjects()
    {
        auto it = std::find_if(instances.begin(), instances.end(), [](const INSTANCE& a)
            {
                return a.first->name == "stressObject";
            });

        instances.erase(it, instances.end());
    }

    void remove(std::string& name)
    {
        auto it = std::find_if(instances.begin(), instances.end(), [&name](const INSTANCE& a)
            {
                return a.first->name == name;
            });

        delete it->first;
        instances.erase(it);
    }

    void removeAll()
    {
        for (int i = instances.size()-1; i >= 0; --i)
        {
            if(instances[i].first->name != "floor")
                remove(instances[i].first->name);
        }
    }
};

#endif
