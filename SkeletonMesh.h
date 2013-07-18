#include <vector>
#include <list>
#include <map>
#include <GL/glew.h>
#include <GL/gl.h>
#include <GL/glut.h>
#include <GL/glew.h>
#include "Transformation.h"
#include "matrix.h"
#include "Skeleton.h"
#include "shader.h"

class Frame;
class SkeletonMesh 
{
public:
   SkeletonMesh();
   virtual ~SkeletonMesh();
   virtual void load(const char* filename, const char* weights, const char* bindPoseName);
   virtual void load(const std::vector<const char*>& filename, 
                     const std::vector<const char*>& weights, 
                     const char* bindPoseName);
   virtual void draw();
   virtual bool getBoundingBox(vec3& mmin, vec3& mmax);  
   virtual void updateSkin(const Skeleton& skeleton);
   virtual void setupSkin(const Skeleton& skeleton);
   virtual void setColor(const vec3& color);
   virtual void setPose(const Frame& frame, const Skeleton& skeleton);
   virtual const Skeleton& getSkeleton() const { return mSkeleton; }

public:
   vec3 Translation;
   vec3 Rotation;
   vec3 Scale;
   Shader SkinShader;

protected:

   virtual void clear();
   virtual void initSkeleton(const char* bindPoseFile);
   virtual void initGeometry();
   virtual void load (const char* filename); // called from ctor
   virtual void parse (char* line);
   virtual void parseVertex(char* line);
   virtual void parseUV(char* line);
   virtual void parseNormal(char* line);
   virtual void parseFace(char* line);

   virtual void loadSkinWeights(const char* filename);
   virtual void drawGeometry();

   // Get the local-to-world model matrix 
   const math::matrix<double>& getLocalToWorld() const;
   const math::matrix<double>& getWorldToLocal() const;

protected:
   
   Skeleton mSkeleton;
   vec3 mColor;
   vec3 myMin, myMax; 
   mutable math::matrix<double> myModelMatrix;

   struct Vertex { GLuint pos; GLuint normal; GLuint uv; };

   typedef std::vector<Vertex> Face; 
   typedef std::list<Face> FaceList;
   typedef std::list<Face>::const_iterator FaceIt;
   FaceList myQuads;
   FaceList myTris;
   FaceList myPolys;
   int myOffset;

   // Geometry from file
   std::vector<vec3> myVertices;
   std::vector<vec2> myUvs;
   std::vector<vec3> myNormals;
   std::vector<std::vector<GLint>> myVertexJointIndices; // for each vertex, list joint indices
   std::vector<std::vector<float>> myVertexWeights; // for each vertex, list joint weights

   // Geometry for drawing
   GLfloat* quadns;
   GLfloat* quadvs;
   GLfloat* quadws;
   GLint* quadinds;

   GLfloat* trins;
   GLfloat* trivs;
   GLfloat* triws;
   GLint* triinds;

   // Shader parameters for skeleton
   std::vector<Transform> myBindPose_Global2Local;  // for each joint, cache world to rest local
   std::vector<Transform> myAnimPose_Local2Global;  // for each joint, cache world to rest local
   GLfloat* global2local;
   GLfloat* local2global;

public:
    static void LoadTurtle(SkeletonMesh& model);
    static void LoadBear(SkeletonMesh& model);
};

