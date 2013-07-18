
#include "SkeletonMesh.h"
#include "stdio.h"
#include "stdlib.h"
#include <iostream>
#include <fstream>
#include <string>
#include <algorithm>
#include <vector>
#include "Frame.h"
#include "Player.h"

#define MAX_JOINTS 30
#define SKIN_DIR "../Mesh/"
#define INCH_2_CM 2.5

SkeletonMesh::SkeletonMesh() 
{
   quadvs = 0;
   quadns = 0;
   quadws = 0;
   quadinds = 0;

   trivs = 0;
   trins = 0;
   triws = 0;
   triinds = 0;

   local2global = 0;
   global2local = 0;

   Translation.set(0,0,0);
   Rotation.set(0,0,0);
   Scale.set(1,1,1);
   mColor.set(0.8,0.8,0.8);
}

SkeletonMesh::~SkeletonMesh()
{
   delete[] quadvs;
   delete[] quadns;
   delete[] quadws;
   delete[] quadinds;

   delete[] trivs;
   delete[] trins;
   delete[] triws;
   delete[] triinds;

   delete[] global2local;
   delete[] local2global;
}

void SkeletonMesh::setColor(const vec3& color)
{
   mColor = color;
}

void ToGLMatrix(Transform t, float* pData)
{
	pData[0] = t.m_rotation[0][0]; pData[4] = t.m_rotation[0][1]; pData[8]  = t.m_rotation[0][2]; pData[12] = t.m_translation[0];
	pData[1] = t.m_rotation[1][0]; pData[5] = t.m_rotation[1][1]; pData[9]  = t.m_rotation[1][2]; pData[13] = t.m_translation[1];
	pData[2] = t.m_rotation[2][0]; pData[6] = t.m_rotation[2][1]; pData[10] = t.m_rotation[2][2]; pData[14] = t.m_translation[2];
	pData[3] = 0.0f;			 pData[7] = 0.0f;			  pData[11] = 0.0f;				pData[15] = 1.0f;
}

void SkeletonMesh::clear()
{
   myMin.set(9999999999.0, 9999999999.0, 9999999999.0);
   myMax.set(-9999999999.0, -9999999999.0, -9999999999.0);
   myVertexJointIndices.clear();
   myVertexWeights.clear(); 
   myQuads.clear(); 
   myTris.clear(); 
   myPolys.clear(); 
   myVertices.clear(); 
   myUvs.clear(); 
   myNormals.clear();   
}

void SkeletonMesh::load(const char* filename, const char* weights, const char* bindPoseName)
{   
   clear();

   initSkeleton(bindPoseName);
   load(filename);
   loadSkinWeights(weights);
   initGeometry();
   setupSkin(mSkeleton);
}

void SkeletonMesh::load(const std::vector<const char*>& filenames, 
                 const std::vector<const char*>& weights, 
                 const char* bindPoseName)
{
   clear();

   initSkeleton(bindPoseName);
   for (unsigned int i = 0; i < filenames.size(); i++)
   {
       load(filenames[i]);
       loadSkinWeights(weights[i]);
   }
   initGeometry();
   setupSkin(mSkeleton);
}

// Initialize bone weigthts, bindPose_world2local, jointIndices
void SkeletonMesh::setupSkin(const Skeleton& skeleton)
{
    SkinShader.init("skin.vert", "skin.frag");

    global2local = new GLfloat[MAX_JOINTS*16];
    memset(global2local, 0, sizeof(GLfloat)*MAX_JOINTS*16);

    local2global = new GLfloat[MAX_JOINTS*16];
    memset(local2global, 0, sizeof(GLfloat)*MAX_JOINTS*16);

    assert(skeleton.GetNumJoints() < MAX_JOINTS);

    int index = 0;
    myBindPose_Global2Local.clear();
    myAnimPose_Local2Global.clear();
    for (unsigned int j = 0; j < skeleton.GetNumJoints(); j++)
    {
        Joint* joint = skeleton.GetJointByID(j);

        Transform jglobal2local = joint->GetGlobalTransform().Inverse();
        Transform jlocal2global = joint->GetGlobalTransform();
        myBindPose_Global2Local.push_back(jglobal2local);
        myAnimPose_Local2Global.push_back(jlocal2global);

       GLfloat matg2l[16];
       GLfloat matl2g[16];
       // Transform identity(vec3(0,0,0), identity3D);
       ToGLMatrix(jglobal2local, matg2l);
       ToGLMatrix(jlocal2global, matl2g);
       for (unsigned int k = 0; k < 16; k++) 
       {
           global2local[index] = matg2l[k];
           local2global[index] = matl2g[k];
           index++;
       }
    }



   // Allocate attribute arrays
   quadinds = new GLint[myQuads.size()*4*4];
   quadws = new GLfloat[myQuads.size()*4*4];

   index = 0;
   for (FaceIt it = myQuads.begin(); it != myQuads.end(); ++it)
   {
     Face face = *it;
     for (unsigned int i = 0; i < face.size(); i++)
     {
        Vertex v = face[i];

        quadinds[index+0] = myVertexJointIndices[v.pos][0];
        quadinds[index+1] = myVertexJointIndices[v.pos][1];
        quadinds[index+2] = myVertexJointIndices[v.pos][2];
        quadinds[index+3] = myVertexJointIndices[v.pos][3];

        quadws[index+0] = myVertexWeights[v.pos][0];
        quadws[index+1] = myVertexWeights[v.pos][1];
        quadws[index+2] = myVertexWeights[v.pos][2];
        quadws[index+3] = myVertexWeights[v.pos][3];

        index += 4;
     }
   }

   triinds = new GLint[myTris.size()*3*4];
   triws = new GLfloat[myTris.size()*3*4];

   index = 0;
   for (FaceIt it = myTris.begin(); it != myTris.end(); ++it)
   {
     Face face = *it;
     for (unsigned int i = 0; i < face.size(); i++)
     {
        Vertex v = face[i];

        triinds[index+0] = myVertexJointIndices[v.pos][0];
        triinds[index+1] = myVertexJointIndices[v.pos][1];
        triinds[index+2] = myVertexJointIndices[v.pos][2];
        triinds[index+3] = myVertexJointIndices[v.pos][3];

        triws[index+0] = myVertexWeights[v.pos][0];
        triws[index+1] = myVertexWeights[v.pos][1];
        triws[index+2] = myVertexWeights[v.pos][2];
        triws[index+3] = myVertexWeights[v.pos][3];

        index += 4;
     }
   }
}

void SkeletonMesh::updateSkin(const Skeleton& skeleton)
{
    myAnimPose_Local2Global.clear();
    int index = 0;
    for (unsigned int j = 0; j < skeleton.GetNumJoints(); j++)
    {
        Joint* joint = skeleton.GetJointByID(j);
        Transform jlocal2global = joint->GetGlobalTransform();
        myAnimPose_Local2Global.push_back(jlocal2global);

        GLfloat mat[16];
        ToGLMatrix(jlocal2global, mat);
        for (unsigned int k = 0; k < 16; k++) local2global[index++] = mat[k];
    }    
}

void SkeletonMesh::draw()
{
    SkinShader.bind();

    GLint global2localId = glGetUniformLocation(SkinShader.id(), "bindPose_world2local");
    glUniformMatrix4fv(global2localId, MAX_JOINTS, 0, global2local); // row major order

    GLint local2globalId = glGetUniformLocation(SkinShader.id(), "animPose_local2world");
    glUniformMatrix4fv(local2globalId, MAX_JOINTS, 0, local2global); // row major order

    drawGeometry();
    SkinShader.unbind();
/*
    glDisable(GL_LIGHTING);
    glBegin(GL_LINES);
    glColor3f(0.0, 1.0, 0.0);
    for (FaceIt it = myQuads.begin(); it != myQuads.end(); ++it)
    {
         Face face = *it;
         for (unsigned int i = 0; i < face.size(); i++)
         {
        Vertex v = face[i];

        vec3 normal = myNormals[v.normal]*2;
        vec3 vertex = myVertices[v.pos];
        glVertex3f(vertex[0], vertex[1], vertex[2]);
        glVertex3f(vertex[0]+normal[0], vertex[1]+normal[1], vertex[2]+normal[2]);
        }
    }
    glEnd();
    glEnable(GL_LIGHTING);
    */

}

void SkeletonMesh::drawGeometry()
{
    static GLfloat ambient[4] = {0,0,0,0.5};
    static GLfloat diffuse[4] = {0.8, 0.8, 0.8, 0.5};
    static GLfloat specular[4] = {0.0,0.0,0.0,0.5};

    diffuse[0] = mColor[0];
    diffuse[1] = mColor[1];
    diffuse[2] = mColor[2];

	glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT, ambient);
	glMaterialfv(GL_FRONT_AND_BACK, GL_DIFFUSE, diffuse);
    glMaterialfv(GL_FRONT_AND_BACK, GL_SPECULAR, specular);

    glEnable(GL_LIGHTING);

    glEnableClientState(GL_NORMAL_ARRAY);
    glEnableClientState(GL_VERTEX_ARRAY);

    GLint weightId = glGetAttribLocation(SkinShader.id(), "weights");
	glEnableVertexAttribArray(weightId);

    GLint indicesId = glGetAttribLocation(SkinShader.id(), "indices");
	glEnableVertexAttribArray(indicesId);

    glPushMatrix();

    glNormalPointer(GL_FLOAT, 0, quadns);
	glVertexAttribPointer(weightId, 4, GL_FLOAT, 0, 0, quadws);
    glVertexAttribPointer(indicesId, 4, GL_FLOAT, 0, 0, quadinds);
    glVertexPointer(3, GL_FLOAT, 0, quadvs);
    glDrawArrays(GL_QUADS, 0, myQuads.size()*4);

    glNormalPointer(GL_FLOAT, 0, trins);
    glVertexPointer(3, GL_FLOAT, 0, trivs);
	glVertexAttribPointer(weightId, 4, GL_FLOAT, 0, 0, triws);
    glVertexAttribPointer(indicesId, 4, GL_FLOAT, 0, 0, triinds);
    glDrawArrays(GL_TRIANGLES, 0, myTris.size()*3);

    glDisableClientState(GL_VERTEX_ARRAY);  // disable vertex arrays
    glDisableClientState(GL_NORMAL_ARRAY);
	glDisableVertexAttribArray(weightId);
	glDisableVertexAttribArray(indicesId);

    for (FaceIt it = myPolys.begin(); it != myPolys.end(); ++it)
    {
     Face face = *it;
     glBegin(GL_POLYGON);
     for (unsigned int i = 0; i < face.size(); i++)
     {
        Vertex v = face[i];

        vec3 normal = myNormals[v.normal];
        glNormal3f(normal[0], normal[1], normal[2]);

        std::vector<float> weights = myVertexWeights[v.pos];
	    glVertexAttrib4f(weightId, weights[0], weights[1], weights[2], weights[3]);
            
        std::vector<int> indices = myVertexJointIndices[v.pos];
	    glVertexAttribI4i(indicesId, indices[0], indices[1], indices[2], indices[3]);        

        vec3 vertex = myVertices[v.pos];
        glVertex3f(vertex[0], vertex[1], vertex[2]);
     }
     glEnd();
    }

    glPopMatrix();
}

void SkeletonMesh::initGeometry()
{
    quadvs = new GLfloat[myQuads.size()*4*3];
    quadns = new GLfloat[myQuads.size()*4*3];

    unsigned int indexn = 0;
    unsigned int indexv = 0;
    for (FaceIt it = myQuads.begin(); it != myQuads.end(); ++it)
    {
     Face face = *it;
     for (unsigned int i = 0; i < face.size(); i++)
     {
        Vertex v = face[i];
        if (myNormals.size() > 0)
        {
            vec3 normal = myNormals[v.normal];
            quadns[indexn++] = normal[0];
            quadns[indexn++] = normal[1];
            quadns[indexn++] = normal[2];
        }
        vec3 vertex = myVertices[v.pos];
        quadvs[indexv++] = vertex[0];
        quadvs[indexv++] = vertex[1];
        quadvs[indexv++] = vertex[2];
     }
    }


    trivs = new GLfloat[myTris.size()*3*3];
    trins = new GLfloat[myTris.size()*3*3];

    indexn = 0;
    indexv = 0;
    for (FaceIt it = myTris.begin(); it != myTris.end(); ++it)
    {
     Face face = *it;
     for (unsigned int i = 0; i < face.size(); i++)
     {
        Vertex v = face[i];
        if (myNormals.size() > 0)
        {
           vec3 normal = myNormals[v.normal];
            trins[indexn++] = normal[0];
            trins[indexn++] = normal[1];
            trins[indexn++] = normal[2];
        }
        vec3 vertex = myVertices[v.pos];
        trivs[indexv++] = vertex[0];
        trivs[indexv++] = vertex[1];
        trivs[indexv++] = vertex[2];
     }
    }
}

void SkeletonMesh::loadSkinWeights(const char* filename)
{   
   // Skin weights from file
   std::map<std::string, unsigned int> skinWeightJoints;
   std::vector<std::vector<float>> skinWeights; // for each joint, list weight for each vertex

   std::ifstream file (filename);
   if (file.is_open())
   {
      while (! file.eof() )
      {
        std::string line;
        std::getline (file, line);
        if (line.length() == 0) continue;

        std::string copy(line);
        char* token = strtok((char*) copy.c_str(), ":");
        if (strncmp(token, "vertex", 6) == 0) // First line
        {
            token = strtok(NULL, ":");
            while (token)
            {
                skinWeightJoints[token] = skinWeights.size();
                skinWeights.push_back(std::vector<float>());
                token = strtok(NULL, ":");
            }
        }
        else
        {
            int index = 0;
            token = strtok(NULL, ":");
            while (token)
            {
                float weight = atof(token);
                skinWeights[index++].push_back(weight);
                token = strtok(NULL, ":");
            }
        }
      }

      file.close();

    for (unsigned int i = 0; i < skinWeights[0].size(); i++)
    {
        std::vector<int> jointInds;
        std::vector<float> weights;
        for (unsigned int j = 0; j < mSkeleton.GetNumJoints(); j++)
        {
            Joint* joint = mSkeleton.GetJointByID(j);    
            if (skinWeightJoints.count(joint->GetName()) == 0) continue;
            float weight = skinWeights[skinWeightJoints[joint->GetName()]][i];
            if (weight > 0.0)
            {
                jointInds.push_back(j);
                weights.push_back(weight);
            }
        }

        bool renorm = true;
        while (jointInds.size() > 4) // get largest and renormalize
        {
            renorm = true;
            //std::cout << "Warning: too many joint weights\n";
            std::vector<float>::iterator it = 
                std::min_element(weights.begin(), weights.end());

            unsigned int k;
            for (k = 0; k < jointInds.size(); k++)
            {
                if (weights[k] == *it)
                {
                    break;
                }
            }
            weights.erase(it);
            jointInds.erase(jointInds.begin()+k);
        }

        // Normalize weights
        if (renorm)
        {
        double denom = 0;
        for (unsigned int k = 0; k < weights.size(); k++) denom += weights[k];
        for (unsigned int k = 0; k < weights.size(); k++) weights[k] = weights[k] / denom;   
        }

        myVertexJointIndices.push_back(std::vector<int>(4, 0));
        myVertexWeights.push_back(std::vector<float>(4, 0));
        for (unsigned int k = 0; k < jointInds.size(); k++)
        {
            myVertexJointIndices[i+myOffset][k] = jointInds[k];
            myVertexWeights[i+myOffset][k] = weights[k];
        }
    }    
   }
   else std::cout << "Unable to open file: " << filename;   
}

bool SkeletonMesh::getBoundingBox(vec3& mmin, vec3& mmax)  // returns bbox in world coord
{
   if (myMax == myMin) return false;

   const math::matrix<double>& mat = getLocalToWorld();
   mmin = mat*myMin;
   mmax = mat*myMax;
   return true;
}

const math::matrix<double>& SkeletonMesh::getLocalToWorld() const
{
   myModelMatrix = math::ScaleMatrix<double>(Scale[0], Scale[1], Scale[2]);
   myModelMatrix = math::RotationMatrix<double>(2, Rotation[2]*M_PI/180)*myModelMatrix;
   myModelMatrix = math::RotationMatrix<double>(1, Rotation[1]*M_PI/180)*myModelMatrix;
   myModelMatrix = math::RotationMatrix<double>(0, Rotation[0]*M_PI/180)*myModelMatrix;
   myModelMatrix = math::TranslationMatrix<double>(Translation[0], Translation[1], Translation[2])*myModelMatrix;
   return myModelMatrix;
}

const math::matrix<double>& SkeletonMesh::getWorldToLocal() const
{
   myModelMatrix = getLocalToWorld(); // calculate myModelMatrix
   myModelMatrix = myModelMatrix.Inv();
   return myModelMatrix;
}


/*
From WIKIPEDIA: OBJ File Specification:

# this is a comment
# Here is the first vertex, with (x,y,z) coordinates.
v 0.123 0.234 0.345 
v ... 
...

#Texture coordinates
vt ...
...

#Normals in (x,y,z) form; normals might not be unit. 
vn ...
..

#Each face is given by a set of indices to the vertex/texture/normal
#coordinate array that precedes this.
#Hence f 1/1/1 2/2/2 3/3/3 is a triangle having texture coordinates and 
#normals for those 3 vertices,
#and having the vertex 1 from the "v" list, texture coordinate 2 from 
#the "vt" list, and the normal 3 from the "vn" list

f v0/vt0/vn0 v1/vt1/vn1 ...
f ...
...

# when there are named polygon groups or materials groups the following 
# tags appear in the face section,
g [group name]
usemtl [material name]
# the latter matches the named material definitions in the external .mtl file.
# each tag applies to all faces following, until another tag of the same type appears.
...

...
*/

void SkeletonMesh::load(const char* filename)
{
   myOffset = myVertices.size();
   std::ifstream file (filename);
   if (file.is_open())
   {
      while (! file.eof() )
      {
         std::string line;
         std::getline (file, line);
         std::string copy(line);
         parse((char*) copy.c_str());
      }
      file.close();
   }
   else std::cout << "Unable to open file: " << filename;   
}

void SkeletonMesh::parse (char* line)
{
   if (!strncmp(line, "v ",  2)) parseVertex(&(line[2]));
   if (!strncmp(line, "vt ", 3)) parseUV(&(line[3]));
   if (!strncmp(line, "vn ", 3)) parseNormal(&(line[3]));
   if (!strncmp(line, "f ",  2)) parseFace(&(line[2]));   
}


void SkeletonMesh::parseVertex(char* line)
{
   // Space delimited float list
   char* token1 = strtok(line, " ");
   char* token2 = strtok(NULL, " ");
   char* token3 = strtok(NULL, " ");

   GLfloat x = atof(token1);
   GLfloat y = atof(token2);
   GLfloat z = atof(token3);

   if (x < myMin[0]) myMin[0] = x;
   if (y < myMin[1]) myMin[1] = y;
   if (z < myMin[2]) myMin[2] = z;

   if (x > myMax[0]) myMax[0] = x;
   if (y > myMax[1]) myMax[1] = y;
   if (z > myMax[2]) myMax[2] = z;

   myVertices.push_back(vec3(x, y, z));   
}

void SkeletonMesh::parseUV(char* line)
{
   // Space delimited float list
   char* token1 = strtok((char*) line, " ");
   char* token2 = strtok(NULL, " ");

   GLfloat u = atof(token1);
   GLfloat v = atof(token2);

   myUvs.push_back(vec2(u, v));   
}

void SkeletonMesh::parseNormal(char* line)
{
   // Space delimited float list
   char* token1 = strtok((char*) line, " ");
   char* token2 = strtok(NULL, " ");
   char* token3 = strtok(NULL, " ");

   GLfloat x = atof(token1);
   GLfloat y = atof(token2);
   GLfloat z = atof(token3);

   myNormals.push_back(vec3(x, y, z));   
}

void SkeletonMesh::parseFace(char* line)
{
   SkeletonMesh::Face face;

   // Assumes vertex/UV/normal
   char* token = strtok((char*) line, " ");
   while (token)
   {
      size_t len = strlen(token);

      // Find first slash
      unsigned int i;
      for (i = 0; i < len && token[i] != '/'; i++);
      size_t slashIndex1 = i;

      // Find second slash
      for (i = slashIndex1+1; i < len && token[i] != '/'; i++);
      size_t slashIndex2 = i;

      SkeletonMesh::Vertex v;
      v.normal = v.pos = v.uv = 0; // Initialize vertex
      if (slashIndex2+1 < len) // have a normal
      {
         v.normal = atoi(&(token[slashIndex2+1]))-1+myOffset; 
      }
      token[slashIndex2] = '\0';
      if (slashIndex1+1 != slashIndex2) // have a uv
      {
         v.uv = atoi(&(token[slashIndex1+1]))-1+myOffset; 
      }
      token[slashIndex1] = '\0';
      v.pos = atoi(token)-1+myOffset; 

      face.push_back(v);
      token = strtok(NULL, " ");
   }
   if (face.size() == 3) myTris.push_back(face);
   else if (face.size() == 4) myQuads.push_back(face);
   else myPolys.push_back(face);
}

Joint* addJoint(const std::string& name, const vec3& t, 
              const std::string order, const vec3& r)
{
    Joint* joint = new Joint(name);
    joint->SetLocalTranslation(t);

    mat3 rot;
	if (order == "xyz") rot.FromEulerAnglesZYX(r*Deg2Rad);
	else if (order == "xzy") rot.FromEulerAnglesYZX(r*Deg2Rad);
	else if (order == "yxz") rot.FromEulerAnglesZXY(r*Deg2Rad);
	else if (order == "yzx") rot.FromEulerAnglesXZY(r*Deg2Rad);
	else if (order == "zxy") rot.FromEulerAnglesYXZ(r*Deg2Rad);
	else if (order == "zyx") rot.FromEulerAnglesXYZ(r*Deg2Rad);
    joint->SetLocalRotation(rot);

    return joint;
}

void SkeletonMesh::initSkeleton(const char* bindPoseFile)
{   
    Player player;
    if (player.LoadBVHFile(bindPoseFile))
    {
        mSkeleton = player.GetSkeleton();
        mSkeleton.ReadFromFrame(player.GetMotion().GetFrame(0));
    }
    else
    {
        std::cout << "ERROR: Cannot load " << bindPoseFile << std::endl;
    }
}

void SkeletonMesh::setPose(const Frame& frame, const Skeleton& skeleton)
{
    mSkeleton.GetRootJoint()->SetLocalTranslation(frame.GetRootTranslation() * INCH_2_CM);

    for (unsigned int i = 0; i < mSkeleton.GetNumJoints(); i++)
    {
        Joint* joint = mSkeleton.GetJointByID(i);
        Joint* playerJoint = skeleton.GetJointByName(joint->GetName());
        if (!playerJoint) continue;
        mat3 rot = frame.GetJointRotation(playerJoint->GetID());
        joint->SetLocalRotation(rot);
    }

    mSkeleton.UpdateFK();
    updateSkin(mSkeleton);
}

void SkeletonMesh::LoadTurtle(SkeletonMesh& model)
{
    std::vector<const char*> objfiles;
    objfiles.push_back(SKIN_DIR"oliver_body.obj");
    objfiles.push_back(SKIN_DIR"oliver_shell.obj");
    objfiles.push_back(SKIN_DIR"oliver_leg1.obj");
    objfiles.push_back(SKIN_DIR"oliver_leg.obj");

    std::vector<const char*> wgtfiles;
    wgtfiles.push_back(SKIN_DIR"oliver_body_weights.txt");
    wgtfiles.push_back(SKIN_DIR"oliver_shell_weights.txt");
    wgtfiles.push_back(SKIN_DIR"oliver_leg1_weights.txt");
    wgtfiles.push_back(SKIN_DIR"oliver_leg_weights.txt");

    model.load(objfiles, wgtfiles, SKIN_DIR"oliverBindPose.bvh");

}

void SkeletonMesh::LoadBear(SkeletonMesh& model)
{
    model.load(SKIN_DIR"manny.obj", 
               SKIN_DIR"mannyWeights.txt", 
               SKIN_DIR"mannyBindPose.bvh");
}

