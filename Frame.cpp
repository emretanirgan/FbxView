//-----------------------------------------------------------------------------
// Copyright (C) 2013 by Aline Normoyle, Liming Zhao, Alla Safonova, Teresa Fan

#include "Frame.h"
#include <string>
#include <fstream>
#include <iomanip>
#include <iostream>

#define MOCAP_SCALE 0.05644444f // for AMC files

vec3 Lerp(float fPerc, const vec3& t0, const vec3& t1)
{
   return (1-fPerc)*t0 + fPerc*t1;
}

vec3 Vec3Cubic(float fPerc, const vec3& t0, const vec3& t1, const vec3& t2, const vec3& t3)
{
   vec3 tmp1 = Lerp(fPerc, t0, t1);
   vec3 tmp2 = Lerp(fPerc, t1, t2);
   vec3 tmp3 = Lerp(fPerc, t2, t3);

   vec3 tmp12 = Lerp(fPerc, tmp1, tmp2);
   vec3 tmp23 = Lerp(fPerc, tmp2, tmp3);

   return Lerp(fPerc, tmp12, tmp23);
}


Frame::Frame() : m_rootTranslation(0,0,0)
{
}

Frame::~Frame()
{
}

Frame::Frame(const Frame& orig)
{
	
    *this = orig;
}

Frame& Frame::operator=(const Frame& orig)
{
    if (this == &orig)
    {
        return *this;
    }

	m_rootTranslation = orig.m_rootTranslation;

    m_eulerData.clear();
    m_rotationData.clear();
    m_quaternionData.clear();

	for (unsigned int i = 0; i < orig.m_rotationData.size(); i++)
	{
		m_eulerData.push_back(orig.m_eulerData[i]);
		m_rotationData.push_back(orig.m_rotationData[i]);
		m_quaternionData.push_back(orig.m_quaternionData[i]);
	}

    return *this;
}

unsigned int Frame::GetNumJoints() const
{
	return m_rotationData.size();
}

void Frame::SetNumJoints(unsigned int num)
{
    m_eulerData.resize(num, vec3(0,0,0));
    m_rotationData.resize(num, identity3D);
    m_quaternionData.resize(num, Quaternion(0,0,0,0));
}

mat3 ComputeBVHRot(float r1, float r2, float r3, const std::string& rotOrder) // For BVH
{
    mat3 m;
	float ry, rx, rz;

    if (rotOrder == "xyz") 
    {
        rx = r1; ry = r2; rz = r3;
        m.FromEulerAnglesXYZ(vec3(rx, ry, rz) * Deg2Rad);
    }
    else if (rotOrder == "xzy")
    {
        rx = r1; rz = r2; ry = r3;
        m.FromEulerAnglesXZY(vec3(rx, ry, rz) * Deg2Rad);
    }
    else if (rotOrder == "yxz")
    {
        ry = r1; rx = r2; rz = r3;
        m.FromEulerAnglesYXZ(vec3(rx, ry, rz) * Deg2Rad);
    }
    else if (rotOrder == "yzx")
    {
        ry = r1; rz = r2; rx = r3;
        m.FromEulerAnglesYZX(vec3(rx, ry, rz) * Deg2Rad);
    }
    else if (rotOrder == "zxy")
    {
        rz = r1; rx = r2; ry = r3;
        m.FromEulerAnglesZXY(vec3(rx, ry, rz) * Deg2Rad);
    }
    else if (rotOrder == "zyx")
    {
        rz = r1; ry = r2; rx = r3;
        m.FromEulerAnglesZYX(vec3(rx, ry, rz) * Deg2Rad);
    }
    return m;
}

mat3 ComputeAMCRot(float r1, float r2, float r3, const std::string& rotOrder)
{
    mat3 m;
	float ry, rx, rz;
    rx = r1; ry = r2; rz = r3;

    if (rotOrder == "xyz") 
    {
        m.FromEulerAnglesXYZ(vec3(rx, ry, rz) * Deg2Rad);
    }
    else if (rotOrder == "xzy")
    {
        m.FromEulerAnglesXZY(vec3(rx, ry, rz) * Deg2Rad);
    }
    else if (rotOrder == "yxz")
    {
        m.FromEulerAnglesYXZ(vec3(rx, ry, rz) * Deg2Rad);
    }
    else if (rotOrder == "yzx")
    {
        m.FromEulerAnglesYZX(vec3(rx, ry, rz) * Deg2Rad);
    }
    else if (rotOrder == "zxy")
    {
        m.FromEulerAnglesZXY(vec3(rx, ry, rz) * Deg2Rad);
    }
    else if (rotOrder == "zyx")
    {
        m.FromEulerAnglesZYX(vec3(rx, ry, rz) * Deg2Rad);
    }
    return m;
}

void Frame::LoadFromAMCFile(std::ifstream& inFile, const Skeleton& pSkeleton)
{
    m_eulerData.resize(pSkeleton.GetNumJoints(), vec3Zero);
    m_rotationData.resize(pSkeleton.GetNumJoints(), identity3D);
    m_quaternionData.resize(pSkeleton.GetNumJoints(), identity3D.ToQuaternion());

	char name[256];
	inFile >> name >> m_rootTranslation[VX] >> m_rootTranslation[VY] >> m_rootTranslation[VZ];
	m_rootTranslation *= MOCAP_SCALE; //MOCAP_SCALE;
    float rx, ry, rz;
	inFile >> rx >> ry >> rz;

    Joint* root = pSkeleton.GetJointByName(name);
    if (!root) return;

    mat3 rot = ComputeAMCRot(rx, ry, rz, root->GetRotationOrder());
    m_eulerData[root->GetID()] = vec3(rx, ry, rz);
    m_rotationData[root->GetID()] = rot;
    m_quaternionData[root->GetID()] = rot.ToQuaternion();

	for (unsigned int i = 1; i < pSkeleton.GetNumJoints(); i++)
	{
		inFile >> name;
        Joint* joint = pSkeleton.GetJointByName(name);
        if (!joint) break;

        rx = ry = rz = 0.0;
        if (joint->GetDOFs() & DOF_X) inFile >> rx;
        if (joint->GetDOFs() & DOF_Y) inFile >> ry;
        if (joint->GetDOFs() & DOF_Z) inFile >> rz;

        rot = ComputeAMCRot(rx, ry, rz, "zyx");
        m_eulerData[joint->GetID()] = vec3(rx,ry,rz);
        m_rotationData[joint->GetID()] = rot;
        m_quaternionData[joint->GetID()] = rot.ToQuaternion();
	}
}

void Frame::LoadFromBVHFile(std::ifstream& inFile, const Skeleton& pSkeleton)
{
	float tx, ty, tz, r1, r2, r3;
	for (unsigned int i = 0; i < pSkeleton.GetNumJoints(); i++)
	{
        tx = ty = tz = 0.0f;
		r1 = r2 = r3 = 0.0f;

		Joint* pJoint = pSkeleton.GetJointByID(i);
		if (pJoint->GetNumChannels() == 6)
		{
            inFile >> tx >> ty >> tz;
            inFile >> r1 >> r2 >> r3;
        }
        else if (pJoint->GetNumChannels() == 3)
        {
            inFile >> r1 >> r2 >> r3;
        }
        else
        {
        }

        if (i == 0) m_rootTranslation = vec3(tx, ty, tz);

        mat3 m = ComputeBVHRot(r1, r2, r3, pJoint->GetRotationOrder());
        m_rotationData.push_back(m); 

        Quaternion q;
        q.FromRotation(m);
		m_quaternionData.push_back(q);
	}
}

void Frame::SaveToBVHFile(std::ofstream& outFile, const Skeleton& pSkeleton)
{
    outFile << std::setprecision(6);
	outFile << m_rootTranslation[0] << "\t" << m_rootTranslation[1] << "\t" << m_rootTranslation[2];

    if (pSkeleton.AMC)
    {
        static Skeleton dummy;
        dummy = pSkeleton;
        dummy.ReadFromFrame(*this);

        vec3 angles;
        for (unsigned int i = 0; i < m_rotationData.size(); i++)
        {
	        Joint* pJoint = dummy.GetJointByID(i);
            assert(pJoint);

	        if (pJoint->GetNumChannels() > 0)
	        {
                vec3 X(1,0,0);
                vec3 Y(0,1,0);
                vec3 Z(0,0,1);
                X = pJoint->GetLocalTransform().m_rotation * X;
                Y = pJoint->GetLocalTransform().m_rotation * Y;
                Z = pJoint->GetLocalTransform().m_rotation * Z;

                mat3 rotation = mat3(X,Y,Z).Transpose();
                rotation.ToEulerAnglesZXY(angles);
		        angles *= Rad2Deg;
                outFile << "\t" << angles[VZ] << "\t" << angles[VX] << "\t" << angles[VY];
	        }
        }
    }
    else
    {
        vec3 angles;
        for (unsigned int i = 0; i < pSkeleton.GetNumJoints(); i++)
        {
	        Joint* pJoint = pSkeleton.GetJointByID(i);
            assert(pJoint);

	        if (pJoint->GetNumChannels() > 0)
	        {
                mat3 rotation = m_rotationData[pJoint->GetID()];
		        rotation.ToEulerAnglesZXY(angles);
		        angles *= Rad2Deg;
                outFile << "\t" << angles[VZ] << "\t" << angles[VX] << "\t" << angles[VY];
	        }
        }
    }

    outFile << std::endl;
}


void Frame::SaveToAMCFile(std::ofstream& outFile, const Skeleton& pSkeleton) 
{
    Joint* root = pSkeleton.GetRootJoint();

    vec3 pos = m_rootTranslation / MOCAP_SCALE;; // / MOCAP_SCALE;
	outFile << root->GetName() << " " << pos[0] << " " << pos[1] << " " << pos[2] << " ";

    // root has ZYX order, all others XYZ
    vec3 angles;
    mat3 m = m_rotationData[root->GetID()];
    if (root->GetRotationOrder() == "xyz") m.ToEulerAnglesXYZ(angles);
    else if (root->GetRotationOrder() == "xzy") m.ToEulerAnglesXZY(angles);
    else if (root->GetRotationOrder() == "yxz") m.ToEulerAnglesYXZ(angles);
    else if (root->GetRotationOrder() == "yzx") m.ToEulerAnglesYZX(angles);
    else if (root->GetRotationOrder() == "zxy") m.ToEulerAnglesZXY(angles);
    else if (root->GetRotationOrder() == "zyx") m.ToEulerAnglesZYX(angles);
    angles = angles * Rad2Deg;
    outFile << angles[0] << " " << angles[1] << " " << angles[2] << std::endl;

	for (unsigned int i = 1; i < pSkeleton.GetNumJoints(); i++)
	{
        Joint* joint = pSkeleton.GetJointByID(i);
        if (joint->GetDOFs() == 0) continue;
        outFile << joint->GetName();

        vec3 angles;
        m_rotationData[i].ToEulerAnglesZYX(angles);
        angles = angles * Rad2Deg;
		if (joint->GetDOFs() & DOF_X) outFile << " " << angles[VX];
		if (joint->GetDOFs() & DOF_Y) outFile << " " << angles[VY];
		if (joint->GetDOFs() & DOF_Z) outFile << " " << angles[VZ];
        outFile << std::endl;
	}
}


void Frame::SetRootTranslation(const vec3& translation)
{
	m_rootTranslation = translation;
}

void Frame::SetJointRotation(unsigned int index, const vec3& eulerZXY)
{
   m_eulerData[index]  = eulerZXY;

   mat3 m;
   m.FromEulerAnglesZXY(eulerZXY);
   SetJointRotation(index, m);
}

void Frame::SetJointRotation(unsigned int index, const mat3& rotation)
{
	assert(index >= 0 && index < GetNumJoints());
	m_rotationData[index] = rotation;
	m_quaternionData[index].FromRotation(rotation);
}

void Frame::SetJointQuaternion(unsigned int index, const Quaternion& rotation)
{
	assert(index >= 0 && index < GetNumJoints());	
	m_quaternionData[index] = rotation;
	m_rotationData[index].FromQuaternion(rotation);
}

vec3 CubicVec3(const vec3 &d1, const vec3 &d2, const vec3 &d3, const vec3 &d4, float t)
{
	vec3 a = d2; 
	vec3 b = d2 - d1; 
	vec3 c = (d3 - d2) * 3 - (d2 - d1) * 2 - (d4 - d3); 
	vec3 d = (d2 - d3) * 2 + d2 - d1 + d4 - d3;
	return a + b * t + c * t * t + d * t * t * t;
}
