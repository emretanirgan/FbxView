//-----------------------------------------------------------------------------
// Copyright (C) 2013 by Aline Normoyle, Liming Zhao, Alla Safonova, Teresa Fan


#ifndef Frame_H_
#define Frame_H_

#include "Skeleton.h"
#include <vector>

class Frame
{

public:
	Frame();
    Frame(const Frame& orig);
    virtual Frame& operator=(const Frame& orig);
	virtual ~Frame();

    void LoadFromBVHFile(std::ifstream& inFile, const Skeleton& pSkeleton);	// Read from BVH file and assume ZXY rotation order
    void LoadFromAMCFile(std::ifstream& inFile, const Skeleton& pSkeleton);	
    void SaveToBVHFile(std::ofstream& outFile, const Skeleton& pSkeleton);	// Write to BVH file and assume ZXY rotation order
    void SaveToAMCFile(std::ofstream& outFile, const Skeleton& pSkeleton);	// Write to BVH file and assume ZXY rotation order

    void SetNumJoints(unsigned int num);
	unsigned int GetNumJoints() const;

    void SetRootTranslation(const vec3& translation);
	const vec3& GetRootTranslation() const { return m_rootTranslation; }

    void SetJointRotation(unsigned int index, const vec3& eulerZXY);
	void SetJointRotation(unsigned int index, const mat3& rotation);
	const mat3& GetJointRotation(unsigned int index) const { return m_rotationData[index]; }

	void SetJointQuaternion(unsigned int index, const Quaternion& rotation);
	const Quaternion& GetJointQuaternion(unsigned int index) const {return m_quaternionData[index]; }

    std::vector<vec3> m_eulerData;

private:
	vec3 m_rootTranslation;
    std::vector<mat3> m_rotationData;
    std::vector<Quaternion> m_quaternionData;
};

#endif