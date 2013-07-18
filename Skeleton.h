//-----------------------------------------------------------------------------
// Copyright (C) 2013 by Aline Normoyle, Liming Zhao, Alla Safonova, Teresa Fan


#ifndef Skeleton_H_
#define Skeleton_H_

#include "Joint.h"

class Frame;
class Player;
class Skeleton
{
public:
	Skeleton();
    Skeleton(const Skeleton& skeleton); // Deep copy
    virtual Skeleton& operator=(const Skeleton& orig); // Deep copy
	virtual ~Skeleton();
    virtual void Clear();

    void UpdateFK(Joint* pRoot = NULL);

    bool LoadFromBVHFile(std::ifstream& inFile);
    void SaveToBVHFile(std::ofstream& outFile);
    bool LoadASFFile(const std::string& filename);

	Joint* GetJointByName(const std::string& name) const;
	Joint* GetJointByID(unsigned int id) const;
	Joint* GetRootJoint() const;
    void AddJoint(Joint* joint, bool isRoot = false);
	size_t GetNumJoints() const { return m_joints.size(); }

	void ReadFromFrame(const Frame& pFrame);
	void WriteToFrame(Frame& frame) const;

    vec3 GetDimensions() ;
    void SetScale(float s);
    float GetScale() const {return mScale;}
    std::vector<Joint*> m_joints;
    bool AMC;

protected:

    bool LoadFromFileBVHRec(std::ifstream& inFile, Joint* pParent, std::string prefix);
    void ReadJointFromASFFile(std::ifstream& inFile);
    void SaveToFileBVHRec(std::ofstream& outFile, Joint* pJoint, unsigned int level);

    float mScale;
    
	Joint* m_pRoot;
};


#endif