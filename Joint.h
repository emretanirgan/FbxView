//-----------------------------------------------------------------------------
// Copyright (C) 2013 by Aline Normoyle, Liming Zhao, Alla Safonova, Teresa Fan


#ifndef Joint_H_
#define Joint_H_

#include <string>
#include <vector>
#include "Transformation.h"

#define DOF_X 0x1
#define DOF_Y 0x10
#define DOF_Z 0x100

class Joint
{
public:
	Joint();
	Joint(const std::string& name);
    Joint(const Joint& joint);
	virtual ~Joint();

    virtual Joint& operator=(const Joint& joint);

	Joint* GetParent();
    void SetParent(Joint* parent);

	unsigned int GetNumChildren() const;
	Joint* GetChildAt(unsigned int index);
    void AppendChild(Joint* child);

	void UpdateTransformation(bool bRecursive = false);

	void SetName(const std::string& name);
	void SetID(int id);
	void SetNumChannels(unsigned int count);
    void SetRotationOrder(const std::string& order);
	void SetLocalTransform(const Transform& transform);
	void SetLocalTranslation(const vec3& translation);
	void SetLocalRotation(const mat3& rotation);
    void SetDOFs(unsigned int dofFlags);
    void SetJointLimits(const vec3& lower, const vec3& upper);

	int GetID() const;
	const std::string& GetName() const;
	unsigned int GetNumChannels() const;
    const std::string& GetRotationOrder() const;
    const Transform& GetLocalTransform() const;
	const vec3& GetLocalTranslation() const;
	const mat3& GetLocalRotation() const;
	const Transform& GetGlobalTransform() const;
	const vec3& GetGlobalTranslation() const;
	const mat3& GetGlobalRotation() const;
    unsigned int GetDOFs() const;
    vec3 GetLowerJointLimit() const;
    vec3 GetUpperJointLimit() const;

	static void AttachJoints(Joint* pParent, Joint* pChild);
	static void DetachJoints(Joint* pParent, Joint* pChild);

    bool AMC;
    vec3 m_translation;  // for amc files
    mat3 m_axisRotation; // for amc files

private:
	std::string m_name;
	int m_id;
	unsigned int m_channelCount;
    std::string m_rotOrder;
    unsigned int m_dofs;
    vec3 m_lowerLimits;
    vec3 m_upperLimits;

    Joint* m_pParent;
	std::vector<Joint*> m_children;

public:
    Transform m_local;
	Transform m_global;
};

inline std::ostream& operator << (std::ostream& ostrm, const Joint& j)
{
   ostrm << j.GetName() << "LOCAL: " << j.GetLocalTransform() << "GLOBAL: " << j.GetGlobalTransform() << std::endl;
   return ostrm;
}


#endif