//------------------------------------------------------------------------
// Copyright (C) 2013 by Aline Normoyle, Liming Zhao, Alla Safonova, Teresa Fan
// See License.txt for usage permissions. 


#include "Joint.h"

Joint::Joint()
{
	m_name = "";
	m_id = -1;
	m_pParent = NULL;
    //mCollision = false;
    m_rotOrder = "xyz";
    m_dofs = DOF_X | DOF_Y | DOF_Z;
    m_lowerLimits = vec3(-360, -360, -360) * Deg2Rad;
    m_upperLimits = vec3(360, 360, 360) * Deg2Rad;
    m_axisRotation = identity3D;
    m_translation = vec3(0,0,0);
    AMC = false;
}

Joint::Joint(const std::string& name)
{
    m_id = 0;
	SetName(name);
	m_pParent = NULL;
    //mCollision = false;
    m_rotOrder = "xyz";
    m_dofs = DOF_X | DOF_Y | DOF_Z;
    m_lowerLimits = vec3(-360, -360, -360) * Deg2Rad;
    m_upperLimits = vec3(360, 360, 360) * Deg2Rad;
    m_axisRotation = identity3D;
    m_translation = vec3(0,0,0);
    AMC = false;
}

Joint::Joint(const Joint& joint)
{
    *this = joint;
}

Joint& Joint::operator=(const Joint& orig)
{
    if (&orig == this)
    {
        return *this;
    }
	m_name = orig.m_name;
	m_id = orig.m_id;
	m_channelCount = orig.m_channelCount;
	m_pParent = 0; 
	m_children.clear();
	m_local = orig.m_local;
	m_global = orig.m_global;
    //mCollision = orig.mCollision;
    m_rotOrder = orig.m_rotOrder;
    m_dofs = orig.m_dofs;
    m_lowerLimits = orig.m_lowerLimits;
    m_upperLimits = orig.m_upperLimits;
    m_translation = orig.m_translation;  // for amc files
    m_axisRotation = orig.m_axisRotation; // for amc files
    AMC = orig.AMC;
    return *this;
}

// Destructor
Joint::~Joint()
{
}

void Joint::SetParent(Joint* parent)
{
    m_pParent = parent;
}

// Member functions
Joint* Joint::GetParent(void)
{
	return m_pParent;
}

unsigned int Joint::GetNumChildren() const
{
	return m_children.size();
}

Joint* Joint::GetChildAt(unsigned int index)
{
	return m_children[index];
}

void Joint::AppendChild(Joint* child)
{
    m_children.push_back(child);
}

void Joint::SetName(const std::string& name)
{
	m_name = name;
    if (strncmp("Site", name.c_str(), 4)==0)
    {
        char dummy[32];
        sprintf(dummy, "Site%d", m_id);
        m_name = dummy;
    }
}

void Joint::SetID(int id)
{
	m_id = id;
    if (strncmp("Site", m_name.c_str(), 4)==0)
    {
        char dummy[32];
        sprintf(dummy, "Site%d", m_id);
        m_name = dummy;
    }
}

void Joint::SetNumChannels(unsigned int count)
{
	m_channelCount = count;
}

void Joint::SetRotationOrder(const std::string& rotOrder)
{
    std::string order = rotOrder;

    if (order.find("Zrotation Xrotation Yrotation") != std::string::npos) m_rotOrder = "zxy";
    else if (order.find("Zrotation Yrotation Xrotation") != std::string::npos) m_rotOrder = "zyx";
    else if (order.find("Xrotation Yrotation Zrotation") != std::string::npos) m_rotOrder = "xyz";
    else if (order.find("Xrotation Zrotation Yrotation") != std::string::npos) m_rotOrder = "xzy";
    else if (order.find("Yrotation Xrotation Zrotation") != std::string::npos) m_rotOrder = "yxz";
    else if (order.find("Yrotation Zrotation Xrotation") != std::string::npos) m_rotOrder = "yzx";
    else m_rotOrder = order;
}

void Joint::SetDOFs(unsigned int dofFlags)
{
    m_dofs = dofFlags;
}

void Joint::SetJointLimits(const vec3& lower, const vec3& upper)
{
    m_lowerLimits = lower;
    m_upperLimits = upper;
}

unsigned int Joint::GetDOFs() const
{
    return m_dofs;
}

vec3 Joint::GetLowerJointLimit() const
{
    return m_lowerLimits;
}

vec3 Joint::GetUpperJointLimit() const
{
    return m_upperLimits;
}

void Joint::SetLocalTransform(const Transform& transform)
{
	m_local = transform;
}

void Joint::SetLocalTranslation(const vec3& translation)
{
	m_local.m_translation = translation;
    m_translation = translation;
}

void Joint::SetLocalRotation(const mat3& rotation)
{
    if (AMC && m_pParent)
    {
        m_local.m_rotation = m_axisRotation * rotation * m_axisRotation.Transpose();
        m_local.m_translation = m_local.m_rotation * m_translation;
    }
    else
    {
    	m_local.m_rotation = rotation;
    }
}

const std::string& Joint::GetName() const
{
	return m_name;
}

int Joint::GetID() const
{
	return m_id;
}

unsigned int Joint::GetNumChannels() const
{
	return m_channelCount;
}

const std::string& Joint::GetRotationOrder() const
{
	return m_rotOrder;
}


const Transform& Joint::GetLocalTransform() const
{
	return m_local;
}

const vec3& Joint::GetLocalTranslation() const
{
	return m_local.m_translation;
}

const mat3& Joint::GetLocalRotation() const
{
	return m_local.m_rotation;
}

const Transform& Joint::GetGlobalTransform() const
{
	return m_global;
}

const vec3& Joint::GetGlobalTranslation() const
{
	return m_global.m_translation;
}

const mat3& Joint::GetGlobalRotation() const
{
	return m_global.m_rotation;
}

void Joint::AttachJoints(Joint* pParent, Joint* pChild)
{
	if (pChild)
	{
		Joint* pOldParent = pChild->m_pParent;
		if (pOldParent)
		{
			// erase the child from old parent's children list
			std::vector<Joint*>::iterator iter;
			for (iter = pOldParent->m_children.begin(); iter != pOldParent->m_children.end(); iter++)
			{
				if ((*iter) == pChild)
				{
					iter = pOldParent->m_children.erase(iter);
				}
			}
		}
		// Set the new parent
		pChild->m_pParent = pParent;
		// Add child to new parent's children list
		if (pParent)
		{
			pParent->m_children.push_back(pChild);
		}
	}
}

void Joint::DetachJoints(Joint* pParent, Joint* pChild)
{
	if (pChild && pChild->m_pParent == pParent)
	{
		if (pParent)
		{
			// erase the child from parent's children list
			std::vector<Joint*>::iterator iter;
			for (iter = pParent->m_children.begin(); iter != pParent->m_children.end(); iter++)
			{
				if ((*iter) == pChild)
				{
					iter = pParent->m_children.erase(iter);
				}
			}
		}
		pChild->m_pParent = NULL;
	}
}

//////////////////////////////////////////////////////////////////////////
// UpdateTransformation
// Input: whether to propagate the transformation update to its children joints
// If bRecursive == true, you need to update the all children joints transformations
// This basically implements the forward kinematics from current joint
void Joint::UpdateTransformation(bool bRecursive)
{
   if (m_pParent)
   {
      m_global = m_pParent->m_global * m_local;
   }
   else
   {
  	   m_global = m_local; 
   }

	if (bRecursive)
	{
		std::vector<Joint*>::const_iterator iter;
		for (iter = m_children.begin(); iter != m_children.end(); iter++)
		{
			(*iter)->UpdateTransformation(true);
		}
	}
}

