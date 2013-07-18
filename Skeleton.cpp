//------------------------------------------------------------------------
// Copyright (C) 2013 by Aline Normoyle, Liming Zhao, Alla Safonova, Teresa Fan


#include "Skeleton.h"
#include "Frame.h"
#include "Transformation.h"
#include <fstream>

#define MOCAP_SCALE 0.05644444f

Skeleton::Skeleton()
{
	m_pRoot = NULL;
    mScale = 1.0;
    AMC = false;
}

Skeleton::~Skeleton()
{	
    Clear();	
}

void Skeleton::Clear()
{
	m_pRoot = NULL;
	m_joints.clear();
}

Skeleton::Skeleton(const Skeleton& skeleton)
{
    *this = skeleton;
}

Skeleton& Skeleton::operator=(const Skeleton& orig)
{
    // Performs a deep copy
    if (&orig == this)
    {
        return *this;
    }

    m_joints.clear();
    m_pRoot = 0;
    AMC = orig.AMC;

    // Copy joints
	for(unsigned int i = 0; i < orig.m_joints.size(); i++)
	{
		Joint* joint = new Joint(*(orig.m_joints[i]));
        m_joints.push_back(joint);
	}

    // Copy parent/children relationships
	for(unsigned int i = 0; i < orig.m_joints.size(); i++)
	{
		Joint* joint = orig.m_joints[i];
        if (joint->GetParent())
        {
            Joint* parent = m_joints[joint->GetParent()->GetID()];
            m_joints[i]->SetParent(parent);
        }
        else 
        {
            m_pRoot = m_joints[i];
            m_pRoot->SetParent(0);
        }

        for (unsigned int j = 0; j < joint->GetNumChildren(); j++)
        {
            Joint* child = m_joints[joint->GetChildAt(j)->GetID()];
            m_joints[i]->AppendChild(child);
        }        
	}

    return *this;
}

bool Skeleton::LoadASFFile(const std::string& filename)
{
    std::ifstream inFile(filename.c_str());
	if (!inFile.is_open()) return false;

    Clear();
    AMC = true;

    char buffer[1024];
	char keyword[256];
	char *token;
	Joint *pJoint, *pParent;

	inFile.getline(buffer, 1024);
	sscanf(buffer, "%s", keyword);
	while(strcmp(buffer, ":bonedata") != 0)
	{
		inFile.getline(buffer, 1024);
		sscanf(buffer, "%s", keyword);
	}

	m_pRoot = new Joint;
	m_pRoot->SetID(0);
	m_pRoot->SetName("root");
    m_pRoot->SetNumChannels(6);
    m_pRoot->SetRotationOrder("zyx"); // TODO, Read from file
    m_pRoot->AMC = AMC;
	AddJoint(m_pRoot, true);

	inFile.getline(buffer, 1024);// Skip begin
	sscanf(buffer, "%s", keyword);
	while(strcmp(keyword, "begin") == 0)
	{
		ReadJointFromASFFile(inFile);
		inFile.getline(buffer, 1024);// Skip begin
		sscanf(buffer, "%s", keyword);
	}
	while(strcmp(buffer, ":hierarchy") != 0)
	{
		inFile.getline(buffer, 1024);
		sscanf(buffer, "%s", keyword);
	}
	inFile.getline(buffer, 1024); // Skip ":hierarchy"
	inFile.getline(buffer, 1024); // Skip "begin"
	sscanf(buffer, "%s", keyword);
	while(strcmp(keyword, "end") != 0)
	{
		token = strtok(buffer, " ");
		pParent = GetJointByName(token);
		token = strtok(NULL, " ");
		while(token != NULL)
		{
			pJoint = GetJointByName(token);
			Joint::AttachJoints(pParent, pJoint);
			token=strtok(NULL, " ");
		}
		inFile.getline(buffer, 1024);
		sscanf(buffer, "%s", keyword);
	}
	inFile.close();
    UpdateFK();

	return true;
}

void Skeleton::ReadJointFromASFFile( std::ifstream& inFile )
{
	char buffer[1024], keyword[256], name[128], *token;
	vec3 direction, translation, axisAngle;
    int id;
	float length;
    unsigned int dofs = 0;
    vec3 lower(0,0,0), upper(0,0,0);
    mat3 axisRotation = identity3D;

	inFile.getline(buffer, 1024);
	sscanf(buffer, "%s", keyword);
	while(strcmp(keyword, "end"))
	{
		if (strcmp(keyword, "id") == 0)
        {
			sscanf(buffer, "%s %d", keyword, &id);
        }
		else if(strcmp(keyword, "name") == 0)
		{
			sscanf(buffer, "%s %s", keyword, name);
		}
		else if (strcmp(keyword, "direction") == 0)
		{
            float x, y, z;
	    	sscanf(buffer, "%s %f %f %f", keyword, &x, &y, &z);
	    	direction.set(x, y, z);
	    }
		else if (strcmp(keyword, "length") == 0)
		{
			sscanf(buffer, "%s %f", keyword, &length);
			translation =  direction * length* MOCAP_SCALE;  // * MOCAP_SCALE
		}
		else if (strcmp(keyword, "axis") == 0)
		{
            float x, y, z;
			sscanf(buffer, "%s %f %f %f", keyword, &x, &y, &z);
			axisAngle.set(x, y, z);
            axisAngle *= Deg2Rad;			
            axisRotation.FromEulerAnglesZYX(axisAngle); // ASN TODO - shouldn't hard-code!
		}
		else if(strcmp(keyword, "dof") == 0) 
		{
            dofs = 0x0;
			token=strtok(buffer, " ");
			while(token != NULL)      
			{
				if(strcmp(token, "rx") == 0) dofs = dofs | DOF_X;
				if(strcmp(token, "ry") == 0) dofs = dofs | DOF_Y;
				if(strcmp(token, "rz") == 0) dofs = dofs | DOF_Z;
				token=strtok(NULL, " ");
			}
		}
		else if(strcmp(keyword, "limits") == 0) 
		{
			if (dofs & DOF_X)
			{
				token = strtok(buffer, "(");
				token = strtok(NULL, " ");
				lower[VX] = float(atof(token));
				token = strtok(NULL, " ");
				upper[VX] = float(atof(token));

				if (dofs & DOF_Y || dofs & DOF_Z)
					inFile.getline(buffer, 1024);	
			}

			if (dofs & DOF_Y)
			{
				token = strtok(buffer, "(");
				token = strtok(NULL, " ");
				lower[VY] = float(atof(token));
				token = strtok(NULL, " ");
				upper[VY] = float(atof(token));

				if (dofs & DOF_Z)
					inFile.getline(buffer, 1024);	
			}

			if (dofs & DOF_Z)
			{
				token = strtok(buffer, "(");
				token = strtok(NULL, " ");
				lower[VZ] = float(atof(token));
				token = strtok(NULL, " ");
				upper[VZ] = float(atof(token));
			}            
		}
		inFile.getline(buffer, 1024);
		sscanf(buffer, "%s", keyword);
	}

    Joint* pJoint = new Joint;

    pJoint->m_translation = translation;
    pJoint->m_axisRotation = axisRotation;
    pJoint->m_local.m_translation = translation;
    pJoint->SetID(m_joints.size());
    pJoint->SetName(name);
    pJoint->SetNumChannels(3);
    pJoint->SetRotationOrder("zyx"); 
    pJoint->SetDOFs(dofs);
    pJoint->SetJointLimits(lower, upper);
    pJoint->AMC = AMC;

    m_joints.push_back(pJoint);
}


bool Skeleton::LoadFromBVHFile(std::ifstream& inFile)
{
    Clear();
    AMC = false;

    vec3 offsets;
	std::string readString, jointname;
	unsigned int channelCount;
		
	inFile >> readString;
	if (readString != "HIERARCHY")
		return false;
	inFile >> readString;
	if (readString != "ROOT" && readString != "JOINT")
		return false;
	inFile.get(); //" "
	getline(inFile, jointname);// joint name
	Joint* joint = new Joint(jointname);
    joint->SetID(m_joints.size());
	joint->SetNumChannels(6);
    joint->AMC = AMC;
	m_joints.push_back(joint);
	m_pRoot = joint;
	inFile >> readString; // "{"
	inFile >> readString; // "OFFSET"
    inFile >> offsets[0] >> offsets[1] >> offsets[2];
	joint->SetLocalTranslation(offsets);
	inFile >> readString;
	if (readString != "CHANNELS")
		return false;
	inFile >> channelCount;
	joint->SetNumChannels(channelCount);
	getline(inFile, readString);	// " Xposition Yposition Zposition Zrotation Xrotation Yrotation"
    joint->SetRotationOrder(readString);
	inFile >> readString;
	while(readString != "}") 
	{		
		if (!LoadFromFileBVHRec(inFile, joint, readString))
		{
			return false;
		}
		inFile >> readString;
	}
	if (readString != "}")
		return false;

	return true;
}

bool Skeleton::LoadFromFileBVHRec(std::ifstream &inFile, Joint *pParent, std::string prefix)
{
	std::string readString, jointname;
	vec3 offsets;
	unsigned int channelCount;
	if (prefix == "JOINT")
	{
		inFile.get(); //" "
		getline(inFile, jointname);// joint name
		Joint* joint = new Joint(jointname);
        joint->SetID(m_joints.size());
        joint->AMC = AMC;
		m_joints.push_back(joint);
		Joint::AttachJoints(pParent, joint);
		inFile >> readString; // "{"
		inFile >> readString; // "OFFSET"
		inFile >> offsets[0] >> offsets[1] >> offsets[2];
		joint->SetLocalTranslation(offsets);
		inFile >> readString; // "CHANNELS"
		inFile >> channelCount;
		joint->SetNumChannels(channelCount);

		getline(inFile, readString);// " Zrotation Xrotation Yrotation"
        joint->SetRotationOrder(readString);

		inFile >> readString; // "Joint" or "}" or "End"
		while (readString != "}")
		{
			if (LoadFromFileBVHRec(inFile, joint, readString) == false)
				return false;
			inFile >> readString; // "Joint" or "}" or "End"
		}
		return true;
	}else if (prefix == "End")
	{	
		inFile.get(); //" "
		getline(inFile, jointname);// joint name
		Joint* joint = new Joint(jointname);
        joint->SetID(m_joints.size());
		joint->SetNumChannels(0);
        joint->AMC = AMC;
		m_joints.push_back(joint);
		Joint::AttachJoints(pParent, joint);
		inFile >> readString; // "{"
		inFile >> readString; // "OFFSET"
		inFile >> offsets[0] >> offsets[1] >> offsets[2];
		joint->SetLocalTranslation(offsets);
		inFile >> readString; // "}"
		return true;
	}else
		return false;
}

void Skeleton::SaveToBVHFile(std::ofstream& outFile)
{
	outFile << "HIERARCHY" << std::endl;
	outFile << "ROOT " << m_pRoot->GetName() << std::endl;
	outFile << "{" << std::endl;
	outFile << "\tOFFSET 0.00 0.00 0.00" << std::endl;
	outFile << "\tCHANNELS " << m_pRoot->GetNumChannels() << " Xposition Yposition Zposition Zrotation Xrotation Yrotation" << std::endl;
	unsigned int childCount = m_pRoot->GetNumChildren();
	for (unsigned int i = 0; i < childCount; i++)
	{
		Joint* pChild = m_pRoot->GetChildAt(i);
		SaveToFileBVHRec(outFile, pChild, 1);
	}
	outFile << "}" << std::endl;
}

void Skeleton::SaveToFileBVHRec(std::ofstream& outFile, Joint* pJoint, unsigned int level)
{
	std::string indentation = "";
	for (unsigned int i = 0; i < level; i++)
	{
		indentation += "\t";
	}

	if (pJoint->GetNumChannels() == 3)
	{
		outFile << indentation << "JOINT " << pJoint->GetName() << std::endl;
		outFile << indentation << "{" << std::endl;
		vec3 offsets = pJoint->GetLocalTranslation();
        if (AMC) offsets = pJoint->m_translation;
		outFile << indentation << "\tOFFSET " << offsets[0] << " " << offsets[1] << " " << offsets[2] << std::endl;
		outFile << indentation << "\tCHANNELS " << pJoint->GetNumChannels() << " Zrotation Xrotation Yrotation" << std::endl;
	}else
	{
		outFile << indentation << "End " << pJoint->GetName() << std::endl;
		outFile << indentation << "{" << std::endl;
		vec3 offsets = pJoint->GetLocalTranslation();
        if (AMC) offsets = pJoint->m_translation;
        outFile << indentation << "\tOFFSET " << offsets[0] << " " << offsets[1] << " " << offsets[2] << std::endl;		
	}
	unsigned int childCount = pJoint->GetNumChildren();
	for (unsigned int i = 0; i < childCount; i++)
	{
		Joint* pChild = pJoint->GetChildAt(i);
		SaveToFileBVHRec(outFile, pChild, level + 1);
	}
	outFile << indentation << "}" << std::endl;
}

Joint* Skeleton::GetJointByName(const std::string& name) const
{
	std::vector<Joint*>::const_iterator iter;
	for (iter = m_joints.begin(); iter != m_joints.end(); iter++)
	{
		if (name == ((*iter)->GetName()))
			return (*iter);
	}
	return NULL;
}

Joint* Skeleton::GetJointByID(unsigned int id) const
{
	if (id >= 0 && id < m_joints.size())
	{
		return m_joints[id];
	}else
		return NULL;
}

Joint* Skeleton::GetRootJoint() const
{
	return m_pRoot;
}

void Skeleton::AddJoint(Joint* joint, bool isRoot)
{
    joint->SetID(m_joints.size());
    m_joints.push_back(joint);
    if (isRoot) m_pRoot = joint;
}

void Skeleton::ReadFromFrame(const Frame& pFrame)
{
	if (m_joints.size() != pFrame.GetNumJoints())
    {
        std::cout << "ERROR: Cannot read pose from frame: number of joints differ\n";
		return;
    }

    m_pRoot->SetLocalTranslation(pFrame.GetRootTranslation() * mScale);
	for (unsigned int i = 0; i < m_joints.size(); i++)
	{
        Joint* joint = m_joints[i];
        joint->SetLocalRotation(pFrame.GetJointRotation(joint->GetID()));
	}

	UpdateFK(m_pRoot);
}

void Skeleton::WriteToFrame(Frame& pFrame) const
{
	pFrame.SetRootTranslation(m_pRoot->GetLocalTranslation() / GetScale());
    pFrame.SetNumJoints(m_joints.size());
	for (unsigned int i = 0; i < m_joints.size(); i++)
	{
        Joint* joint = m_joints[i];
        mat3 m = joint->GetLocalRotation();
        if (AMC) m = joint->m_axisRotation.Transpose() * m * joint->m_axisRotation;
		pFrame.SetJointRotation(joint->GetID(), m);
	}
}

//////////////////////////////////////////////////////////////////////////
// Update forward kinematics from given Joint pRoot
// If pRoot == NULL, update from root of the skeleton, which is m_pRoot

void Skeleton::UpdateFK(Joint* pRoot)
{
   if (!pRoot) pRoot = m_pRoot;
   if (!pRoot) return; // Nothing loaded
   pRoot->UpdateTransformation(true);
}

vec3 Skeleton::GetDimensions() 
{
    UpdateFK(m_pRoot);

    vec3 min(9999999999.0,9999999999.0,9999999999.0);
    vec3 max(-9999999999.0,-9999999999.0,-9999999999.0);
    for (unsigned int i = 0; i < m_joints.size(); i++)
    {
        vec3 pos = m_joints[i]->GetGlobalTranslation();
        min[0] = std::min<float>(min[0], pos[0]);
        min[1] = std::min<float>(min[1], pos[1]);
        min[2] = std::min<float>(min[2], pos[2]);

        max[0] = std::max<float>(max[0], pos[0]);
        max[1] = std::max<float>(max[1], pos[1]);
        max[2] = std::max<float>(max[2], pos[2]);
    }

    vec3 dim(0,0,0);
    dim = max - min;
    return dim;
}

void Skeleton::SetScale(float s)
{
    if (s == mScale) return;
    mScale = s;
    for (int i = 0; i < m_joints.size(); i++)
    {
        if (AMC) m_joints[i]->m_translation *= mScale;
        else
        {
            vec3 translation = m_joints[i]->GetLocalTranslation();
            m_joints[i]->SetLocalTranslation(translation * mScale);
        }
    }
}