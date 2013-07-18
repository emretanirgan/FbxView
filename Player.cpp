//-----------------------------------------------------------------------------
// Copyright (C) 2013 by Aline Normoyle, Liming Zhao, Alla Safonova, Teresa Fan

#include "Player.h"
#include <fstream>
#pragma warning(disable : 4244)

Player::Player()
{
}

Player::~Player()
{
}

bool Player::LoadAMCFile(const std::string& amcfile, float fps)
{
    if (m_skeleton.GetNumJoints() == 0)
    {
        std::cout << "Load an ASF file before loading an AMC file!\n";
        return false;
    }

	bool status = m_motion.LoadAMCFile(amcfile, m_skeleton, fps);
    if (status)
    {
        m_motion.SetName(amcfile);

        // Try to load annotations, if possible
        std::string motionName = amcfile;
        std::string annName = motionName.replace(motionName.end()-4,motionName.end(), ".ann");

        m_motion.SetCurrentIndex(0);
        if (m_motion.GetNumFrames() > 0) m_skeleton.ReadFromFrame(m_motion.GetCurrentFrame());
    }
    else
    {
        m_motion.SetName("None");
    }

    init();
	return status;
}

bool Player::LoadASFFile(const std::string& asffile)
{
    return m_skeleton.LoadASFFile(asffile);
    init();
}

bool Player::LoadAMCFile(const std::string& asffile, const std::string& amcfile, float fps)
{
	bool status = LoadASFFile(asffile) && LoadAMCFile(amcfile, fps);
    init();
	return status;
}

bool Player::LoadBVHFile(const std::string& filename)
{
    std::ifstream inFile(filename.c_str());
	if (!inFile.is_open())
	{
        std::cout << "WARNING: Could not open " << filename.c_str() << std::endl;
        return false;
	}

	bool status = m_skeleton.LoadFromBVHFile(inFile) && 
                  m_motion.LoadFromBVHFile(inFile, m_skeleton);

    if (status)
    {
        m_motion.SetName(filename);
        // Try to load annotations, if possible
        std::string motionName = filename;
        std::string annName = motionName.replace(motionName.end()-4,motionName.end(), ".ann");

        m_motion.SetCurrentIndex(0);
        if (m_motion.GetNumFrames() > 0) m_skeleton.ReadFromFrame(m_motion.GetCurrentFrame());
    }
    else
    {
        m_motion.SetName("None");
    }

    init();
    inFile.close();				
	return status;
}

bool Player::SaveBVHFile(const std::string& filename)
{
    std::ofstream outFile(filename.c_str());
	if (!outFile.is_open())
	{
        std::cout << "WARNING: Could not open " << filename.c_str() << " for writing" << std::endl;
        return false;
	}

	m_skeleton.SaveToBVHFile(outFile);
	m_motion.SaveToBVHFile(outFile, m_skeleton);

    init();
    outFile.close();
	return true;
}

bool Player::SaveAMCFile(const std::string& filename)
{
    return m_motion.SaveAMCFile(filename, m_skeleton);
}

void Player::Update(int frameNum)
{
	if (!IsValid()) return;
    m_skeleton.ReadFromFrame(m_motion.GetFrame(frameNum));
}

void Player::Update()
{
	if (!IsValid()) return;
    m_skeleton.ReadFromFrame(m_motion.GetCurrentFrame());
}

Motion& Player::GetMotion()
{
	return m_motion;  
}

const Motion& Player::GetMotion() const
{
	return m_motion;  
}

void Player::SetMotion(const Motion& motion)
{
    m_motion = motion;
}

void Player::SetSkeleton(const Skeleton& skeleton)
{
    m_skeleton = skeleton;
}

Skeleton& Player::GetSkeleton()
{
	return m_skeleton;
}

const Skeleton& Player::GetSkeleton() const
{
	return m_skeleton;
}

bool Player::IsValid()
{
	return (m_skeleton.GetNumJoints() > 0 && 
        m_motion.GetNumFrames() > 0);
}

void Player::ConvertAMC2BVH()
{
    Skeleton& skeleton = GetSkeleton();
    skeleton.AMC = false; // convert from AMC to BVH skeleton

    int siteId = 0;
    int numJoints = skeleton.GetNumJoints();
    std::vector<Joint*> endEffectors;
    for (int i = 0; i < numJoints; i++)
    {
        Joint* joint = skeleton.GetJointByID(i);
        if (joint->GetNumChildren() == 0) // add end site
        {
            endEffectors.push_back(joint);
        }
    }

    for (int i = 0; i < endEffectors.size(); i++)
    {
        Joint* joint = endEffectors[i];

        // Create end site
        Joint* newJoint = new Joint();
        newJoint->AMC = false;
        newJoint->m_local.m_rotation = identity3D;
        newJoint->m_local.m_translation = joint->m_translation;
        newJoint->SetNumChannels(3);

        char buff[32];
        sprintf(buff, "Site%d", siteId++);
        newJoint->SetName(buff);
        skeleton.AddJoint(newJoint);
        Joint::AttachJoints(joint, newJoint);
        newJoint->UpdateTransformation();

        while (joint->GetParent())
        {
            joint->AMC  = false;

            if (joint->GetParent() == GetSkeleton().GetRootJoint())
            {
                joint->m_local.m_translation = vec3Zero;
            }
            else
            {
                joint->m_local.m_translation = joint->GetParent()->m_translation;
            }
            joint = joint->GetParent();
        }
    }

    for (int i = 0; i < GetMotion().GetNumFrames(); i++)
    {
        Frame frame = GetMotion().GetFrame(i);

        Frame newFrame;
        newFrame.SetNumJoints(skeleton.GetNumJoints());
        newFrame.SetRootTranslation(frame.GetRootTranslation());

        for (int j = 0; j < skeleton.GetNumJoints(); j++)
        {
            Joint* joint = skeleton.GetJointByID(j);
            if (j < frame.GetNumJoints())
            {
                mat3 rot = frame.GetJointRotation(j);
                rot = joint->m_axisRotation * rot * joint->m_axisRotation.Transpose();
                newFrame.SetJointRotation(j, rot);
            }
            else
            {
                newFrame.SetJointRotation(j, identity3D);
            }
        }
        GetMotion().SetFrame(i, newFrame);
    }
}