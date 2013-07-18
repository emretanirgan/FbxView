//------------------------------------------------------------------------
// Copyright (C) 2013 by Aline Normoyle, Liming Zhao, Alla Safonova, Teresa Fan
// See License.txt for usage permissions. 


#include "Motion.h"
#include "Frame.h"
#include <string>
#include <fstream>
#include <iomanip>
#include <iostream>

#pragma warning(disable : 4244)

std::string Motion::PruneName(const std::string& name)
{
    std::string temp = name;

    int i = temp.rfind("."); 
    if (i != std::string::npos) temp = temp.replace(i, temp.size(), "");

    i = name.rfind("/");
    if (i == std::string::npos) i = name.rfind("\\"); 
    if (i != std::string::npos) temp = temp.replace(0,i+1, "");

    return temp;
}

Motion::Motion()
{
	m_name = "None";
	m_currentFrame = 0;
    m_fps = 0;
}

Motion::Motion(const Motion& orig)
{
    *this = orig;
}

Motion& Motion::operator=(const Motion& orig)
{
    if (&orig == this) return *this;
    
	m_name = orig.m_name;
    m_currentFrame = orig.m_currentFrame;
    m_fps = orig.m_fps;
    m_keyFrames = orig.m_keyFrames;
    //m_phases = orig.m_phases;

    return *this;
}

Motion::~Motion()
{
}

void Motion::Clear()
{
    //m_phases.clear();
    m_keyFrames.clear();
    m_currentFrame = 0;
    m_name = "None";
    m_fps = 0;
}

bool Motion::LoadAMCFile(const std::string& amcfile, const Skeleton& pSkeleton, float fps)
{
    std::ifstream inFile(amcfile.c_str());
	if (!inFile.is_open()) return false;

    Clear();
	m_name = amcfile;
    m_fps = fps; // Unfortunately FPS is not part of the spec

	char buffer[1024], keyword[256];
	inFile.getline(buffer, 1024);
	sscanf(buffer, "%s", keyword);

	while(strcmp(keyword, ":DEGREES") != 0)
	{
		inFile.getline(buffer, 1024);
		sscanf(buffer, "%s", keyword);
		if (strcmp(keyword, ":ROOT_YXZ") == 0)
		{
			pSkeleton.GetRootJoint()->SetRotationOrder("yxz");
		}
		if (strcmp(keyword, ":ROOT_YZX") == 0)
		{
			pSkeleton.GetRootJoint()->SetRotationOrder("yzx");
		}
		if (strcmp(keyword, ":FOOT_3DOF") == 0)
		{            
            Joint* lfoot = pSkeleton.GetJointByName("lfoot");
            Joint* rfoot = pSkeleton.GetJointByName("rfoot");
            if (lfoot) lfoot->SetDOFs(DOF_X | DOF_Y | DOF_Z);
            if (rfoot) rfoot->SetDOFs(DOF_X | DOF_Y | DOF_Z);
		}
	}
	inFile.getline(buffer, 1024); // frame number
	while(inFile.good())
	{
		Frame frame;
        frame.LoadFromAMCFile(inFile, pSkeleton);
		m_keyFrames.push_back(frame);
	}

	inFile.close();
    return true;
}

bool Motion::LoadFromBVHFile(std::ifstream& inFile, const Skeleton& pSkeleton)
{
    Clear();

    std::string readString;
	unsigned int frameCount;
	inFile >> readString;
	if (readString != "MOTION")
		return false;
	inFile >> readString;
	if(readString != "Frames:") 
		return false;
	inFile >> frameCount;
	inFile >> readString; // "Frame"
	getline(inFile, readString); // " Time: 0.033333"
    m_fps = 1.0/atof(&(readString.c_str()[6]));

    // Read frames
    m_keyFrames.resize(frameCount);
	for (unsigned int i = 0; i < frameCount; i++)
	{
		m_keyFrames[i].LoadFromBVHFile(inFile, pSkeleton);
	}
	
	return true;
}

std::vector<std::string> parse(const std::string& str)
{
    std::vector<std::string> tokens;
    std::string temp = str;
    char* token = strtok((char*) temp.c_str(), " ");
    while (token)
    {
        tokens.push_back(token);
        token = strtok(NULL, " ");
    }
    return tokens;
}

/*bool Motion::LoadANNFile(const std::string& annfile, const Skeleton& skeleton)
{
    if (m_keyFrames.size() == 0) 
    {
        std::cout << "ERROR: LoadANNFile - not motion loaded\n";
        return false;
    }

    std::ifstream inFile(annfile.c_str());
    if (!inFile.is_open()) return false;

    try
    {
        unsigned int index = 0;
	    while(!inFile.eof())
        {
            char frames[512];
            char joints[512];
            char beh[512];
	        inFile.getline(frames, 512); 
	        inFile.getline(joints, 512); 
	        inFile.getline(beh, 512); 

            if (strlen(frames) == 0) break;
            std::vector<std::string> frameTokens = parse(frames);
            std::vector<std::string> jointTokens = parse(joints);
            std::vector<std::string> behTokens = parse(beh);

            int startFrame = atof(frameTokens[0].c_str());
            int endFrame = atof(frameTokens[1].c_str());
            for (unsigned int j = 0; j < jointTokens.size(); j++)
            {
                Joint* joint = skeleton.GetJointByName(jointTokens[j]);
                if (!joint) 
                {
                    std::cout << "Could not find joint: " << jointTokens[j] << std::endl;
                    continue;
                }
                for (unsigned int k = startFrame; k < endFrame; k++)
                {
                    if (k >= m_keyFrames.size()) std::cout << "ERROR IN ANN FILE " << annfile << std::endl;
                    //m_keyFrames[k].SetContact(joint->GetID(), true);
                }
            }

            if (behTokens.size() > 0)
            {
                for (unsigned int k = startFrame; k < endFrame; k++)
                {
                    m_keyFrames[k].SetBehaviorType(behTokens[0]);
                }
            }
        }
        //ComputePhases();
	}
    catch(...)
    {
	    inFile.close();
        return false;
    }
    inFile.close();
    return true;
}

bool Motion::SaveANNFile(const std::string& annfile, const Skeleton& pSkeleton)
{
    std::ofstream outFile(annfile.c_str());
	if (!outFile.is_open())
	{
        std::cout << "WARNING: Could not open " << annfile.c_str() << " for writing" << std::endl;
        return false;
	}

    //ComputePhases(); // Make sure these are up to date
    for (unsigned int i = 0; i < m_phases.size(); i++)
    {
        unsigned int startFrame = m_phases[i].first;
        unsigned int endFrame = m_phases[i].second;
        const Frame& frame = GetFrame(startFrame);
        //unsigned int contacts = frame.GetContactFlags();
        unsigned int behavior = frame.GetBehaviorType();

        outFile << startFrame << " " << endFrame << std::endl;
        //outFile << frame.GetContactString(pSkeleton) << std::endl;
        outFile << frame.GetBehaviorTypeString() << std::endl;
    }

    outFile.close();
    return true;
}*/

void Motion::SaveToBVHFile(std::ofstream& outFile, const Skeleton& pSkeleton)
{
	outFile << "MOTION" << std::endl;
	outFile << "Frames: " << GetNumFrames() << std::endl;
	outFile << "Frame Time: " << 1.0/m_fps << std::endl;
	for (unsigned int i = 0; i < m_keyFrames.size(); i++)
	{
		m_keyFrames[i].SaveToBVHFile(outFile, pSkeleton);
	}
}

bool Motion::SaveAMCFile(const std::string& filename, const Skeleton& pSkeleton)
{
    std::ofstream outFile(filename.c_str());
	if (outFile.is_open())
	{
		outFile << "#Unknown ASF file" << std::endl;

        Joint* root = pSkeleton.GetRootJoint();
        if (root->GetRotationOrder() == "xyz") outFile << ":ROOT_XYZ" << std::endl;
        else if (root->GetRotationOrder() == "xzy") outFile << ":ROOT_XZY" << std::endl;
        else if (root->GetRotationOrder() == "yxz") outFile << ":ROOT_YXZ" << std::endl;
        else if (root->GetRotationOrder() == "yzx") outFile << ":ROOT_YZX" << std::endl;
        else if (root->GetRotationOrder() == "zxy") outFile << ":ROOT_ZXY" << std::endl;
        else if (root->GetRotationOrder() == "zyx") outFile << ":ROOT_ZYX" << std::endl;
			
        Joint* foot = pSkeleton.GetJointByName("lfoot");
        unsigned int mask = DOF_X|DOF_Y|DOF_Z;
        if (foot && (foot->GetDOFs() == mask))
        {
            outFile << ":FOOT_3DOF" << std::endl;
        }
		outFile << ":FULLY-SPECIFIED" << std::endl;
		outFile << ":DEGREES" << std::endl;
		for (unsigned int i = 0; i < m_keyFrames.size(); i++)
		{
			outFile << i + 1 << std::endl;
			m_keyFrames[i].SaveToAMCFile(outFile, pSkeleton);
		}
		outFile.close();
        return true;
	}
    return false;
}

double Motion::GetFps() const
{
    return m_fps;
}

void Motion::SetFps(double fps)
{
    m_fps = fps;
}

void Motion::SetCurrentIndex(unsigned int currentFrame)
{
    m_currentFrame = std::max<unsigned int>(0, 
                     std::min<unsigned int>(m_keyFrames.size()-1, currentFrame));
}

const Frame& Motion::GetFrame(unsigned int index) const
{
	assert(index >= 0 && index < m_keyFrames.size());
	return m_keyFrames[index];
}

Frame& Motion::GetFrame(unsigned int index)
{
	assert(index >= 0 && index < m_keyFrames.size());
	return m_keyFrames[index];
}

const Frame& Motion::GetCurrentFrame() const
{
   return GetFrame(m_currentFrame);
}

void Motion::SetFrame(unsigned int index, const Frame& f)
{
    m_keyFrames[index] = f;
}

unsigned int Motion::GetNumJoints() const
{
    if (m_keyFrames.size() == 0) return 0;
    return m_keyFrames[0].GetNumJoints();
}

unsigned int Motion::GetNumFrames() const
{
    return m_keyFrames.size();
}

const std::string& Motion::GetName() const
{
    return m_name;
}

void Motion::SetName(const std::string& name)
{
    m_name = name;
}

void Motion::ReOrient(const vec3& startPos, const mat3& startOri)
{
   if (m_keyFrames.size() == 0) return;

   Transform transformDesired(startPos, startOri);

   Frame firstKey = m_keyFrames[0];
   Transform transformInv(firstKey.GetRootTranslation(), firstKey.GetJointRotation(0));
   transformInv = transformInv.Inverse();

   for (unsigned int i = 0; i < m_keyFrames.size(); i++)
   {
      Frame& key = m_keyFrames[i];
      
      Transform keyTransform(key.GetRootTranslation(), key.GetJointRotation(0));
      keyTransform = transformDesired * transformInv * keyTransform;

      key.SetRootTranslation(keyTransform.m_translation);
      key.SetJointRotation(0, keyTransform.m_rotation);
   }
}

void Motion::AppendFrame(const Frame& frame)
{
    m_keyFrames.push_back(frame);
}

void Motion::Append(const Motion& motion)
{
	for (unsigned int i = 0; i < motion.GetNumFrames(); i++)
	{
		AppendFrame(motion.GetFrame(i));
	}
}

Motion Motion::SubMotion(int startFrame, int endFrame)
{
    startFrame = std::max<unsigned int>(0, startFrame);
    endFrame = std::min<unsigned int>(GetNumFrames(), endFrame);

    Motion m;
    m.SetFps(GetFps());
	for (unsigned int i = startFrame; i < endFrame; i++)
	{
		m.AppendFrame(GetFrame(i));
	}
    return m;
}

void Motion::SetSubMotion(int startFrame, int endFrame, const Motion& m)
{
    assert (endFrame-startFrame <= m.GetNumFrames());

    int index = 0;
    for (unsigned int i = startFrame; i < endFrame; i++)
    {
        SetFrame(i, m.GetFrame(index++));
    }
}
/*
void Motion::ComputeAnnotations(Skeleton& skeleton, const vec3& upAxis, float floorThresh, float velThresh)
{
    if (GetNumFrames() == 0) return;

    CachePointData(skeleton);
    float fps = GetFps();

    for (unsigned int i = 0; i < GetNumFrames(); i++)
    {
        int startFrame = std::min(i, GetNumFrames()-2);
        int endFrame = std::min(i+1, GetNumFrames()-1);
        std::cout << "---------------- " << i << std::endl;

        // Check end effectors...Are they close to the floor and unmoving?
        //GetFrame(startFrame).SetContactFlags(0);

        
//unsigned long contact = 0;
        for (unsigned int j = 0; j < skeleton.GetNumJoints(); j++)
        {
            Joint* joint = skeleton.GetJointByID(j);
            if (joint->GetNumChildren() != 0) continue;

            vec3 vel = GetFrame(endFrame).GetJointPosition(j) - 
                       GetFrame(startFrame).GetJointPosition(j);
            vel /= fps;
            float height = Dot(upAxis, GetFrame(startFrame).GetJointPosition(j));
            
            std::cout << joint->GetName() << " speed = " << vel.Length() << " height = " << height << std::endl;
            if (vel.Length() < velThresh && height < floorThresh)
            {
                //GetFrame(startFrame).SetContact(j, true);
            }
        }
    }

    std::string motionName = GetName();
    std::string annName = motionName.replace(motionName.end()-4,motionName.end(), ".ann");
    std::ofstream ofile(annName.c_str());
    //unsigned long contact = GetFrame(0).GetContactFlags();
    unsigned int startFrame = 0;
    for (unsigned int i = 0; i < GetNumFrames(); i++)
    {
        // Format: startFrame endFrame\ncontactName(s)\nbehaviorName
        if (contact != GetFrame(i).GetContactFlags())
        {
            ofile << startFrame << " " << i << std::endl;
            std::string contactStr = GetFrame(startFrame).GetContactString(skeleton);
            ofile << contactStr.c_str() << std::endl; 
            ofile << "" << std::endl;
            startFrame = i;
            contact = GetFrame(i).GetContactFlags();
        }
    }
    ofile << startFrame << " " << GetNumFrames() << std::endl;
    //ofile << GetFrame(startFrame).GetContactString(skeleton) << std::endl; 
    ofile << "" << std::endl;
    ofile.close();
}

void Motion::CachePointData(Skeleton& skeleton)
{
    if (GetNumFrames() == 0) return;
    if (GetFrame(0).HasPointCacheData()) return;
    for (unsigned int i = 0; i < GetNumFrames(); i++)
    {
        GetFrame(i).CachePointData(skeleton);
    }
}

unsigned int Motion::GetNumPhases() const
{
    return m_phases.size();
}

Motion::Phase Motion::GetPhase(unsigned int id) const
{
    assert(id >= 0 && id < m_phases.size());
    return m_phases[id];
}

Motion::Phase Motion::GetPhaseContainingFrameId(unsigned int id) const
{
    static Motion::Phase empty = std::make_pair<unsigned int, unsigned int>(0,0);

    for (unsigned int i = 0; i < m_phases.size(); i++)
    {
        if (id >= m_phases[i].first && id < m_phases[i].second)
        {
            return m_phases[i];
        }
    }
    return empty;
}

std::vector<Frame> Motion::GetFrames(const Motion::Phase& phase)
{
    std::vector<Frame> results;
    for (unsigned int i = phase.first; i < phase.second; i++)
    {
        results.push_back(GetFrame(i));
    }
    return results;
}

const Frame& Motion::GetFrame(const Motion::Phase& phase, float percent) const
{
    unsigned int totalFrames = phase.second - phase.first - 1;
    int frameOffset = (int) (percent * totalFrames);
    return GetFrame(phase.first + frameOffset);
}

unsigned int Motion::GetContactFlags(const Motion::Phase& phase)
{
    if (m_phases.size() == 0) return 0;
    if (m_keyFrames.size() == 0) return 0;
    return GetFrame(phase.first).GetContactFlags();
}

void Motion::ComputePhases()
{
    m_phases.clear();
    if (m_keyFrames.size() == 0) return;

    Phase phase;
    phase.first = 0;
    unsigned int contacts = m_keyFrames[0].GetContactFlags();
    for (unsigned int i = 1; i < m_keyFrames.size(); i++)
    {
        unsigned int newContacts = m_keyFrames[i].GetContactFlags();
        if (newContacts != contacts)
        {
            phase.second = i;
            m_phases.push_back(phase);
            phase.first = i;
            contacts = newContacts;
        }
    }

    phase.second = m_keyFrames.size();
    m_phases.push_back(phase);
}

void Motion::Downsample(int factor)
{
    Motion origMotion = *this;
    Clear();

    for (unsigned int i = 0; i < origMotion.GetNumFrames(); i += factor)
    {
        AppendFrame(origMotion.GetFrame(i));
    }
    m_fps = origMotion.GetFps()/(float) factor;
}*/