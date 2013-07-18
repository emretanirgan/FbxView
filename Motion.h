//-----------------------------------------------------------------------------
// Copyright (C) 2013 by Aline Normoyle, Liming Zhao, Alla Safonova, Teresa Fan


#ifndef Motion_H_
#define Motion_H_

#include "Skeleton.h"


class Frame;
class Motion
{
public:
	Motion();
    Motion(const Motion& orig); // Deep copy
    Motion& operator=(const Motion& orig); // Deep copy
	virtual ~Motion();
	
    bool LoadFromBVHFile(std::ifstream& inFile, const Skeleton& pSkeleton);	// Read from BVH file
    bool LoadAMCFile(const std::string& amcfile, const Skeleton& pSkeleton, float fps = 120.0);	// Read from BVH file
    void SaveToBVHFile(std::ofstream& outFile, const Skeleton& pSkeleton);	// Write to BVH file
    bool SaveAMCFile(const std::string& filename, const Skeleton& pSkeleton);

    double GetFps() const;
    void SetFps(double fps);
	void SetCurrentIndex(unsigned int currentFrame);
	unsigned int GetCurrentIndex() const { return m_currentFrame; }

   	const Frame& GetFrame(unsigned int index) const;	// Get frame at index, allowing reading and writing
   	Frame& GetFrame(unsigned int index);	// Get frame at index, allowing reading and writing
	const Frame& GetCurrentFrame() const;
    void SetFrame(unsigned int index, const Frame& f);

    void AppendFrame(const Frame& frame);
    void Append(const Motion& motion);
    Motion SubMotion(int startFrame, int endFrame);
    void SetSubMotion(int startFrame, int endFrame, const Motion& m);

    void Clear();

	unsigned int GetNumJoints() const;
	unsigned int GetNumFrames() const;

	const std::string& GetName() const;
    void SetName(const std::string& name);
    void ReOrient(const vec3& startPos, const mat3& startOri);

private:

    std::vector<Frame> m_keyFrames;
	unsigned int m_currentFrame;
    std::string m_name;
    double m_fps;

public:
    static std::string PruneName(const std::string& name);
};



#endif