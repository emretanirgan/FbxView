//-----------------------------------------------------------------------------
// Copyright (C) 2013 by Aline Normoyle, Liming Zhao, Alla Safonova, Teresa Fan

#ifndef Player_H_
#define Player_H_

#include "Frame.h"
#include "Motion.h"
#include "Skeleton.h"
#include <vector>

class Player
{
public:
	Player();
	virtual ~Player();

    virtual bool LoadBVHFile(const std::string& filename);
    virtual bool LoadAMCFile(const std::string& asffile, const std::string& amcfile, float fps = 120.0);
    virtual bool LoadAMCFile(const std::string& amcfile, float fps = 120.0);
    virtual bool LoadASFFile(const std::string& asffile);
	virtual bool SaveBVHFile(const std::string& filename);
    virtual bool SaveAMCFile(const std::string& filename);

    virtual void ConvertAMC2BVH();

	virtual bool IsValid();
    virtual void Update(int frameNum);
    virtual void Update();

    Motion& GetMotion();
    const Motion& GetMotion() const;
    void SetMotion(const Motion& motion);

    Skeleton& GetSkeleton();
    const Skeleton& GetSkeleton() const;
    void SetSkeleton(const Skeleton& skeleton);

protected:
    virtual void init() {}

protected:
	Skeleton m_skeleton;
    Motion m_motion;
};

#endif