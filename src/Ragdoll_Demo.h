/***************************************************************************

This source file is part of OGREBULLET
(Object-oriented Graphics Rendering Engine Bullet Wrapper)
For the latest info, see http://www.ogre3d.org/phpBB2addons/viewforum.php?f=10

Copyright (c) 2007 tuan.kuranes@gmail.com (Use it Freely, even Statically, but have to contribute any changes)



This source file is not LGPL, it's public source code that you can reuse.
-----------------------------------------------------------------------------*/
#ifndef _OGREBULLET_Ragdoll_Demo__H
#define _OGREBULLET_Ragdoll_Demo__H

#include "OgreBulletDynamics.h"
#include "OgreBulletListener.h"

// -------------------------------------------------------------------------
class Ragdoll_Demo : public OgreBulletListener 
{
public:

    Ragdoll_Demo() : OgreBulletListener()
      {
          mName = "Ragdoll Demo";
      };
    virtual ~Ragdoll_Demo(){};

    void init(Ogre::Root *root, Ogre::RenderWindow *win, OgreBulletApplication *application);

    void keyPressed(BULLET_KEY_CODE key);

	void shootToKill();

	void button0Pressed();

	bool frameStarted(Ogre::Real elapsedTime);
	bool frameEnded(Ogre::Real elapsedTime);

 };


#endif //_OGREBULLET_Constraints_Demo__H

