#include "aActor.h"

#pragma warning(disable : 4018)



/****************************************************************
*
*    	    Actor functions
*
****************************************************************/

AActor::AActor() 
{
	m_pInternalSkeleton = new ASkeleton();
	m_pSkeleton = m_pInternalSkeleton;

	m_BVHController = new BVHController();
	m_BVHController->setActor(this);

	m_IKController = new IKController();
	m_IKController->setActor(this);

	// code to update additional Actor data goes here
	resetGuide();

}

AActor::AActor(const AActor* actor)
{
	*this = *actor;
}

AActor& AActor::operator = (const AActor& actor)
{
	// Performs a deep copy
	if (&actor == this)
	{
		return *this;
	}
	m_pSkeleton = actor.m_pSkeleton;

	// code to update additional Actor data goes here


	return *this;
}

AActor::~AActor()
{
	 delete m_IKController;
	 delete m_BVHController;
	 delete m_pInternalSkeleton;

}

void AActor::clear()
{
	// looks like it is clearing more times than the number of actors.  as a result, m_pSkeleton is not defined for last case.
	m_pSkeleton->clear();  

	// code to update additional Actor data goes here
}

void AActor::update()
{
	if (!m_pSkeleton->getRootNode() )
		 return; // Nothing loaded
	else m_pSkeleton->update();

	// code to update additional Actor data goes here

}

ASkeleton* AActor::getSkeleton()
{
	return m_pSkeleton;
}

void AActor::setSkeleton(ASkeleton* pExternalSkeleton)
{
	m_pSkeleton = pExternalSkeleton;
}

void AActor::resetSkeleton()
{
	m_pSkeleton = m_pInternalSkeleton;
}

BVHController* AActor::getBVHController()
{
	return m_BVHController;
}

IKController* AActor::getIKController()
{
	return m_IKController;
}

void AActor::updateGuideJoint(vec3 guideTargetPos)
{
	if (!m_pSkeleton->getRootNode()) { return; }

	// TODO: 
	// 1.	Set the global position of the guide joint to the global position of the root joint
	// 2.	Set the y component of the guide position to 0
	// 3.	Set the global rotation of the guide joint towards the guideTarget

	// step 1
	vec3 pos = m_Guide.getGlobalRotation() * m_pSkeleton->getRootNode()->getGlobalTranslation() +
			   m_Guide.getGlobalTranslation(); // root position in global space

	// step 2 
	guideTargetPos[1] = 0;
	pos[1] = 0;
	m_Guide.setGlobalTranslation(pos);

	// step 3 
	// NOTE: my character orientation is not updating whatsoever
	vec3 look = (guideTargetPos - pos).Normalize();
	vec3 up = m_Guide.getGlobalRotation().GetCol(1); // up vector
	m_Guide.setGlobalRotation(mat3(up.Cross(look), up, look).Transpose());
}

void AActor::solveFootIK(float leftHeight, float rightHeight, bool rotateLeft, bool rotateRight, vec3 leftNormal, vec3 rightNormal)
{
	if (!m_pSkeleton->getRootNode()) { return; }
	AJoint* leftFoot = m_pSkeleton->getJointByID(m_IKController->mLfootID);
	AJoint* rightFoot = m_pSkeleton->getJointByID(m_IKController->mRfootID);

	// TODO: 
	// The normal and the height given are in the world space

	// 1.Update the local translation of the root based on the left height and the right height
	// get the min height because the other leg can bend upwards
	float minHeight = leftHeight > rightHeight ? rightHeight : leftHeight;

	AJoint* root = m_pSkeleton->getRootNode();
	vec3 pos = root->getLocalTranslation();
	root->setLocalTranslation(vec3(pos[0], pos[1] + minHeight, pos[2])); // new pos from world to guide to root space

	// 2.	Update the character with Limb-based IK 
	vec3 leftPos = leftFoot->getGlobalTranslation();
	vec3 rightPos = rightFoot->getGlobalTranslation();

	// update the left leg
	leftPos = vec3(leftPos[0], leftPos[1] + leftHeight, leftPos[2]);
	ATarget leftTarget = ATarget();
	leftTarget.setGlobalTranslation(leftPos);
	m_IKController->IKSolver_Limb(m_IKController->mLfootID, leftTarget);

	// update the right leg
	rightPos = vec3(rightPos[0], rightPos[1] + rightHeight, rightPos[2]);
	ATarget rightTarget = ATarget();
	rightTarget.setGlobalTranslation(rightPos);
	m_IKController->IKSolver_Limb(m_IKController->mRfootID, rightTarget);
	
	// Rotate Foot
	if (rotateLeft)
	{
		// Update the local orientation of the left foot based on the left normal
		vec3 right = m_Guide.getGlobalRotation().GetCol(0);
		leftNormal[0] = 0;
		leftFoot->setLocalRotation(mat3(right, leftNormal, leftNormal.Cross(right)));
	}
	if (rotateRight)
	{
		// Update the local orientation of the right foot based on the right normal
		vec3 right = m_Guide.getGlobalRotation().GetCol(0);
		rightNormal[0] = 0;
		rightFoot->setLocalRotation(mat3(right, rightNormal, rightNormal.Cross(right)));
	}
	m_pSkeleton->update();
}
