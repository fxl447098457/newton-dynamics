/* Copyright (c) <2003-2022> <Newton Game Dynamics>
* 
* This software is provided 'as-is', without any express or implied
* warranty. In no event will the authors be held liable for any damages
* arising from the use of this software.
* 
* Permission is granted to anyone to use this software for any purpose,
* including commercial applications, and to alter it and redistribute it
* freely
*/

#include "ndSandboxStdafx.h"
#include "ndMeshLoader.h"
#include "ndPhysicsUtils.h"
#include "ndPhysicsWorld.h"
#include "ndMakeStaticMap.h"
#include "ndUnicyclePlayer.h"
#include "ndDemoEntityNotify.h"
#include "ndDemoEntityManager.h"

namespace ndUnicyclePlayer
{
	class ndHelpLegend_Sac : public ndDemoEntityManager::ndDemoHelper
	{
		virtual void PresentHelp(ndDemoEntityManager* const scene) override
		{
			ndVector color(1.0f, 1.0f, 0.0f, 0.0f);
			scene->Print(color, "Cart Pole is the classic hello world of reinforcement learning");
			scene->Print(color, "it is use to test the correctness of an algorithm implementation.");
			scene->Print(color, "The model is trained using Soft Actor Critic(SAC).");
			scene->Print(color, "It consists of a pole attached by a hinge to a sliding cart.");
			scene->Print(color, "The objective goal was to train a neural network to keep");
			scene->Print(color, "the pole balanced in an upright position.");
			scene->Print(color, "You can interact with the simulation and try.");
			scene->Print(color, "to knock the pole over using the mouse.");
		}
	};
	
	class ndHelpLegend_Ppo : public ndDemoEntityManager::ndDemoHelper
	{
		virtual void PresentHelp(ndDemoEntityManager* const scene) override
		{
			ndVector color(1.0f, 1.0f, 0.0f, 0.0f);
			scene->Print(color, "Cart Pole is the classic hello world of reinforcement learning");
			scene->Print(color, "It is used to test the correctness of an algorithm implementation.");
			scene->Print(color, "The model is trained using Proximal Policy Gradient (PPO).");
			scene->Print(color, "It consists of a pole attached by a hinge to a sliding cart.");
			scene->Print(color, "The objective goal was to train a neural network to keep");
			scene->Print(color, "the pole balanced in an upright position.");
			scene->Print(color, "You can interact with the simulation and try.");
			scene->Print(color, "to knock the pole over using the mouse.");
		}
	};

	class ndPlaybackController : public ndController
	{
		public:
		ndPlaybackController()
			:ndController()
		{
		}
	};

	ndController::ndController()
		:ndModelNotify()
		,m_agent(nullptr)
		,m_timestep(0.0f)
	{
	}

	void ndController::Update(ndFloat32 timestep)
	{
		m_timestep = timestep;
		m_agent->Step();
	}

	void ndController::ResetModel()
	{
		ndTrace(("%s\n", __FUNCTION__));
		//ndMatrix cartMatrix(ndGetIdentityMatrix());
		//cartMatrix.m_posit = m_cart->GetMatrix().m_posit;
		//cartMatrix.m_posit.m_x = ndFloat32(0.0f);
		////cartMatrix.m_posit.m_x = ndFloat32(10.0f) * (ndRand() - ndFloat32(0.5f));
		//cartMatrix.m_posit.m_y = ndFloat32(0.1f);
		//m_cart->SetMatrix(cartMatrix);
		//
		//const ndMatrix poleMatrix(m_poleHinge->CalculateGlobalMatrix1());
		//m_pole->SetMatrix(poleMatrix);
		//
		//m_pole->SetOmega(ndVector::m_zero);
		//m_pole->SetVelocity(ndVector::m_zero);
		//
		//m_cart->SetOmega(ndVector::m_zero);
		//m_cart->SetVelocity(ndVector::m_zero);
		//
		//GetModel()->GetAsModelArticulation()->ClearMemory();
	}

	bool ndController::IsTerminal() const
	{
		ndTrace(("%s\n", __FUNCTION__));
		return false;
		//const ndJointHinge* const hinge = (ndJointHinge*)*m_poleHinge;
		//const ndJointSlider* const slider = (ndJointSlider*)*m_slider;
		//ndFloat32 angle = hinge->GetAngle();
		//ndFloat32 speed = slider->GetSpeed();
		//bool isdead = ndAbs(angle) > (REWARD_MIN_ANGLE * ndFloat32(2.0f));
		//isdead = isdead || (ndAbs(speed) > ndFloat32(3.0f));
		//return isdead;
	}

	ndBrainFloat ndController::CalculateReward() const
	{
		ndTrace(("%s\n", __FUNCTION__));
		return 0;
		//if (IsTerminal())
		//{
		//	// a terminal reward of zero should make for smoother MDPs. 
		//	// training small networks could be much harder with negative terminal rewards..
		//	// return ndBrainFloat(-1.0f);
		//	return ndBrainFloat(-1.0f);
		//}
		//
		//ndJointHinge* const hinge = (ndJointHinge*)*m_poleHinge;
		//ndJointSlider* const slider = (ndJointSlider*)*m_slider;
		//
		//ndFloat32 angle = hinge->GetAngle();
		//ndFloat32 omega = hinge->GetOmega();
		//ndFloat32 speed = slider->GetSpeed();
		//
		//ndFloat32 invSigma2 = ndFloat32(50.0f);
		//ndFloat32 speedReward = ndExp(-invSigma2 * speed * speed);
		//ndFloat32 omegaReward = ndExp(-invSigma2 * omega * omega);
		//ndFloat32 angleReward = ndExp(-invSigma2 * angle * angle);
		//
		//// make sure the reward is never negative, to avoid the possibility of  
		//// MDP states with negative values.
		//ndFloat32 reward = ndFloat32(0.3f) * angleReward + ndFloat32(0.3f) * omegaReward + ndFloat32(0.4f) * speedReward;
		//return ndBrainFloat(reward);
	}

	void ndController::ApplyActions(ndBrainFloat* const actions)
	{
		ndTrace(("%s\n", __FUNCTION__));
		//ndBrainFloat action = actions[0];
		//ndBrainFloat accel = PUSH_ACCEL * action;
		//ndFloat32 pushForce = accel * (m_cart->GetAsBodyDynamic()->GetMassMatrix().m_w);
		//
		//ndJointSlider* const slider = (ndJointSlider*)*m_slider;
		//const ndMatrix matrix(slider->CalculateGlobalMatrix0());
		//
		//ndVector force(m_cart->GetAsBodyDynamic()->GetForce() + matrix.m_front.Scale(pushForce));
		//m_cart->GetAsBodyDynamic()->SetForce(force);
	}

	ndBrainFloat ndController::IsOnAir() const
	{
		ndBodyKinematic::ndContactMap& contacts = m_ball->GetAsBodyKinematic()->GetContactMap();
		ndBodyKinematic::ndContactMap::Iterator it(contacts);
		for (it.Begin(); it; it++)
		{
			ndContact* const contact = *it;
			if (contact->IsActive())
			{
				return ndBrainFloat(0.0f);
			}
		}
		return ndBrainFloat(1.0f);
	};

	void ndController::GetObservation(ndBrainFloat* const observation)
	{
		ndFloat32 poleOmega = ((ndJointHinge*)*m_poleHinge)->GetOmega();
		ndFloat32 poleAngle = ((ndJointHinge*)*m_poleHinge)->GetAngle() / ND_MAX_LEG_JOINT_ANGLE;
		ndFloat32 wheelOmega = ((ndJointRoller*)*m_ballRoler)->GetOmega();
		wheelOmega *= 1.0f / 20.0f;

		//ndBrainFloat speed = ndClamp(((ndJointSlider*)*m_slider)->GetSpeed(), ndBrainFloat(-6.0f), ndBrainFloat(6.0f)) / 6.0f;
		ndBrainFloat speed = 0.0f;

		observation[m_velocity] = speed;
		observation[m_poleAngle] = ndBrainFloat(poleAngle);
		observation[m_poleOmega] = ndBrainFloat(poleOmega);
		observation[m_wheelOmega] = ndBrainFloat(wheelOmega);
		observation[m_hasSupportContact] = IsOnAir();
	}

	void ndController::CreateArticulatedModel(
		ndDemoEntityManager* const scene,
		ndModelArticulation* const model,
		ndSharedPtr<ndMesh> mesh,
		ndSharedPtr<ndRenderSceneNode> visualMesh)
	{
		auto CreateRigidBody = [scene](ndSharedPtr<ndMesh>& mesh, ndSharedPtr<ndRenderSceneNode>& visualMesh, ndFloat32 mass, ndBodyDynamic* const parentBody)
		{
			ndSharedPtr<ndShapeInstance> shape(mesh->CreateCollision());

			ndBodyKinematic* const body = new ndBodyDynamic();
			body->SetNotifyCallback(new ndDemoEntityNotify(scene, visualMesh, parentBody));
			body->SetMatrix(mesh->CalculateGlobalMatrix());
			body->SetCollisionShape(*(*shape));
			body->GetAsBodyDynamic()->SetMassMatrix(mass, *(*shape));
			return body;
		};

		// add the root body
		m_topBox = ndSharedPtr<ndBody>(CreateRigidBody(mesh, visualMesh, BOX_MASS, nullptr));
		ndModelArticulation::ndNode* const modelRootNode = model->AddRootBody(m_topBox);

		// add the pole mesh and body
		ndSharedPtr<ndMesh> poleMesh(mesh->GetChildren().GetFirst()->GetInfo());
		ndSharedPtr<ndRenderSceneNode> poleEntity(visualMesh->GetChildren().GetFirst()->GetInfo());
		m_pole = ndSharedPtr<ndBody>(CreateRigidBody(poleMesh, poleEntity, POLE_MASS, m_topBox->GetAsBodyDynamic()));

		// add ball mesh and body
		ndSharedPtr<ndMesh> ballMesh(poleMesh->GetChildren().GetFirst()->GetInfo());
		ndSharedPtr<ndRenderSceneNode> ballEntity(poleEntity->GetChildren().GetFirst()->GetInfo());
		m_ball = ndSharedPtr<ndBody>(CreateRigidBody(ballMesh, ballEntity, BALL_MASS, m_pole->GetAsBodyDynamic()));

		// add links
		const ndMatrix poleMatrix(m_pole->GetMatrix());
		m_poleHinge = ndSharedPtr<ndJointBilateralConstraint>(new ndJointHinge(poleMatrix, m_pole->GetAsBodyKinematic(), m_topBox->GetAsBodyKinematic()));
		ndModelArticulation::ndNode* const poleNode = model->AddLimb(modelRootNode, m_pole, m_poleHinge);

		const ndMatrix ballMatrix(m_ball->GetMatrix());
		m_ballRoler = ndSharedPtr<ndJointBilateralConstraint>(new ndJointRoller(ballMatrix, m_ball->GetAsBodyKinematic(), m_pole->GetAsBodyKinematic()));
		((ndJointRoller*)*m_ballRoler)->SetAsSpringDamperPosit(0.01f, 1000.0f, 15.0f);
		model->AddLimb(poleNode, m_ball, m_ballRoler);

		// fix to the word with a plane joint
		ndWorld* const world = scene->GetWorld();
		const ndMatrix planeMatrix(m_topBox->GetMatrix());
		m_plane = ndSharedPtr<ndJointBilateralConstraint>(new ndJointPlane(planeMatrix.m_posit, planeMatrix.m_right, m_topBox->GetAsBodyKinematic(), world->GetSentinelBody()));
		model->AddCloseLoop(m_plane);
	}

	ndModelArticulation* ndController::CreateModel(ndDemoEntityManager* const scene, const ndMatrix& location, const ndRenderMeshLoader& loader, const char* const name)
	{
		ndAssert(0);
		ndMatrix matrix(location);
		matrix.m_posit = FindFloor(*scene->GetWorld(), matrix.m_posit, 200.0f);
		matrix.m_posit.m_y += ndFloat32(0.1f);
		loader.m_mesh->m_matrix = loader.m_mesh->m_matrix * matrix;
		
		ndSharedPtr<ndRenderSceneNode> visualMesh(loader.m_renderMesh->Clone());
		visualMesh->SetTransform(loader.m_mesh->m_matrix);
		visualMesh->SetTransform(loader.m_mesh->m_matrix);
		
		ndModelArticulation* const model = new ndModelArticulation();
		ndSharedPtr<ndModelNotify> controller(new ndPlaybackController());
		model->SetNotifyCallback(controller);
		ndPlaybackController* const playerController = (ndPlaybackController*)(*controller);
		playerController->CreateArticulatedModel(scene, model, loader.m_mesh, visualMesh);

		char nameExt[256];
		snprintf(nameExt, sizeof(nameExt) - 1, "%s.dnn", name);
		ndString fileName(ndGetWorkingFileName(nameExt));
		ndSharedPtr<ndBrain> policy(ndBrainLoad::Load(fileName.GetStr()));
		playerController->m_agent = ndSharedPtr<ndBrainAgent>(new ndController::ndAgent(policy, playerController));

		// add model a visual mesh to the scene and world
		ndWorld* const world = scene->GetWorld();
		world->AddModel(model);
		scene->AddEntity(visualMesh);
		model->AddBodiesAndJointsToWorld();

		return model;
	}
}

using namespace ndUnicyclePlayer;

void ndUnicyclePlayer_SAC(ndDemoEntityManager* const scene)
{
	ndSharedPtr<ndBody> mapBody(BuildFloorBox(scene, ndGetIdentityMatrix(), "marbleCheckBoard.png", 0.1f, true));

	// add a help message
	ndSharedPtr<ndDemoEntityManager::ndDemoHelper> demoHelper(new ndHelpLegend_Sac());
	scene->SetDemoHelp(demoHelper);

	ndMatrix matrix(ndGetIdentityMatrix());
	ndRenderMeshLoader loader(*scene->GetRenderer());
	loader.LoadMesh(ndGetWorkingFileName("unicycle.nd"));
	ndController::CreateModel(scene, matrix, loader, CONTROLLER_NAME_SAC);

	matrix.m_posit.m_x -= 0.0f;
	matrix.m_posit.m_y += 0.5f;
	matrix.m_posit.m_z += 2.0f;
	ndQuaternion rotation(ndVector(0.0f, 1.0f, 0.0f, 0.0f), 90.0f * ndDegreeToRad);
	scene->SetCameraMatrix(rotation, matrix.m_posit);
}

void ndUnicyclePlayer_PPO(ndDemoEntityManager* const scene)
{
	ndSharedPtr<ndBody> mapBody(BuildFloorBox(scene, ndGetIdentityMatrix(), "marbleCheckBoard.png", 0.1f, true));

	// TO DO
	ndAssert(0);
#if 0
	// add a help message
	ndSharedPtr<ndDemoEntityManager::ndDemoHelper> demoHelper(new ndHelpLegend_Ppo());
	scene->SetDemoHelp(demoHelper);

	ndMatrix matrix(ndGetIdentityMatrix());
	ndRenderMeshLoader loader(*scene->GetRenderer());
	loader.LoadMesh(ndGetWorkingFileName("unicycle.nd"));
	ndController::CreateModel(scene, matrix, loader, CONTROLLER_NAME_PPO);

	matrix.m_posit.m_x -= 0.0f;
	matrix.m_posit.m_y += 0.5f;
	matrix.m_posit.m_z += 2.0f;
	ndQuaternion rotation(ndVector(0.0f, 1.0f, 0.0f, 0.0f), 90.0f * ndDegreeToRad);
	scene->SetCameraMatrix(rotation, matrix.m_posit);
#endif
}
