// Copyright (c) 2021, Members of Yale Interactive Machines Group, Yale University,
// Nathan Tsoi
// All rights reserved.
// This source code is licensed under the BSD-style license found in the
// LICENSE file in the root directory of this source tree. 

using System.Collections.Generic;
using UnityEngine;

namespace SEAN.Scenario.Agents
{
    public class CrowdCrossing : BaseAgentManager
    {
        public List<IVI.INavigable> agents;
        public List<Trajectory.TrackedGroup> groups;

        public GameObject agentPrefab;

        public Collider zoneA;
        public Collider zoneB;

        public float desiredSpeed = 0.5f;
        public float maxSpeed = 0.5f;

        public float perception = 3.0f;

        public GameObject agentsPool;
        private List<List<Pose>> positions;
        private Dictionary<IVI.INavigable, List<Pose>> agentGoals;

        public Pose openGroupLocation = Pose.identity;

        public int numObjects = 20;


        public string scenario_name
        {
            get
            {
                return "CrowdCrossing";
            }
        }

        protected override void Awake()
        {
            base.Awake();
            agentGoals = new Dictionary<IVI.INavigable, List<Pose>>();
        }

        void Update()
        {
            // UnityEngine.Debug.Log(current);
            // no need to plan if we only have static groups
            foreach (var agent in agents)
            {
                if (agent.CloseEnough())
                {
                    // Set the next goal
                    agent.InitDest(agentGoals[agent][2].position);
                    Pose currentGoal = agentGoals[agent][0];
                    agentGoals[agent].RemoveAt(0);
                    agentGoals[agent].Add(currentGoal);
                }
            }
            
        }

        public void NewScenario()
        {
            // Must clear before registering the new group
            Clear();

            for (int i = 0; i < numObjects; i++)
            {   
                List<Pose> agentPosition = new List<Pose>();
                int choice = Random.Range( 0, 2);

                if (choice == 0)
                {
                    Vector3 startPosition = RandomPointInBounds(zoneA.bounds);
                    startPosition.y = 0;
                    agentPosition.Add(new Pose(startPosition, new Quaternion()));
                    Vector3 endPosition = RandomPointInBounds(zoneB.bounds);
                    endPosition.y = 0;
                    agentPosition.Add(new Pose(endPosition, new Quaternion()));
                }
                else if (choice == 1)
                {
                    Vector3 startPosition = RandomPointInBounds(zoneB.bounds);
                    startPosition.y = 0;
                    agentPosition.Add(new Pose(startPosition, new Quaternion()));
                    Vector3 endPosition = RandomPointInBounds(zoneA.bounds);
                    endPosition.y = 0;
                    agentPosition.Add(new Pose(endPosition, new Quaternion()));
                }


                int size = agentPosition.Count;

                for (int j = 0; j < 10; j++)
                {
                    for (int x = 0; x < size; x++)
                    {
                        agentPosition.Add(agentPosition[x]);
                    }
                    
                }

                // Debug.Log(agentPosition.Count);

                IVI.INavigable agent = SpawnAgent("Agent_" + (i+1), desiredSpeed, maxSpeed, false, perception, agentPosition[0]);
                agentGoals.Add(agent, agentPosition);
                if (agentPosition.Count > 1)
                    agent.InitDest(agentPosition[1].position);
                else
                    agent.staticAgent = true;
            }
        }

        void Clear()
        {
            if (!agentsPool) { return; }
            agents = new List<IVI.INavigable>();
            groups = new List<Trajectory.TrackedGroup>();
            foreach (Transform child in agentsPool.transform)
            {
                GameObject.Destroy(child.gameObject);
            }
            
            openGroupLocation = Pose.identity;
            agentGoals.Clear();
        }

        IVI.INavigable SpawnAgent(string name, float desiredSpeed, float maxSpeed, bool staticAgent, float perception, Pose pose)
        {
            var sfRandom = Instantiate(agentPrefab, Vector3.zero, Quaternion.identity);
            IVI.INavigable agent = sfRandom.GetComponentInChildren<IVI.INavigable>();
            agent.name = name;
            agent.desiredSpeed = desiredSpeed;
            agent.maxSpeed = maxSpeed;
            agent.staticAgent = staticAgent;
            agent.perception = perception;
            agent.transform.position = pose.position;
            agent.transform.rotation = pose.rotation;
            agent.transform.parent = agentsPool.transform;
            agents.Add(agent);
            return agent;
        }

        void SpawnGroup(Pose groupCenter)
        {
            IVI.GroupDataLoader.GroupData group = IVI.GroupDataLoader.groupData[Random.Range(0, IVI.GroupDataLoader.groupData.Count)];
            int openGroupIdx = Random.Range(0, group.pos.Count);
            for (int i = 0; i < group.pos.Count; i++)
            {

                float angle = Mathf.Atan(group.pos[i].x / group.pos[i].z);
                if (group.pos[i].z > 0)
                {
                    angle += Mathf.PI;
                }
                Quaternion rotation = Quaternion.Euler(0, angle * Mathf.Rad2Deg, 0);
                Pose pose = new Pose(groupCenter.position + group.pos[i], rotation);
                if (i == openGroupIdx)
                {
                    openGroupLocation = pose;
                }
                // else
                // {
                //     SpawnAgent("Agent_" + i, pose);
                // }
            }
        }

        void SpawnAgents()
        {
            // for (int i = 0; i < positions.Count; i++)
            // {
            //     Pose spawnPose = positions[i][0];
            //     IVI.INavigable agent = SpawnAgent("Agent_" + (i+1), spawnPose);
            //     agentGoals.Add(agent, positions[i]);
            //     agent.InitDest(positions[i][1].position);
            // }
            
            
            // if (situation == PedestrianBehavior.SocialSituation.JoinGroup || situation == PedestrianBehavior.SocialSituation.LeaveGroup)
            // {
            //     foreach (Pose pose in positions)
            //     {
            //         SpawnGroup(pose);
            //     }
            //     return;
            // }
        }

        
        public static Vector3 RandomPointInBounds(Bounds bounds) {
            return new Vector3(
                Random.Range(bounds.min.x, bounds.max.x),
                Random.Range(bounds.min.y, bounds.max.y),
                Random.Range(bounds.min.z, bounds.max.z)
            );
        }
    }
}