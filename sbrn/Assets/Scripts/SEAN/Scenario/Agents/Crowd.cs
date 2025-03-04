// Copyright (c) 2021, Members of Yale Interactive Machines Group, Yale University,
// Nathan Tsoi
// All rights reserved.
// This source code is licensed under the BSD-style license found in the
// LICENSE file in the root directory of this source tree. 

using System.Collections.Generic;
using UnityEngine;

namespace SEAN.Scenario.Agents
{
    public class Crowd : BaseAgentManager
    {
        public List<IVI.INavigable> agents;
        public List<Trajectory.TrackedGroup> groups;

        public GameObject agentPrefab;

        public GameObject agentsPool;
        private List<List<Pose>> positions;
        private Dictionary<IVI.INavigable, List<Pose>> agentGoals;

        public Pose openGroupLocation = Pose.identity;

        public enum Formation
        {
            Square,
            Circle
        }

        [SerializeField]
        Formation m_Formation;

        public Transform squadMember;
        public int columns = 4;
        public int space = 10;
        public int numObjects = 20;
        public float yOffset = 1;

        public string scenario_name
        {
            get
            {
                return "Crowd";
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

            switch (m_Formation)
            {
                case Formation.Circle:

                    Vector3 center = transform.position;
                    for (int i = 0; i < numObjects; i++)
                    {
                        Vector3 pos = RandomCircle(center, 5.0f);
                        var rot = Quaternion.LookRotation(center - pos);

                        if (Terrain.activeTerrain != null)
                        {
                            pos.y = Terrain.activeTerrain.SampleHeight(pos);
                        }

                        pos.y = pos.y + yOffset;
                        //    Transform insObj = Instantiate(squadMember, pos, rot);
                        Transform agentPositions = Instantiate(squadMember, new Vector3(0, 0, 0), rot);
                        List<Pose> agentPosition = new List<Pose>();
                        foreach (Transform agent_pos in agentPositions.transform)
                        {
                            Vector3 temp = agent_pos.position;
                            temp.y = 0;
                            agentPosition.Add(new Pose(temp, agent_pos.rotation));
                        }

                        int size = agentPosition.Count;

                        for (int j = 0; j < 10; j++)
                        {
                            for (int x = 0; x < size; x++)
                            {
                                agentPosition.Add(agentPosition[x]);
                            }
                            
                        }
                        // agentPosition.Add(agentPosition[0]);

                        float desiredSpeed = agentPositions.GetComponent<PositionAgent>().desiredSpeed;
                        float maxSpeed = agentPositions.GetComponent<PositionAgent>().maxSpeed;
                        float perception = agentPositions.GetComponent<PositionAgent>().perception;
                        bool staticAgent = agentPositions.GetComponent<PositionAgent>().staticAgent;

                        IVI.INavigable agent = SpawnAgent("Agent_" + (i+1), desiredSpeed, maxSpeed, staticAgent, perception, agentPosition[0]);
                        agentGoals.Add(agent, agentPosition);
                        if (agentPosition.Count > 1)
                            agent.InitDest(agentPosition[1].position);
                        else
                            agent.staticAgent = true;
                        Destroy(agentPositions.gameObject);
                    }
                    break;
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

        Vector2 FormationSquare(int index) // call this func for all your objects
        {
            float posX = (index % columns) * space;
            float posY = (index / columns) * space;
            return new Vector2(posX, posY);
        }

        Vector3 RandomCircle(Vector3 center, float radius)
        {
            float ang = UnityEngine.Random.value * 360;
            Vector3 pos;
            pos.x = center.x + radius * Mathf.Sin(ang * Mathf.Deg2Rad);
            pos.z = center.z + radius * Mathf.Cos(ang * Mathf.Deg2Rad);
            pos.y = center.y;
            return pos;
        }
    }
}