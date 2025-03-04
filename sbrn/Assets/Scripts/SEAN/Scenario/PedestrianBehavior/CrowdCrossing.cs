// Copyright (c) 2021, Members of Yale Interactive Machines Group, Yale University,
// Nathan Tsoi
// All rights reserved.
// This source code is licensed under the BSD-style license found in the
// LICENSE file in the root directory of this source tree. 

using System.Collections.Generic;
using System.Collections;
using System.Linq;
using UnityEngine;

namespace SEAN.Scenario.PedestrianBehavior
{
    public class CrowdCrossing : Base
    {
        Agents.CrowdCrossing agentManager;
        // Top level situation in the environment
        // public GameObject socialSituationEnv;
        // Top level of handcrafted situations, named with the same name as the enum
        // public GameObject spawnLocations;

        public float delayToSpawn;

        public override string scenario_name
        {
            get
            {
                return "CrowdCrossing";
            }
        }

        void Start(){
            base.Start();
            NewScenario();
        }

        private IEnumerator Wait(){
            yield return new WaitForSeconds(delayToSpawn);
        }

        public void NewScenario()
        {
            Wait();
            // socialSituationEnv.SetActive(true);
            // spawnLocations.SetActive(true);
            agentManager = (Agents.CrowdCrossing)Agents.BaseAgentManager.instance;
            // agentManager.NewScenario(socialSituationEnv, spawnLocations);
            agentManager.NewScenario();
        }

        public override Trajectory.TrackedGroup[] groups
        {
            get
            {
                if (agentManager == null)
                {
                    return new Trajectory.TrackedGroup[0];
                }
                return agentManager.groups.ToArray();
            }
        }

        public override Trajectory.TrackedAgent[] agents
        {
            get
            {
                if (agentManager == null)
                {
                    return new Trajectory.TrackedAgent[0];
                }
                return agentManager.agents.ToArray();
            }
        }
    }
}