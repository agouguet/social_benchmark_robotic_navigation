// Copyright (c) 2021, Members of Yale Interactive Machines Group, Yale University,
// Nathan Tsoi
// All rights reserved.
// This source code is licensed under the BSD-style license found in the
// LICENSE file in the root directory of this source tree. 

using System;
using System.Collections.Generic;
using UnityEngine;

namespace SEAN.Tasks
{
    public class ABNavHandCraft : Base
    {
        public GameObject pointA, pointB;

        public override void Start()
        {
            base.Start();
            InitializeTaskPositions();
            onNewTask.Invoke();
            Publish(interactiveGoal);
        }

        

        private void InitializeTaskPositions()
        {
            pointA.SetActive(false);
            pointB.SetActive(false);

            robotStart.transform.position = pointA.transform.position;
            robotStart.transform.rotation = pointA.transform.rotation;
            playerStart.transform.position = pointA.transform.position + new Vector3(0, 1, 0);
            playerStart.transform.rotation = pointA.transform.rotation;

            robotGoal.transform.position = pointB.transform.position;
            robotGoal.transform.rotation = pointB.transform.rotation;
            playerGoal.transform.position = pointB.transform.position;
            playerGoal.transform.rotation = pointB.transform.rotation;

            Publish(interactiveGoal);
        }

        protected override bool NewTask()
        {

            InitializeTaskPositions();
            sean.robot.detector.Clear();
            Scenario.PedestrianBehavior.HandCrafted scenario = (Scenario.PedestrianBehavior.HandCrafted)sean.pedestrianBehavior;
            // starts the scenario by spawning people and setting their destinations
            scenario.NewScenario();
            return true;
        }

        // protected override void CheckNewTask()
        // {
        //     float distToWayPoint = Vector3.Distance(sean.robot.transform.position, robotGoal.transform.position);
        //     // print(sean.robot.transform.position + "   " + robotGoal.transform.position+"   "+distToWayPoint);
        //     if (distToWayPoint < completionDistance)
        //     {
        //         // NewTask();
        //         if (maximumNumberOfTasks > 0 && number >= maximumNumberOfTasks)
        //         {
        //             Debug.Log("Completed " + number + " of  " + maximumNumberOfTasks + " tasks, exiting");
        //             #if UNITY_EDITOR
        //             UnityEditor.EditorApplication.isPlaying = false;
        //             #else
        //             Application.Quit();
        //             #endif
        //         }
        //         StartNewTask();
        //     }
        // }

    }
}