﻿/*
    Copyright (c) 2020, Members of Yale Interactive Machines Group, Yale University,
    Nathan Tsoi
    All rights reserved.
    This source code is licensed under the BSD-style license found in the
    LICENSE file in the root directory of this source tree. 
*/

using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using Unity.Robotics.ROSTCPConnector;

public class BoolPublisher : MonoBehaviour
{
    public string Topic;
    ROSConnection ros;

    private RosMessageTypes.Std.BoolMsg message;
    private bool isInfoUpdated;

    void Start()
    {
        ros = ROSConnection.GetOrCreateInstance();
        ros.RegisterPublisher<RosMessageTypes.Std.BoolMsg>(Topic);
        //base.Start();
        InitializeMessage();
    }

    private void Update()
    {
        if (isInfoUpdated)
            ros.Publish(Topic, message);
    }

    private void InitializeMessage()
    {
        message = new RosMessageTypes.Std.BoolMsg();
    }

    public void UpdateInfo(bool data)
    {
        message.data = data;
        isInfoUpdated = true;
    }
}