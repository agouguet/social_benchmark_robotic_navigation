using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using Unity.Robotics.ROSTCPConnector.ROSGeometry;
using RosMessageTypes.Std;

namespace SEAN.TF
{
    public class AMCLPublisher : MonoBehaviour
    {
        ROSConnection ros;
        public float publishMessageFrequency = 0.5f;
        private float timeElapsed;
        public string topicName;

        public Transform PublishedTransform;
        public string FrameId = "map";

        private RosMessageTypes.Geometry.PoseWithCovarianceStampedMsg message;

        private float previousRealTime;
        private Vector3 previousPosition = Vector3.zero;
        private Quaternion previousRotation = Quaternion.identity;

        private double[] identityMatrix = {1, 0, 0, 0, 0, 0,
                                          0, 1, 0, 0, 0, 0,
                                          0, 0, 1, 0, 0, 0,
                                          0, 0, 0, 1, 0, 0,
                                          0, 0, 0, 0, 1, 0,
                                          0, 0, 0, 0, 0, 1};

        void Start()
        {
            ros = ROSConnection.GetOrCreateInstance();
            ros.RegisterPublisher<RosMessageTypes.Geometry.PoseWithCovarianceStampedMsg>(topicName);
            InitializeMessage();

        }

        private void FixedUpdate()
        {
            UpdateMessage();
        }

        private void InitializeMessage()
        {
            message = new RosMessageTypes.Geometry.PoseWithCovarianceStampedMsg();
            message.pose.covariance = identityMatrix;
            message.header.frame_id = FrameId;
        }

        private void UpdateMessage()
        {
            float deltaTime = Time.realtimeSinceStartup - previousRealTime;
            timeElapsed += Time.deltaTime;

            if (timeElapsed <= publishMessageFrequency)
            {
                return;
            }

            previousRealTime = Time.realtimeSinceStartup;
            previousPosition = PublishedTransform.position;
            previousRotation = PublishedTransform.rotation;

            SEAN.instance.clock.UpdateMHeader(message.header);
            message.pose.pose.position = Util.Geometry.GetGeometryPoint(PublishedTransform.position.To<FLU>());
            message.pose.pose.orientation = Util.Geometry.GetGeometryQuaternion(PublishedTransform.rotation.To<FLU>());
            ros.Publish(topicName, message);
            timeElapsed = 0;
        }
    }
}