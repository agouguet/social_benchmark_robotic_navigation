using UnityEditor;
using UnityEngine;

[CustomEditor(typeof(CameraAgentDetector))]
public class FieldOfViewEditor : Editor
{
    private void OnSceneGUI()
    {
        CameraAgentDetector fov = (CameraAgentDetector)target;
        Handles.color = Color.white;
        Handles.DrawWireArc(fov.transform.position, Vector3.up, Vector3.forward, 360, fov.radius);

        Vector3 viewAngle01 = DirectionFromAngle(fov.transform.eulerAngles.y, -fov.angle / 2);
        Vector3 viewAngle02 = DirectionFromAngle(fov.transform.eulerAngles.y, fov.angle / 2);

        Handles.color = Color.yellow;
        Handles.DrawLine(fov.transform.position, fov.transform.position + viewAngle01 * fov.radius);
        Handles.DrawLine(fov.transform.position, fov.transform.position + viewAngle02 * fov.radius);

        Handles.color = Color.green;
        foreach(var agent in fov.agentsView)
        {   
            if(agent.Value)
                Handles.DrawLine(fov.transform.position, agent.Key.transform.position + new Vector3(0f, 0.5f, 0f));
        }
    }

    private Vector3 DirectionFromAngle(float eulerY, float angleInDegrees)
    {
        angleInDegrees += eulerY;

        return new Vector3(Mathf.Sin(angleInDegrees * Mathf.Deg2Rad), 0, Mathf.Cos(angleInDegrees * Mathf.Deg2Rad));
    }
}