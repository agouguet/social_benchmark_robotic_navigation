using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class PositionAgent : MonoBehaviour
{
    private float _desiredSpeed;
    public float minSpeed = 0.0f;
    public float maxSpeed = Parameters.MAX_VEL;
    public bool staticAgent = false;
    public float perception = Parameters.PERCEPTION_RADIUS_AGENT;

    void Start ()
    {
        _desiredSpeed = Random.Range(minSpeed, maxSpeed);
    }

    public float desiredSpeed
    {
        get
        {
            if (_desiredSpeed == null)
                _desiredSpeed = Random.Range(minSpeed, maxSpeed);
            return _desiredSpeed;
        }
    }

    public float newDesiredSpeed()
    {
        _desiredSpeed = Random.Range(minSpeed, maxSpeed);
        return _desiredSpeed;
    }

    void OnDrawGizmosSelected()
    {
        for(int i = 1; i < this.transform.childCount; i++)
        {
            Transform Go_one = this.transform.GetChild(i-1);
            Transform Go_two = this.transform.GetChild(i);

            Gizmos.color = Color.blue;
            Gizmos.DrawLine(Go_one.position, Go_two.position);
        }
    }
}
