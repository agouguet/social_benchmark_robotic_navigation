using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class CameraAgentDetector : MonoBehaviour
{

    public float radius;
    [Range(0,360)]
    public float angle;

    public Dictionary<GameObject, bool> agentsView = new Dictionary<GameObject, bool>();

    public LayerMask targetMask;
    public LayerMask obstructionMask;

    // Start is called before the first frame update
    void Start()
    {
        StartCoroutine(LateStart(0.2f));
        
    }

    IEnumerator LateStart(float waitTime)
    {
        yield return new WaitForSeconds(waitTime);
        foreach(GameObject agent in GameObject.FindGameObjectsWithTag("NavAgent"))
        {
            agentsView.Add(agent, false);
        }
        
        StartCoroutine(FOVRoutine());   
    }

    private IEnumerator FOVRoutine()
    {
        WaitForSeconds wait = new WaitForSeconds(0.2f);

        while (true)
        {
            yield return wait;
            FieldOfViewCheck();
        }
    }

    private void FieldOfViewCheck()
    {
        Collider[] rangeChecks = Physics.OverlapSphere(transform.position, radius, targetMask);

        if (rangeChecks.Length != 0)
        {
            foreach(Collider agentCollider in rangeChecks)
            {
                Transform target = agentCollider.transform;
                Vector3 target_position = target.position + new Vector3(0f, 0.5f, 0f);
                Vector3 directionToTarget = (target_position - transform.position).normalized;

                if (Vector3.Angle(transform.forward, directionToTarget) < angle / 2)
                {
                    float distanceToTarget = Vector3.Distance(transform.position, target_position);

                    // print(target + "    " + Physics.Raycast(transform.position, directionToTarget, distanceToTarget, obstructionMask));

                    if (!Physics.Raycast(transform.position, directionToTarget, distanceToTarget, obstructionMask))
                        agentsView[target.gameObject] = true;
                    else
                        agentsView[target.gameObject] = false;
                }
                else
                    agentsView[target.gameObject] = false;
            }
        }
        else
        {
            List<GameObject> keys = new List<GameObject>(agentsView.Keys);
            foreach(GameObject agent in keys)
            {
                agentsView[agent] = false;
            }
        }
    }

    public void Clear()
    {
        agentsView = new Dictionary<GameObject, bool>();
    }
}
