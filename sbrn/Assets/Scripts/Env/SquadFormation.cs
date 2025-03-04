using System;
using System.Collections;
using System.Collections.Generic;
using System.Linq;
using UnityEngine;

public class SquadFormation : MonoBehaviour
{
   public enum Formation
   {
       Square,
       Circle
   }

   [SerializeField]
   Formation m_Formation;

   public Transform squadMember;
   public GameObject parent;
   public int columns = 4;
   public int space = 10;
   public int numObjects = 20;
   public float yOffset = 1;

   // Use this for initialization
   void Start()
   {
       ChangeFormation();
   }

   // Update is called once per frame
   void Update()
   {

   }

   private void ChangeFormation()
   {
       switch (m_Formation)
       {
           case Formation.Square:

               for (int i = 0; i < numObjects; i++)
               {
                   Transform go = Instantiate(squadMember);
                   go.parent = transform;
                   Vector3 pos = FormationSquare(i);
                   go.position = new Vector3(0, 0, 0);
                //    go.position = new Vector3(transform.position.x + pos.x, 0, transform.position.y + pos.y);
                   go.Rotate(new Vector3(0, -90, 0));
               }
               break;

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
                   Transform insObj = Instantiate(squadMember, new Vector3(0, 0, 0), rot);
                   insObj.parent = transform;
                   insObj.rotation = rot;
               }
               break;
       }
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