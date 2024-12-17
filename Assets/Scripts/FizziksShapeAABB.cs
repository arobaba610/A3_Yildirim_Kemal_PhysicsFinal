using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class FizziksShapeAABB : FizziksShape
{
    public Vector3 min; // World-space minimum point
    public Vector3 max; // World-space maximum point



    void Start()
    {

    }

    // Recalculate bounds using the object's world-space renderer
    public void RecalculateBounds()
    {
        MeshFilter meshFilter = GetComponent<MeshFilter>();
        if (meshFilter == null)
        {

            return;
        }

        Mesh mesh = meshFilter.mesh;
        Vector3 worldPosition = transform.position;

        // Calculate bounds in world space with the pivot offset
        Vector3 localMin = mesh.bounds.min;
        Vector3 localMax = mesh.bounds.max;
        min = worldPosition + Vector3.Scale(localMin, transform.localScale);
        max = worldPosition + Vector3.Scale(localMax, transform.localScale);


    }


    void OnDrawGizmos()
    {
        Gizmos.color = Color.green;
        Gizmos.DrawWireCube((min + max) / 2, max - min); // Visualize AABB
    }


    public override Shape GetShape()
    {
        return Shape.AABB;
    }
}
