using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class FizziksShapeAABB : FizziksShape
{
    public Vector3 min;
    public Vector3 max;

    void Start()
    {
        RecalculateBounds();
    }

    public void RecalculateBounds()
    {
        Vector3 halfExtents = transform.localScale * 0.5f;
        min = transform.position - halfExtents;
        max = transform.position + halfExtents;
    }

    public override Shape GetShape()
    {
        return Shape.AABB;
    }
}
