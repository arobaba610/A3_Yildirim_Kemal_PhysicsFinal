using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class FizziksShapeSphere : FizziksShape
{
    public override Shape GetShape()
    {
        return Shape.Sphere;
    }

    public float radius = 1f; // Radius of the sphere

    public void UpdateScale()
    {
        // Update the object's scale based on the radius
        transform.localScale = new Vector3(radius, radius, radius) * 2f;
    }

    private void OnValidate()
    {
        // Ensure the radius is within reasonable bounds
        radius = Mathf.Max(0.01f, radius); // Radius cannot be less than 0.01
        UpdateScale();
    }

    private void Update()
    {
        // Optional: Debug draw the sphere for visualization
        Debug.DrawLine(transform.position, transform.position + Vector3.up * radius, Color.green);
    }
}
