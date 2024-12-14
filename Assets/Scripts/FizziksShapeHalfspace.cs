using UnityEngine;

public class FizziksShapeHalfspace : FizziksShape
{
    public override Shape GetShape()
    {
        return Shape.Halfspace;
    }

    public Vector3 GetPosition()
    {
        return transform.position; // Position of the halfspace
    }

    public Vector3 Normal()
    {
        return transform.up.normalized; // Unit vector normal to the halfspace
    }

    private void OnValidate()
    {
        // Ensure the object's transform has a valid rotation for the halfspace
        transform.up = transform.up.normalized;
    }

    private void Update()
    {
        // Optional: Debug draw the normal vector for visualization
        Debug.DrawLine(transform.position, transform.position + Normal() * 2f, Color.blue);
    }
}
