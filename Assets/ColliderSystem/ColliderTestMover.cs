using UnityEngine;

public class ColliderTestMover : MonoBehaviour
{
    private Bounds movementBounds;
    private Vector2 velocity;
    private bool initialized;

    public void Initialize(Bounds bounds, Vector2 direction, float speed)
    {
        movementBounds = bounds;
        velocity = direction.sqrMagnitude > 0f ? direction.normalized * speed : Vector2.right * speed;
        initialized = true;
        ClampToBounds();
    }

    private void Update()
    {
        if (!initialized)
        {
            return;
        }

        Vector3 position = transform.position;
        position.x += velocity.x * Time.deltaTime;
        position.y += velocity.y * Time.deltaTime;
        transform.position = position;
        BounceAtBounds();
    }

    private void BounceAtBounds()
    {
        Vector3 position = transform.position;
        Vector3 min = movementBounds.min;
        Vector3 max = movementBounds.max;

        if (position.x < min.x)
        {
            position.x = min.x;
            velocity.x = Mathf.Abs(velocity.x);
        }
        else if (position.x > max.x)
        {
            position.x = max.x;
            velocity.x = -Mathf.Abs(velocity.x);
        }

        if (position.y < min.y)
        {
            position.y = min.y;
            velocity.y = Mathf.Abs(velocity.y);
        }
        else if (position.y > max.y)
        {
            position.y = max.y;
            velocity.y = -Mathf.Abs(velocity.y);
        }

        transform.position = position;
    }

    private void ClampToBounds()
    {
        Vector3 position = transform.position;
        Vector3 min = movementBounds.min;
        Vector3 max = movementBounds.max;
        position.x = Mathf.Clamp(position.x, min.x, max.x);
        position.y = Mathf.Clamp(position.y, min.y, max.y);
        transform.position = position;
    }
}
