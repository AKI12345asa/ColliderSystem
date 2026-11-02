using System.Collections.Generic;
using UnityEngine;

public class ColliderSystemTest : MonoBehaviour
{
    [Header("Collider prefabs")]
    [SerializeField] private GameObject boxPrefab;
    [SerializeField] private GameObject spherePrefab;
    [SerializeField] private GameObject capsulePrefab;
    [SerializeField] private GameObject meshPrefab;

    [Header("Spawn counts")]
    [Min(0), SerializeField] private int boxCount;
    [Min(0), SerializeField] private int sphereCount;
    [Min(0), SerializeField] private int capsuleCount;
    [Min(0), SerializeField] private int meshCount;

    [Header("Test area")]
    [SerializeField] private Bounds movementBounds = new Bounds(Vector3.zero, new Vector3(20f, 12f, 2f));
    [SerializeField, Min(0f)] private float minSpeed = 1f;
    [SerializeField, Min(0f)] private float maxSpeed = 4f;
    [SerializeField] private int randomSeed = 12345;
    [SerializeField] private bool detectCollisions = true;

    private readonly List<GameObject> spawnedObjects = new List<GameObject>();
    private readonly List<Collider> spawnedColliders = new List<Collider>();
    private readonly List<GameObject> prefabSnapshot = new List<GameObject>(4);
    private readonly int[] countSnapshot = new int[4];
    private bool hasSnapshot;

    private void Awake()
    {
        Physics.autoSyncTransforms = false;
        Physics.simulationMode = SimulationMode.Script;
        RegenerateObjects();
    }

    private void Update()
    {
        if (ConfigurationChanged())
        {
            RegenerateObjects();
        }
    }

    private void FixedUpdate()
    {
       ColliderSystem.UpdateCollision();
    }

    private bool ConfigurationChanged()
    {
        return !hasSnapshot ||
               prefabSnapshot[0] != boxPrefab ||
               prefabSnapshot[1] != spherePrefab ||
               prefabSnapshot[2] != capsulePrefab ||
               prefabSnapshot[3] != meshPrefab ||
               countSnapshot[0] != boxCount ||
               countSnapshot[1] != sphereCount ||
               countSnapshot[2] != capsuleCount ||
               countSnapshot[3] != meshCount;
    }

    private void RegenerateObjects()
    {
        ClearSpawnedObjects();

        prefabSnapshot.Clear();
        prefabSnapshot.Add(boxPrefab);
        prefabSnapshot.Add(spherePrefab);
        prefabSnapshot.Add(capsulePrefab);
        prefabSnapshot.Add(meshPrefab);

        countSnapshot[0] = Mathf.Max(0, boxCount);
        countSnapshot[1] = Mathf.Max(0, sphereCount);
        countSnapshot[2] = Mathf.Max(0, capsuleCount);
        countSnapshot[3] = Mathf.Max(0, meshCount);
        hasSnapshot = true;

        var random = new System.Random(randomSeed);
        SpawnPrefab(boxPrefab, countSnapshot[0], random);
        SpawnPrefab(spherePrefab, countSnapshot[1], random);
        SpawnPrefab(capsulePrefab, countSnapshot[2], random);
        SpawnPrefab(meshPrefab, countSnapshot[3], random);
    }

    private void SpawnPrefab(GameObject prefab, int count, System.Random random)
    {
        if (count == 0)
        {
            return;
        }

        if (prefab == null)
        {
            Debug.LogError($"Collider test prefab is missing for requested count {count}.", this);
            return;
        }

        for (int i = 0; i < count; i++)
        {
            Vector3 position = new Vector3(
                RandomRange(random, movementBounds.min.x, movementBounds.max.x),
                RandomRange(random, movementBounds.min.y, movementBounds.max.y),
                RandomRange(random, movementBounds.min.z, movementBounds.max.z));
            GameObject instance = Instantiate(prefab, position, prefab.transform.rotation, transform);
            spawnedObjects.Add(instance);

            Collider[] colliders = instance.GetComponentsInChildren<Collider>();
            if (colliders.Length == 0)
            {
                Debug.LogError($"Prefab '{prefab.name}' has no Collider component.", instance);
                continue;
            }

            spawnedColliders.AddRange(colliders);
            ColliderTestMover mover = instance.GetComponent<ColliderTestMover>();
            if (mover == null)
            {
                mover = instance.AddComponent<ColliderTestMover>();
            }

            mover.Initialize(movementBounds, RandomDirection(random), RandomRange(random, minSpeed, maxSpeed));
        }
    }

    private static Vector2 RandomDirection(System.Random random)
    {
        float angle = RandomRange(random, 0f, Mathf.PI * 2f);
        return new Vector2(Mathf.Cos(angle), Mathf.Sin(angle));
    }

    private static float RandomRange(System.Random random, float min, float max)
    {
        if (max < min)
        {
            (min, max) = (max, min);
        }

        return min + (float)random.NextDouble() * (max - min);
    }

    private void ClearSpawnedObjects()
    {
        spawnedColliders.Clear();
        foreach (GameObject instance in spawnedObjects)
        {
            if (instance == null)
            {
                continue;
            }

            if (Application.isPlaying)
            {
                Destroy(instance);
            }
            else
            {
                DestroyImmediate(instance);
            }
        }

        spawnedObjects.Clear();
    }

    private void OnDestroy()
    {
        ClearSpawnedObjects();
        ColliderSystem.Shutdown();
    }

    private void OnDrawGizmosSelected()
    {
        Gizmos.color = Color.cyan;
        Gizmos.DrawWireCube(movementBounds.center, movementBounds.size);
    }
}
